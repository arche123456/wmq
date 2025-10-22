#include "rm_armor_finder/ArmorIdentify.h"
#include <cuda_runtime.h>
#include <opencv2/core/base.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/hal/interface.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#if USE_MLP
#include "rm_armor_finder/ArmorFinder.h"
#include <c10/core/DeviceType.h>
#include <filesystem>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <torch/script.h>
namespace rm_armor_finder {
    void ArmorIdentify::identifyAll(const cv::cuda::GpuMat &src, std::vector<ContourPair> &pairs,
                                    std::vector<Armor> &good, std::vector<Armor> &bad, bool is_small) {
        if (!values->use_predict) {
            for (const auto &p: pairs) {
                auto type = p.is_large ? ifr_interface::defs::rm::armor_id::arm_unknown_lg : ifr_interface::defs::rm::armor_id::arm_unknown_sm;
                good.emplace_back(p.pts, p.is_large, -1, type);
            }
            return;
        }
        int cnt_sm = 0, cnt_lg = 0;
        cv::Point2f pts[4];
        for (size_t i = 0; i < pairs.size(); ++i) {
            auto &p = pairs[i];
            if (p.skip) continue;
            const auto &net = (p.is_large ? net_lg : net_sm);
            auto &cnt = p.is_large ? cnt_lg : cnt_sm;
            if (is_small)
                for (int i = 0; i < 4; i++) {
                    pts[i] = p.pts.number_pt[i];
                    pts[i] /= 2;
                }
            auto rot = cv::getPerspectiveTransform(is_small ? pts : p.pts.number_pt, target);
            cv::cuda::warpPerspective(src, net->tensors->getGpuMat(cnt), rot, net->img_size, cv::INTER_NEAREST);
            // net->tensors->setGpuMat(net->number_mat, cnt);
            net->idx[cnt] = i;
            cnt++;
            if (cnt == net->batch_size) {
                RCLCPP_WARN_STREAM(values->get_node()->get_logger(), "Found too much armors. Multiple 'identifyBatch' are required: pairs = " << pairs.size());
                identifyBatch(net, pairs, cnt, p.is_large, good, bad);
                cnt = 0;
            }
        }
        identifyBatch(net_lg, pairs, cnt_lg, true, good, bad);
        identifyBatch(net_sm, pairs, cnt_sm, false, good, bad);
    }

    void ArmorIdentify::identifyBatch(const std::unique_ptr<MLP> &net, std::vector<ContourPair> &pairs,
                                      const int &cnt, bool is_lg,
                                      std::vector<Armor> &good, std::vector<Armor> &bad) {
        if (cnt <= 0) return;
        auto tensor = net->tensors->getTensor(cnt);
        auto result = net->model->forward(tensor).to(torch::kCPU).max(1);
        auto result_val = std::get<0>(result).data_ptr<float>();
        auto result_idx = std::get<1>(result).data_ptr<long>();

        using namespace ifr_interface::defs::rm::armor_id;
        const auto &types = is_lg ? arm_lg_ids : arm_sm_ids;
        for (int i = 0; i < cnt; ++i) {
            auto type = types[result_idx[i]];
            auto confidence = result_val[i];
            bool is_bad = confidence < values->model_threshold || arm_ignore_ids.count(type);
            (is_bad ? bad : good).emplace_back(pairs[net->idx[i]].pts, is_lg, confidence, type);
        }
    }
    cv::cuda::GpuMat ArmorIdentify::debug_number_mat(bool is_lg) const {
        return (is_lg ? net_lg : net_sm)->tensors->getGpuMat(0);
    }
    ArmorIdentify::ArmorIdentify(Values *values,
                                 const std::string &model_name,
                                 const std::vector<uint8_t> &ignores)
        : values(values), arm_ignore_ids(ignores.begin(), ignores.end()) {
        std::filesystem::path dir = ament_index_cpp::get_package_share_directory("rm_armor_finder");
        dir = dir / "model";
        const auto size = cv::Size(values->arm_to_w, values->arm_to_h);
        using namespace ifr_interface::defs::rm::armor_id;
        net_lg = std::make_unique<MLP>(values, values->get_node(), dir, "lg", size, arm_lg_num);
        net_sm = std::make_unique<MLP>(values, values->get_node(), dir, "sm", size, arm_sm_num);

        const auto h = float(values->arm_to_h), w = float(values->arm_to_w);
        auto target = const_cast<cv::Point2f *>(this->target);
        target[0] = {0, 0}, target[1] = {0, h}, target[2] = {w, h}, target[3] = {w, 0};
    }
    SharedMemoryTensor::SharedMemoryTensor(Values *values, int n, int w, int h)
        : batch_size(values->use_predict ? n : 1), width(w), height(h), img_size(width * height * sizeof(float)),
          options(torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCUDA)) {
        if (values->predict_img_maybe_save) {
            auto armor_finder_ptr = dynamic_cast<ArmorFinder *>(values->get_node());
            CV_Assert(armor_finder_ptr != nullptr);
            armor_finder_ptr->memMapPool->malloc(&host_memory, &device_memory, batch_size * img_size);
        } else {
            cudaMalloc(&device_memory, batch_size * img_size);
        }
        tensor = torch::from_blob(device_memory, {batch_size, height, width}, options);
    }
    SharedMemoryTensor::~SharedMemoryTensor() { cudaFree(device_memory); }
    cv::cuda::GpuMat SharedMemoryTensor::getGpuMat(int index) {
        return cv::cuda::GpuMat(height, width, CV_32FC1, get_ptr(index));
    }
    void SharedMemoryTensor::setGpuMat(const cv::cuda::GpuMat &src, int index) {
        CV_Assert(src.rows == height && src.cols == width && src.type() == CV_8UC1);
        cv::cuda::GpuMat dst(height, width, CV_32FC1, get_ptr(index));
        src.convertTo(dst, CV_32FC1, 1.0);
    }
    torch::Tensor SharedMemoryTensor::getTensor(int size) {
        CV_Assert(0 < size && size <= batch_size);
        return torch::from_blob(device_memory, {size, height, width}, options);
    }
    void *SharedMemoryTensor::get_ptr(int index) {
        CV_Assert(0 <= index && index < batch_size);
        char *dst_ptr = static_cast<char *>(device_memory) + index * img_size;
        return dst_ptr;
    }
    MLP::MLP(Values *values, rclcpp::Node *node, std::filesystem::path dir, std::string type,
             cv::Size img_size, size_t class_num, int batch_size)
        : type(type), model(class_num, img_size.area()), batch_size(batch_size), img_size(img_size) {
        idx.resize(batch_size);
        if (values->use_predict) {
            std::string filepath = dir / (type + ".pt");
            if (!std::filesystem::exists(filepath)) {
                std::stringstream ss;
                ss << "[MLP] Not Found Model File[" << type << "]: " << filepath;
                RCLCPP_FATAL(node->get_logger(), ss.str().c_str());
                throw std::runtime_error(ss.str());
            }
            RCLCPP_INFO_STREAM(node->get_logger(), "[MLP] Loading model: " << filepath);
            try {
                torch::load(model, filepath);
                model->to(at::kCUDA);
            } catch (c10::Error &e) {
                RCLCPP_FATAL_STREAM(node->get_logger(), "Failed to load " << filepath << ", c10::Error: " << e.what());
                throw e;
            } catch (std::exception &e) {
                RCLCPP_FATAL_STREAM(node->get_logger(), "Failed to load " << filepath << ", std::exception: " << e.what());
                throw e;
            }
            RCLCPP_INFO(node->get_logger(), "Loaded Model[%s]: %s", type.c_str(), filepath.c_str());
        } else {
            RCLCPP_INFO(node->get_logger(), "Skip Load Model[%s]: %s", type.c_str());
        }
        tensors = std::make_shared<SharedMemoryTensor>(values, batch_size, img_size.width, img_size.height);
        number_mat.create(img_size, CV_8UC1);
    }

    ArmorSaver::ArmorSaver(Values *values, int max_queue_size, int max_batch, int max_save_size)
        : node(values->get_node()->create_sub_node("ArmorSaver")),
          values(values), dir(get_save_dir()),
          max_queue_size(max_queue_size), max_batch(max_batch), max_save_size(max_save_size),
          save_thread(std::bind(&ArmorSaver::save, this)) {
        CV_Assert(max_queue_size > 0);
        namespace fs = std::filesystem;
        if (values->predict_img_maybe_save) {
            std::error_code err;
            if (fs::create_directories(dir, err)) {
                enable = true;
                RCLCPP_INFO_STREAM(node->get_logger(), "Create save dir: " << dir);
            } else {
                RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to create save dir: " << dir << ", error = " << err << " / " << err.message());
            }
        }
    }
    ArmorSaver::~ArmorSaver() {
        if (enable) {
            enable = false;
            cv.notify_all();
        }
    }
    void ArmorSaver::add_img(void *data, size_t count, int w, int h) {
        if (!enable || !values->predict_img_saving || imgs.size() >= max_queue_size) return;
        std::lock_guard lock(mtx);
        for (size_t i = 0; i < count; i++) {
            if (imgs.size() >= max_queue_size) return;
            imgs.emplace(cv::Mat{h, w, CV_8UC1, data}.clone());
            data = static_cast<uint8_t *>(data) + w * h;// 移动指针以指向下一图像数据
        }
        if (enable)
            cv.notify_one();
        else
            while (!imgs.empty()) imgs.pop();
    }
    void ArmorSaver::save() {
        std::unique_lock<std::mutex> lk(mtx);
        while (enable && rclcpp::ok()) {
            cv.wait(lk, [&]() { return !imgs.empty() || !enable || !rclcpp::ok(); });
            int cnt = 0;
            while (!imgs.empty()) {
                const auto data = imgs.front();
                imgs.pop();
                lk.unlock();
                if (++save_idx > max_save_size) return;
                std::string name = std::to_string(save_idx) + ".png";
                cv::imwrite(dir / name, data);
                lk.lock();
                if (++cnt >= max_batch) {
                    enable = false;
                    while (!imgs.empty()) imgs.pop();
                    RCLCPP_WARN_STREAM(node->get_logger(), "Reached maximum limit " << max_batch << ", stop saving. All images have been saved to: " << dir);
                    return;
                }
            }
        }
    }

    /// 检查 path 是否以 end 结尾
    /// 忽略路径符不一致问题
    constexpr bool __path_checker(const char *path, const char *end) {
        std::size_t path_len = 0, end_len = 0;
        while (path[path_len]) ++path_len;
        while (end[end_len]) ++end_len;

        while (end_len > 0 && path_len > 0) {
            const auto &p = path[path_len - 1];
            const auto &e = end[end_len - 1];
            --path_len;
            --end_len;
            if (p == e) continue;
            if ((p == '/' || p == '\\') && (e == '/' || e == '\\')) continue;
            return false;
        }
        return end_len <= 0;
    }
    std::filesystem::path ArmorSaver::get_save_dir() {
        auto now = std::chrono::system_clock::now();
        std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
        std::tm *now_tm = std::localtime(&now_time_t);
        std::ostringstream oss;
        oss << std::put_time(now_tm, "%Y%m%d-%H-%M-%S");
        static_assert(__path_checker(__FILE__, "src/ArmorIdentify.cpp"), "Bad src path!!! if move src, change this");
        std::filesystem::path SELF = __FILE__;
        // std::filesystem::dir
        auto root = SELF.parent_path().parent_path();
        auto save_dir = root / "rm_armor_imgs" / oss.str();
        return save_dir;
    }
}// namespace rm_armor_finder
#else

namespace rm_armor_finder {

    ArmorIdentify::ArmorIdentify(Values *values,
                                 const std::string &model_name,
                                 const std::vector<uint8_t> &ignores) {
    }
    void ArmorIdentify::identifyAll(const cv::cuda::GpuMat &src, std::vector<ContourPair> &pairs,
                                    std::vector<Armor> &good, std::vector<Armor> &bad, bool is_small) {
        for (const auto &p: pairs) {
            auto type = p.is_large ? ifr_interface::defs::rm::armor_id::arm_unknown_lg : ifr_interface::defs::rm::armor_id::arm_unknown_sm;
            good.emplace_back(p.pts, p.is_large, -1, type);
        }
    }

}// namespace rm_armor_finder
#endif