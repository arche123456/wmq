#ifndef IFR_ROS2_CV__PACKAGE_RM_ARMOR_FINDER__ARMORIDENTIFY__H
#define IFR_ROS2_CV__PACKAGE_RM_ARMOR_FINDER__ARMORIDENTIFY__H
#include "Armor.h"
#include "Values.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdint>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/timer.hpp>
#include <string>
#if USE_MLP
#include "ArmorNet.hpp"
#include <atomic>
#include <chrono>
#include <filesystem>
#include <memory>
#include <mutex>
#include <queue>
#include <torch/torch.h>
namespace rm_armor_finder {
    class MLP;
    class ArmorIdentify;
    /// 共享显存的tensor
    class SharedMemoryTensor {
    public:
        /// @param n 批大小
        /// @param w 宽度
        /// @param h 高度
        SharedMemoryTensor(Values *values, int n, int w, int h);

        ~SharedMemoryTensor();

        /// @brief 获取第index个GpuMat的引用
        cv::cuda::GpuMat getGpuMat(int index);

        /// @brief 复制到第index个GpuMat上
        void setGpuMat(const cv::cuda::GpuMat &src, int index);

        torch::Tensor getTensor(int size);


        torch::Tensor tensor;// n * w * h
        const int batch_size, width, height;
        const size_t img_size;//图像大小, w * h

    private:
        void *get_ptr(int index);
        void *host_memory = nullptr;  //cpu内存, 只在部分调试情况下才会非null
        void *device_memory = nullptr;//gpu显存
        const torch::TensorOptions options = torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCUDA);
    };
    /// @brief 模型及其缓存数据管理
    /// @details 存储一个网络运行所需的所有数据
    class MLP {
        friend class ArmorIdentify;
        std::string type;                           // 类型名称
        mlp::ArmorNet model;                        //模型
        std::shared_ptr<SharedMemoryTensor> tensors;//tensor转换缓存
        std::vector<size_t> idx;                    //索引缓存
        const int batch_size;                       //批大小
        const cv::Size img_size;                    //图像大小
        cv::cuda::GpuMat number_mat;                //gpumat缓存

    public:
        MLP(Values *values, rclcpp::Node *node, std::filesystem::path dir, std::string type,
            cv::Size img_size, size_t class_num, int batch_size = 64);
    };

    /// @brief 装甲板识别器
    class ArmorIdentify {
        Values *values;
        std::unique_ptr<MLP> net_lg, net_sm;
        const std::set<uint8_t> arm_ignore_ids;
        const cv::Point2f target[4];

    public:
        ArmorIdentify(Values *values,
                      const std::string &model_name,
                      const std::vector<uint8_t> &ignores);
        /// @brief 识别所有装甲板
        /// @param src 原始图像(二值图)
        /// @param pairs 所有装甲板轮廓
        /// @param good 好装甲板结果集
        /// @param bad 坏装甲板结果集
        /// @param is_small 是否是缩小一半的src
        void identifyAll(const cv::cuda::GpuMat &src, std::vector<ContourPair> &pairs,
                         std::vector<Armor> &good, std::vector<Armor> &bad, bool is_small = true);

        /// @brief 识别一批装甲板
        /// @param net 所使用的网络
        /// @param pairs 所有装甲板轮廓
        /// @param cnt 批大小
        /// @param is_lg 此批是否是大装甲板
        /// @param good 好装甲板结果集
        /// @param bad 坏装甲板结果集
        void identifyBatch(const std::unique_ptr<MLP> &net, std::vector<ContourPair> &pairs,
                           const int &cnt, bool is_lg,
                           std::vector<Armor> &good, std::vector<Armor> &bad);

        /// @brief 返回数字区域的图片
        cv::cuda::GpuMat debug_number_mat(bool is_lg) const;
    };

    /// @brief 会依据参数保存装甲板
    class ArmorSaver {
        rclcpp::Node::SharedPtr node;   // 子节点
        std::queue<cv::Mat> imgs;       /// 待保存队列
        std::mutex mtx;                 //线程锁
        Values *const values;           // 参数
        const std::filesystem::path dir;// 保存文件夹
        const int max_queue_size;
        const int max_batch;
        const int max_save_size;
        bool enable = false;// 是否开启, 如果为false, 此类不会做任何事
        std::thread save_thread;
        std::condition_variable cv;

        uint64_t save_idx = 0;

        static std::filesystem::path get_save_dir();
        void save();

    public:
        /// @brief 创建装甲板图片保存器
        /// @param values 所有参数
        /// @param max_queue_size 队列最大大小
        /// @param max_batch 单批保存最大大小
        /// @param max_save_size 最大保存数量
        ArmorSaver(Values *values, int max_queue_size = 128, int max_batch = 64, int max_save_size = 4096);
        ~ArmorSaver();

        /// @brief 添加一批图像
        /// @param data cpu数据指针
        /// @param count 此批图片量
        /// @param w 图片宽
        /// @param h 图片高
        /// @details data是一个连续数据指针, 大小为count*w*h字节
        void add_img(void *data, size_t count, int w, int h);
    };

}// namespace rm_armor_finder
#else

namespace rm_armor_finder {
    /// @brief 装甲板识别器
    class ArmorIdentify {

    public:
        ArmorIdentify(Values *values,
                      const std::string &model_name,
                      const std::vector<uint8_t> &ignores);
        /// @brief 识别所有装甲板
        /// @param src 原始图像(二值图)
        /// @param pairs 所有装甲板轮廓
        /// @param good 好装甲板结果集
        /// @param bad 坏装甲板结果集
        /// @param is_small 是否是缩小一半的src
        void identifyAll(const cv::cuda::GpuMat &src, std::vector<ContourPair> &pairs,
                         std::vector<Armor> &good, std::vector<Armor> &bad, bool is_small = true);
    };
}// namespace rm_armor_finder
#endif// USE_MLP
#endif// IFR_ROS2_CV__PACKAGE_RM_ARMOR_FINDER__ARMORIDENTIFY__H
