#include "ifr_common/defs.h"
#include "rm_armor_finder/ArmorFinder.h"
#include <cstdint>
#include <filesystem>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
namespace rm_armor_finder {
    class ArmorSaverNode : public ArmorFinder {

        uint64_t frame_id;

        std::filesystem::path save_dir;                         ///< 文件夹根目录
        std::unordered_map<uint8_t, std::filesystem::path> dirs;///< 类别对应文件夹
        std::unordered_map<uint8_t, bool> created;              ///< 类别对应文件夹


        void step8_saveTarget() {
            good_targets.clear();
            good_targets.reserve(goodPair.size());

            bad_targets.clear();
            bad_targets.reserve(goodPair.size());

            int sub_id = 0;
            ++frame_id;
            //TODO 做并行处理
            stream1.waitForCompletion();
            std::vector<Armor> result;
            identify->identifyAll(gpu_gray, goodPair, result, result);
            for (const auto &r: result) {//将所有可能的装甲板做仿射变换提取图像
                std::stringstream ss;
                ss << frame_id << '_' << (++sub_id) << "_" << static_cast<int>(r.confidence * 100) << ".jpg";

                cv::Mat cpu_armor;
                gpu_armor.download(cpu_armor);
                if (!created[r.type]) std::filesystem::create_directories(dirs[r.type]);
                cv::imwrite((dirs[r.type] / ss.str()).string(), cpu_armor * 255);
            }
        }


        void image_input(const ifr_interface::msg::BayerImage::SharedPtr msg) {
            src = cv::Mat(msg->height, msg->width, CV_8UC1, reinterpret_cast<void *>(msg->data));
            // if (pub_debug_target->get_subscription_count() > 0) cpu_debug_target = src.clone();
            frame_header = msg->header;

            step0_prepare();
            msg_back_ptr.ptr = msg->data;
            pub_back_ptr->publish(msg_back_ptr);
            step1_findContours();
            step2_getMinRect();
            step3_goodIndex();
            step4_goodPair();
            step5_calcPairRR();
            step6_testConflict();
            step7_sortPair();
            step8_saveTarget();
            step8_toTarget();
            step9_pnp();
            stepA_publish();
        }

    public:
        ArmorSaverNode(const rclcpp::NodeOptions &options) : ArmorFinder(options) {
            sub_image.reset();

            save_dir = this->declare_parameter("save_dir", "/home/ifr/ros2-cv/models/armor_real");
            save_dir /= std::to_string(static_cast<uint64_t>((this->now().seconds() * 1000)));

            for (const auto &id: ifr_interface::defs::rm::armor_id::arm_lg_ids) {
                auto dir = save_dir / std::to_string(static_cast<int>(id));
                created.insert(std::make_pair(id, false));
                dirs.insert(std::make_pair(id, dir));
            }
            for (const auto &id: ifr_interface::defs::rm::armor_id::arm_sm_ids) {
                auto dir = save_dir / std::to_string(static_cast<int>(id));
                created.insert(std::make_pair(id, false));
                dirs.insert(std::make_pair(id, dir));
            }


            frame_id = 0;

            sub_image = this->create_subscription<ifr_interface::msg::BayerImage>(
                    "/image_bayer/image", rclcpp::SensorDataQoS(),
                    std::bind(&ArmorSaverNode::image_input, this, std::placeholders::_1));
        }
    };
}// namespace rm_armor_finder

// Register node as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_armor_finder::ArmorSaverNode)