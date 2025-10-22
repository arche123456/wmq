#ifndef IFR_ROS2_CV__PACKAGE_RM_ARMOR_FINDER__ARMORFINDER__H
#define IFR_ROS2_CV__PACKAGE_RM_ARMOR_FINDER__ARMORFINDER__H

#include "Armor.h"
#include "ArmorIdentify.h"
#include "DebugPublisher.h"
#include "PnP.h"
#include "Values.h"
#include "ifr_common/defs.h"
#include "ifr_watcher/cpu_watcher.hpp"
#include "mem_map.h"
#include <cuda_runtime.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <ifr_interface/msg/aim_type.hpp>
#include <ifr_interface/msg/detail/bayer_image__struct.hpp>
#include <ifr_interface/msg/detail/common_ptr__struct.hpp>
#include <memory>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/node.hpp>
#include <rm_interface/msg/detail/rm_armor__struct.hpp>
#include <rm_interface/msg/detail/rm_armors__struct.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>
#include <visualization_msgs/msg/image_marker.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace rm_armor_finder {

    class ArmorFinder : public rclcpp::Node {
        ifr_watcher::cpu::RegisterClient cpu_watcher{"node", this};

    public:
        ///字段

        cv::cuda::Stream stream1{cudaStreamNonBlocking}, stream2{cudaStreamNonBlocking};///< 异步流
        std::unique_ptr<MemMapPool> memMapPool;                                         ///< 内存映射池
        std::unique_ptr<Values> values;                                                 ///< 所有参数
        std::unique_ptr<ArmorIdentify> identify;                                        ///< 装甲板识别器
        std::unique_ptr<PnPSolver> pnp;                                                 ///< pnp解算器
        /// 缓存机制相关
        // std::vector<cv::Point> cached_contour;///< 缓存轮廓点
        // cv::RotatedRect last_valid_result;    ///< 最后有效的旋转矩形

    protected:
        ///数据

        ifr_interface::defs::Team self_team = ifr_interface::defs::Team::BLUE;///< 自身队伍
        std_msgs::msg::Header frame_header;


    protected:
        ///步骤数据
        bool can_run = true;///< 根据自瞄模式选择是否运行

        cv::Mat src;             ///< cpu原图
        cv::cuda::GpuMat gpu_src;///< gpu原图

        cv::cuda::GpuMat gpu_gray;///< gpu灰度图(阈值)

        cv::Mat cpu_enemy;         ///< cpu 的颜色结果  (仅敌人的颜色)
        cv::cuda::GpuMat gpu_enemy;///< gpu 的颜色结果  (仅敌人的颜色)

        cv::Mat cpu_debug_target;///< 调试模式下目标信息图


        cv::cuda::GpuMat gpu_armor;///< gpu 的装甲板

        // 保存相机参数，用于三分法优化
        cv::Mat camera_matrix_for_trisection;
        cv::Mat dist_coeffs_for_trisection;

        float src_size;                              ///< 原图尺寸(像素数量)
        std::vector<std::vector<cv::Point>> contours;///< 所有轮廓
        std::vector<cv::Vec4i> hierarchy;            ///< 轮廓关系
        std::vector<LightBar> lbs;                   ///< 所有轮廓的最小包围
        std::vector<size_t> goodIndex;               ///< 所有较好的轮廓下标
        std::vector<ContourPair> goodPair;           ///< 所有较好的轮廓对
        std::vector<Armor> good_targets;             ///< 好结果
        std::vector<Armor> bad_targets;              ///< 坏结果


        rm_interface::msg::RmArmor msg_armor;///< 单个装甲板消息

        visualization_msgs::msg::Marker msg_armor_marker; ///< 单个装甲板标记
        visualization_msgs::msg::Marker msg_badpnp_marker;///< pnp另一组解标记
        visualization_msgs::msg::Marker msg_text_marker;  ///< 单个装甲板类别标记

        // 三分法优化yaw相关常量和函数声明
        const double ARMOR_FIXED_PITCH = M_PI / 12.0;// 15度 = π/12
        const double ARMOR_FIXED_ROLL = 0.0;         // 固定为0
        const double FIXED_ROLL_RAD = 0.0;
        const double FIXED_PITCH_RAD = ARMOR_FIXED_PITCH;
        const int FIND_ANGLE_ITERATIONS = 12;    // 三分法迭代次数
        const double YAW_SEARCH_RANGE = M_PI / 2;// 搜索范围(±90度)

        // 获取四元数的RPY角
        void getEulerFromQuaternion(const geometry_msgs::msg::Quaternion &q, double &roll, double &pitch, double &yaw);

        // 设置四元数的RPY角
        void setQuaternionFromEuler(geometry_msgs::msg::Quaternion &q, double roll, double pitch, double yaw);

        // 三分法优化yaw角
        void optimizeYawWithTrisection(const Armor &armor, geometry_msgs::msg::Pose &pose);

        // 计算投影误差（代价函数）
        double calculateProjectionError(const Armor &armor, const geometry_msgs::msg::Pose &pose, double yaw);

    protected:
        ///消息

        ImgPublisher img_publisher;
        ImgMarkerPublisher img_marker_publisher;

        rm_interface::msg::RmArmors msg_armors;///< 所有装甲板消息
        rclcpp::Publisher<decltype(msg_armors)>::SharedPtr pub_armors;

        geometry_msgs::msg::Pose msg_debug_armor;
        rclcpp::Publisher<decltype(msg_debug_armor)>::SharedPtr pub_debug_armor;

        visualization_msgs::msg::MarkerArray msg_marker_arr;///< 所有标记
        rclcpp::Publisher<decltype(msg_marker_arr)>::SharedPtr pub_marker_arr;

        visualization_msgs::msg::ImageMarker msg_2d_marker;///< 所有图像标记
        rclcpp::Publisher<decltype(msg_2d_marker)>::SharedPtr pub_2d_marker;

        ifr_interface::msg::CommonPtr msg_back_ptr;///< 图像指针返还
        rclcpp::Publisher<decltype(msg_back_ptr)>::SharedPtr pub_back_ptr;

        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_cam_info;///< 相机参数
        rclcpp::Subscription<ifr_interface::msg::BayerImage>::SharedPtr sub_image; ///< 图像输入
        rclcpp::Subscription<ifr_interface::msg::Team>::SharedPtr sub_team;        ///< 队伍输入
        rclcpp::Subscription<ifr_interface::msg::AimType>::SharedPtr sub_aim_type; ///< 自瞄类型

        // std_msgs::msg::Float32MultiArray msg_debug_angle;///< pnp解算测试角度
        // rclcpp::Publisher<decltype(msg_debug_angle)>::SharedPtr pub_debug_angle;

    protected:
        ///执行步骤

        void step0_prepare();///< 准备数据

        void step1_findContours();///< 寻找轮廓

        void step2_getMinRect();///< 最小包围矩形

        void step3_goodIndex();///< 初步筛选

        void step4_goodPair();///< 灯条配对

        void step5_calcPairRR();///< 计算灯条组的包围矩形

        void step6_testConflict();///< 灯条对的交集测试, 当一个灯条对的区域内包含另一个灯条的中心时, 则忽略此灯条对

        void step7_sortPair();///< 排序灯条, 将跳过的放在最后，将分数小的放在前面

        void step8_toTarget();///< 将包围矩形做仿射变换并使用net判别

        void step9_pnp();///< 将装甲板进行PNP解算, 得到结果

        void stepA_publish();///< 发布数据

    protected:
        ///其它

        /**
         * 图像输入
         * @param msg 图像数据
         */
        void image_input(const ifr_interface::msg::BayerImage::SharedPtr msg);

        void drawDebug();

    public:
        ArmorFinder(const rclcpp::NodeOptions &options);

    private:
        cv::RotatedRect computeStableRotatedRect(const std::vector<cv::Point> &contour);
    };
}// namespace rm_armor_finder

#endif// IFR_ROS2_CV__PACKAGE_RM_ARMOR_FINDER__ARMORFINDER__H
