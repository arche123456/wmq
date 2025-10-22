#ifndef IFR_ROS2_CV__PACKAGE_RM_SERIAL_PORT__TARGETCALC__H
#define IFR_ROS2_CV__PACKAGE_RM_SERIAL_PORT__TARGETCALC__H
#include "ifr_common/ext_funcs.h"
#include <geometry_msgs/msg/detail/point_stamped__struct.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rm_interface/msg/detail/rm_target__struct.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
namespace rm_serial_port::utils {
    ///@brief 目标计算模块
    ///@details 用于从预测器输出结果选择最佳攻击
    class TargetCalc {
    public:
        struct ArmorTarget {
            bool can_attack{};
            double move_x{};
            double move_y{};
        };

    private:
        static constexpr const double r2d = 180 / M_PI;
        static constexpr const double d2r = M_PI / 180;

        static constexpr const double g = 9.80665;///< 重力加速度
        double v0, g_v02;                         //g_v02 = g/(v0^2)

        double attack_max_yaw;///<  攻击最大偏航角度差值, 即向左或向右此度数范围内可攻击

        double gyro_threshold;///< 可以被判定为小陀螺的yaw轴转速

        double time_delta;///< 预测时间补偿(s)

        std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
        std::string frame_gun;//枪口坐标系名称

        rclcpp::Node *node;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle;///< 监听回调句柄

        rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &params);
        void setV0(double v0);
        void setAMY(double attack_max_yaw);
        void setGyroThreshold(double gyro_threshold);
        void setTimeDelta(double time_delta);


        /// 获取飞行时间 t
        /// @param data 目标数据
        /// @param dt 数据采集到最新的时间差
        /// @return 从数据采集到子弹命中的时间差
        double getT(const rm_interface::msg::RmTarget::SharedPtr &data, double dt) const;

        /// 计算弹道补偿值
        /// @param p 攻击位置
        /// @return 枪口上抬角度
        double getAlpha(const geometry_msgs::msg::Point &p) const;

        /// 转换结果
        /// @param attack 是否要攻击
        /// @param 变换前攻击位置
        /// @details 会叠加弹道补偿数据
        ArmorTarget getResult(bool attack, geometry_msgs::msg::PointStamped &position) const;

        /// 将坐标计算平移到枪口
        void transformToGun(geometry_msgs::msg::PointStamped &position) const;

        bool canAttack(const cv::Point2d &car_center, const cv::Point2d &armor_center, double yaw, const bool &is_lg) const;

        FORCE_INLINE bool cross_test(const cv::Point2d &B, const cv::Point2d &C, const cv::Point2d &D);

    public:
        TargetCalc(rclcpp::Node *node);

        /// 获取最佳装甲板击打位置
        /// @param data 目标数据
        /// @return 攻击信息
        ArmorTarget getArmor(const rm_interface::msg::RmTarget::SharedPtr &data) const;
    };
}// namespace rm_serial_port::utils

#endif// IFR_ROS2_CV__PACKAGE_RM_SERIAL_PORT__TARGETCALC__H
