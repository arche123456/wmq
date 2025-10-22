#ifndef IFR_ROS2_CV__PACKAGE_RM_SERIAL_PORT__SERIALPORTMOTOR__H
#define IFR_ROS2_CV__PACKAGE_RM_SERIAL_PORT__SERIALPORTMOTOR__H
#include "SerialPort.h"
#include "TargetCalc.h"
#include "packet.h"
#include <atomic>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <ifr_interface/msg/aim_type.hpp>
#include <ifr_interface/msg/detail/serial_control__struct.hpp>
#include <ifr_interface/msg/detail/serial_imu_data__struct.hpp>
#include <ifr_interface/msg/team.hpp>
#include <message_filters/subscriber.h>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rm_interface/msg/detail/rm_target__struct.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/detail/bool__struct.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/detail/marker__struct.hpp>
namespace rm_serial_port {
    class SerialPortMotor : public rclcpp::Node {
    private:
        SerialPort serial;      /// 串口管理
        utils::TargetCalc tc;   /// 目标计算器
        packet::MotorControl mc;/// 移动控制数据包

        tf2::Quaternion imu_pose = tf2::Quaternion::getIdentity();/// 陀螺仪

    private:
        // 通信

        ifr_interface::msg::Team msg_team;///< 队伍数据
        rclcpp::Publisher<decltype(msg_team)>::SharedPtr pub_team;

        ifr_interface::msg::AimType msg_aim_type;///< 自瞄类型
        rclcpp::Publisher<decltype(msg_aim_type)>::SharedPtr pub_aim_type;

        std_msgs::msg::Bool msg_use_forecast;///< 预测数据
        rclcpp::Publisher<decltype(msg_use_forecast)>::SharedPtr pub_use_forecast;

        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_srv_;///< 刷新自瞄客户端
        std::atomic_bool reset_tracker_srv_busy_;
        rclcpp::Time reset_tracker_srv_last_time_;

        rclcpp::TimerBase::SharedPtr timer_cam_transform;
        geometry_msgs::msg::TransformStamped msg_cam_transform;///< 相机坐标系转换
        std::unique_ptr<tf2_ros::TransformBroadcaster> pub_cam_transform;


        rclcpp::Subscription<rm_interface::msg::RmTarget>::SharedPtr sub_target;///< 攻击目标

        ifr_interface::msg::SerialControl msg_serial_send;///< 串口数据
        rclcpp::Publisher<decltype(msg_serial_send)>::SharedPtr pub_serial_send;

        ifr_interface::msg::SerialImuData msg_serial_receive;///< 串口数据
        rclcpp::Publisher<decltype(msg_serial_receive)>::SharedPtr pub_serial_receive;

        void readData(const packet::IMU_Data &data);
        void sendData(const rm_interface::msg::RmTarget::SharedPtr data);

        void needToFlash();///< 自瞄刷新客户端发送数据

    public:
        SerialPortMotor(const rclcpp::NodeOptions &opt = rclcpp::NodeOptions());
    };
}// namespace rm_serial_port
#endif// IFR_ROS2_CV__PACKAGE_RM_SERIAL_PORT__SERIALPORTMOTOR__H
