#ifndef IFR_ROS2_CV__PACKAGE_RM_SERIAL_PORT__SERIALPORTFULLCTRL__H
#define IFR_ROS2_CV__PACKAGE_RM_SERIAL_PORT__SERIALPORTFULLCTRL__H
#include "SerialPort.h"
#include "TargetCalc.h"
#include "packet.h"
#include <cstdint>
#include <functional>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ifr_common/common.h>
#include <ifr_interface/msg/team.hpp>
#include <memory>
#include <message_filters/subscriber.h>
#include <mutex>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <rm_common/referee_datas.h>
#include <rm_interface/msg/detail/rm_target__struct.hpp>
#include <rm_referee_interface/msg/detail/game_status__struct.hpp>
#include <rm_referee_interface/msg/game_status.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/detail/bool__struct.hpp>
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
#include <tuple>
#include <unordered_map>
#include <visualization_msgs/msg/detail/marker__struct.hpp>
namespace rm_serial_port {
    class SerialPortFullCtrl : public rclcpp::Node {
        using RSFH = RefereeSystem::packet::FrameHeader;
        template<class T>
        using RSFB = RefereeSystem::packet::FrameBody<T>;
        using RSBodyReadFunc = std::function<bool(const RSFH &, const uint16_t &)>;

    private:
        // 数据
        tf2::Quaternion imu_pose = tf2::Quaternion::getIdentity();///< 陀螺仪


        SerialPort serial;     /// 串口管理
        utils::TargetCalc tc;  /// 目标计算器
        packet::FullControl fc;/// 移动控制数据包

        std::mutex serial_write_mtx;

    private:
        // 通信

        ifr_interface::msg::Team msg_team;///< 队伍数据
        rclcpp::Publisher<decltype(msg_team)>::SharedPtr pub_team;

        std_msgs::msg::Bool msg_use_forecast;///< 预测数据
        rclcpp::Publisher<decltype(msg_use_forecast)>::SharedPtr pub_use_forecast;

        rclcpp::TimerBase::SharedPtr timer_cam_transform;
        geometry_msgs::msg::TransformStamped msg_cam_transform1, msg_cam_transform2;///< 相机坐标系转换
        std::unique_ptr<tf2_ros::TransformBroadcaster> pub_cam_transform;


        rclcpp::Subscription<rm_interface::msg::RmTarget>::SharedPtr sub_target;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_move;

        ifr_interface::msg::SerialControl msg_serial_send;
        rclcpp::Publisher<decltype(msg_serial_send)>::SharedPtr pub_serial_send;

        ifr_interface::msg::SerialImuData msg_serial_receive;
        rclcpp::Publisher<decltype(msg_serial_receive)>::SharedPtr pub_serial_receive;

        std::unordered_map<uint16_t, std::tuple<std::string, Buffer::SharedPtr, std::function<void()>>> referees;//裁判系统数据, cmd_id -> [name, buffer, handler]
        Buffer tmp_buffer;

        rclcpp::TimerBase::SharedPtr timer_send;

        bool in_game = false;
        rclcpp ::Subscription<rm_referee_interface::msg::GameStatus>::SharedPtr in_game_receive;

        struct CtrlShare {
            bool has_target;
            uint64_t clear_move = 0;
            static constexpr const uint64_t clear_move_t = 50;
            uint64_t clear_aim = 0;
            static constexpr const uint64_t clear_aim_t = 50;
        } ctrl_share;

        double yaw_follow; ///< 跟随运动的Yaw速度
        double cmd_vel_max;///< /cmd_vel的最大速度

    public:
        explicit SerialPortFullCtrl(const rclcpp::NodeOptions &options);

        void sendData();
        void updateSendDataRmTarget(const rm_interface::msg::RmTarget::SharedPtr data);
        void updateSendDataTwist(const geometry_msgs::msg::Twist::SharedPtr data);
        void readDataIMU(const packet::IMU_Data &data);
        void readDataRS(const RSFH &data);
        void updateIsGame(const rm_referee_interface::msg::GameStatus::SharedPtr data);

        template<class T>
        void add_rs_reader(rclcpp::QoS qos = rclcpp::SensorDataQoS()) {
            static constexpr const auto ID = T::ID;
            using pkg_info = RefereeSystem::pkg_info<T>;

            auto topic = RefereeSystem::getTopic<T>();//发布topic
            auto pub = this->create_publisher<typename pkg_info::ROS_TYPE>(topic, qos);
            Buffer::SharedPtr buffer = std::make_shared<Buffer>(sizeof(T));//数据缓冲区

            std::function<void()> handler;
            if constexpr (pkg_info::has_ros_type) {// 对于有对应ROS消息类型的, 将消息转换为ROS消息再推送
                handler = [pub = std::move(pub), ros = std::make_shared<typename pkg_info::ROS_TYPE>(), buffer]() {
                    auto &ros_msg = buffer->data<T>()->to(ros);//从二进制流转换为ros消息
                    pub->publish(ros_msg);
                };
            } else {// 对于没有对应ROS消息类型的, 使用原始的数据串推送(对应std_msgs::msg::String)
                handler = [pub = std::move(pub), buffer]() {
                    auto &ros_msg = buffer->raw();
                    pub->publish(ros_msg);
                };
            }
            referees[T::ID] = std::make_tuple(T::NAME, buffer, handler);
            RCLCPP_INFO_STREAM(get_logger(), "add reader: " << T::NAME << "(0x" << std::hex << ID << "), topic = " << topic);
        }
    };
}// namespace rm_serial_port
#endif// IFR_ROS2_CV__PACKAGE_RM_SERIAL_PORT__SERIALPORTFULLCTRL__H
