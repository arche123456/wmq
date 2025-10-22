#include "rm_serial_port/SerialPortMotor.h"
#include "ifr_common/defs.h"
#include "rm_serial_port/SerialPort.h"
#include "rm_serial_port/packet.h"
#include <chrono>
#include <cstdint>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/detail/point_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/vector3_stamped__struct.hpp>
#include <ifr_common/defs.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/timer.hpp>
#include <rm_interface/msg/detail/rm_target__struct.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/create_timer_ros.h>

namespace rm_serial_port {

    SerialPortMotor::SerialPortMotor(const rclcpp::NodeOptions &options)
        : rclcpp::Node("serial_motor", options), serial(this), tc(this) {
        RCLCPP_INFO(this->get_logger(), "SerialPort is starting...");

        // Create Publisher
        pub_team = this->create_publisher<decltype(msg_team)>("/rm/self_team", 10);
        pub_use_forecast = this->create_publisher<decltype(msg_use_forecast)>("/serial/use_forecast", rclcpp::SensorDataQoS());
        pub_serial_send = this->create_publisher<decltype(msg_serial_send)>("/serial/send", rclcpp::SensorDataQoS());
        pub_serial_receive = this->create_publisher<decltype(msg_serial_receive)>("/serial/revice", rclcpp::SensorDataQoS());
        pub_aim_type = this->create_publisher<decltype(msg_aim_type)>("/rm/aim_type", 10);

        // 创建服务
        reset_tracker_srv_ = this->create_client<std_srvs::srv::Trigger>("/rm/tracker/reset");


        // 监听攻击目标
        sub_target = this->create_subscription<rm_interface::msg::RmTarget>(
                "/rm/tracker/target", rclcpp::SensorDataQoS(),
                std::bind(&SerialPortMotor::sendData, this, std::placeholders::_1));


        //陀螺仪数据广播
        msg_cam_transform.header.frame_id = "odom";
        msg_cam_transform.child_frame_id = "gimbal_link";
        pub_cam_transform = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        timer_cam_transform = this->create_wall_timer(std::chrono::milliseconds(10), [this]() {
            pub_cam_transform->sendTransform(msg_cam_transform);
        });

        serial.setReader<packet::IMU_Data>(std::bind(&SerialPortMotor::readData, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "SerialPort successfully started!");
    }

    void SerialPortMotor::needToFlash() {
        if (reset_tracker_srv_busy_.exchange(true)) return;// 上一次刷新未完成
        const auto now = this->now();
        if (now - reset_tracker_srv_last_time_ < std::chrono::milliseconds(200)) return;// 200ms内不重复刷新
        reset_tracker_srv_last_time_ = std::move(now);

        //RCLCPP_INFO(this->get_logger(), "请求刷新自瞄");
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

        reset_tracker_srv_->async_send_request(request, [this, now](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            reset_tracker_srv_busy_ = false;
            if (!rclcpp::ok()) return;
            const auto ftime = static_cast<int>((this->now() - now).seconds() * 1e3);
            try {
                auto response = future.get();
                RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1e3, (response->success ? "Success" : "Failed") << " To Flash Aim, time=" << ftime << "ms");
            } catch (const std::exception &e) {
                RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1e3, "Failed to call flash aim service: " << e.what());
            }
        });
    }


    void SerialPortMotor::sendData(const rm_interface::msg::RmTarget::SharedPtr data) {
        if (data->has_target) {
            auto at = tc.getArmor(data);
            // RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            //                             "Calc: "//
            //                                     << data->position.x << ", " << data->position.y << ", " << data->position.z << "; "
            //                                     << data->velocity.x << ", " << data->velocity.y << ", " << data->velocity.z << "; "
            //                                     << at.move_x << ", " << at.move_y//
            // );
            mc.set(at.can_attack, at.move_x, at.move_y);
        } else {
            mc.set(false, 0, 0);
        }
        auto now = this->now();
        const auto time = (now - data->receive_time).seconds() * 1e3;
        mc.finish(data->has_target, time);
        serial.writeData(mc);

        mc.toRos(msg_serial_send);
        msg_serial_send.delay = time;
        pub_serial_send->publish(msg_serial_send);


        static constexpr const auto r2d = 180 / M_PI;
        RCLCPP_INFO_STREAM_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Send: [ " << ((int(mc.has_target) << 1) + mc.can_attack)
                           << ", " << (mc.move_x * 1e-4 * r2d)
                           << ", " << (mc.move_y * 1e-4 * r2d)
                           << ", " << int(mc.delay) << "(in->out=" << time << "ms)"
                           << " ] "//
        );
    }

    void SerialPortMotor::readData(const packet::IMU_Data &data) {
        static constexpr const auto d2r = M_PI / 180;

        data.toRos(msg_serial_receive);

        switch (static_cast<packet::IMU_Data::Type>(data.imu_type)) {
            case packet::IMU_Data::Type::Abt:
                imu_pose.setRPY(data.getRoll() * d2r, data.getPitch() * d2r, data.getYaw() * d2r);/// 艾博特坐标系2
                break;
            case packet::IMU_Data::Type::Lxs: {
                tf2::Quaternion q_roll_yaw, q_pitch;
                q_roll_yaw.setRPY(data.getRoll() * d2r, 0, data.getYaw() * d2r);
                q_pitch.setRPY(0, data.getPitch() * d2r, 0);
                imu_pose = q_roll_yaw * q_pitch;/// 廖旭昇坐标系 PRY
                break;
            }
            // case packet::IMU_Data::Type::Jyh:
            //     imu_pose.setRPY(data.getRoll() * d2r, -data.getPitch() * d2r, data.getYaw() * d2r);/// 贾宇航坐标系
            //     break;
            default:
                RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1e3,
                                             "Unknown IMU data type: " << int(data.imu_type));
                break;
                // imu_pose.setEuler(data.getYaw() * d2r, data.getPitch() * d2r, data.getRoll() * d2r);/// 艾博特坐标系1
                // imu_pose.setRPY(data.getRoll() * d2r, data.getPitch() * d2r, data.getYaw() * d2r);
        }

        msg_cam_transform.transform.rotation.set__x(imu_pose.x())
                .set__y(imu_pose.y())
                .set__z(imu_pose.z())
                .set__w(imu_pose.w());
        msg_cam_transform.header.stamp = this->now();

        msg_aim_type.set__type(data.aim_type);
        msg_team.set__team(static_cast<uint8_t>(data.is_red ? ifr_interface::defs::Team::RED : ifr_interface::defs::Team::BLUE));
        msg_use_forecast.set__data(data.use_forecast);

        if (pub_aim_type) pub_aim_type->publish(msg_aim_type);
        if (pub_team) pub_team->publish(msg_team);
        if (pub_use_forecast) pub_use_forecast->publish(msg_use_forecast);
        if (pub_serial_receive) pub_serial_receive->publish(msg_serial_receive);
        if (data.need_flash) needToFlash();

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5e3, "receive: [%f %f %f], t=%c p=%c",
                             data.getYaw(), data.getPitch(), data.getRoll(),
                             data.is_red ? 'R' : 'B', data.use_forecast ? 'f' : 'd');
    }
}// namespace rm_serial_port


// Register node as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_port::SerialPortMotor)