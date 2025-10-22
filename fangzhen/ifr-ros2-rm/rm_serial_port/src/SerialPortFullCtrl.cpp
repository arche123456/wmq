#include "rm_serial_port/SerialPortFullCtrl.h"
#include "ifr_common/defs.h"
#include "rm_serial_port/packet.h"
#include <chrono>
#include <cstdint>
#include <functional>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/detail/point_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <geometry_msgs/msg/detail/vector3_stamped__struct.hpp>
#include <ifr_common/crc.h>
#include <ifr_common/defs.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/timer.hpp>
#include <rm_common/referee_datas.h>
#include <rm_interface/msg/detail/rm_target__struct.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/create_timer_ros.h>

namespace rm_serial_port {

    SerialPortFullCtrl::SerialPortFullCtrl(const rclcpp::NodeOptions &options)
        : rclcpp::Node("serial_sentry", options), serial(this), tc(this) {
        RCLCPP_INFO(this->get_logger(), "SerialPort is starting...");

        // Create Publisher
        pub_team = this->create_publisher<decltype(msg_team)>("/rm/self_team", 10);
        pub_use_forecast = this->create_publisher<decltype(msg_use_forecast)>("/serial/use_forecast", 10);
        pub_serial_send = this->create_publisher<decltype(msg_serial_send)>("/serial/send", rclcpp::SensorDataQoS());
        pub_serial_receive = this->create_publisher<decltype(msg_serial_receive)>("/serial/revice", rclcpp::SensorDataQoS());

        // 监听攻击目标
        sub_target = this->create_subscription<rm_interface::msg::RmTarget>(
                "/rm/tracker/target", rclcpp::SensorDataQoS(),
                std::bind(&SerialPortFullCtrl::updateSendDataRmTarget, this, std::placeholders::_1));
        sub_move = this->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel", rclcpp::SensorDataQoS(),
                std::bind(&SerialPortFullCtrl::updateSendDataTwist, this, std::placeholders::_1));
        in_game_receive = this->create_subscription<rm_referee_interface::msg::GameStatus>("/serial/referee/GameStatus", rclcpp::SensorDataQoS(), std::bind(&SerialPortFullCtrl::updateIsGame, this, std::placeholders::_1));
        timer_send = this->create_wall_timer(std::chrono::milliseconds(10),
                                             std::bind(&SerialPortFullCtrl::sendData, this));

        yaw_follow = this->declare_parameter("yaw_follow", 0.1);
        cmd_vel_max = this->declare_parameter("cmd_vel_max", 1.0);


        //陀螺仪数据广播
        msg_cam_transform1.header.frame_id = "odom";
        msg_cam_transform1.child_frame_id = "gimbal_middle_link";
        msg_cam_transform2.header.frame_id = "gimbal_middle_link";
        msg_cam_transform2.child_frame_id = "gimbal_link";
        msg_cam_transform2.transform.translation.set__x(-0.0690247);
        pub_cam_transform = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        timer_cam_transform = this->create_wall_timer(std::chrono::milliseconds(10), [this]() {
            pub_cam_transform->sendTransform(msg_cam_transform1);
            pub_cam_transform->sendTransform(msg_cam_transform2);
        });

        {
            using namespace RefereeSystem::packet;
            add_rs_reader<GameStatus>();
            add_rs_reader<GameResult>();
            add_rs_reader<GameRobotHP>();
            add_rs_reader<EventData>();
            add_rs_reader<GameRobotStatus>();
            add_rs_reader<Buff>();
            add_rs_reader<RobotHurt>();
            add_rs_reader<ShootData>();
            add_rs_reader<BulletRemaining>();
            add_rs_reader<SentryInfo>();
        }
        //serial.setMultiHeadReader<RSFH>(std::bind(&SerialPortFullCtrl::readDataRS, this, std::placeholders::_1));
        // serial.setReader<packet::IMU_Data>(std::bind(&SerialPortFullCtrl::readDataIMU, this, std::placeholders::_1));
        serial.setMultiHeadReader<packet::IMU_Data, RSFH>(
                std::bind(&SerialPortFullCtrl::readDataIMU, this, std::placeholders::_1),
                std::bind(&SerialPortFullCtrl::readDataRS, this, std::placeholders::_1));

        //std::bind(&SerialPortFullCtrl::readDataIMU, this, std::placeholders::_1),


        RCLCPP_INFO(this->get_logger(), "SerialPort successfully started!");
    }
    void SerialPortFullCtrl::updateIsGame(const rm_referee_interface::msg::GameStatus::SharedPtr data) {
        if (data->game_progress == 4) in_game = true;

        else
            in_game = false;
        // RCLCPP_INFO_STREAM(this->get_logger(), "value" << in_game << " " << data->game_progress << ", " << data->stage_remain_time);
    }

    void SerialPortFullCtrl::sendData() {
        {
            std::lock_guard lock(serial_write_mtx);
            if (!in_game) {
                fc.setMove(0, 0);
                fc.setGyro(false);
                fc.setShoot(false);
                fc.setScan(false);
            }
            fc.finish();
            serial.writeData(fc);


            pub_serial_send->publish(msg_serial_send);

            RCLCPP_INFO_STREAM_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "Send: [ "
                            << "move = " << (msg_serial_send.move_fb)
                            << ", " << (msg_serial_send.move_lr)
                            << "; gun = " << (msg_serial_send.gun_yaw)
                            << ", " << (msg_serial_send.gun_pitch)
                            << "; G = " << (msg_serial_send.gyro)
                            << ", S = " << (msg_serial_send.scan)
                            << ", A = " << (msg_serial_send.can_attack)
                            << " ] " << serial.toHex(msg_serial_send)//
            );

            if (ctrl_share.clear_move > ctrl_share.clear_move_t) {
                fc.setMove(0, 0);
            }
            if (ctrl_share.clear_aim > ctrl_share.clear_aim_t) {
                ctrl_share.has_target = false;
            }
            if (ctrl_share.clear_move > ctrl_share.clear_move_t &&
                ctrl_share.clear_aim > ctrl_share.clear_aim_t) {
                fc.setShoot(false);
                fc.setGyro(true);
                fc.setScan(true);
            }
            ctrl_share.clear_move++;
            ctrl_share.clear_aim++;


            fc.toRos(msg_serial_send);
        }
    }

    void SerialPortFullCtrl::updateSendDataRmTarget(const rm_interface::msg::RmTarget::SharedPtr data) {
        {
            std::lock_guard lock(serial_write_mtx);
            ctrl_share.has_target = data->has_target;
            if (data->has_target) {
                auto at = tc.getArmor(data);
                fc.setGunYaw(at.move_x);
                fc.setGunPitch(at.move_y);
                fc.setShoot(at.can_attack);
                fc.setGyro(true);
                fc.setScan(false);
                ctrl_share.clear_aim = 0;
                RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1e3, "AT: " << at.move_y << ", " << data->position.x << ", " << data->position.y << ", " << data->position.z);
            } else {
                fc.setShoot(false);
            }
        }
        const auto time = (this->now() - data->receive_time).seconds() * 1e3;
        msg_serial_send.delay = time;

        RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5e3, "RmTarget: [ " << data->has_target << ", " << time << "ms ]");
    }

    void SerialPortFullCtrl::updateSendDataTwist(const geometry_msgs::msg::Twist::SharedPtr data) {
        const double speed_factor = 1 / cmd_vel_max * 660;
        // static constexpr const double d2r = M_PI / 180;
        // if (ctrl_share.has_target) return;// TODO: 临时：有目标就打, 不再移动
        tf2::Matrix3x3 m(imu_pose);
        double r, p, y;
        m.getRPY(r, p, y);


        std::lock_guard lock(serial_write_mtx);

        fc.setMove(-data->linear.x * speed_factor, data->linear.y * speed_factor);
        if (!ctrl_share.has_target) {
            // fc.setGun(y - data->angular.z * d2r, p - data->angular.y * d2r);
            // note: 此处忽略/cmd_vel传入的yaw轴旋转信息: 在导航中已经禁用了yaw旋转, 转为手动旋转
            if (abs(data->linear.y) + abs(data->linear.x) > 0.3 * cmd_vel_max) {

                auto delta = std::atan2(data->linear.y, data->linear.x);
                delta = delta > yaw_follow    ? yaw_follow
                        : delta < -yaw_follow ? -yaw_follow
                                              : delta;
                fc.setGun(y - delta, p);
                fc.setScan(false);
                fc.setGyro(false);
            } else {
                fc.setScan(true);
                fc.setGyro(true);
            }
        }
        ctrl_share.clear_move = 0;
    }

    void SerialPortFullCtrl::readDataIMU(const packet::IMU_Data &data) {
        static constexpr const auto d2r = M_PI / 180;

        data.toRos(msg_serial_receive);

        switch (static_cast<packet::IMU_Data::Type>(data.imu_type)) {
            case packet::IMU_Data::Type::Abt:
                imu_pose.setRPY(data.getRoll() * d2r, data.getPitch() * d2r, data.getYaw() * d2r);/// 艾博特坐标系2
                msg_cam_transform1.transform.rotation.set__x(0)                                   // yaw
                        .set__y(0)
                        .set__z(std::sin(data.getYaw() * d2r / 2))
                        .set__w(std::cos(data.getYaw() * d2r / 2));
                msg_cam_transform2.transform.rotation.set__x(0)
                        .set__y(std::sin(data.getPitch() * d2r / 2))
                        .set__z(0)
                        .set__w(std::cos(data.getPitch() * d2r / 2));
                break;
            // case packet::IMU_Data::Type::Lxs: {
            //     tf2::Quaternion q_roll_yaw, q_pitch;
            //     q_roll_yaw.setRPY(data.getRoll() * d2r, 0, data.getYaw() * d2r);cc
            //     q_pitch.setRPY(0, data.getPitch() * d2r, 0);
            //     imu_pose = q_roll_yaw * q_pitch;/// 廖旭昇坐标系 PRY
            //     break;
            // }
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

        msg_cam_transform1.header.stamp = msg_cam_transform2.header.stamp = this->now();

        msg_team.set__team(static_cast<uint8_t>(data.is_red ? ifr_interface::defs::Team::RED : ifr_interface::defs::Team::BLUE));
        msg_use_forecast.set__data(data.use_forecast);

        if (pub_team) pub_team->publish(msg_team);
        if (pub_use_forecast) pub_use_forecast->publish(msg_use_forecast);
        if (pub_serial_receive) pub_serial_receive->publish(msg_serial_receive);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5e3, "receive: [%f %f %f], t=%c p=%c",
                             data.getYaw(), data.getPitch(), data.getRoll(),
                             data.is_red ? 'R' : 'B', data.use_forecast ? 'f' : 'd');
    }
    void SerialPortFullCtrl::readDataRS(const RSFH &head) {
        uint16_t cmd_id, crc16;
        if (!serial.read(reinterpret_cast<uint8_t *>(&cmd_id), sizeof(cmd_id))) return;
        auto itr = referees.find(cmd_id);
        if (itr != referees.end()) {
            auto &[name, buffer, handler] = itr->second;
            IFR_ASSERT(head.data_length == buffer->length(), "Pkg length miss match: " + name + ", ID = " + std::to_string(cmd_id) +//
                                                                     ", size(T) = " + std::to_string(buffer->length()) +            //
                                                                     ", data_len = " + std::to_string(head.data_length));
            if (!serial.read(buffer)) return;
            if (!serial.read(reinterpret_cast<uint8_t *>(&crc16), sizeof(crc16))) return;
            if (auto crc = crc::CRC16::summon_crc(buffer->data(), buffer->length(), crc::CRC16::summon_all_crc(head, cmd_id));
                crc16 != crc) {
                RCLCPP_WARN_STREAM_THROTTLE(
                        this->get_logger(), *this->get_clock(), 2e3,
                        "Pkg frame tail crc check fail: "
                                << name
                                << ", ID = " << cmd_id
                                << ", get = " << serial.toHex(crc16)
                                << ", calc = " << serial.toHex(crc)//
                );
            }
            handler();
        } else {//不需要的数据包, 跳过数据
            tmp_buffer.resize(head.data_length + sizeof(crc16));
            if (!serial.read(tmp_buffer)) return;
        }
    }
}// namespace rm_serial_port


// Register node as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_port::SerialPortFullCtrl)