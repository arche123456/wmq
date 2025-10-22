#include "rm_serial_port/SerialPort.h"
#include "rm_serial_port/packet.h"
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
namespace rm_serial_port {

    class CoordinateDisplay : public rclcpp::Node {
        geometry_msgs::msg::TransformStamped msg;
        std::unique_ptr<tf2_ros::TransformBroadcaster> publisher;
        SerialPort serial;


    public:
        explicit CoordinateDisplay(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
            : rclcpp::Node("coordinate_display", options), serial(this) {
            msg.header.frame_id = "odom";
            msg.transform.translation.set__x(0).set__y(0).set__z(0);
            publisher = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            serial.setReader<packet::IMU_Data>(std::bind(&CoordinateDisplay::readData, this, std::placeholders::_1));
        }

        void readData(const packet::IMU_Data &data) {
            msg.header.stamp = this->now();
            static constexpr const auto d2r = M_PI / 180;

#define SEND(name, ...)                                                                         \
    do {                                                                                        \
        __VA_ARGS__                                                                             \
        msg.child_frame_id = #name;                                                             \
        msg.transform.rotation.set__w(qua.w()).set__x(qua.x()).set__y(qua.y()).set__z(qua.z()); \
        publisher->sendTransform(msg);                                                          \
    } while (0)
            tf2::Quaternion qua;
            SEND(eu1, qua.setEuler(data.getYaw() * d2r, data.getPitch() * d2r, data.getRoll() * d2r););
            SEND(eu2, qua.setRPY(data.getRoll() * d2r, data.getPitch() * d2r, data.getYaw() * d2r););
            SEND(eu3, qua.setRPY(data.getRoll() * d2r, -data.getPitch() * d2r, data.getYaw() * d2r););


            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Received!");
        }
    };
}// namespace rm_serial_port

// Register node as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_port::CoordinateDisplay)