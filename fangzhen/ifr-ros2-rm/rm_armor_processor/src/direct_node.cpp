#include "rm_armor_processor/direct_node.h"
#include <algorithm>
#include <functional>
#include <rclcpp/qos.hpp>
#include <rm_interface/msg/detail/rm_armor__struct.hpp>
#include <rm_interface/msg/detail/rm_armors__struct.hpp>


namespace rm_armor_processor {

    ArmorDirectNode::ArmorDirectNode(const rclcpp::NodeOptions &options)
        : Node("rm_armor_direct", options) {
        RCLCPP_INFO(this->get_logger(), "Starting DirectNode!");
        sub_armors = this->create_subscription<rm_interface::msg::RmArmors>(
                "/rm/finder/armors", rclcpp::SensorDataQoS(),
                std::bind(&ArmorDirectNode::armorsCallback, this, std::placeholders::_1));

        pub_target = this->create_publisher<decltype(msg_target)>("/rm/tracker/target", rclcpp::SensorDataQoS());
    }

    void ArmorDirectNode::armorsCallback(const rm_interface::msg::RmArmors::SharedPtr armors_msg) {
        using Armor = rm_interface::msg::RmArmor;
        auto ele = std::min_element(armors_msg->armors.begin(), armors_msg->armors.end(), [](const Armor &l, const Armor &r) {
            return l.distance_to_image_center < r.distance_to_image_center;
        });
        if ((msg_target.has_target = (!armors_msg->armors.empty()))) {
            msg_target.header = armors_msg->header;
            msg_target.receive_time = armors_msg->header.stamp;
            msg_target.type = ele->type;
            msg_target.position = ele->pose.position;
            msg_target.velocity.set__x(0).set__y(0).set__z(0);
        }
        pub_target->publish(msg_target);
    }

}// namespace rm_armor_processor

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_armor_processor::ArmorDirectNode)
