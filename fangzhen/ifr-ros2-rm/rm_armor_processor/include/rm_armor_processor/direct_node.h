#ifndef IFR_ROS2_CV__PACKAGE_RM_ARMOR_PROCESSOR__DIRECT_NODE__H
#define IFR_ROS2_CV__PACKAGE_RM_ARMOR_PROCESSOR__DIRECT_NODE__H

#include <message_filters/subscriber.h>
#include <rclcpp/rclcpp.hpp>
#include <rm_interface/msg/detail/rm_armors__struct.hpp>
#include <rm_interface/msg/detail/rm_target__struct.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>


namespace rm_armor_processor {
    class ArmorDirectNode : public rclcpp::Node {
        rm_interface::msg::RmTarget msg_target;
        rclcpp::Publisher<decltype(msg_target)>::SharedPtr pub_target;

        rclcpp::Subscription<rm_interface::msg::RmArmors>::SharedPtr sub_armors;

    public:
        explicit ArmorDirectNode(const rclcpp::NodeOptions &options);
        void armorsCallback(const rm_interface::msg::RmArmors::SharedPtr armors_ptr);
    };

}// namespace rm_armor_processor

#endif// IFR_ROS2_CV__PACKAGE_RM_ARMOR_PROCESSOR__DIRECT_NODE__H
