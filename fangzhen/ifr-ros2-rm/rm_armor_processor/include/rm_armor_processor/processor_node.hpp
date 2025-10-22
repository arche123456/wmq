#ifndef IFR_ROS2_CV__PACKAGE_RM_ARMOR_PROCESSOR__PROCESSOR_NODE__HPP
#define IFR_ROS2_CV__PACKAGE_RM_ARMOR_PROCESSOR__PROCESSOR_NODE__HPP

#include "ifr_watcher/cpu_watcher.hpp"
#include "rm_armor_processor/tracker.hpp"
#include <memory>
#include <message_filters/subscriber.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rm_interface/msg/detail/rm_armors__struct.hpp>
#include <rm_interface/msg/detail/rm_target__struct.hpp>
#include <rm_interface/msg/detail/rm_tracker_info__struct.hpp>
#include <std_msgs/msg/detail/bool__struct.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>


namespace rm_armor_processor {
    // using tf2_filter = tf2_ros::MessageFilter<rm_interface::msg::RmArmors>;
    class ArmorTrackerNode : public rclcpp::Node {
        ifr_watcher::cpu::RegisterClient cpu_watcher{"node", this};

    public:
        explicit ArmorTrackerNode(const rclcpp::NodeOptions &options);

    private:
        void update_forecast(const std_msgs::msg::Bool::SharedPtr use_forecast);

        void armorsCallback(const rm_interface::msg::RmArmors::SharedPtr armors_ptr);

        void publishMarkers(const rm_interface::msg::RmTarget &target_msg);

        // Maximum allowable armor distance in the XOY plane
        double max_armor_distance_;

        // The time when the last message was received
        rclcpp::Time last_time_;
        double dt_;

        // Armor tracker
        double s2qxyz_, s2qyaw_, s2qr_;
        double r_xyz_factor, r_yaw;
        double lost_time_thres_;
        std::unique_ptr<Tracker> tracker_;

        // Reset tracker service
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_tracker_srv_;

        // Subscriber with tf2 message_filter
        std::string target_frame_;
        std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
        rclcpp::Subscription<rm_interface::msg::RmArmors>::SharedPtr armors_sub_;

        // Tracker info publisher
        rclcpp::Publisher<rm_interface::msg::RmTrackerInfo>::SharedPtr info_pub_;

        // Publisher
        rclcpp::Publisher<rm_interface::msg::RmTarget>::SharedPtr target_pub_;

        // Visualization marker publisher
        visualization_msgs::msg::Marker position_marker_;
        visualization_msgs::msg::Marker linear_v_marker_;
        visualization_msgs::msg::Marker yaw_marker_;
        visualization_msgs::msg::Marker angular_v_marker_;
        visualization_msgs::msg::Marker armor_marker_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

        // use forecast
        bool lst_use_forecast;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr use_forecast_sub_;
    };

}// namespace rm_armor_processor

#endif// IFR_ROS2_CV__PACKAGE_RM_ARMOR_PROCESSOR__PROCESSOR_NODE__HPP
