#ifndef IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___PUB_CAN_INIT__HPP
#define IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___PUB_CAN_INIT__HPP

#include <behaviortree_ros2/bt_topic_pub_node.hpp>
#include <rclcpp/node.hpp>
#include <std_msgs/msg/detail/bool__struct.hpp>
namespace rm_sentry_ai::bt {
    class PubCanInit final : public BT::RosTopicPubNode<std_msgs::msg::Bool> {
    public:
        PubCanInit(const std::string &name,
                   const BT::NodeConfig &conf,
                   const BT::RosNodeParams &params);

        static BT::PortsList providedPorts();
        bool setMessage(std_msgs::msg::Bool &msg);

    private:
    };
    class ShutDownScan final : public BT::RosTopicPubNode<std_msgs::msg::Bool> {
    public:
        ShutDownScan(const std::string &name,
                     const BT::NodeConfig &conf,
                     const BT::RosNodeParams &params);

        static BT::PortsList providedPorts();
        bool setMessage(std_msgs::msg::Bool &msg);

    private:
    };
}// namespace rm_sentry_ai::bt
#endif// IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___PUB_CAN_INIT__HPP
