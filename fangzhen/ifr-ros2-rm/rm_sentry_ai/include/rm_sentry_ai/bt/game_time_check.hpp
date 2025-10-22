#ifndef IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___GAME_TIME_CHECK__HPP
#define IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___GAME_TIME_CHECK__HPP

#include <behaviortree_cpp/decorator_node.h>
#include <rclcpp/node.hpp>
namespace rm_sentry_ai::bt {
    class GameTimeCheck final : public BT::DecoratorNode {
    public:
        GameTimeCheck(const std::string &name, const BT::NodeConfig &conf,
                      const rclcpp::Node::SharedPtr &node);

        static BT::PortsList providedPorts();
        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
    };
}// namespace rm_sentry_ai::bt
#endif// IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___GAME_TIME_CHECK__HPP
