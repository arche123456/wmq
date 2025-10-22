#ifndef IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___STATUS_MANAGER__HPP
#define IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___STATUS_MANAGER__HPP
#include "rm_sentry_ai/bt_cvt.hpp"
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/decorator_node.h>
#include <behaviortree_cpp/tree_node.h>
#include <chrono>
#include <ifr_common/common.h>
#include <rclcpp/node.hpp>
namespace rm_sentry_ai::bt {
    /// @brief 进入某个状态, 持续一段时间
    class EnterStatus final : public BT::SyncActionNode {
        using time_point = std::chrono::steady_clock::time_point;

    public:
        EnterStatus(const std::string &name, const BT::NodeConfig &conf, const rclcpp::Node::SharedPtr &node);
        static BT::PortsList providedPorts();
        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
    };
    /// @brief 退出某个状态, 持续一段时间
    class ExitStatus final : public BT::SyncActionNode {
        using time_point = std::chrono::steady_clock::time_point;

    public:
        ExitStatus(const std::string &name, const BT::NodeConfig &conf, const rclcpp::Node::SharedPtr &node);
        static BT::PortsList providedPorts();
        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
    };

    /// @brief 检查是否在某个状态中
    class CheckStatus final : public BT::DecoratorNode {
        using time_point = std::chrono::steady_clock::time_point;

    public:
        CheckStatus(const std::string &name, const BT::NodeConfig &conf, const rclcpp::Node::SharedPtr &node);
        static BT::PortsList providedPorts();
        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
    };
}// namespace rm_sentry_ai::bt
#endif// IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___STATUS_MANAGER__HPP
