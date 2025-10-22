#ifndef IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___HEALTH_CHECK__HPP
#define IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___HEALTH_CHECK__HPP

#include "rm_sentry_ai/bt/referee_system.hpp"
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/decorator_node.h>
#include <chrono>
#include <cstdint>
#include <limits>
#include <rclcpp/node.hpp>
namespace rm_sentry_ai::bt {
    /// @brief 检查机器人血量
    class HealthCheck final : public BT::DecoratorNode {

    public:
        HealthCheck(const std::string &name, const BT::NodeConfig &conf,
                    const rclcpp::Node::SharedPtr &node);

        static BT::PortsList providedPorts();
        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
    };

    /// @brief 检查机器人血量
    class SelfHealthCheck final : public BT::DecoratorNode {

    public:
        SelfHealthCheck(const std::string &name, const BT::NodeConfig &conf,
                        const rclcpp::Node::SharedPtr &node);

        static BT::PortsList providedPorts();
        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
    };

    /// @brief 判断自身可恢复剩余血量
    /// @brief SUCCESS: 还有可恢复的空间
    class RecoveryHealthCheck final : public BT::DecoratorNode {
        using time_point = std::chrono::steady_clock::time_point;

    public:
        RecoveryHealthCheck(const std::string &name,
                            const BT::NodeConfig &conf,
                            const rclcpp::Node::SharedPtr &node);

        static BT::PortsList providedPorts();
        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
        uint16_t lst_HP = std::numeric_limits<uint16_t>::max();
        time_point lst_in_health = time_point::max();
        int recovery_sec = 0;     //总回血时间
        uint16_t all_recovery = 0;//总回血量
    };
    class OutpostCheck final : public BT::DecoratorNode {
    public:
        OutpostCheck(const std::string &name, const BT::NodeConfig &conf,
                     const rclcpp::Node::SharedPtr &node);

        static BT::PortsList providedPorts();
        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
    };
}// namespace rm_sentry_ai::bt
#endif// IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___HEALTH_CHECK__HPP
