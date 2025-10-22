#ifndef IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___BULLET_CHECK__HPP
#define IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___BULLET_CHECK__HPP

#include "rm_sentry_ai/bt/referee_system.hpp"
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/decorator_node.h>
#include <chrono>
#include <cstdint>
#include <limits>
namespace rm_sentry_ai::bt {
    /// @brief 检查机器人血量
    class BulletCheck final : public BT::DecoratorNode {

    public:
        enum class Type {
            bullet_17mm,
            bullet_42mm,
            coin,
        };
        BulletCheck(const std::string &name, const BT::NodeConfig &conf, const Type &type,
                    const rclcpp::Node::SharedPtr &node);

        static BT::PortsList providedPorts();
        BT::NodeStatus tick() override;

    private:
        Type type_;
        rclcpp::Node::SharedPtr node_;
    };
    class BulletRedemption final : public BT::DecoratorNode {

    public:
        BulletRedemption(const std::string &name, const BT::NodeConfig &conf, const rclcpp::Node::SharedPtr &node);

        static BT::PortsList providedPorts();
        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
        uint16_t last_bullet_num_ = 0;
        bool is_exchanged = false;
    };
}// namespace rm_sentry_ai::bt
#endif// IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___BULLET_CHECK__HPP
