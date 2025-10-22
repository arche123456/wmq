#ifndef IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___ATTACK_MANAGER__HPP
#define IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___ATTACK_MANAGER__HPP

#include "rm_sentry_ai/bt/all.hpp"
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/decorator_node.h>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <cstddef>
#include <rm_interface/msg/detail/rm_target__struct.hpp>
namespace rm_sentry_ai::bt {
    /// @brief 检查机器人血量
    class RmTargetUpdater final : public BT::RosTopicSubNode<rm_interface::msg::RmTarget> {
        using MsgT = decltype(last_msg_);

    public:
        RmTargetUpdater(const std::string &instance_name,
                        const BT::NodeConfig &conf,
                        const BT::RosNodeParams &params);

        static BT::PortsList providedPorts();
        BT::NodeStatus onTick(const MsgT &msg) override;
    };

    /// @brief 判断是否有目标
    class HasAttackTarget final : public BT::DecoratorNode {
        using MsgT = std::shared_ptr<rm_interface::msg::RmTarget>;

    public:
        HasAttackTarget(const std::string &instance_name,
                        const BT::NodeConfig &config);

        static BT::PortsList providedPorts();
        BT::NodeStatus tick() override;

    private:
        MsgT msg;
    };

    /// @brief 切换是否允许攻击
    class SwitchAttack final : public BT::SyncActionNode {
        using MsgT = std::shared_ptr<rm_interface::msg::RmTarget>;

    public:
        SwitchAttack(const std::string &instance_name,
                     const BT::NodeConfig &config,
                     DataExchange::Ptr dataEx);

        static BT::PortsList providedPorts();
        BT::NodeStatus tick() override;

    private:
        MsgT msg;
        DataExchange::Ptr dataEx;
        rclcpp::Node::SharedPtr node_;
    };
}// namespace rm_sentry_ai::bt
#endif// IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___ATTACK_MANAGER__HPP
