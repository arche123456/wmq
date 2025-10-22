#ifndef IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___GYRO_MANAGER__HPP
#define IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___GYRO_MANAGER__HPP

#include "rm_sentry_ai/bt/all.hpp"
#include "rm_sentry_ai/bt/referee_system.hpp"
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/decorator_node.h>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <cstddef>
#include <rclcpp/node.hpp>
#include <rm_interface/msg/detail/rm_target__struct.hpp>
namespace rm_sentry_ai::bt {
    /// @brief 是否需要进入小陀螺
    class NeedGyro final : public BT::DecoratorNode {
        using RmTargetPtr = std::shared_ptr<rm_interface::msg::RmTarget>;

    public:
        NeedGyro(const std::string &instance_name,
                 const BT::NodeConfig &conf,
                 DataExchange::Ptr dataEx);

        static BT::PortsList providedPorts();
        BT::NodeStatus tick() override;

    private:
        uint64_t RobotHurt_lstid = -1;
        DataExchange::Ptr dataEx;
        rclcpp::Node::SharedPtr node_;
    };


    /// @brief 切换是否进入小陀螺
    class SwitchGyro final : public BT::SyncActionNode {
        using MsgT = std::shared_ptr<rm_interface::msg::RmTarget>;

    public:
        SwitchGyro(const std::string &instance_name,
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
#endif// IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___GYRO_MANAGER__HPP
