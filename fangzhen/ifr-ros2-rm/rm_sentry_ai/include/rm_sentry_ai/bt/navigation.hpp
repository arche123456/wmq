#ifndef IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___NAVIGATION__HPP
#define IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___NAVIGATION__HPP
#include <behaviortree_ros2/bt_action_node.hpp>
#include <nav2_msgs/action/detail/navigate_to_pose__struct.hpp>
#include <rclcpp/logging.hpp>
namespace rm_sentry_ai::bt {
    class Navigation final : public BT::RosActionNode<nav2_msgs::action::NavigateToPose> {
    public:
        Navigation(const std::string &name,
                   const BT::NodeConfig &conf,
                   const BT::RosNodeParams &params);

        static BT::PortsList providedPorts();

        // 当TreeNode被触发时会调用此方法，并且它应该向动作服务器发送请求
        bool setGoal(RosActionNode::Goal &goal) override;

        // 当收到回复时执行的回调。基于回复，你需要决定返回SUCCESS或FAILURE。
        BT::NodeStatus onResultReceived(const WrappedResult &wr) override;

        // 当客户端和服务器之间的通信级别出现错误时调用的回调。
        // 这将根据返回值将 TreeNode 的状态设置为 SUCCESS 或 FAILURE。
        // 如果不重写，默认返回 FAILURE。
        virtual BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;

        // 我们还支持反馈回调，如原始教程中所示。
        // 通常，此回调应返回 RUNNING，但您可能会根据反馈值决定中止操作，并认为 TreeNode 已完成。
        // 在这种情况下，返回成功或失败。
        // 取消请求将自动发送到服务器。
        BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;
    };
}// namespace rm_sentry_ai::bt
#endif// IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___NAVIGATION__HPP
