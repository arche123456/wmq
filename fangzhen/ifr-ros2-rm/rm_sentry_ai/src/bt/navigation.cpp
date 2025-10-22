#include "rm_sentry_ai/bt/navigation.hpp"
#include "ifr_common/outputs.hpp"
#include "rm_sentry_ai/bt_cvt.hpp"
#include "rm_sentry_ai/bt_defs.hpp"
namespace rm_sentry_ai::bt {
    Navigation::Navigation(const std::string &name,
                           const BT::NodeConfig &conf,
                           const BT::RosNodeParams &params)
        : BT::RosActionNode<ActionType>(name, conf, params) {}
    BT::PortsList Navigation::providedPorts() {
        return providedBasicPorts({
                BT::InputPort<geometry_msgs::msg::PoseStamped>("pose"),
        });
    }
    bool Navigation::setGoal(RosActionNode::Goal &goal) {
        auto success = getInput("pose", goal.pose);
        if (success) {
            goal.pose.header.stamp = node_->now();
            BTROS_INFO_STREAM_THROTTLE(1e2, __func__ << ":  success: " << goal.pose);
        } else {
            BTROS_ERROR_STREAM_THROTTLE(1e2, __func__ << ": " << success.error());
        }
        return !!success;
    }
    BT::NodeStatus Navigation::onResultReceived(const WrappedResult &wr) {
        BTROS_INFO_STREAM_THROTTLE(1e2, __func__ << ": result = " << int(wr.code) << ", " << rclcpp_action::to_string(wr.goal_id));
        return BT::NodeStatus::SUCCESS;
    }
    BT::NodeStatus Navigation::onFailure(BT::ActionNodeErrorCode error) {
        BTROS_ERROR_STREAM_THROTTLE(1e2, __func__ << ": " << error << '(' << BT::toStr(error) << ')');
        return BT::NodeStatus::FAILURE;
    }
    BT::NodeStatus Navigation::onFeedback(const std::shared_ptr<const Feedback> feedback) {
        BTROS_INFO_STREAM_THROTTLE(1e3, __func__ << ": cp = " << feedback->current_pose <<  //
                                                ", dr = " << feedback->distance_remaining <<//
                                                ", nt = " << feedback->navigation_time <<   //
                                                ", nor = " << feedback->number_of_recoveries//
        );
        return BT::NodeStatus::RUNNING;
    }
}// namespace rm_sentry_ai::bt