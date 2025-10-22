#include "rm_sentry_ai/bt/status_manager.hpp"
#include "rm_sentry_ai/bt_defs.hpp"
#include <rclcpp/logging.hpp>
namespace rm_sentry_ai::bt {
    EnterStatus::EnterStatus(const std::string &name, const BT::NodeConfig &conf, const rclcpp::Node::SharedPtr &node)
        : BT::SyncActionNode(name, conf), node_(node) {
    }
    BT::PortsList EnterStatus::providedPorts() {
        return {
                BT::OutputPort<time_point>("value", "{in_status}", "状态变量"),
                BT::InputPort<long>("time", 8, "状态维持时间, 秒"),
        };
    }
    BT::NodeStatus EnterStatus::tick() {
        auto now = std::chrono::steady_clock::now();
        BTROS_INPUT_OR_FAILED(long, time, "time");
        if (time <= 0) throw BT::RuntimeError("Bad [time]: " + std::to_string(time));

        setOutput<time_point>("value", now + std::chrono::seconds(time));

        return BT::NodeStatus::SUCCESS;
    }


    ExitStatus::ExitStatus(const std::string &name, const BT::NodeConfig &conf, const rclcpp::Node::SharedPtr &node)
        : BT::SyncActionNode(name, conf), node_(node) {
    }
    BT::PortsList ExitStatus::providedPorts() {
        return {
                BT::OutputPort<time_point>("value", "{in_status}", "状态变量"),
        };
    }
    BT::NodeStatus ExitStatus::tick() {
        static constexpr const auto unset = time_point::min();
        setOutput<time_point>("value", unset);
        // RCLCPP_INFO_STREAM(node_->get_logger(), "Success exit");
        return BT::NodeStatus::SUCCESS;
    }

    CheckStatus::CheckStatus(const std::string &name, const BT::NodeConfig &conf, const rclcpp::Node::SharedPtr &node)
        : BT::DecoratorNode(name, conf), node_(node) {
    }
    BT::PortsList CheckStatus::providedPorts() {
        return {
                BT::InputPort<time_point>("value", "{in_status}", "状态变量"),
        };
    }
    BT::NodeStatus CheckStatus::tick() {
        auto now = std::chrono::steady_clock::now();
        auto time = getInput<time_point>("value");

        if (time && time.value() > now) {
            setStatus(BT::NodeStatus::RUNNING);
            return child_node_->executeTick();
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }
}// namespace rm_sentry_ai::bt