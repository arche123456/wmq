#include "rm_sentry_ai/bt/game_time_check.hpp"
#include "rm_sentry_ai/bt/referee_system.hpp"
#include "rm_sentry_ai/bt_defs.hpp"
#include <limits>

namespace rm_sentry_ai::bt {
    GameTimeCheck::GameTimeCheck(const std::string &name, const BT::NodeConfig &conf, const rclcpp::Node::SharedPtr &node)
        : BT::DecoratorNode(name, conf), node_(node) {
    }
    BT::PortsList GameTimeCheck::providedPorts() {
        return {
                getRefereeSystemPort<RefereeSystem::packet::GameStatus>(),
                BT::InputPort<int>("stage", 4, "Check Game Stage: [0, 5]"),
                BT::InputPort<int>("time_min", 0, "Minimum remaining time (include)"),
                BT::InputPort<int>("time_max", std::numeric_limits<int>::max(), "Maximum remaining time (include)"),
        };
    }
    BT::NodeStatus GameTimeCheck::tick() {
        BTROS_NOTNULL_OR_FAILED(status, getRefereeSystemPkg<RefereeSystem::packet::GameStatus>(this));
        BTROS_INPUT_OR_FAILED(int, stage, "stage");
        BTROS_INPUT_OR_FAILED(int, time_min, "time_min");
        BTROS_INPUT_OR_FAILED(int, time_max, "time_max");


        if (stage == status->msg.game_progress &&
            time_min <= status->msg.stage_remain_time &&
            time_max >= status->msg.stage_remain_time) {
            setStatus(BT::NodeStatus::RUNNING);
            return child_node_->executeTick();
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }
}// namespace rm_sentry_ai::bt