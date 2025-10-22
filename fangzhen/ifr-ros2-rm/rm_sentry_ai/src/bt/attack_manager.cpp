#include "rm_sentry_ai/bt/attack_manager.hpp"
#include "rm_sentry_ai/bt_defs.hpp"
#include <behaviortree_cpp/basic_types.h>
#include <limits>

namespace rm_sentry_ai::bt {

    RmTargetUpdater::RmTargetUpdater(const std::string &instance_name,
                                     const BT::NodeConfig &conf,
                                     const BT::RosNodeParams &params)
        : RosTopicSubNode(instance_name, conf, params) {}
    BT::PortsList RmTargetUpdater::providedPorts() {
        return providedBasicPorts({
                BT::OutputPort<MsgT>("msg", "{RmTarget}", "RmTarget ptr output"),
        });
    }
    BT::NodeStatus RmTargetUpdater::onTick(const MsgT &msg) {
        setOutput("msg", msg);
        return BT::NodeStatus::SUCCESS;
    }
    HasAttackTarget::HasAttackTarget(const std::string &instance_name,
                                     const BT::NodeConfig &config)
        : BT::DecoratorNode(instance_name, config) {}
    BT::PortsList HasAttackTarget::providedPorts() {
        return {
                BT::InputPort<MsgT>("msg", "{RmTarget}", "RmTarget ptr input"),
        };
    }
    BT::NodeStatus HasAttackTarget::tick() {
        if (!getInput("msg", msg) || !msg) {
            return BT::NodeStatus::FAILURE;
        }

        if (msg->has_target) {
            setStatus(BT::NodeStatus::RUNNING);
            return child_node_->executeTick();
        } else {
            return BT::NodeStatus::SUCCESS;
        }
    }
    SwitchAttack::SwitchAttack(const std::string &instance_name,
                               const BT::NodeConfig &config, DataExchange::Ptr dataEx)
        : BT::SyncActionNode(instance_name, config),
          dataEx(std::move(dataEx)), node_(this->dataEx->node_) {}
    BT::PortsList SwitchAttack::providedPorts() {
        return {
                BT::InputPort<bool>("allow", false, "切换到的状态, 是否允许攻击"),
                BT::InputPort<int>("priority", 0, "决策优先级, 大数字决策可以覆盖小数字决策"),
        };
    }
    BT::NodeStatus SwitchAttack::tick() {
        BTROS_INPUT_OR_FAILED(int, priority, "priority");
        BTROS_INPUT_OR_FAILED(bool, allow, "allow");
        if (dataEx->allow_attack.tryOverride(priority)) {
            dataEx->allow_attack.v = allow;
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
}// namespace rm_sentry_ai::bt