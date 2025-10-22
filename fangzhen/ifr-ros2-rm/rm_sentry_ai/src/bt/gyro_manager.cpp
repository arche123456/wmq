#include "rm_sentry_ai/bt/gyro_manager.hpp"
#include "rm_sentry_ai/bt_defs.hpp"

namespace rm_sentry_ai::bt {

    NeedGyro::NeedGyro(const std::string &instance_name,
                       const BT::NodeConfig &conf,
                       DataExchange::Ptr dataEx)
        : BT::DecoratorNode(instance_name, conf),
          dataEx(std::move(dataEx)), node_(this->dataEx->node_) {}

    BT::PortsList NeedGyro::providedPorts() {
        return {
                BT::InputPort<bool>("hurt", true, "Gyro when attacked by bullet"),
                BT::InputPort<bool>("see", true, "Gyro when see target"),

                getRefereeSystemPort<RefereeSystem::packet::RobotHurt>(),
                BT::InputPort<RmTargetPtr>("target", "{RmTarget}", "RmTarget ptr input"),
        };
    }
    BT::NodeStatus NeedGyro::tick() {
        bool need_gyro = true;
        BTROS_INPUT_OR_FAILED(bool, hurt, "hurt");
        BTROS_INPUT_OR_FAILED(bool, see, "see");
        if (hurt) {
            auto pkg = getRefereeSystemPkg<RefereeSystem::packet::RobotHurt>(this);
            if (pkg && RobotHurt_lstid != pkg->update_id) {
                RobotHurt_lstid = pkg->update_id;
                if (pkg->msg.hurt_type == 0) {
                    need_gyro = true;
                }
            }
        }
        if (see) {
            auto msg = getInput<RmTargetPtr>("target");
            if (msg && msg.value()->has_target) {
                need_gyro = true;
            }
        }

        if (need_gyro) {
            setStatus(BT::NodeStatus::RUNNING);
            return child_node_->executeTick();
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }


    SwitchGyro::SwitchGyro(const std::string &instance_name,
                           const BT::NodeConfig &config, DataExchange::Ptr dataEx)
        : BT::SyncActionNode(instance_name, config),
          dataEx(std::move(dataEx)), node_(this->dataEx->node_) {}
    BT::PortsList SwitchGyro::providedPorts() {
        return {
                BT::InputPort<bool>("allow", false, "切换到的状态, 是否允许攻击"),
                BT::InputPort<int>("priority", 0, "决策优先级, 大数字决策可以覆盖小数字决策"),
        };
    }
    BT::NodeStatus SwitchGyro::tick() {
        BTROS_INPUT_OR_FAILED(int, priority, "priority");
        BTROS_INPUT_OR_FAILED(bool, allow, "allow");
        if (dataEx->need_gyro.tryOverride(priority)) {
            dataEx->need_gyro.v = allow;
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
}// namespace rm_sentry_ai::bt