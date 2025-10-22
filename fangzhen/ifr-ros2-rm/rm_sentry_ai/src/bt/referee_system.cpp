#include "rm_sentry_ai/bt/referee_system.hpp"
#include <rclcpp/executors.hpp>
#include <rm_common/referee_datas.h>

namespace rm_sentry_ai::bt {
    RefereeSystemUpdate::RefereeSystemUpdate(const std::string &name,
                                             const BT::NodeConfig &conf,
                                             const rclcpp::Node::SharedPtr &node)
        : BT::SyncActionNode(name, conf), node_(node) {
    }
    BT::PortsList RefereeSystemUpdate::providedPorts() {
        return registerTopics(nullptr);
    }
    BT::PortsList RefereeSystemUpdate::registerTopics(RefereeSystemUpdate *rsu) {
        return {
                registerTopic<RefereeSystem::packet::GameStatus>(rsu),
                registerTopic<RefereeSystem::packet::GameResult>(rsu),
                registerTopic<RefereeSystem::packet::GameRobotHP>(rsu),
                registerTopic<RefereeSystem::packet::EventData>(rsu),
                registerTopic<RefereeSystem::packet::GameRobotStatus>(rsu),
                registerTopic<RefereeSystem::packet::Buff>(rsu),
                registerTopic<RefereeSystem::packet::RobotHurt>(rsu),
                registerTopic<RefereeSystem::packet::ShootData>(rsu),
                registerTopic<RefereeSystem::packet::BulletRemaining>(rsu),
                registerTopic<RefereeSystem::packet::SentryInfo>(rsu),
        };
    }
    BT::NodeStatus RefereeSystemUpdate::tick() {
        rclcpp::spin_some(node_);
        if (handlers_.empty()) registerTopics(this);
        BT::NodeStatus result = BT::NodeStatus::SUCCESS;
        for (const auto &handler: handlers_) {
            if (!handler()) result = BT::NodeStatus::FAILURE;
        }
        return result;
    }
}// namespace rm_sentry_ai::bt