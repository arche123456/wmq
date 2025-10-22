#include "rm_sentry_ai/bt/bullet_check.hpp"
#include "rm_sentry_ai/bt_defs.hpp"
#include <behaviortree_cpp/basic_types.h>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <rclcpp/logging.hpp>
#include <rm_common/referee_datas.h>
namespace rm_sentry_ai::bt {

    BulletCheck::BulletCheck(const std::string &name, const BT::NodeConfig &conf, const Type &type, const rclcpp::Node::SharedPtr &node)
        : BT::DecoratorNode(name, conf), type_(type), node_(node) {
    }
    BT::PortsList BulletCheck::providedPorts() {
        static constexpr const auto min_val = std::numeric_limits<uint16_t>::min();

        return {
                getRefereeSystemPort<RefereeSystem::packet::BulletRemaining>(),
                BT::InputPort<size_t>("min", min_val, "min value, include"),

        };
    }
    BT::NodeStatus BulletCheck::tick() {
        BTROS_NOTNULL_OR_FAILED(br, getRefereeSystemPkg<RefereeSystem::packet::BulletRemaining>(this));
        BTROS_INPUT_OR_FAILED(size_t, min_val, "min");


        size_t val = 0;
        switch (type_) {
            case Type::bullet_17mm:
                val = br->msg.bullet_remaining_num_17mm;
                break;
            case Type::bullet_42mm:
                val = br->msg.bullet_remaining_num_42mm;
                break;
            case Type::coin:
                val = br->msg.coin_remaining_num;
                break;
        }

        if (min_val > val) {
            setStatus(BT::NodeStatus::RUNNING);
            return child_node_->executeTick();
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }
    BulletRedemption::BulletRedemption(const std::string &name, const BT::NodeConfig &conf, const rclcpp::Node::SharedPtr &node)
        : BT::DecoratorNode(name, conf), node_(node) {
    }
    BT::PortsList BulletRedemption::providedPorts() {
        return {

                getRefereeSystemPort<RefereeSystem::packet::GameStatus>(),
                getRefereeSystemPort<RefereeSystem::packet::BulletRemaining>(),
        };
    }
    BT::NodeStatus BulletRedemption::tick() {
        BTROS_NOTNULL_OR_FAILED(status, getRefereeSystemPkg<RefereeSystem::packet::GameStatus>(this));
        const auto &bullet_num = getRefereeSystemPkg<RefereeSystem::packet::BulletRemaining>(this)->msg.bullet_remaining_num_17mm;
        if (status->msg.stage_remain_time == 300 || status->msg.stage_remain_time == 180) {
            last_bullet_num_ = getRefereeSystemPkg<RefereeSystem::packet::BulletRemaining>(this)->msg.bullet_remaining_num_17mm;
            is_exchanged = false;
        }
        RCLCPP_INFO_STREAM(node_->get_logger(), "bullet " << bullet_num);
        RCLCPP_INFO_STREAM(node_->get_logger(), "Time " << status->msg.stage_remain_time);

        if (abs(bullet_num - last_bullet_num_) <= 50 && is_exchanged == false) {
            setStatus(BT::NodeStatus::RUNNING);
            return child_node_->executeTick();
        } else {
            is_exchanged = true;
            return BT::NodeStatus::FAILURE;
        }
    }
}// namespace rm_sentry_ai::bt