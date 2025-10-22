#include "rm_sentry_ai/bt/health_check.hpp"
#include "rm_sentry_ai/bt/referee_system.hpp"
#include "rm_sentry_ai/bt_cvt.hpp"
#include "rm_sentry_ai/bt_defs.hpp"
#include <behaviortree_cpp/basic_types.h>
#include <chrono>
#include <ifr_common/defs.h>
#include <limits>
#include <rclcpp/logging.hpp>
#include <rm_common/referee_datas.h>
namespace rm_sentry_ai::bt {
    HealthCheck::HealthCheck(const std::string &name, const BT::NodeConfig &conf, const rclcpp::Node::SharedPtr &node)
        : BT::DecoratorNode(name, conf), node_(node) {
    }
    BT::PortsList HealthCheck::providedPorts() {
        return {
                getRefereeSystemPort<RefereeSystem::packet::GameRobotHP>(),
                getRefereeSystemPort<RefereeSystem::packet::GameRobotStatus>(),
                BT::InputPort<HealthCheckDatas>("data", "id,min_health,max_health;... id=team*100+robot"),
                BT::InputPort<bool>("need_all", 0, "1/true='and', 0/false='or'"),
        };
    }
    BT::NodeStatus HealthCheck::tick() {
        BTROS_NOTNULL_OR_FAILED(hp, getRefereeSystemPkg<RefereeSystem::packet::GameRobotHP>(this));
        BTROS_NOTNULL_OR_FAILED(grs, getRefereeSystemPkg<RefereeSystem::packet::GameRobotStatus>(this));
        BTROS_INPUT_OR_FAILED(HealthCheckDatas, datas, "data");
        BTROS_INPUT_OR_FAILED(bool, need_all, "need_all");

        bool pass = need_all;
        auto self_team = ifr_interface::defs::Team(grs->msg.robot_id / 100);
        auto other_team = self_team == ifr_interface::defs::Team::RED ? ifr_interface::defs::Team::BLUE : ifr_interface::defs::Team::RED;
        for (const auto &data: datas) {
            auto health = hp->msg.getByRobot(data.is_self_team ? self_team : other_team, data.robot);
            if (data.min_health <= health && health <= data.max_health) {
                pass = true;
                if (!need_all) break;
            } else {
                pass = false;
                if (need_all) break;
            }
        }

        if (pass) {
            setStatus(BT::NodeStatus::RUNNING);
            return child_node_->executeTick();
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }

    SelfHealthCheck::SelfHealthCheck(const std::string &name, const BT::NodeConfig &conf, const rclcpp::Node::SharedPtr &node)
        : BT::DecoratorNode(name, conf), node_(node) {
    }
    BT::PortsList SelfHealthCheck::providedPorts() {
        return {
                getRefereeSystemPort<RefereeSystem::packet::GameRobotStatus>(),
                BT::InputPort<int>("min", "0", "min health"),
                BT::InputPort<int>("max", std::numeric_limits<int>::max(), "max health"),
        };
    }
    BT::NodeStatus SelfHealthCheck::tick() {
        BTROS_INPUT_OR_FAILED(int, min_hp, "min");
        BTROS_INPUT_OR_FAILED(int, max_hp, "max");
        const auto &hp = getRefereeSystemPkg<RefereeSystem::packet::GameRobotStatus>(this)->__msg_update.remain_HP;

        if (min_hp <= hp && hp <= max_hp) {
            setStatus(BT::NodeStatus::RUNNING);
            return child_node_->executeTick();
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }


    RecoveryHealthCheck::RecoveryHealthCheck(const std::string &name,
                                             const BT::NodeConfig &conf,
                                             const rclcpp::Node::SharedPtr &node)
        : BT::DecoratorNode(name, conf), node_(node) {
    }

    BT::PortsList RecoveryHealthCheck::providedPorts() {
        return {
                getRefereeSystemPort<RefereeSystem::packet::GameRobotStatus>(),
                BT::InputPort<int>("max_recovery", "600", "最大回血量"),
                BT::InputPort<int>("once_recovery", "100", "单次回血量"),
                BT::InputPort<int>("max_recovery_sec", "8", "最大回血时间, 秒(超时时间)"),
                BT::InputPort<time_point>("value", "{is_healthing}", "是否处于回血状态的数据"),

        };
    }
    BT::NodeStatus RecoveryHealthCheck::tick() {
        auto now = std::chrono::steady_clock::now();
        BTROS_NOTNULL_OR_FAILED(pkg_grs, getRefereeSystemPkg<RefereeSystem::packet::GameRobotStatus>(this));
        BTROS_INPUT_OR_FAILED(int, once_recovery, "once_recovery");
        BTROS_INPUT_OR_FAILED(int, max_recovery, "max_recovery");
        BTROS_INPUT_OR_FAILED(int, max_recovery_sec, "max_recovery_sec");
        auto time = getInput<time_point>("value");
        if (time && time.value() >= now) {// healthing
            if (lst_in_health > now) {
                lst_in_health = now;
            } else if (now >= lst_in_health + std::chrono::seconds(1)) {
                lst_in_health = now;
                ++recovery_sec;
            }
        } else {
            lst_in_health = time_point::max();
        }

        const auto &hp = pkg_grs->msg.remain_HP;
        //计算回血量, 排除回血时扣血的干扰
        auto recovery = std::max(((hp - lst_HP) + (once_recovery - 1)) / once_recovery * once_recovery, 0);
        all_recovery += recovery;
        if (recovery > 0)
            BTROS_INFO_STREAM("Recovery: " << recovery << ", hp_now: " << hp << "/" << pkg_grs->msg.max_HP << "; HP = " << all_recovery << " / " << max_recovery <<//
                              "; time = " << recovery_sec << " / " << max_recovery_sec);

        bool allow = all_recovery < max_recovery && recovery_sec <= max_recovery_sec && hp < pkg_grs->msg.max_HP;

        BTROS_INFO_STREAM_THROTTLE(1e3, "Check: " << allow <<                                                           //
                                                "; hp: " << hp << "/" << pkg_grs->msg.max_HP <<                         //
                                                "; HP = " << recovery << "/" << all_recovery << " / " << max_recovery <<//
                                                "; time = " << recovery_sec << " / " << max_recovery_sec);

        if (allow) {
            setStatus(BT::NodeStatus::RUNNING);
            return child_node_->executeTick();
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }
    OutpostCheck::OutpostCheck(const std::string &name, const BT::NodeConfig &conf, const rclcpp::Node::SharedPtr &node)
        : BT::DecoratorNode(name, conf), node_(node) {
    }
    BT::PortsList OutpostCheck::providedPorts() {
        return {
                getRefereeSystemPort<RefereeSystem::packet::GameRobotStatus>(),
                getRefereeSystemPort<RefereeSystem::packet::GameRobotHP>(),

        };
    }

    BT::NodeStatus OutpostCheck::tick() {
        uint16_t outpost_hp = 10;
        const auto &robot_id = getRefereeSystemPkg<RefereeSystem::packet::GameRobotStatus>(this)->msg.robot_id;
        if (robot_id == 7) {
            outpost_hp = getRefereeSystemPkg<RefereeSystem::packet::GameRobotHP>(this)->msg.red_outpost_HP;

        } else {
            outpost_hp = getRefereeSystemPkg<RefereeSystem::packet::GameRobotHP>(this)->msg.blue_outpost_HP;
        }
        //RCLCPP_INFO_STREAM(node_->get_logger(), "Starting " << outpost_hp);
        if (outpost_hp == 0) {
            setStatus(BT::NodeStatus::RUNNING);
            return child_node_->executeTick();
        } else
            return BT::NodeStatus::FAILURE;
    }
}// namespace rm_sentry_ai::bt