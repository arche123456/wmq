#include "rm_sentry_ai/bt/all.hpp"
#include "rm_sentry_ai/bt/attack_manager.hpp"
#include "rm_sentry_ai/bt/bullet_check.hpp"
#include "rm_sentry_ai/bt/game_time_check.hpp"
#include "rm_sentry_ai/bt/gyro_manager.hpp"
#include "rm_sentry_ai/bt/health_check.hpp"
#include "rm_sentry_ai/bt/navigation.hpp"
#include "rm_sentry_ai/bt/parallel_async.hpp"
#include "rm_sentry_ai/bt/pub_can_init.hpp"
#include "rm_sentry_ai/bt/referee_system.hpp"
#include "rm_sentry_ai/bt/status_manager.hpp"
#include "rm_sentry_ai/bt/wait_fallback.h"
#include <memory>
namespace rm_sentry_ai::bt {

    std::unique_ptr<BT::BehaviorTreeFactory> getBehaviorTreeFactory(rclcpp::Node::SharedPtr node, DataExchange::Ptr dataEx) {
        auto factory = std::make_unique<BT::BehaviorTreeFactory>();

        {
            BT::RosNodeParams params;
            params.nh = node;
            params.default_port_value = "navigate_to_pose";
            factory->registerNodeType<Navigation>("Navigation", params);
        }
        {
            factory->registerNodeType<RefereeSystemUpdate>("RefereeSystemUpdate", node);
        }
        {
            factory->registerNodeType<GameTimeCheck>("GameTimeCheck", node);
        }
        {
            factory->registerNodeType<HealthCheck>("HealthCheck", node);
            factory->registerNodeType<SelfHealthCheck>("SelfHealthCheck", node);
            factory->registerNodeType<RecoveryHealthCheck>("RecoveryHealthCheck", node);
            factory->registerNodeType<OutpostCheck>("OutpostCheck", node);
        }
        {
            factory->registerNodeType<EnterStatus>("EnterStatus", node);
            factory->registerNodeType<ExitStatus>("ExitStatus", node);
            factory->registerNodeType<CheckStatus>("CheckStatus", node);
        }
        {
            factory->registerNodeType<ParallelAsync>("ParallelAsync");
        }
        {
            factory->registerNodeType<BulletCheck>("BulletCheck17mm", BulletCheck::Type::bullet_17mm, node);
            factory->registerNodeType<BulletCheck>("BulletCheck42mm", BulletCheck::Type::bullet_42mm, node);
            factory->registerNodeType<BulletCheck>("BulletCheckCoin", BulletCheck::Type::coin, node);
            factory->registerNodeType<BulletRedemption>("BulletRedemption", node);
        }


        {
            BT::RosNodeParams params;
            params.nh = node;
            params.default_port_value = "/rm/tracker/target";
            factory->registerNodeType<RmTargetUpdater>("RmTargetUpdater", params);
            factory->registerNodeType<HasAttackTarget>("HasAttackTarget");
            factory->registerNodeType<SwitchAttack>("SwitchAttack", dataEx);
        }
        {
            factory->registerNodeType<NeedGyro>("NeedGyro", dataEx);
            factory->registerNodeType<SwitchGyro>("SwitchGyro", dataEx);
        }
        {
            factory->registerNodeType<WaitFallback>("WaitFallback");
        }
        {
            BT::RosNodeParams params;
            params.nh = node;
            params.default_port_value = "/can_init";
            factory->registerNodeType<PubCanInit>("PubCanInit", params);
        }
        {
            BT::RosNodeParams params;
            params.nh = node;
            params.default_port_value = "/lidar/dyn_scan";
            factory->registerNodeType<rm_sentry_ai::bt::ShutDownScan>("ShutDownScan", params);
        }
        return factory;
    }
}// namespace rm_sentry_ai::bt