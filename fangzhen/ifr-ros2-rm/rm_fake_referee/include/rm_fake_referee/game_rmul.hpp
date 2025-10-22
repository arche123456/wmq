#ifndef IFR_ROS2_CV__PACKAGE_RM_FAKE_REFEREE__GAME_RMUL__HPP
#define IFR_ROS2_CV__PACKAGE_RM_FAKE_REFEREE__GAME_RMUL__HPP
#include "game.hpp"
#include <chrono>
#include <ifr_common/defs.h>
#include <ios>
#include <mutex>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <rm_common/referee_datas.h>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <std_srvs/srv/detail/trigger__struct.hpp>
#include <type_traits>
namespace rm_fake_referee::game {
    using namespace std::chrono_literals;
    using namespace RefereeSystem::packet;
    using namespace std::chrono;
    using clock = std::chrono::steady_clock;
    class RMUL_Sentry : public Game {

    public:
        void start() override {
            Game::start();
            lst_in_gaming_ = false;
            bullet_remaining_num_17mm = 750;
        }

        RMUL_Sentry(rclcpp::Node *__node) : Game(__node) {
            reigsterPkg();

            sub_ShootData = RefereeSystem::create_subscription<ShootData>(
                    node.get(), [this](const auto &, auto &) { bullet_remaining_num_17mm--; });
            // int value;
            declare_parameter("robot_level", 1, 5, 10);
            declare_parameter("remain_HP", 1, 600, 600);
           
           
        }


    protected:
        friend Game;
        static constexpr const seconds GTPS[] = {1s, 1s, 1s, 5s, 5min, 1s};
        bool work(GameStatus &pkg) {
            if (!in_gaming_) return false;
            pkg.SyncTimeStamp = std::chrono::duration_cast<seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            pkg.game_type = 4;
            pkg.game_progress = 0;
            {
                seconds time = seconds(long(game_now_sec_));
                for (const auto &timepoint: GTPS) {
                    if (time < timepoint) {
                        pkg.stage_remain_time = (timepoint - time).count();
                        time = -1s;
                        break;
                    }
                    time -= timepoint;
                    ++pkg.game_progress;
                }
                if (time >= 0s) {
                    in_gaming_ = false;
                    return false;// game end
                }
            }
            return true;
        }
        bool lst_in_gaming_ = false;
        bool work(GameResult &pkg) {
            if (!in_gaming_ && lst_in_gaming_) {
                pkg.winner = 0;
                lst_in_gaming_ = false;
                return true;
            }
            lst_in_gaming_ = in_gaming_;
            return false;
        }
        bool work(GameRobotHP &pkg) {
            if (!in_gaming_) return false;
            pkg.blue_1_robot_HP = pkg.red_1_robot_HP = 300;
            pkg.blue_2_robot_HP = pkg.red_2_robot_HP = 0;
            pkg.blue_3_robot_HP = pkg.red_3_robot_HP = 300;
            pkg.blue_4_robot_HP = pkg.red_4_robot_HP = 0;
            pkg.blue_5_robot_HP = pkg.red_5_robot_HP = 0;
            pkg.blue_7_robot_HP = pkg.red_7_robot_HP = get_parameter_or("remain_HP", 600);
            pkg.blue_outpost_HP = pkg.red_outpost_HP = 0;
            pkg.blue_base_HP = pkg.red_base_HP = 1500;
            return true;
        }
        bool work(EventData &pkg) {
            if (!in_gaming_) return false;
            pkg.isOccupy1 = false;
            pkg.isOccupy2 = false;
            pkg.isOccupy3 = false;
            pkg.isOccupyEM = pkg.isActiveEMsm = pkg.isActiveEMlg = false;
            pkg.isR2B2Occupy = pkg.isR3B3Occupy = pkg.isR4B4Occupy = false;
            pkg.hasVirtualShield = true;
            pkg.isOutpostBattleSurvival = false;
            return true;
        }
        bool work(GameRobotStatus &pkg) {
            if (!in_gaming_) return false;
            pkg.robot_id = int(team_) * 100 + int(robot_);
            pkg.robot_level = get_parameter_or("robot_level", 10);
            pkg.remain_HP = get_parameter_or("remain_HP", 400);
            pkg.max_HP = 600;
            pkg.shooter_barrel_cooling_value = 0;
            pkg.shooter_barrel_heat_limit = 0;
            pkg.chassis_power_limit = 0;
            pkg.chassis_power_limit = 0;
            pkg.mains_power_gimbal_output = 0;
            pkg.mains_power_chassis_output = 0;
            pkg.mains_power_shooter_output = 0;
            return true;
        }
        bool work(Buff &pkg) {
            if (!in_gaming_) return false;
            pkg.recovery_buff = 0;
            pkg.cooling_buff = 0;
            pkg.defence_buff = 0;
            pkg.vulnerability_buff = 0;
            pkg.attack_buff = 0;
            return true;
        }
        bool work(RobotHurt &pkg) {
            return false;
        }
        bool work(ShootData &pkg) {
            return false;
        }

        int bullet_remaining_num_17mm = 750;
        RefereeSystem::pkg_info<ShootData>::Subscription::SharedPtr sub_ShootData;
        bool work(BulletRemaining &pkg) {
            if (!in_gaming_) return false;
            pkg.bullet_remaining_num_17mm = bullet_remaining_num_17mm;
            return true;
        }


        void reigsterPkg() {
            add_pkg<RMUL_Sentry, GameStatus>();
            add_pkg<RMUL_Sentry, GameResult>();
            add_pkg<RMUL_Sentry, GameRobotHP>();
            add_pkg<RMUL_Sentry, EventData>();
            add_pkg<RMUL_Sentry, GameRobotStatus>();
            add_pkg<RMUL_Sentry, Buff>();
            add_pkg<RMUL_Sentry, RobotHurt>();
            add_pkg<RMUL_Sentry, ShootData>();
            add_pkg<RMUL_Sentry, BulletRemaining>();
        }
    };
}// namespace rm_fake_referee::game
#endif// IFR_ROS2_CV__PACKAGE_RM_FAKE_REFEREE__GAME_RMUL__HPP
