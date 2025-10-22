#ifndef IFR_ROS2_CV__PACKAGE_RM_FAKE_REFEREE__GAME__HPP
#define IFR_ROS2_CV__PACKAGE_RM_FAKE_REFEREE__GAME__HPP

#include <any>
#include <chrono>
#include <ifr_common/defs.h>
#include <memory>
#include <opencv2/core/base.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>
#include <rm_common/referee_datas.h>
#include <rm_referee_interface/all.hpp>
#include <std_srvs/srv/detail/trigger__struct.hpp>
#include <unordered_map>
#include <unordered_set>
namespace rm_fake_referee::game {

    using namespace std::chrono_literals;
    using namespace RefereeSystem::packet;
    using namespace std::chrono;
    using clock = std::chrono::steady_clock;
    class Game {
    protected:
        rclcpp::TimerBase::SharedPtr game_time_updater;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start;

        rclcpp::Node::SharedPtr node;
        double game_now_sec_;// 游戏秒数
        bool in_gaming_;     // 是否在游戏中
        ifr_interface::defs::Team team_;
        ifr_interface::defs::rm::RobotType robot_;


    public:
        virtual void start() {
            game_now_sec_ = 0;
            in_gaming_ = true;

            {
                int v;
                node->get_parameter_or("team", v, 0);
                team_ = static_cast<decltype(team_)>(v);
            }
            {
                int v;
                node->get_parameter_or("robot", v, 7);
                robot_ = static_cast<decltype(robot_)>(v);
            }

            game_time_updater = node->create_wall_timer(1s, [this]() {
                double speed;
                node->get_parameter_or("game_speed", speed, 1.0);
                game_now_sec_ += speed;
            });
        }

        Game(rclcpp::Node *__node) {
            node = __node->create_sub_node("rmul");

            {
                rcl_interfaces::msg::ParameterDescriptor desc;
                desc.set__read_only(false).set__description("0=RED,1=BLUE");
                desc.integer_range.push_back(rcl_interfaces::msg::IntegerRange()
                                                     .set__from_value(0)
                                                     .set__to_value(1)
                                                     .set__step(1));
                node->declare_parameter("team", 0, desc);
            }
            {
                rcl_interfaces::msg::ParameterDescriptor desc;
                desc.set__read_only(false).set__description("1=HERO,2=ENGINEER,3=INFANTRY,4=INFANTRY,5=INFANTRY,6=AERIAL,7=SENTRY,8=DARTS,9=RADAR");
                desc.integer_range.push_back(rcl_interfaces::msg::IntegerRange()
                                                     .set__from_value(1)
                                                     .set__to_value(9)
                                                     .set__step(1));
                node->declare_parameter("robot", 7, desc);
            }

            srv_start = node->create_service<std_srvs::srv::Trigger>(
                    "start", [this](
                                     const std_srvs::srv::Trigger::Request::SharedPtr,
                                     std_srvs::srv::Trigger::Response::SharedPtr response) {
                        response->message = in_gaming_ ? "Restart" : "Start";
                        this->start();
                        response->success = true;
                        RCLCPP_INFO(this->node->get_logger(), "RMUL Game %s!", response->message.c_str());
                    });
        }

    protected:
        template<class Pkg, typename Enable = void>
        struct PkgManager {
            using info = RefereeSystem::pkg_info<Pkg>;
            using ROS_TYPE = typename info::ROS_TYPE;
            ROS_TYPE _raw;
            Pkg _pkg;
            rclcpp::TimerBase::SharedPtr timer;
            typename rclcpp::Publisher<ROS_TYPE>::SharedPtr publisher;

            constexpr Pkg &pkg() { return _pkg; }
            constexpr const Pkg &pkg() const { return _pkg; }
            constexpr ROS_TYPE &raw() {
                _pkg.to(_raw);
                return _raw;
            }
            constexpr const ROS_TYPE &raw() const {
                _pkg.to(_raw);
                return _raw;
            }
        };
        template<class Pkg>
        struct PkgManager<Pkg, std::enable_if_t<!RefereeSystem::pkg_info<Pkg>::has_ros_type>> {
            using info = RefereeSystem::pkg_info<Pkg>;
            using ROS_TYPE = typename info::ROS_TYPE;
            ROS_TYPE _raw;
            rclcpp::TimerBase::SharedPtr timer;
            typename rclcpp::Publisher<ROS_TYPE>::SharedPtr publisher;

            constexpr Pkg &pkg() { return *reinterpret_cast<Pkg *>(_raw.data.data()); }
            constexpr const Pkg &pkg() const { return *reinterpret_cast<Pkg *>(_raw.data.data()); }
            constexpr ROS_TYPE &raw() { return _raw; }
            constexpr const ROS_TYPE &raw() const { return _raw; }
        };

    private:
        std::unordered_map<uint16_t, std::shared_ptr<PkgManager<std::any>>> all_pkg;

    protected:
        template<class GameT, class T>
        void add_pkg(rclcpp::QoS qos = rclcpp::SensorDataQoS()) {
            static constexpr const auto NAME = T::NAME;
            static constexpr const auto ID = T::ID;
            static constexpr const auto INTERVAL = T::INTERVAL;
            using TYPE = typename RefereeSystem::pkg_info<T>::ROS_TYPE;
            auto pm = std::make_shared<PkgManager<T>>();
            auto topic = RefereeSystem::getTopic<T>();
            pm->publisher = node->create_publisher<TYPE>(topic, qos);

            pm->timer = node->create_wall_timer(INTERVAL > 0 ? milliseconds(INTERVAL) : 500ms, [pm, this]() {
                if (!static_cast<GameT *>(this)->work(pm->pkg())) return;
                pm->publisher->publish(pm->raw());
            });

            RCLCPP_INFO_STREAM(node->get_logger(), "add pkg: " << NAME << "(0x" << std::hex << ID << "), topic = " << topic);
        }
        template<class T>
        auto get_pkg() {
            static constexpr const auto ID = T::ID;
            CV_Assert(all_pkg.count(ID));
            return std::any_cast<std::shared_ptr<PkgManager<T>>>(all_pkg[ID]);
        }

    protected:
        template<bool read_only = false>
        void declare_parameter(const std::string &name, const int &min, const int &val, const int &max) {
            rcl_interfaces::msg::ParameterDescriptor desc;
            desc.read_only = read_only;
            desc.integer_range.push_back(rcl_interfaces::msg::IntegerRange()
                                                 .set__from_value(min)
                                                 .set__to_value(max)
                                                 .set__step(1));

            // RCLCPP_INFO_STREAM(node->get_logger(), "node1_namespace:" << node->get_namespace());
            node->declare_parameter(name, val, desc);
          
            // int value;
            // node->get_parameter(name, value);
            // RCLCPP_INFO_STREAM(node->get_logger(), name << "'s value:" << value);
        }
        template<class T>
        T get_parameter_or(const std::string &name, T default_value) {

            T t;
            node->get_parameter_or(name, t, default_value);
            return t;
        }
    };
}// namespace rm_fake_referee::game
#endif// IFR_ROS2_CV__PACKAGE_RM_FAKE_REFEREE__GAME__HPP
