#ifndef IFR_ROS2_CV__PACKAGE_RM_FAKE_REFEREE__NODE__HPP
#define IFR_ROS2_CV__PACKAGE_RM_FAKE_REFEREE__NODE__HPP
#include "rm_fake_referee/game_rmul.hpp"
#include <chrono>
#include <cstdint>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/timer.hpp>
#include <rm_common/referee_datas.h>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <unordered_map>

namespace rm_fake_referee {
    class FakeReferee : public rclcpp::Node {


        game::RMUL_Sentry game{this};

    public:
        FakeReferee(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    };
}// namespace rm_fake_referee
#endif// IFR_ROS2_CV__PACKAGE_RM_FAKE_REFEREE__NODE__HPP
