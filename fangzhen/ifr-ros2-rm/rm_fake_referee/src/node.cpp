#include "rm_fake_referee/node.hpp"


namespace rm_fake_referee {
    FakeReferee::FakeReferee(const rclcpp::NodeOptions &options)
        : rclcpp::Node("rm_fake_referee", options) {
        game.start();
    }
}// namespace rm_fake_referee
// Register node as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_fake_referee::FakeReferee)