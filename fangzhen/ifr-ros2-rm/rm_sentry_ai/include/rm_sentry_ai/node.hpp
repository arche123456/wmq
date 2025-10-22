#ifndef IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__NODE__HPP
#define IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__NODE__HPP
#include "rm_sentry_ai/bt/all.hpp"
#include <behaviortree_cpp/loggers/bt_file_logger_v2.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
namespace rm_sentry_ai {
    class Node : public rclcpp::Node {
    public:
        Node(const rclcpp::NodeOptions &options);

    private:
        rclcpp::Node::SharedPtr bt_node_;
        bt::DataExchange::Ptr dataEx;
        BT::Tree tree;
        std::shared_ptr<BT::Groot2Publisher> groot2_publisher;

        std::filesystem::path btlog_file;
        std::shared_ptr<BT::FileLogger2> btlog;

        void call_bt();
    };
}// namespace rm_sentry_ai
#endif// IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__NODE__HPP
