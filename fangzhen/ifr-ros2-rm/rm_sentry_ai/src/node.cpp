#include "rm_sentry_ai/node.hpp"
#include "rm_sentry_ai/bt/all.hpp"
#include "rm_sentry_ai/bt_json.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_file_logger_v2.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <rclcpp/logging.hpp>
#include <sstream>
namespace rm_sentry_ai {
    Node::Node(const rclcpp::NodeOptions &options)
        : rclcpp::Node("rm_sentry_ai", options),
          bt_node_(create_sub_node("bt")) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Starting " << this->get_name());


        RCLCPP_INFO_STREAM(this->get_logger(), "create data exchange");
        dataEx = std::make_shared<bt::DataExchange>(bt_node_);
        RCLCPP_INFO_STREAM(this->get_logger(), "create bt factory");
        auto factory = bt::getBehaviorTreeFactory(bt_node_, dataEx);
        RCLCPP_INFO_STREAM(this->get_logger(), "RegisterJsonDefinitions");
        rm_sentry_ai::bt::json_export::RegisterJsonDefinitions();

        RCLCPP_INFO_STREAM(this->get_logger(), "finding tree");
        std::filesystem::path dir = ament_index_cpp::get_package_share_directory("rm_sentry_ai");
        auto xml_filepath = dir / "trees" / this->declare_parameter("tree", "root.xml");

        RCLCPP_INFO_STREAM(this->get_logger(), "creating tree: " << xml_filepath);
        tree = factory->createTreeFromFile(xml_filepath);

        RCLCPP_INFO_STREAM(this->get_logger(), "created tree, struct:");
        BT::printTreeRecursively(tree.rootNode());


        if (this->declare_parameter("motor_groot2", true)) {
            unsigned port = this->declare_parameter("motor_groot2_port", 1667);
            groot2_publisher = std::make_shared<BT::Groot2Publisher>(tree, port);
            RCLCPP_INFO_STREAM(this->get_logger(), "Enable: motor groot2: localhost:" << port);
        } else {
            RCLCPP_INFO(this->get_logger(), "Disable: motor groot2");
        }

        {
            std::filesystem::path CPP = __FILE__;
            auto dir = CPP.parent_path().parent_path() / "logs";
            std::filesystem::path log_dir = this->declare_parameter<std::string>("log_dir", dir.string());
            if (!log_dir.empty()) {
                std::filesystem::create_directories(log_dir);
                auto now = std::chrono::system_clock::now();
                auto time_t_now = std::chrono::system_clock::to_time_t(now);
                std::tm bt_tm = *std::localtime(&time_t_now);
                std::ostringstream oss;
                oss << std::put_time(&bt_tm, "%Y%m%d-%H%M%S") << ".btlog";
                btlog_file = log_dir / oss.str();
                RCLCPP_INFO_STREAM(this->get_logger(), "Enable: btlog >> " << btlog_file);
                btlog = std::make_shared<BT::FileLogger2>(tree, btlog_file);
            } else {
                btlog_file = "";
                RCLCPP_INFO(this->get_logger(), "Disable: btlog");
            }
        }

        static constexpr const auto sleep = std::chrono::milliseconds(10);
        while (rclcpp::ok()) {
            dataEx->updateTick();
            tree.tickOnce();
            tree.sleep(sleep);
        }

        RCLCPP_INFO_STREAM(this->get_logger(), "Success start " << this->get_name());
    }
}// namespace rm_sentry_ai

// Register node as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_sentry_ai::Node)
