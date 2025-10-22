#include "rm_sentry_ai/bt/all.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/xml_parsing.h>
#include <filesystem>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
namespace fs = std::filesystem;
namespace rm_sentry_ai {
    class ExportBtXml : public rclcpp::Node {
        void export_xml() {
            RCLCPP_INFO(get_logger(), "Registering behavior tree nodes...");
            auto btf = rm_sentry_ai::bt::getBehaviorTreeFactory(this->create_sub_node("bt"), nullptr);
            std::string xml = BT::writeTreeNodesModelXML(*btf);

            RCLCPP_INFO(get_logger(), "Getting path...");
            fs::path dir = ament_index_cpp::get_package_share_directory("rm_sentry_ai");
            auto filepath = fs::absolute(dir / "bt_nodes.xml");
            RCLCPP_INFO_STREAM(get_logger(), "path: " << filepath);

            std::ofstream file(filepath, std::ios::out | std::ios::trunc);
            if (!file) {
                RCLCPP_FATAL(get_logger(), "Cannot open file: %s", filepath.c_str());
                return;
            }
            file << xml;
            file.close();
            RCLCPP_INFO(get_logger(), "String saved to %s", filepath.c_str());
        }

        rclcpp::TimerBase::SharedPtr timer_;

    public:
        ExportBtXml(const rclcpp::NodeOptions &options)
            : rclcpp::Node("export_bt_xml", options) {
            export_xml();
            rclcpp::shutdown();
        }
    };
}// namespace rm_sentry_ai

// Register node as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_sentry_ai::ExportBtXml)
