#include "rm_sentry_ai/bt/pub_can_init.hpp"
#include "rm_sentry_ai/bt_defs.hpp"
#include <limits>


namespace rm_sentry_ai::bt {
    PubCanInit::PubCanInit(const std::string &name,
                           const BT::NodeConfig &conf,
                           const BT::RosNodeParams &params)
        : BT::RosTopicPubNode<std_msgs::msg::Bool>(name, conf, params) {
    }
    BT::PortsList PubCanInit::providedPorts() {
        return {
                BT::InputPort<bool>("can_init", true, "发布是否允许初始化"),
        };
    }
    bool PubCanInit::setMessage(std_msgs::msg::Bool &msg) {
        auto result = getInput<bool>("can_init");
        if (result) {
            msg.data = result.value();
            return true;
        } else {
            BTROS_ERROR_STREAM("Failed get input 'can_init': " << result.error());
            return false;
        }
    }
    ShutDownScan ::ShutDownScan(const std::string &name,
                                const BT::NodeConfig &conf,
                                const BT::RosNodeParams &params)
        : BT::RosTopicPubNode<std_msgs::msg::Bool>(name, conf, params) {
    }
    BT::PortsList ShutDownScan::providedPorts() {
        return {
                BT::InputPort<bool>("shut_scan", false, "发布是否允许扫描"),
        };
    }
    bool ShutDownScan::setMessage(std_msgs::msg::Bool &msg) {
        auto result = getInput<bool>("shut_scan");
        if (result) {
            msg.data = result.value();
            return true;
        } else {
            BTROS_ERROR_STREAM("Failed get input 'shut_scan': " << result.error());
            return false;
        }
    }
}// namespace rm_sentry_ai::bt