#ifndef IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___WAIT_FALLBACK__H
#define IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___WAIT_FALLBACK__H

#include "behaviortree_cpp/control_node.h"

namespace rm_sentry_ai::bt {

    class WaitFallback : public BT::ControlNode {
    public:
        WaitFallback(const std::string &name) : ControlNode(name, {}) {}


    private:
        BT::NodeStatus tick() override;

        void halt() override;

        int running_child_ = -1;
    };
}// namespace rm_sentry_ai::bt
#endif// IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___WAIT_FALLBACK__H
