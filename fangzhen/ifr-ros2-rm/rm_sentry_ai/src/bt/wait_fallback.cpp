
#include "rm_sentry_ai/bt/wait_fallback.h"
namespace rm_sentry_ai::bt {


    BT::NodeStatus WaitFallback::tick() {
        bool all_skipped = true;
        if (status() == BT::NodeStatus::IDLE) {
            running_child_ = -1;
        }
        setStatus(BT::NodeStatus::RUNNING);

        for (size_t index = 0; index < childrenCount(); index++) {
            TreeNode *current_child_node = children_nodes_[index];
            const BT::NodeStatus child_status = current_child_node->executeTick();

            // switch to RUNNING state as soon as you find an active child
            all_skipped &= (child_status == BT::NodeStatus::SKIPPED);

            switch (child_status) {
                case BT::NodeStatus::RUNNING: {
                    if (running_child_ != -1 && size_t(running_child_) != index) {
                        children_nodes_[running_child_]->haltNode();
                    }
                    running_child_ = int(index);
                    return BT::NodeStatus::RUNNING;
                }

                case BT::NodeStatus::FAILURE:
                    break;

                case BT::NodeStatus::SUCCESS: {
                    resetChildren();
                    return BT::NodeStatus::SUCCESS;
                }

                case BT::NodeStatus::SKIPPED: {
                    // to allow it to be skipped again, we must reset the node
                    haltChild(index);
                } break;

                case BT::NodeStatus::IDLE: {
                    throw BT::LogicError("[", name(), "]: A children should not return IDLE");
                }
            }// end switch
        }    // end for

        resetChildren();
        return all_skipped ? BT::NodeStatus::SKIPPED : BT::NodeStatus::FAILURE;
    }

    void WaitFallback::halt() {
        running_child_ = -1;
        ControlNode::halt();
    }

}// namespace rm_sentry_ai::bt
