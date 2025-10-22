#include "rm_sentry_ai/bt/parallel_async.hpp"
#include "rm_sentry_ai/bt_cvt.hpp"

namespace rm_sentry_ai::bt {

    ParallelAsync::ParallelAsync(const std::string &name, const BT::NodeConfig &config)
        : ControlNode::ControlNode(name, config) {
        IndexMap map;
        if (!getInput("success_limit", map)) {
            throw BT::RuntimeError("Missing parameter [success_limit] in ParallelNode");
        } else {
            success_thresholds_ = map.map;
        }
        if (!getInput("failure_limit", map)) {
            throw BT::RuntimeError("Missing parameter [failure_limit] in ParallelNode");
        } else {
            failure_thresholds_ = map.map;
        }
    }
    BT::PortsList ParallelAsync::providedPorts() {
        return {
                BT::InputPort<size_t>("max_failures", 1,
                                      "If the number of children returning FAILURE exceeds this value, "
                                      "ParallelAll returns FAILURE"),
                BT::InputPort<IndexMap>("success_limit", "",
                                        "成功限制 [子节点索引=限制;子节点索引=限制;...]"),
                BT::InputPort<IndexMap>("failure_limit", "",
                                        "失败限制 [子节点索引=限制;子节点索引=限制;...]"),
        };
    }
    BT::NodeStatus ParallelAsync::tick() {
        const size_t children_count = children_nodes_.size();
        size_t success_count = 0, failure_count = 0, skipped_count = 0;//单帧状态
        size_t success_counts = 0, failure_counts = 0;                 //单循环达到限制, 强制结束
        for (size_t index = 0; index < children_count; index++) {
            const auto &child_node = children_nodes_[index];
            const BT::NodeStatus child_status = child_node->executeTick();

            switch (child_status) {
                case BT::NodeStatus::SUCCESS: {
                    ++success_count;
                    if (success_thresholds_.count(index))
                        if (++success_counts_[index] >= success_thresholds_[index])
                            ++success_counts;
                } break;

                case BT::NodeStatus::FAILURE: {
                    ++failure_count;
                    if (failure_thresholds_.count(index))
                        if (++failure_counts_[index] >= failure_thresholds_[index])
                            ++failure_counts;
                } break;

                case BT::NodeStatus::RUNNING: {
                    // Still working. Check the next
                } break;

                case BT::NodeStatus::SKIPPED: {
                    ++skipped_count;
                } break;

                case BT::NodeStatus::IDLE: {
                    throw BT::LogicError("[", name(), "]: A children should not return IDLE");
                }
            }
        }

        if (skipped_count >= children_count) {
            return BT::NodeStatus::SKIPPED;
        }
        BT::NodeStatus status = BT::NodeStatus::RUNNING;


        if (skipped_count + success_count + failure_count >= children_count) {
            size_t max_failures = 0;
            if (!getInput("max_failures", max_failures)) {
                throw BT::RuntimeError("Missing parameter [max_failures] in ParallelNode");
            }
            status = (failure_count >= max_failures) ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
        } else if (success_counts || failure_counts) {
            status = failure_count >= success_counts ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
        }

        if (status != BT::NodeStatus::RUNNING) {
            haltChildren();
            success_counts_.clear();
            failure_counts_.clear();
        }
        return status;
    }
}// namespace rm_sentry_ai::bt