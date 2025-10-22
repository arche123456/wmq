#ifndef IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___PARALLEL_ASYNC__HPP
#define IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___PARALLEL_ASYNC__HPP

#include "rm_sentry_ai/bt_cvt.hpp"
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/control_node.h>
#include <cstddef>
#include <set>
#include <unordered_map>

namespace rm_sentry_ai::bt {
    /// @brief AsyncParallelNode 并行地执行所有子节点，
    /// 允许每个子节点根据自己的速度运行，而不需要等待其他子节点完成。这个节点旨在高效地处理执行时间不同的任务，
    /// 使得执行较快的任务能够更频繁地执行。
    ///
    /// @details 与 ParallelAllNode 不同，后者需要等待所有子节点完成后才能继续，
    /// AsyncParallelNode 一旦任何子节点返回 RUNNING，就可以返回 RUNNING，使其响应更加迅速。只有当特定子节点的成功或失败
    /// 阈值达到时，它才会返回 SUCCESS 或 FAILURE，否则它将继续运行。
    ///
    /// @details 假设有两个子节点 NodeA, NodeB, 如果NodeA最高可以以10Hz的频率运行, NodeB最高只能以1Hz的频率运行,
    /// 在没有任何成功/失败限制的理想情况下:
    /// AsyncParallelNode 可以执行1秒, 期间一直为RUNNING, NodeA会被执行10次, NodeB会被执行1次.
    /// 理想情况下, 一秒后NodeA与NodeB会同时返回一个非RUNNING的结果, 此时AsyncParallelNode也会返回非RUNNING
    ///
    /// @details 另一种假设: 假设有两个子节点 NodeA, NodeB, 如果NodeA是一个立即执行节点, NodeB最高只能以1Hz的频率运行,
    /// 在没有任何成功/失败限制的理想情况下, AsyncParallelNode被以100Hz的频率调用:
    /// AsyncParallelNode 可以执行1秒, 期间一直为RUNNING, NodeA会被执行100次, NodeB会被执行1次.
    /// 理想情况下, 一秒后NodeB会返回一个非RUNNING的结果, 此时AsyncParallelNode也会返回非RUNNING
    ///
    /// @details 可以为单个子节点设置成功和失败限制，以控制节点的整体行为。当一个子节点的成功或失败计数达到其限制时，
    /// 节点可能会强制返回 SUCCESS 或 FAILURE。限制以子节点索引和限制值的对形式指定，
    /// 其中 -1 代表无限制，意味着该子节点的结果不会强制返回。
    ///
    /// @details 限额示例说明：
    /// 成功限制: [子节点索引=限制;子节点索引=限制;...]
    /// 失败限制: [子节点索引=限制;子节点索引=限制;...]
    /// 如果没有为子节点指定限制，则认为它没有限制.
    /// 允许所有子节点根据自己的结果贡献到整体状态中，做整体结果判断。
    ///
    /// @note 注意：虽然此节点允许并行执行，但它不会自动为每个子节点管理单独的线程。如果需要真正的并行性，
    /// 子节点本身必须管理自己的线程或异步执行。
    class ParallelAsync : public BT::ControlNode {
    public:
        ParallelAsync(const std::string &name, const BT::NodeConfig &config);


        static BT::PortsList providedPorts();

        ~ParallelAsync() override = default;

        virtual void halt() override {
            success_counts_.clear();
            failure_counts_.clear();
            ControlNode::halt();
        }


    private:
        std::unordered_map<size_t, size_t> success_thresholds_;
        std::unordered_map<size_t, size_t> failure_thresholds_;

        std::unordered_map<size_t, size_t> success_counts_;
        std::unordered_map<size_t, size_t> failure_counts_;

        BT::NodeStatus tick() override;
    };

}// namespace rm_sentry_ai::bt
#endif// IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___PARALLEL_ASYNC__HPP
