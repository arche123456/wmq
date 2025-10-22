#ifndef IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___ALL__HPP
#define IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___ALL__HPP
#include "rm_sentry_ai/bt_cvt.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <memory>
#include <rclcpp/node.hpp>
namespace rm_sentry_ai::bt {

    class DataExchange {
        int tickId = 0;
        /// 覆盖管理
        /// 防止同一个tick高优被低优覆盖
        /// 拥有过期机制, 在DataExchange中执行updateNext()即可使旧数据过期
        struct OverrideInfo {
            friend class DataExchange;

        private:
            int *expireNowFlag;
            int expireLstFlag_ = false;
            int priority_ = 0;
            OverrideInfo(int *expireNowFlag) : expireNowFlag(expireNowFlag) {}
            OverrideInfo(DataExchange *de) : OverrideInfo(&de->tickId) {}

        public:
            /// @brief 尝试覆盖数据
            /// @return 是否可以覆盖
            /// 如果返回false, 则不可对被保护的数据进行修改.
            /// 如果返回true, 则必须对数据进行修改, 否则会导致其他决策无法执行, 沿用历史数据
            inline bool tryOverride(int priority) {
                if (*expireNowFlag == expireLstFlag_ && priority <= priority_) return false;
                expireLstFlag_ = *expireNowFlag;
                priority_ = priority;
                return true;
            }
        };

    public:
        using Ptr = std::shared_ptr<DataExchange>;

    public:
        inline void updateTick() { ++tickId; }

#define SENTRY_BT_DE_BOX(field, ...)                    \
    struct DataBox_##field {                            \
    private:                                            \
        friend class DataExchange;                      \
        OverrideInfo override_info;                     \
        DataBox_##field(DataExchange *de)               \
            : override_info(de) {}                      \
                                                        \
    public:                                             \
        inline bool tryOverride(int priority) {         \
            return override_info.tryOverride(priority); \
        }                                               \
                                                        \
    public:                                             \
        __VA_ARGS__                                     \
    } field { this }
        SENTRY_BT_DE_BOX(hc,
                         float yaw;
                         float pitch;
                         bool fast;);

        SENTRY_BT_DE_BOX(allow_attack,
                         bool v;);

        SENTRY_BT_DE_BOX(need_gyro,
                         bool v;);

    public:
        DataExchange(rclcpp::Node::SharedPtr node) : node_(node) {}
        rclcpp::Node::SharedPtr node_;

    private:
    };

    /// @brief 获取行为树Factory
    /// @details 自动注册所有BT Node
    /// @param node 行为树ros节点
    [[nodiscard]] std::unique_ptr<BT::BehaviorTreeFactory> getBehaviorTreeFactory(rclcpp::Node::SharedPtr node, DataExchange::Ptr dataEx);
}// namespace rm_sentry_ai::bt
#endif// IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___ALL__HPP
