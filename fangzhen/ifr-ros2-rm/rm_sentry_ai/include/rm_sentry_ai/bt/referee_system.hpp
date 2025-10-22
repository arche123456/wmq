#ifndef IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___REFEREE_SYSTEM__HPP
#define IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___REFEREE_SYSTEM__HPP
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <chrono>
#include <cstdint>
#include <ifr_common/ext_funcs.h>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rm_common/referee_datas.h>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <vector>
namespace rm_sentry_ai::bt {
    class RefereeSystemUpdate final : public BT::SyncActionNode {
    public:
        template<class T>
        struct Data {
            T msg;                                            /// 消息
            uint64_t update_id = 0;                           /// 更新id
            std::chrono::steady_clock::time_point update_time;/// 更新时间

            Data() {}
            bool __has_msg;
            T __msg_update;// ros更新的消息
        };

    private:
        /// @brief 注册一个裁判系统数据包
        /// @param rsu 实例化对象
        /// @details 提供一个节点指针rsu, 将会对其创建事件监听处理的handler. 不提供则不会做任何事
        /// @returns 返回注册的 BT::OutputPort, 可供providedPorts使用
        template<class T>
        static std::pair<std::string, BT::PortInfo> registerTopic(RefereeSystemUpdate *rsu);
        static BT::PortsList registerTopics(RefereeSystemUpdate *rsu);

    public:
        RefereeSystemUpdate(const std::string &name,
                            const BT::NodeConfig &conf,
                            const rclcpp::Node::SharedPtr &node);
        static BT::PortsList providedPorts();

        /// 更新port数据
        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
        std::vector<std::function<bool()>> handlers_;
    };

    template<class T>
    [[nodiscard]] std::string getRefereeSystemPortDefault() {
        static_assert(T::ID || T::NAME || 1, "");
        std::string ptr = "{REFEREE_MSG_";
        ptr += T::NAME;
        ptr += '}';
        return ptr;
    }

    /// @brief 从BT::Node获取裁判系统的数据
    template<class T>
    [[nodiscard]] FORCE_INLINE std::shared_ptr<RefereeSystemUpdate::Data<T>> getRefereeSystemPkg(BT::TreeNode *node) {
        static_assert(T::ID || T::NAME || 1, "");
        auto data = node->getInput<std::shared_ptr<RefereeSystemUpdate::Data<T>>>(T::NAME);
        if (data) {
            auto ptr = data.value();
            if (ptr->update_id) return ptr;
            return nullptr;
        }
        return nullptr;
    }

    /// @brief 获取裁判系统的Port声明(InputPort)
    template<class T>
    [[nodiscard]] FORCE_INLINE auto getRefereeSystemPort() {
        using PortT = std::shared_ptr<RefereeSystemUpdate::Data<T>>;
        static_assert(T::ID || T::NAME || 1, "");
        std::string ptr = "{REFEREE_MSG_";
        ptr += T::NAME;
        ptr += '}';
        return BT::InputPort<PortT>(T::NAME, getRefereeSystemPortDefault<T>(), std::string("Referee System Package: ") + T::NAME);
    }


    template<class T>
    std::pair<std::string, BT::PortInfo> RefereeSystemUpdate::registerTopic(RefereeSystemUpdate *rsu) {
        if (rsu) {
            using RosT = typename RefereeSystem::pkg_info<T>::ROS_TYPE;
            auto data = std::make_shared<Data<T>>();
            auto topic = RefereeSystem::getTopic<T>();
            std::cout << topic << std::endl;
            auto sub = rsu->node_->create_subscription<RosT>(topic, rclcpp::SensorDataQoS(), [data](typename RosT::SharedPtr ros) {
                data->__msg_update = RefereeSystem::fromRosMsg<T>(ros);

                data->__has_msg = true;
            });
            rsu->handlers_.push_back([rsu, sub, data]() {
                if (data->__has_msg) {
                    data->msg = data->__msg_update;
                    //RCLCPP_INFO_STREAM(rsu->node_->get_logger(),"get a value");
                    data->__has_msg = false;
                    ++data->update_id;
                    data->update_time = std::chrono::steady_clock::now();
                    auto res = rsu->setOutput<std::shared_ptr<Data<T>>>(T::NAME, data);
                    if (!res) RCLCPP_ERROR_STREAM(rsu->node_->get_logger(), '[' << rsu->name() << "] Failed Set '" << T::NAME << "': " << res.error());
                    return !!res;
                }
                return true;
            });
            RCLCPP_INFO_STREAM(rsu->node_->get_logger(), '[' << rsu->name() << "] " << __func__ << ": registered topic: " << T::NAME << " -> " << topic);
        }
        return BT::OutputPort<std::shared_ptr<Data<T>>>(T::NAME, getRefereeSystemPortDefault<T>(), std::string("Referee System Package: ") + T::NAME);
    }
}// namespace rm_sentry_ai::bt
#endif// IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT___REFEREE_SYSTEM__HPP
