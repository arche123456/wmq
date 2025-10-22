#ifndef IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT_CVT__HPP
#define IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT_CVT__HPP
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/exceptions.h>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <ifr_common/defs.h>
#include <limits>
#include <rm_common/defs.h>
#include <string>
#include <unordered_map>
namespace rm_sentry_ai::bt {

    struct HealthCheckData {
        bool is_self_team;
        ifr_interface::defs::rm::RobotType robot;
        uint16_t min_health = 0;
        uint16_t max_health = std::numeric_limits<uint16_t>::max();
    };
    using HealthCheckDatas = std::vector<HealthCheckData>;

    struct IndexMap {
        std::unordered_map<size_t, size_t> map;
    };


}// namespace rm_sentry_ai::bt
namespace BT {
    template<>
    [[nodiscard]] inline geometry_msgs::msg::PoseStamped convertFromString(StringView str) {
        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ';');
        if (parts.size() < 4) {
            throw RuntimeError("invalid input " + std::string{str});
        } else {
            geometry_msgs::msg::PoseStamped pos;
            pos.header.frame_id = convertFromString<std::string>(parts[0]);
            pos.pose.position.x = convertFromString<double>(parts[1]);
            pos.pose.position.y = convertFromString<double>(parts[2]);
            pos.pose.position.z = convertFromString<double>(parts[3]);
            if (parts.size() > 7) {
                pos.pose.orientation.w = convertFromString<double>(parts[4]);
                pos.pose.orientation.x = convertFromString<double>(parts[5]);
                pos.pose.orientation.y = convertFromString<double>(parts[6]);
                pos.pose.orientation.z = convertFromString<double>(parts[7]);
            } else {
                pos.pose.orientation.w = 1;
                pos.pose.orientation.x = 0;
                pos.pose.orientation.y = 0;
                pos.pose.orientation.z = 0;
            }
            return pos;
        }
    }
    template<>
    [[nodiscard]] inline rm_sentry_ai::bt::HealthCheckData convertFromString(StringView str) {
        auto parts = splitString(str, ',');
        if (parts.size() != 3) {
            throw RuntimeError("invalid input " + std::string{str} + ", need: 'id,min,max'");
        } else {
            auto tid = convertFromString<int>(parts[0]);
            bool is_self_team = !(tid / 100);
            auto id = ifr_interface::defs::rm::RobotType(tid % 100);
            uint16_t min_h = convertFromString<unsigned long>(parts[1]);
            uint16_t max_h = convertFromString<unsigned long>(parts[2]);
            return {is_self_team, id, min_h, max_h};
        }
    }
    template<>
    [[nodiscard]] inline rm_sentry_ai::bt::HealthCheckDatas convertFromString(StringView str) {
        auto parts = splitString(str, ';');
        rm_sentry_ai::bt::HealthCheckDatas results;
        results.reserve(parts.size());
        for (const auto &part: parts) {
            results.emplace_back(convertFromString<rm_sentry_ai::bt::HealthCheckData>(part));
        }
        return results;
    }
    template<class Key, class Value>
    [[nodiscard]] inline std::unordered_map<Key, Value> convertFromString(StringView str) {
        if (str.empty()) return {};
        auto parts = splitString(str, ';');
        std::unordered_map<Key, Value> map;
        for (const auto &part: parts) {
            auto kv = splitString(part, '=');
            if (kv.size() != 2) throw BT::RuntimeError("Bad map part: " + std::string{part});
            map.emplace(convertFromString<Key>(kv[0]), convertFromString<Value>(kv[1]));
        }
        return map;
    }

    template<>
    [[nodiscard]] inline rm_sentry_ai::bt::IndexMap convertFromString(StringView str) {
        if (str.empty()) return {};
        auto parts = splitString(str, ';');
        std::unordered_map<size_t, size_t> map;
        for (const auto &part: parts) {
            auto kv = splitString(part, '=');
            if (kv.size() != 2) throw BT::RuntimeError("Bad map part: " + std::string{part});
            map.emplace(convertFromString<size_t>(kv[0]), convertFromString<size_t>(kv[1]));
        }
        return {map};
    }

}// end namespace BT
#endif// IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT_CVT__HPP
