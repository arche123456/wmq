#ifndef IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT_JSON__HPP
#define IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT_JSON__HPP
#include <behaviortree_cpp/contrib/json.hpp>
#include <behaviortree_cpp/json_export.h>
namespace rm_sentry_ai::bt::json_export {
    inline void Convert(nlohmann::json &dest, const std::chrono::steady_clock::time_point &src) {
        dest["ts"] = src.time_since_epoch().count();
        dest["offset"] = (src - std::chrono::steady_clock::now()).count();
    }
    inline void RegisterJsonDefinitions() {
        BT::RegisterJsonDefinition<std::chrono::steady_clock::time_point>(Convert);
    }

}// namespace rm_sentry_ai::bt::json_export
#endif// IFR_ROS2_CV__PACKAGE_RM_SENTRY_AI__BT_JSON__HPP
