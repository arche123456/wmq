#ifndef IFR_ROS2_CV__PACKAGE_RM_COMMON__DEFS__H
#define IFR_ROS2_CV__PACKAGE_RM_COMMON__DEFS__H

#include "rm_interface/msg/rm_robot_type.hpp"
#include <cstdint>
#include <ifr_common/defs.h>
#include <set>
#include <sys/types.h>

namespace ifr_interface::defs {

    namespace rm {
        /// @brief 机器人类型
        enum class RobotType {
            HERO_1 = 1,     /**< 1 英雄机器人*/
            ENGINEER_2 = 2, /**< 2 工程机器人*/
            INFANTRY_3 = 3, /**< 3 步兵机器人*/
            INFANTRY_4 = 4, /**< 4 步兵机器人*/
            INFANTRY_5 = 5, /**< 5 步兵机器人*/
            AERIAL_6 = 6,   /**< 6 空中机器人*/
            SENTRY_7 = 7,   /**< 7 哨兵机器人*/
            DARTS_8 = 8,    /**< 8 飞镖*/
            RADAR_9 = 9,    /**< 9 雷达*/
            OUTPOST,        /**< 前哨战*/
            BASE,           /**< 基地*/
        };
        IFR_INTERFACE_ENUM_CAST(RobotType, rm_interface::msg::RmRobotType, type)

        namespace const_values {

            static constexpr const float arm_l_h = 55.78F; //< 装甲板 灯条高度
            static constexpr const float arm_h = 127.2F;   //< 装甲板高度
            static constexpr const float arm_sm_w = 132.0F;//< 小装甲板宽度
            // static constexpr const float arm_h = 124.7F;   //< 装甲板高度
            // static constexpr const float arm_sm_w = 140.5F;//< 小装甲板宽度
            static constexpr const float arm_lg_w = 225.4F;//< 大装甲板宽度

            static constexpr const int arm_number_pixel_w = 20;//装甲板透射变换后的宽度
            static constexpr const int arm_number_pixel_h = 28;//装甲板透射变换后的高度

        }// namespace const_values

        namespace armor_id {

            enum class Type {
                ENGINEER_2 = 1, /**< 2 工程机器人*/
                INFANTRY_3 = 2, /**< 3 步兵机器人*/
                INFANTRY_4 = 3, /**< 4 步兵机器人*/
                INFANTRY_5 = 4, /**< 5 步兵机器人*/
                HERO_1 = 5,     /**< 1 英雄机器人*/
                BALANCE_3 = 6,  /**< 3 平衡机器人*/
                BALANCE_4 = 7,  /**< 4 平衡机器人*/
                BALANCE_5 = 8,  /**< 5 平衡机器人*/
                OUTPOST = 9,    /**< 前哨站*/
                BASE_SM = 10,   /**< 基地小装甲*/
                SENTRY_7 = 11,  /**< 7 哨兵机器人*/
                BASE_LG = 12,   /**< 基地大装甲*/
            };

            /// 装甲板ID定义, 详见装甲板素材包说明文档
            static const constexpr uint8_t arm_sm_ids[] = {1, 2, 3, 4, 9, 10, 11},
                                           arm_lg_ids[] = {5, 6, 7, 8, 12};
            static const constexpr size_t arm_sm_num = sizeof(arm_sm_ids) / sizeof(arm_sm_ids[0]);
            static const constexpr size_t arm_lg_num = sizeof(arm_lg_ids) / sizeof(arm_lg_ids[0]);

            /// n块装甲板的类型ID
            static const std::set<uint8_t> arm_num_4 = {1, 2, 3, 4, 5, 11};
            static const std::set<uint8_t> arm_num_2 = {6, 7, 8};
            static const std::set<uint8_t> arm_num_3 = {9, 10, 12};

            ///判断装甲板ID是否是大装甲板
            static const std::set<uint8_t> arm_is_lg{arm_lg_ids,
                                                     arm_lg_ids + sizeof(arm_lg_ids)};

            ///特殊装甲板ID
            static const uint8_t arm_unknown_sm = uint8_t(Type::INFANTRY_3), arm_unknown_lg = uint8_t(Type::HERO_1 );
            static const uint8_t arm_undefined = -1;

            ///@return 一个车上装甲板数量
            static inline uint getNum(uint8_t type) {
                if (arm_num_4.count(type))
                    return 4;
                if (arm_num_2.count(type))
                    return 2;
                if (arm_num_3.count(type))
                    return 3;
                return 1;
            }
        }// namespace armor_id
    }    // namespace rm

#undef IFR_INTERFACE_ENUM_CAST

}// namespace ifr_interface::defs

#endif// IFR_ROS2_CV__PACKAGE_RM_COMMON__DEFS__H
