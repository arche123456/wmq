//
// Created by yuanlu on 2022/12/9.
// NCUT IFR DreamTeam
//

#ifndef IFR_ROS2_CV__PACKAGE_RM_COMMON__REFEREE_DATAS__H
#define IFR_ROS2_CV__PACKAGE_RM_COMMON__REFEREE_DATAS__H

#include "ifr_common/crc.h"
#include "ifr_common/defs.h"
#include "rm_common/defs.h"
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <opencv2/core/cvdef.h>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rm_referee_interface/all.hpp>
#include <rm_referee_interface/msg/detail/game_status__struct.hpp>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <string>
#include <type_traits>

#ifndef RS_PKG_USE_ROS
#define RS_PKG_USE_ROS true
#endif

#define RSPKG_GET CV_NODISCARD_STD inline
#define RSPKG_BASIC(name, id, interval_, maxInterval_)                               \
    static_assert(std::is_class<struct name>::value, #name " is not a struct Name"); \
    static const constexpr char *NAME = #name;                                       \
    static const constexpr uint16_t ID = id;                                         \
    static const constexpr int64_t INTERVAL = interval_;                             \
    static const constexpr int64_t MAX_INTERVAL = maxInterval_;                      \
    //包ID ; 目标间隔时间(ms) ; 最大间隔时间(ms) ; 间隔时间: -1 = 不定,  -2 = 仅发送一次

#if RS_PKG_USE_ROS
#define RSPKG_ROS(name, from_ros, to_ros)                             \
    static_assert(RefereeSystem::str_equal(#name, NAME), "bad name"); \
    using ROS_TYPE = rm_referee_interface::msg::name;                 \
    ROS_TYPE to() const {                                             \
        ROS_TYPE msg;                                                 \
        to_ros;                                                       \
        return msg;                                                   \
    };                                                                \
    ROS_TYPE &to(ROS_TYPE &msg) const {                               \
        to_ros;                                                       \
        return msg;                                                   \
    };                                                                \
    ROS_TYPE &to(ROS_TYPE::SharedPtr msg_ptr) const {                 \
        auto &msg = *msg_ptr;                                         \
        to_ros;                                                       \
        return msg;                                                   \
    };                                                                \
    void from(const ROS_TYPE::SharedPtr &msg) {                       \
        from_ros;                                                     \
    };                                                                \
    void from(const ROS_TYPE::ConstSharedPtr &msg) {                  \
        from_ros;                                                     \
    };                                                                \
    void from(const ROS_TYPE *msg) {                                  \
        from_ros;                                                     \
    };
#else
#define RSPKG_ROS(name, from_ros, to_ros)
#endif

namespace RefereeSystem {

    static constexpr bool str_equal(const char *a, const char *b) {
        while (*a && *b) {
            if (*a != *b) return false;
            ++a;
            ++b;
        }
        return *a == *b;
    }
    namespace packet {
#pragma pack(1)

        /**
         * 每个数据包的帧头
         */
        struct FrameHeader {
            static const constexpr uint8_t HEAD = 0xA5;
            uint8_t SOF;         //数据帧起始字节，固定值为 0xA5
            uint16_t data_length;//数据帧中 data 的长度
            uint8_t seq;         //包序号
            uint8_t CRC8;        //帧头 CRC8 校验

            RSPKG_GET bool check() const { return crc::CRC8::check_crcT(this); }
        };
        template<class Body>
        struct FrameBody {
            Body body;
            uint16_t CRC16;
            RSPKG_GET bool check(uint16_t old = crc::CRC16::init) const { return crc::CRC16::check_crcT(this, old); }
        };


        /**
         * @brief 比赛状态数据，1Hz 周期发送
         * @details 命令码: 0x0001
         * @details 数据段长度: 11
         */
        struct GameStatus {
            RSPKG_BASIC(GameStatus, 0x0001, 1000, 2000)

            /// @brief 比赛类型
            /// @details 1：RoboMaster 机甲大师赛；
            /// @details 2：RoboMaster 机甲大师单项赛；
            /// @details 3：ICRA RoboMaster 人工智能挑战赛
            /// @details 4：RoboMaster 联盟赛 3V3
            /// @details 5：RoboMaster 联盟赛 1V1
            uint8_t game_type : 4;
            /// @brief 当前比赛阶段
            /// @details 0：未开始比赛；
            /// @details 1：准备阶段；
            /// @details 2：自检阶段；
            /// @details 3：5s 倒计时；
            /// @details 4：对战中；
            /// @details 5：比赛结算中
            uint8_t game_progress : 4;
            uint16_t stage_remain_time{};// 当前阶段剩余时间，单位 s
            uint64_t SyncTimeStamp{};    // 机器人接收到该指令的精确 Unix 时间，当机载端收到有效的 NTP 服务器授时后生效


            RSPKG_ROS(
                    GameStatus,
                    {
                        game_type = msg->game_type;
                        game_progress = msg->game_progress;
                        stage_remain_time = msg->stage_remain_time;
                        SyncTimeStamp = msg->sync_time_stamp;
                    },
                    {
                        msg.game_type = game_type;
                        msg.game_progress = game_progress;
                        msg.stage_remain_time = stage_remain_time;
                        msg.sync_time_stamp = SyncTimeStamp;
                    })
        };

        /**
        * @brief 比赛结果数据，比赛结束后发送
        * @details 命令码: 0x0002
        * @details 数据段长度: 1
        */
        struct GameResult {
            RSPKG_BASIC(GameResult, 0x0002, -2, 0)
            uint8_t winner{};//0 平局 1 红方胜利 2 蓝方胜利


            RSPKG_ROS(GameResult, winner = msg->winner, msg.winner = winner)
        };

        /**
        * @brief 比赛机器人血量数据，3Hz 周期发送
        * @details 命令码: 0x0003
        * @details 数据段长度: 32
        */
        struct GameRobotHP {
            RSPKG_BASIC(GameRobotHP, 0x0003, 1000 / 3, 2000 / 3)
            uint16_t red_1_robot_HP{}; //红 1 英雄机器人血量，未上场以及罚下血量为 0
            uint16_t red_2_robot_HP{}; //红 2 工程机器人血量
            uint16_t red_3_robot_HP{}; //红 3 步兵机器人血量
            uint16_t red_4_robot_HP{}; //红 4 步兵机器人血量
            uint16_t red_5_robot_HP{}; //红 5 步兵机器人血量
            uint16_t red_7_robot_HP{}; //红 7 哨兵机器人血量
            uint16_t red_outpost_HP{}; //红方前哨战血量
            uint16_t red_base_HP{};    //红方基地血量
            uint16_t blue_1_robot_HP{};//蓝 1 英雄机器人血量，未上场以及罚下血量为 0
            uint16_t blue_2_robot_HP{};//蓝 2 工程机器人血量
            uint16_t blue_3_robot_HP{};//蓝 3 步兵机器人血量
            uint16_t blue_4_robot_HP{};//蓝 4 步兵机器人血量
            uint16_t blue_5_robot_HP{};//蓝 5 步兵机器人血量
            uint16_t blue_7_robot_HP{};//蓝 7 哨兵机器人血量
            uint16_t blue_outpost_HP{};//蓝方前哨战血量
            uint16_t blue_base_HP{};   //蓝方基地血量


            RSPKG_ROS(
                    GameRobotHP,
                    {
                        red_1_robot_HP = msg->red_1_robot_hp;
                        red_2_robot_HP = msg->red_2_robot_hp;
                        red_3_robot_HP = msg->red_3_robot_hp;
                        red_4_robot_HP = msg->red_4_robot_hp;
                        red_5_robot_HP = msg->red_5_robot_hp;
                        red_7_robot_HP = msg->red_7_robot_hp;
                        red_outpost_HP = msg->red_outpost_hp;
                        red_base_HP = msg->red_base_hp;
                        blue_1_robot_HP = msg->blue_1_robot_hp;
                        blue_2_robot_HP = msg->blue_2_robot_hp;
                        blue_3_robot_HP = msg->blue_3_robot_hp;
                        blue_4_robot_HP = msg->blue_4_robot_hp;
                        blue_5_robot_HP = msg->blue_5_robot_hp;
                        blue_7_robot_HP = msg->blue_7_robot_hp;
                        blue_outpost_HP = msg->blue_outpost_hp;
                        blue_base_HP = msg->blue_base_hp;
                    },
                    {
                        msg.red_1_robot_hp = red_1_robot_HP;
                        msg.red_2_robot_hp = red_2_robot_HP;
                        msg.red_3_robot_hp = red_3_robot_HP;
                        msg.red_4_robot_hp = red_4_robot_HP;
                        msg.red_5_robot_hp = red_5_robot_HP;
                        msg.red_7_robot_hp = red_7_robot_HP;
                        msg.red_outpost_hp = red_outpost_HP;
                        msg.red_base_hp = red_base_HP;
                        msg.blue_1_robot_hp = blue_1_robot_HP;
                        msg.blue_2_robot_hp = blue_2_robot_HP;
                        msg.blue_3_robot_hp = blue_3_robot_HP;
                        msg.blue_4_robot_hp = blue_4_robot_HP;
                        msg.blue_5_robot_hp = blue_5_robot_HP;
                        msg.blue_7_robot_hp = blue_7_robot_HP;
                        msg.blue_outpost_hp = blue_outpost_HP;
                        msg.blue_base_hp = blue_base_HP;
                    })

            using Team = ifr_interface::defs::Team;
            using RobotType = ifr_interface::defs::rm::RobotType;
            /// @brief 通过机器人ID获取对应的血量
            /// @return -1: 无血量
            RSPKG_GET uint16_t getByRobot(const Team &team, const RobotType &type) const {
                switch (team) {
                    case Team::BLUE:
                        switch (type) {
                            case RobotType::HERO_1:
                                return blue_1_robot_HP;
                            case RobotType::ENGINEER_2:
                                return blue_2_robot_HP;
                            case RobotType::INFANTRY_3:
                                return blue_3_robot_HP;
                            case RobotType::INFANTRY_4:
                                return blue_4_robot_HP;
                            case RobotType::INFANTRY_5:
                                return blue_5_robot_HP;
                            case RobotType::SENTRY_7:
                                return blue_7_robot_HP;
                            case RobotType::OUTPOST:
                                return blue_outpost_HP;
                            case RobotType::BASE:
                                return blue_base_HP;
                            default:
                                break;
                        }
                        break;
                    case Team::RED:
                        switch (type) {
                            case RobotType::HERO_1:
                                return red_1_robot_HP;
                            case RobotType::ENGINEER_2:
                                return red_2_robot_HP;
                            case RobotType::INFANTRY_3:
                                return red_3_robot_HP;
                            case RobotType::INFANTRY_4:
                                return red_4_robot_HP;
                            case RobotType::INFANTRY_5:
                                return red_5_robot_HP;
                            case RobotType::SENTRY_7:
                                return red_7_robot_HP;
                            case RobotType::OUTPOST:
                                return red_outpost_HP;
                            case RobotType::BASE:
                                return red_base_HP;
                            default:
                                break;
                        }
                        break;
                    default:
                        break;
                }
                return -1;
            };

            /**
                @brief 通过机器人ID获取对应的血量
                @details 1：红方英雄机器人；
                @details 2：红方工程机器人；
                @details 3/4/5：红方步兵机器人；
                @details 6：红方空中机器人；
                @details 7：红方哨兵机器人；
                @details 8：红方飞镖机器人；
                @details 9：红方雷达站；
                @details 101：蓝方英雄机器人；
                @details 102：蓝方工程机器人；
                @details 103/104/105：蓝方步兵机器人；
                @details 106：蓝方空中机器人；
                @details 107：蓝方哨兵机器人；
                @details 108：蓝方飞镖机器人；
                @details 109：蓝方雷达站。
                @return -1: 无血量
             */
            RSPKG_GET uint16_t getByRobotId(const uint8_t &id) const {
                switch (id) {
                    case 1:
                        return red_1_robot_HP;
                    case 2:
                        return red_2_robot_HP;
                    case 3:
                        return red_3_robot_HP;
                    case 4:
                        return red_4_robot_HP;
                    case 5:
                        return red_5_robot_HP;
                    case 7:
                        return red_7_robot_HP;
                    case 101:
                        return blue_1_robot_HP;
                    case 102:
                        return blue_2_robot_HP;
                    case 103:
                        return blue_3_robot_HP;
                    case 104:
                        return blue_4_robot_HP;
                    case 105:
                        return blue_5_robot_HP;
                    case 107:
                        return blue_7_robot_HP;
                    case 6:
                    case 8:
                    case 9:
                    case 106:
                    case 108:
                    case 109:
                    default:
                        return -1;
                }
            }
        };

        /**
         * @brief 人工智能挑战赛加成与惩罚状态，1Hz 周期发送
         * @details 命令码: 0x0005
         * @details 数据段长度: 13
         */
        struct ICRA_BDZL_status {
            RSPKG_BASIC(ICRA_BDZL_status, 0x0005, 1000, 2000)
            uint8_t f1_activate : 1;     // F1激活状态
            uint8_t f1_status : 3;       // F1状态信息
            uint8_t f2_activate : 1;     // F2激活状态
            uint8_t f2_status : 3;       // F2状态信息
            uint8_t f3_activate : 1;     // F3激活状态
            uint8_t f3_status : 3;       // F3状态信息
            uint8_t f4_activate : 1;     // F4激活状态
            uint8_t f4_status : 3;       // F4状态信息
            uint8_t f5_activate : 1;     // F5激活状态
            uint8_t f5_status : 3;       // F5状态信息
            uint8_t f6_activate : 1;     // F6激活状态
            uint8_t f6_status : 3;       // F6状态信息
            uint16_t red1_bullet_left{}; //红方 1 号剩余弹量
            uint16_t red2_bullet_left{}; //红方 2 号剩余弹量
            uint16_t blue1_bullet_left{};//蓝方 1 号剩余弹量
            uint16_t blue2_bullet_left{};//蓝方 2 号剩余弹量
            uint8_t lurk_mode{};         //阶段
            uint8_t res{};               //保留字节
        };

        /**
         * @brief 场地事件数据，1Hz 周期发送
         * @details 命令码: 0x0101
         * @details 数据段长度: 4
         */
        struct EventData {
            RSPKG_BASIC(EventData, 0x0101, 1000, 2000)

            bool isOccupy1 : 1;// 己方补给站 1 号补血点占领状态, 1 为已占领；
            bool isOccupy2 : 1;// 己方补给站 2 号补血点占领状态, 1 为已占领；
            bool isOccupy3 : 1;// 己方补给站 3 号补血点占领状态, 1 为已占领；

            bool isOccupyEM : 1;  // 打击点占领状态, 1 为占领；
            bool isActiveEMsm : 1;// 小能量机关激活状态，1 为已激活；
            bool isActiveEMlg : 1;// 大能量机关激活状态，1 为已激活；

            bool isR2B2Occupy : 1;// 己方侧 R2/B2 环形高地占领状态 1 为已占领；
            bool isR3B3Occupy : 1;// 己方侧 R3/B3 梯形高地占领状态 1 为已占领；
            bool isR4B4Occupy : 1;// 己方侧 R4/B4 梯形高地占领状态 1 为已占领；

            bool hasVirtualShield : 1;       // 己方基地护盾状态：1 为基地有虚拟护盾血量；0 为基地无虚拟护盾血量；
            bool isOutpostBattleSurvival : 1;// 己方前哨战状态：1 为前哨战存活；0 为前哨战被击毁；


            uint32_t reserve : 21;// bit 11-31: 保留

            RSPKG_ROS(
                    EventData,
                    {
                        isOccupy1 = msg->is_occupy_1;
                        isOccupy2 = msg->is_occupy_2;
                        isOccupy3 = msg->is_occupy_3;
                        isOccupyEM = msg->is_occupy_em;
                        isActiveEMsm = msg->is_active_em_sm;
                        isActiveEMlg = msg->is_active_em_lg;
                        isR2B2Occupy = msg->is_r2b2_occupy;
                        isR3B3Occupy = msg->is_r3b3_occupy;
                        isR4B4Occupy = msg->is_r4b4_occupy;
                        hasVirtualShield = msg->has_virtual_shield;
                        isOutpostBattleSurvival = msg->is_outpost_battle_survival;
                    },
                    {
                        msg.is_occupy_1 = isOccupy1;
                        msg.is_occupy_2 = isOccupy2;
                        msg.is_occupy_3 = isOccupy3;
                        msg.is_occupy_em = isOccupyEM;
                        msg.is_active_em_sm = isActiveEMsm;
                        msg.is_active_em_lg = isActiveEMlg;
                        msg.is_r2b2_occupy = isR2B2Occupy;
                        msg.is_r3b3_occupy = isR3B3Occupy;
                        msg.is_r4b4_occupy = isR4B4Occupy;
                        msg.has_virtual_shield = hasVirtualShield;
                        msg.is_outpost_battle_survival = isOutpostBattleSurvival;
                    })
        };

        /**
         * @brief 场地补给站动作标识数据，动作改变后发送
         * @details 命令码: 0x0102
         * @details 数据段长度: 4
         */
        struct SupplyProjectileAction {
            RSPKG_BASIC(SupplyProjectileAction, 0x0102, -1, 0)
            uint8_t projectile_id{};//补给站口 ID; 1：1 号补给口；2：2 号补给口
            uint8_t robot_id{};     //补弹机器人 ID：0 为当前无机器人补弹，1 为红方英雄机器人补弹，2 为红方工程机 器人补弹，3/4/5 为红方步兵机器人补弹，101 为蓝方英雄机器人补弹，102 为蓝方工程机器人补弹，103/104/105 为蓝方步兵机器人补弹
            uint8_t step{};         //出弹口开闭状态：0 为关闭，1 为子弹准备中，2 为子弹下落
            uint8_t num{};          //补弹数量 50 / 100 /150 /20
        };

        /**
         * @brief 裁判警告数据，警告发生后发送
         * @details 命令码: 0x0104
         * @details 数据段长度: 2
         */
        struct RefereeWarning {
            RSPKG_BASIC(RefereeWarning, 0x0104, -1, 0)
            uint8_t level{};        // 1 = 黄牌 ; 2 = 红牌 ; 3 = 判负
            uint8_t foul_robot_id{};// 犯规机器人 ID ; 判负时为 0
        };

        /**
         * @brief 飞镖发射口倒计时，1Hz 周期发送
         * @details 命令码: 0x0105
         * @details 数据段长度: 1
         */
        struct DartRemaining {
            RSPKG_BASIC(DartRemaining, 0x0105, 1000, 2000)
            uint8_t dart_remaining_time{};//15s倒计时
        };

        /**
         * @brief 机器人状态数据，10Hz 周期发送
         * @details 命令码: 0x0201
         * @details 数据段长度: 13
         * @version 1.6.1 20240122
         */
        struct GameRobotStatus {
            RSPKG_BASIC(GameRobotStatus, 0x0201, 1000 / 10, 1000 / 10 * 2)
            /// 本机器人 ID：
            /// @details 1：红方英雄机器人；
            /// @details 2：红方工程机器人；
            /// @details 3/4/5：红方步兵机器人；
            /// @details 6：红方空中机器人；
            /// @details 7：红方哨兵机器人；
            /// @details 8：红方飞镖机器人；
            /// @details 9：红方雷达站；
            /// @details 101：蓝方英雄机器人；
            /// @details 102：蓝方工程机器人；
            /// @details 103/104/105：蓝方步兵机器人；
            /// @details 106：蓝方空中机器人；
            /// @details 107：蓝方哨兵机器人；
            /// @details 108：蓝方飞镖机器人；
            /// @details 109：蓝方雷达站。

            uint8_t robot_id;
            uint8_t robot_level;                   //机器人等级： 1：一级；2：二级；3：三级。
            uint16_t remain_HP;                    //机器人剩余血量
            uint16_t max_HP;                       //机器人上限血量
            uint16_t shooter_barrel_cooling_value; //机器人枪口热量每秒冷却值
            uint16_t shooter_barrel_heat_limit;    //机器人枪口热量上限
            uint16_t chassis_power_limit;          //机器人底盘功率上限
            uint8_t mains_power_gimbal_output : 1; //gimbal 口输出：1 为有 24V 输出，0 为无 24v 输出；
            uint8_t mains_power_chassis_output : 1;//chassis 口输出：1 为有 24V 输出，0 为无 24v 输出；
            uint8_t mains_power_shooter_output : 1;//shooter 口输出：1 为有 24V 输出，0 为无 24v 输出；


            RSPKG_ROS(
                    GameRobotStatus,
                    {
                        robot_id = msg->robot_id;
                        robot_level = msg->robot_level;
                        remain_HP = msg->remain_hp;
                        max_HP = msg->max_hp;
                        shooter_barrel_cooling_value = msg->shooter_barrel_cooling_value;
                        shooter_barrel_heat_limit = msg->shooter_barrel_heat_limit;
                        chassis_power_limit = msg->chassis_power_limit;
                        chassis_power_limit = msg->chassis_power_limit;
                        mains_power_gimbal_output = msg->mains_power_gimbal_output;
                        mains_power_chassis_output = msg->mains_power_chassis_output;
                        mains_power_shooter_output = msg->mains_power_shooter_output;
                    },
                    {
                        msg.robot_id = robot_id;
                        msg.robot_level = robot_level;
                        msg.remain_hp = remain_HP;
                        msg.max_hp = max_HP;
                        msg.shooter_barrel_cooling_value = shooter_barrel_cooling_value;
                        msg.shooter_barrel_heat_limit = shooter_barrel_heat_limit;
                        msg.chassis_power_limit = chassis_power_limit;
                        msg.chassis_power_limit = chassis_power_limit;
                        msg.mains_power_gimbal_output = mains_power_gimbal_output;
                        msg.mains_power_chassis_output = mains_power_chassis_output;
                        msg.mains_power_shooter_output = mains_power_shooter_output;
                    })
        };

        /**
         * @brief 实时功率热量数据，50Hz 周期发送
         * @details 命令码: 0x0202
         * @details 数据段长度: 16
         */
        struct PowerHeat {
            RSPKG_BASIC(PowerHeat, 0x0202, 1000 / 50, 1000 / 50 * 2)
            uint16_t chassis_volt{};                 //底盘输出电压 单位 毫伏
            uint16_t chassis_current{};              //底盘输出电流 单位 毫安
            float chassis_power{};                   //底盘输出功率 单位 W 瓦
            uint16_t chassis_power_buffer{};         //底盘功率缓冲 单位 J 焦耳 备注：飞坡根据规则增加至 250J
            uint16_t shooter_id1_17mm_cooling_heat{};//1 号 17mm 枪口热量
            uint16_t shooter_id2_17mm_cooling_heat{};//2 号 17mm 枪口热量
            uint16_t shooter_id1_42mm_cooling_heat{};//42mm 枪口热量
        };

        /**
         * @brief 机器人位置数据，10Hz 发送
         * @details 命令码: 0x0203
         * @details 数据段长度: 16
         */
        struct RobotPos {
            RSPKG_BASIC(RobotPos, 0x0203, 1000 / 10, 1000 / 10 * 2)
            float x;  //位置 x 坐标，单位 m
            float y;  //位置 y 坐标，单位 m
            float z;  //位置 z 坐标，单位 m
            float yaw;//位置枪口，单位度
        };

        /**
         * @brief 机器人增益数据，增益状态改变后发送
         * @details 命令码: 0x0204
         * @details 数据段长度: 1
         * @version 1.6.1 20240122
         */
        struct Buff {
            RSPKG_BASIC(Buff, 0x0204, -1, 0)
            uint8_t recovery_buff;     //机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%）
            uint8_t cooling_buff;      //机器人枪口冷却倍率（直接值，值为 5 表示 5 倍冷却）
            uint8_t defence_buff;      //机器人防御增益（百分比，值为 50 表示 50%防御增益）
            uint8_t vulnerability_buff;//机器人负防御增益（百分比，值为 30 表示-30%防御增益）
            uint8_t remaining_energy;
            uint16_t attack_buff;//机器人攻击增益（百分比，值为 50 表示 50%攻击增益）

            RSPKG_ROS(
                    Buff,
                    {
                        recovery_buff = msg->recovery_buff;
                        cooling_buff = msg->cooling_buff;
                        defence_buff = msg->defence_buff;
                        vulnerability_buff = msg->vulnerability_buff;
                        attack_buff = msg->attack_buff;
                        remaining_energy = msg->remaining_energy;
                    },
                    {
                        msg.recovery_buff = recovery_buff;
                        msg.cooling_buff = cooling_buff;
                        msg.defence_buff = defence_buff;
                        msg.vulnerability_buff = vulnerability_buff;
                        msg.attack_buff = attack_buff;
                        msg.remaining_energy = remaining_energy;
                    })
        };


        /**
         * @brief 空中机器人能量状态数据，10Hz 周期发送，只有空中机器人主控发送
         * @details 命令码: 0x0205
         * @details 数据段长度: 3
         */
        struct AerialRobotEnergy {
            RSPKG_BASIC(AerialRobotEnergy, 0x0205, 1000 / 10, 1000 / 10 * 2)
            uint8_t attack_time{};//可攻击时间 单位 s。30s 递减至 0
        };

        /**
         * @brief 伤害状态数据，伤害发生后发送
         * @details 命令码: 0x0206
         * @details 数据段长度: 1
         */
        struct RobotHurt {
            RSPKG_BASIC(RobotHurt, 0x0206, -1, 0)
            /// 当血量变化类型为装甲伤害，代表装甲 ID，其中数值为 0-4 号代表机器人的五个装甲片，其他血量变化类型，该变量数值为 0。
            uint8_t armor_id : 4;
            /// @brief 获取血量变化类型
            /// @details 0x0 装甲伤害扣血；
            /// @details 0x1 模块掉线扣血；
            /// @details 0x2 超射速扣血；
            /// @details 0x3 超枪口热量扣血；
            /// @details 0x4 超底盘功率扣血；
            /// @details 0x5 装甲撞击扣血
            uint8_t hurt_type : 4;


            RSPKG_ROS(
                    RobotHurt,
                    {
                        armor_id = msg->armor_id;
                        hurt_type = msg->hurt_type;
                    },
                    {
                        msg.armor_id = armor_id;
                        msg.hurt_type = hurt_type;
                    })
        };

        struct SentryInfo {
            RSPKG_BASIC(SentryInfo, 0x020D, 1000, 2000)
            uint32_t sentry_info;
            uint16_t sentry_info_2;
            RSPKG_ROS(
                    SentryInfo,
                    {
                        sentry_info = msg->sentry_info;
                        sentry_info_2 = msg->sentry_info_2;
                    },
                    {
                        msg.sentry_info = sentry_info;
                        msg.sentry_info_2 = sentry_info_2;
                    })
        };


        /**
         * @brief 实时射击数据，子弹发射后发送
         * @details 命令码: 0x0207
         * @details 数据段长度: 6
         */
        struct ShootData {
            RSPKG_BASIC(ShootData, 0x0207, -1, 0)
            uint8_t bullet_type{};//子弹类型: 1：17mm 弹丸 2：42mm 弹丸
            uint8_t shooter_id{}; //发射机构 ID： 1：1 号 17mm 发射机构 2：2 号 17mm 发射机构 3：42mm 发射机构
            uint8_t bullet_freq{};//子弹射频 单位 Hz
            float bullet_speed{}; //子弹射速 单位 m/s


            RSPKG_ROS(
                    ShootData,
                    {
                        bullet_type = msg->bullet_type;
                        shooter_id = msg->shooter_id;
                        bullet_freq = msg->bullet_freq;
                        bullet_speed = msg->bullet_speed;
                    },
                    {
                        msg.bullet_type = bullet_type;
                        msg.shooter_id = shooter_id;
                        msg.bullet_freq = bullet_freq;
                        msg.bullet_speed = bullet_speed;
                    })
        };
        /**
         * @brief 允许发弹量，固定以 10Hz 频率发送
         * @details 服务器→己方英雄、步兵、哨兵、空中机器人
         * @details 命令码: 0x0208
         * @details 数据段长度: 6
         * @version 1.6.1 20240122
         */
        struct BulletRemaining {
            RSPKG_BASIC(BulletRemaining, 0x0208, 1000 / 10, 2000 / 10)
            uint16_t bullet_remaining_num_17mm{};//17mm 子弹剩余发射数目
            uint16_t bullet_remaining_num_42mm{};//42mm 子弹剩余发射数目
            uint16_t coin_remaining_num{};       //剩余金币数量

            RSPKG_ROS(
                    BulletRemaining,
                    {
                        bullet_remaining_num_17mm = msg->bullet_remaining_num_17mm;
                        bullet_remaining_num_42mm = msg->bullet_remaining_num_42mm;
                        coin_remaining_num = msg->coin_remaining_num;
                    },
                    {
                        msg.bullet_remaining_num_17mm = bullet_remaining_num_17mm;
                        msg.bullet_remaining_num_42mm = bullet_remaining_num_42mm;
                        msg.coin_remaining_num = coin_remaining_num;
                    })
        };

        /**
         * @brief 机器人 RFID 状态，3Hz 周期发送
         * @details 服务器→己方装有 RFID 模块的机器人
         * @details 命令码: 0x0209
         * @details 数据段长度: 4
         * @version 1.6.1 20240122
         */
        struct RfidStatus {
            RSPKG_BASIC(RfidStatus, 0x0209, 1000 / 3, 2000)
            uint32_t rfid_status{};

            /// bit 0：己方基地增益点
            /// bit 1：己方环形高地增益点
            /// bit 2：对方环形高地增益点
            /// bit 3：己方 R3/B3 梯形高地增益点
            /// bit 4：对方 R3/B3 梯形高地增益点
            /// bit 5：己方 R4/B4 梯形高地增益点
            /// bit 6：对方 R4/B4 梯形高地增益点
            /// bit 7：己方能量机关激活点
            /// bit 8：己方飞坡增益点（靠近己方一侧飞坡前）
            /// bit 9：己方飞坡增益点（靠近己方一侧飞坡后）
            /// bit 10：对方飞坡增益点（靠近对方一侧飞坡前）
            /// bit 11：对方飞坡增益点（靠近对方一侧飞坡后）
            /// bit 12：己方前哨站增益点
            /// bit 13：己方补血点（检测到任一均视为激活）
            /// bit 14：己方哨兵巡逻区
            /// bit 15：对方哨兵巡逻区
            /// bit 16：己方大资源岛增益点
            /// bit 17：对方大资源岛增益点
            /// bit 18：己方兑换区
            /// bit 19：中心增益点（仅 RMUL 适用）
            template<uint8_t id>
            RSPKG_GET bool get() const {
                static_assert(0 <= id && id <= 19, "ID must be between 0 and 19 (inclusive)");
                return rfid_status & (1 << id);
            }
        };

        /**
         * @brief 飞镖机器人客户端指令书，10Hz 周期发送
         * @details 命令码: 0x020A
         * @details 数据段长度: 12
         */
        struct DartClientCmd {
            RSPKG_BASIC(DartClientCmd, 0x020A, -1, 0)
            uint8_t dart_launch_opening_status{};//当前飞镖发射口的状态 1：关闭； 2：正在开启或者关闭中 0：已经开启
            uint8_t dart_attack_target{};        //飞镖的打击目标，默认为前哨站； 0：前哨站； 1：基地。
            uint16_t target_change_time{};       //切换打击目标时的比赛剩余时间，单位秒，从未切换默认为 0。
            uint16_t operate_launch_cmd_time{};  //最近一次操作手确定发射指令时的比赛剩余时间，单位秒, 初始值为 0。
        };
        /**
         * @brief 机器人间交互数据，发送方触发发送，上限 10Hz
         * @details 命令码: 0x0301
         * @details 数据段长度: n
         */
        struct StuInteractiveDataHead {
            RSPKG_BASIC(StuInteractiveDataHead, 0x0301, 1000 / 10, 1000 / 10 * 2)
            uint16_t data_cmd_id{};//0x0200~0x02FF 机器人间通信 ; 0x0100~0x0110 客户端通信

            /// @机器人ID 1，英雄(红)；
            /// @机器人ID 2，工程(红)；
            /// @机器人ID 3/4/5，步兵(红)；
            /// @机器人ID 6，空中(红)；
            /// @机器人ID 7，哨兵(红)；
            /// @机器人ID 9，雷达站（红）；
            /// @机器人ID 101，英雄(蓝)；
            /// @机器人ID 102，工程(蓝)；
            /// @机器人ID 103/104/105，步兵(蓝)；
            /// @机器人ID 106，空中(蓝)；
            /// @机器人ID 107，哨兵(蓝)；
            /// @机器人ID 109，雷达站（蓝）。
            ///
            /// @客户端ID 0x0101 为英雄操作手客户端(红)；
            /// @客户端ID 0x0102，工程操作手客户端((红)；
            /// @客户端ID 0x0103/0x0104/0x0105，步兵操作手客户端(红)；
            /// @客户端ID 0x0106，空中操作手客户端((红)；
            /// @客户端ID 0x0165，英雄操作手客户端(蓝)；
            /// @客户端ID 0x0166，工程操作手客户端(蓝)；
            /// @客户端ID 0x0167/0x0168/0x0169，步兵操作手客户端步兵(蓝)；
            /// @客户端ID 0x016A，空中操作手客户端(蓝)。
            uint16_t sender_ID{};
            uint16_t receiver_ID{};
        };
        namespace StuInteractiveDataBody {
#define RefereeSystem_SDB_BASIC(id) static const constexpr uint16_t ID = id

            //自定义数据包: (操作手或其他机器人指定的)哨兵状态
            struct Custom_SentryStatus {
                RefereeSystem_SDB_BASIC(0x0200);
                uint8_t type = 0;

                /**
                 * @brief
                 * @details 0: 自由模式
                 * @details 1: P (保护基地)
                 * @details 2: S (自保)
                 * @details 3: N (默认)
                 * @details 4: A (攻击)
                 */
                RSPKG_GET uint8_t getStatus() const { return type & 0x7; }

                //                /**
                //                 * 获取维持此状态的持续时间
                //                 * @details 最大31s
                //                 * @return 0 = 永久, 其他: 秒
                //                 */
                //                inline uint8_t getTime() const { return (type >> 3) & 0x1f; }
            };

            ///图形数据
            struct GraphicData {
                uint8_t graphic_name[3] = {};//
                uint32_t data1{};            //图形配置 1
                uint32_t data2{};            //图形配置 2
                uint32_t data3{};            //图形配置 3

                /// @图形操作 0：空操作
                /// @图形操作 1：增加
                /// @图形操作 2：修改
                /// @图形操作 3：删除
                RSPKG_GET uint8_t getOperation() const { return data1 & 0x7; }


                /// @图形类型 0：直线
                /// @图形类型 1：矩形
                /// @图形类型 2：整圆
                /// @图形类型 3：椭圆
                /// @图形类型 4：圆弧
                /// @图形类型 5：浮点数
                /// @图形类型 6：整型数
                /// @图形类型 7：字符
                RSPKG_GET uint8_t getType() const { return (data1 >> 3) & 0x7; }

                ///图层数，0~9
                RSPKG_GET uint8_t getLayer() const { return (data1 >> 6) & 0xf; }

                /// @颜色 0：红蓝主色
                /// @颜色 1：黄色
                /// @颜色 2：绿色
                /// @颜色 3：橙色
                /// @颜色 4：紫红色
                /// @颜色 5：粉色
                /// @颜色 6：青色
                /// @颜色 7：黑色
                /// @颜色 8：白色
                RSPKG_GET uint8_t getColor() const { return (data1 >> 10) & 0xf; }

                ///起始角度，单位：°，范围[0,360]；
                RSPKG_GET uint8_t getStartAngle() const { return (data1 >> 14) & 0x1ff; }

                ///终止角度，单位：°，范围[0,360]。
                RSPKG_GET uint8_t getEndAngle() const { return (data1 >> 23) & 0x1ff; }

                ///线宽
                RSPKG_GET uint8_t getLineWidth() const { return data2 & 0x3ff; }

                ///起点 x 坐标
                RSPKG_GET uint8_t getStartX() const { return (data2 >> 10) & 0x7ff; }

                ///起点 y 坐标
                RSPKG_GET uint8_t getStartY() const { return (data2 >> 21) & 0x7ff; }

                ///字体大小或者半径
                RSPKG_GET uint8_t getFontSize() const { return data3 & 0x3ff; }

                ///终点 x 坐标
                RSPKG_GET uint8_t getEndX() const { return (data3 >> 10) & 0x7ff; }

                ///终点 y 坐标
                RSPKG_GET uint8_t getEndY() const { return (data3 >> 21) & 0x7ff; }
            };

            ///客户端删除图形
            struct Delete {
                RefereeSystem_SDB_BASIC(0x0100);

                /// 图形操作
                /// @details 0: 空操作；
                /// @details 1: 删除图层；
                /// @details 2: 删除所有；
                uint8_t operate_tpye{};
                uint8_t layer{};//图层数：0~9
            };
            template<size_t n>
            struct DrawN {
                GraphicData graphic_data[n];
            };
            ///客户端绘制一个图形
            struct Draw1 {
                RefereeSystem_SDB_BASIC(0x0101);
                DrawN<1> data;
            };
            struct Draw2 {
                RefereeSystem_SDB_BASIC(0x0102);
                DrawN<2> data;
            };
            struct Draw5 {
                RefereeSystem_SDB_BASIC(0x0103);
                DrawN<5> data;
            };
            struct Draw7 {
                RefereeSystem_SDB_BASIC(0x0104);
                DrawN<7> data;
            };
            struct DrawChar {
                RefereeSystem_SDB_BASIC(0x0110);
                GraphicData graphic_data;
                uint8_t data[30];
            };
#undef RefereeSystem_SDB_BASIC
        }// namespace StuInteractiveDataBody

        /**
         * @brief 机器人间交互数据，发送方触发发送
         * @details 头部数据和实际数据的整合
         */
        struct StuInteractiveData {
            StuInteractiveDataHead head;
            std::shared_ptr<void> body{};
        };
        /**
         * @brief 自定义控制器交互数据接口，通过客户端触发发送，上限 30Hz
         * @details 命令码: 0x0302
         * @details 数据段长度: n
         */
        template<size_t x>
        struct RobotInteractiveData {
            RSPKG_BASIC(RobotInteractiveData<x>, 0x0302, 1000 / 30, 1000 / 30 * 2)
            static_assert(0 <= x && x <= 30, "Bad size");
            uint8_t data[x];
        };
        /**
         * @brief 客户端小地图交互数据，触发发送
         * @details 命令码: 0x0303
         * @details 数据段长度: 15
         */
        struct RobotCommandSend {
            RSPKG_BASIC(RobotCommandSend, 0x0303, 1000 / 10, 1000 / 10 * 2)
            float target_position_x{};//目标 x 位置坐标，单位 m 当发送目标机器人 ID 时，该项为 0
            float target_position_y{};//目标 y 位置坐标，单位 m 当发送目标机器人 ID 时，该项为 0
            float target_position_z{};//目标 z 位置坐标，单位 m 当发送目标机器人 ID 时，该项为 0
            uint8_t commd_keyboard{}; //发送指令时，云台手按下的键盘信息 无按键按下则为 0
            /**
                @brief 要作用的目标机器人 ID 当发送位置信息时，该项为 0
                @机器人ID 1，英雄(红)；
                @机器人ID 2，工程(红)；
                @机器人ID 3/4/5，步兵(红)；
                @机器人ID 6，空中(红)；
                @机器人ID 7，哨兵(红)；
                @机器人ID 9，雷达站（红）；
                @机器人ID 10，前哨站（红）；
                @机器人ID 11，基地（红）；
                @机器人ID 101，英雄(蓝)；
                @机器人ID 102，工程(蓝)；
                @机器人ID 103/104/105，步兵(蓝)；
                @机器人ID 106，空中(蓝)；
                @机器人ID 107，哨兵(蓝)；
                @机器人ID 109，雷达站（蓝）；
                @机器人ID 110，前哨站（蓝）；
                @机器人ID 111，基地（蓝）。
             */
            uint16_t target_robot_ID{};
        };


        /**
         * @brief 客户端小地图接收信息
         * @details 命令码: 0x0305
         * @details 数据段长度: 10
         */
        struct RobotCommandReceive {
            RSPKG_BASIC(RobotCommandReceive, 0x0304, 1000 / 30, 1000 / 30 * 2)
            int16_t mouse_x{};         //鼠标 X 轴信息
            int16_t mouse_y{};         //鼠标 Y 轴信息
            int16_t mouse_z{};         //鼠标滚轮信息
            int8_t left_button_down{}; //鼠标左键
            int8_t right_button_down{};//鼠标右键按下
            uint16_t keyboard_value{}; //键盘信息
            uint16_t reserved{};       //保留位

            RSPKG_GET bool getW() const { return keyboard_value & (1 << 0); }

            RSPKG_GET bool getS() const { return keyboard_value & (1 << 1); }

            RSPKG_GET bool getA() const { return keyboard_value & (1 << 2); }

            RSPKG_GET bool getD() const { return keyboard_value & (1 << 3); }

            RSPKG_GET bool getShift() const { return keyboard_value & (1 << 4); }

            RSPKG_GET bool getCtrl() const { return keyboard_value & (1 << 5); }

            RSPKG_GET bool getQ() const { return keyboard_value & (1 << 6); }

            RSPKG_GET bool getE() const { return keyboard_value & (1 << 7); }

            RSPKG_GET bool getR() const { return keyboard_value & (1 << 8); }

            RSPKG_GET bool getF() const { return keyboard_value & (1 << 9); }

            RSPKG_GET bool getG() const { return keyboard_value & (1 << 10); }

            RSPKG_GET bool getZ() const { return keyboard_value & (1 << 11); }

            RSPKG_GET bool getX() const { return keyboard_value & (1 << 12); }

            RSPKG_GET bool getC() const { return keyboard_value & (1 << 13); }

            RSPKG_GET bool getV() const { return keyboard_value & (1 << 14); }

            RSPKG_GET bool getB() const { return keyboard_value & (1 << 15); }
        };

        /**
         * @brief 键盘、鼠标信息，通过图传串口发送
         * @details 命令码: 0x0304
         * @details 数据段长度: 12
         */
        struct ClientMapCommand {
            RSPKG_BASIC(ClientMapCommand, 0x0305, 1000 / 10, 1000 / 10 * 2)
            uint16_t target_robot_ID{};//目标机器人 ID
            float target_position_x{}; //目标 x 位置坐标，单位 m 当 x,y 超出界限时则不显示。
            float target_position_y{}; //目标 y 位置坐标，单位 m 当 x,y 超出界限时则不显示。
        };

#pragma pack()
    }// namespace packet


    /// 裁判系统数据包额外数据
    /// @details 此模板仅在给定数据包没有对应的ROS msg时启用
    /// @details 数据在ros中使用std_msgs/String传输
    template<typename, typename = std::void_t<>>
    struct pkg_info {
        static constexpr bool has_ros_type = false;
        using ROS_TYPE = std_msgs::msg::String;
        using Subscription = rclcpp::Subscription<ROS_TYPE>;
        using Publisher = rclcpp::Publisher<ROS_TYPE>;
    };
    /// 裁判系统数据包额外数据
    /// @details 此模板仅在给定数据包含有对应的ROS msg时启用
    /// @details 数据在ros中使用自身的消息类型传输
    template<typename T>
    struct pkg_info<T, std::void_t<typename T::ROS_TYPE>> {
        static constexpr bool has_ros_type = true;
        using ROS_TYPE = typename T::ROS_TYPE;
        using Subscription = rclcpp::Subscription<ROS_TYPE>;
        using Publisher = rclcpp::Publisher<ROS_TYPE>;
    };

    // 处理具有 ROS_TYPE 的情况
    template<typename T, typename = std::enable_if_t<pkg_info<T>::has_ros_type>>
    T fromRosMsg(const typename T::ROS_TYPE::SharedPtr &msg) {
        T t;
        t.from(msg);
        return t;
    }

    // 处理没有 ROS_TYPE 的情况
    template<typename T, typename = std::enable_if_t<!pkg_info<T>::has_ros_type>>
    T &fromRosMsg(const std_msgs::msg::String::SharedPtr &msg) {
        return *reinterpret_cast<T *>(msg->data.data());
    }

    // 处理具有 ROS_TYPE 的情况
    template<typename T, typename = std::enable_if_t<pkg_info<T>::has_ros_type>>
    T fromRosMsg(const typename T::ROS_TYPE::ConstSharedPtr &msg) {
        T t;
        t.from(msg);
        return t;
    }

    // 处理没有 ROS_TYPE 的情况
    template<typename T, typename = std::enable_if_t<!pkg_info<T>::has_ros_type>>
    const T &fromRosMsg(const std_msgs::msg::String::ConstSharedPtr &msg) {
        return *reinterpret_cast<T *>(msg->data.data());
    }

    /// @brief 获取裁判系统的topic
    template<class T>
    inline std::string getTopic(std::filesystem::path prefix = "/serial/referee") {
        static constexpr const char *name = T::NAME;
        auto x = prefix / name;
        return x;
    }

    /// @brief 创建一个裁判系统数据包订阅器
    /// @param node 所属节点
    /// @param callback 回调函数void(raw,msg)
    /// @param qos QoS profile for Subcription.
    /// @param prefix 监听topic前缀, 如无必要不建议指定
    template<class RefereePkg>
    inline auto create_subscription(rclcpp::Node *node, std::function<void(typename pkg_info<RefereePkg>::ROS_TYPE::SharedPtr raw, RefereePkg &msg)> callback,
                                    rclcpp::QoS qos = rclcpp::SensorDataQoS(), std::filesystem::path prefix = "/serial/referee") {
        using info = pkg_info<RefereePkg>;
        return node->create_subscription<typename info::ROS_TYPE>(
                getTopic<RefereePkg>(prefix), qos, [callback](typename info::ROS_TYPE::SharedPtr ptr) {
                    if constexpr (info::has_ros_type) {
                        RefereePkg pkg;
                        pkg.from(ptr);
                        callback(ptr, pkg);
                    } else {
                        CV_Assert(ptr->data.size() >= sizeof(RefereePkg));
                        callback(ptr, *reinterpret_cast<RefereePkg *>(ptr->data.data()));
                    }
                });
    }
    // using T = RefereeSystem::GameStatus;
}// namespace RefereeSystem
#endif// IFR_ROS2_CV__PACKAGE_RM_COMMON__REFEREE_DATAS__H
