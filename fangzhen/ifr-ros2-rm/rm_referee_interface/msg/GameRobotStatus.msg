# 机器人状态数据，10Hz 周期发送
# 命令码: 0x0201
# 数据段长度: 13

#本机器人 ID：
# 1：红方英雄机器人；
# 2：红方工程机器人；
# 3/4/5：红方步兵机器人；
# 6：红方空中机器人；
# 7：红方哨兵机器人；
# 8：红方飞镖机器人；
# 9：红方雷达站；
# 101：蓝方英雄机器人；
# 102：蓝方工程机器人；
# 103/104/105：蓝方步兵机器人；
# 106：蓝方空中机器人；
# 107：蓝方哨兵机器人；
# 108：蓝方飞镖机器人；
# 109：蓝方雷达站。
uint8 robot_id
uint8 robot_level                    # 机器人等级： 1：一级；2：二级；3：三级。
uint16 remain_hp                     # 机器人剩余血量
uint16 max_hp                        # 机器人上限血量
uint16 shooter_barrel_cooling_value  # 机器人枪口热量每秒冷却值
uint16 shooter_barrel_heat_limit     # 机器人枪口热量上限
uint16 chassis_power_limit           # 机器人底盘功率上限
uint8 mains_power_gimbal_output      # gimbal 口输出：1 为有 24V 输出，0 为无 24v 输出；
uint8 mains_power_chassis_output     # chassis 口输出：1 为有 24V 输出，0 为无 24v 输出；
uint8 mains_power_shooter_output     # shooter 口输出：1 为有 24V 输出，0 为无 24v 输出；