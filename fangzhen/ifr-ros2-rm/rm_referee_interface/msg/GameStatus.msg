# 比赛状态数据，1Hz 周期发送
# 命令码: 0x0001
# 数据段长度: 11

# 比赛类型
# 1：RoboMaster 机甲大师赛；
# 2：RoboMaster 机甲大师单项赛；
# 3：ICRA RoboMaster 人工智能挑战赛
# 4：RoboMaster 联盟赛 3V3
# 5：RoboMaster 联盟赛 1V1
uint8 game_type
# 当前比赛阶段
# 0：未开始比赛；
# 1：准备阶段；
# 2：自检阶段；
# 3：5s 倒计时；
# 4：对战中；
# 5：比赛结算中
uint8 game_progress
uint16 stage_remain_time   # 当前阶段剩余时间，单位 s
uint64 sync_time_stamp     # 机器人接收到该指令的精确 Unix 时间，当机载端收到有效的 NTP 服务器授时后生效
