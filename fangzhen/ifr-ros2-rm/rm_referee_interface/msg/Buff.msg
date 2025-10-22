# 机器人增益数据，增益状态改变后发送
# 命令码: 0x0204
# 数据段长度: 1

uint8 recovery_buff # 机器人血量补血状态
uint8 cooling_buff # 枪口热量冷却加速
uint8 defence_buff # 机器人防御加成
uint8 vulnerability_buff # 机器人攻击加成
uint8 remaining_energy
uint16 attack_buff
