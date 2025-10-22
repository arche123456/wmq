# 实时射击数据，子弹发射后发送
# 命令码: 0x0207
# 数据段长度: 6

uint8 bullet_type # 子弹类型
uint8 shooter_id # 发射机构 ID
uint8 bullet_freq # 子弹射频 单位 Hz
float32 bullet_speed # 子弹射速 单位 m/s
