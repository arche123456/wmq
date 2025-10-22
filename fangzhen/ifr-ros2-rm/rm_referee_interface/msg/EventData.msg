# 场地事件数据，1Hz 周期发送
# 命令码: 0x0101
# 数据段长度: 4

bool is_occupy_1 # 己方补给站 1 号补血点占领状态, 1 为已占领
bool is_occupy_2 # 己方补给站 2 号补血点占领状态, 1 为已占领
bool is_occupy_3 # 己方补给站 3 号补血点占领状态, 1 为已占领
bool is_occupy_em # 打击点占领状态, 1 为占领
bool is_active_em_sm # 小能量机关激活状态，1 为已激活
bool is_active_em_lg # 大能量机关激活状态，1 为已激活
bool is_r2b2_occupy # 己方侧 R2/B2 环形高地占领状态 1 为已占领
bool is_r3b3_occupy # 己方侧 R3/B3 梯形高地占领状态 1 为已占领
bool is_r4b4_occupy # 己方侧 R4/B4 梯形高地占领状态 1 为已占领
bool has_virtual_shield # 己方基地护盾状态：1 为基地有虚拟护盾血量；0 为基地无虚拟护盾血量
bool is_outpost_battle_survival # 己方前哨战状态：1 为前哨战存活；0 为前哨战被击毁
uint32 reserve # bit 11-31: 保留
