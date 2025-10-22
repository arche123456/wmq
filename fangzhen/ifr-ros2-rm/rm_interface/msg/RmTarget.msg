# 代表RoboMaster中攻击目标信息
std_msgs/Header header #数据生成刻
builtin_interfaces/Time receive_time # 原数据输入刻
bool has_target # 是否有目标
uint8 type # 目标类型
geometry_msgs/Point position   # 目标位置
geometry_msgs/Vector3 velocity # 移动速度
float64 yaw
float64 v_yaw
float64 radius_1
float64 radius_2
float64 dz