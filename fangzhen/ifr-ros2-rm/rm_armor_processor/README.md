# rm_armor_processor
> 装甲板预测器

用于预测装甲板运动轨迹, 使用扩展卡尔曼滤波估计装甲板的: xyz坐标、xyz速度、yaw位置、yaw速度、车半径

修改自 https://github.com/chenjunnn/rm_auto_aim/tree/main/armor_tracker


## Publisher

 - `/rm/tracker/info`: 调试用追踪信息 `ifr_interface/msg/RmTrackerInfo`  
 - `/rm/tracker/target`: 目标信息 `ifr_interface/msg/RmTarget`  
 - `/tracker/marker`: 调试用姿态估计 `visualization_msgs/msg/MarkerArray`  


## Subscriber

 - `/rm/finder/armors`: 识别到的装甲板信息 `ifr_interface/msg/RmArmors`  
