# rm_serial_port
> 串口管理

包含通用串口框架`SerialPort`和所有实现如`SerialPortMotor`  
用于读取串口输入并向串口输出数据


## 纯云台控制

### Publisher

 - `/rm/self_team`: 自身队伍信息 `ifr_interface/msg/Team`  
 - `/serial/use_forecast`: 是否使用预测 `std_msgs/msg/Bool`  
 - `/tf`: 陀螺仪数据 `tf2_msgs/msg/TFMessage`  

### Subscriber
 - `/rm/tracker/target`: 瞄准位置信息 `ifr_interface/msg/RmTarget`  
