cmake_minimum_required(VERSION 3.5)
project(laser_to_foot)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)

# 查找必要依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)  # 你在 package.xml 里声明了它，CMake 也要查找
find_package(rosidl_default_runtime REQUIRED)  # 解决 message type support 依赖

# 创建可执行目标
add_executable(laser_to_foot src/node.cpp)

# 链接库
target_link_libraries(laser_to_foot
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  #tf2_sensor_msgs::tf2_sensor_msgs
)

# 这里确保消息类型支持
ament_target_dependencies(laser_to_foot
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
  rosidl_default_runtime  # 解决 PointCloud2 的类型支持问题
)

# 安装和打包配置
install(TARGETS laser_to_foot
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
