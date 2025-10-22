#pragma once

#if __has_include(<rclcpp/version.h>)
#include <rclcpp/version.h>
  #if (RCLCPP_VERSION_MAJOR >= 5)
    #define ROS_GTE_HUMBLE
  #endif
#endif
