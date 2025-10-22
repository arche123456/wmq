cmake_minimum_required(VERSION 3.5)
project(rm_armor_processor LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_definitions(-Wall -Werror)
endif()

if (CMAKE_BUILD_TYPE STREQUAL Release)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -flto -march=native")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -flto -march=native")
    add_definitions(-DDEBUG=0)
elseif (CMAKE_BUILD_TYPE STREQUAL RelWithDebInfo)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -flto -march=native -g")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -flto -march=native -g")
    add_definitions(-DDEBUG=0)
elseif (CMAKE_BUILD_TYPE STREQUAL MinSizeRel)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os -flto -march=native")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Os -flto -march=native")
    add_definitions(-DDEBUG=0)
else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -g")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -g")
    add_definitions(-DDEBUG=1)
endif ()

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


# Target
ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN ${PROJECT_NAME}::ArmorTrackerNode
  EXECUTABLE ${PROJECT_NAME}_tracker_node
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN ${PROJECT_NAME}::ArmorDirectNode
  EXECUTABLE ${PROJECT_NAME}_direct_node
)

# Test
if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   list(APPEND AMENT_LINT_AUTO_EXCLUDE
#     ament_cmake_copyright
#     ament_cmake_uncrustify
#     ament_cmake_cpplint
#   )
#   ament_lint_auto_find_test_dependencies()

#   find_package(ament_cmake_gtest)
#   ament_add_gtest(test_node_startup test/test_node_startup.cpp)
#   target_link_libraries(test_node_startup ${PROJECT_NAME})

#   ament_add_gtest(test_number_cls test/test_number_cls.cpp)
#   target_link_libraries(test_number_cls ${PROJECT_NAME})

endif()

# Install
ament_auto_package(
  INSTALL_TO_SHARE
)