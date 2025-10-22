cmake_minimum_required(VERSION 3.5)
project(ifr_pointcloud_to_laserscan LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Export compile commands for clangd

# add_compile_options(
# "$<$<COMPILE_LANGUAGE:CXX>:-Wall;-Werror>"
# "$<$<AND:$<CXX_COMPILER_ID:GNU,Clang>,$<CONFIG:Release>>:-O3;-flto;-march=native>"
# "$<$<AND:$<CXX_COMPILER_ID:GNU,Clang>,$<CONFIG:RelWithDebInfo>>:-O3;-flto;-march=native;-g>"
# "$<$<AND:$<CXX_COMPILER_ID:GNU,Clang>,$<CONFIG:MinSizeRel>>:-Os;-flto;-march=native>"
# "$<$<AND:$<CXX_COMPILER_ID:GNU,Clang>,$<CONFIG:Debug>>:-O0;-g>"
# )

# if(CMAKE_BUILD_TYPE STREQUAL "Debug")
# list(APPEND CUDA_NVCC_FLAGS "-G" "-g")
# endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

file(GLOB_RECURSE CPP_SOURCE_FILES src/*.*)
ament_auto_add_library(${PROJECT_NAME} SHARED ${CPP_SOURCE_FILES})

target_compile_definitions(${PROJECT_NAME} PUBLIC "$<$<CONFIG:Debug>:DEBUG=1>")
target_compile_definitions(${PROJECT_NAME} PUBLIC "$<$<NOT:$<CONFIG:Debug>>:DEBUG=0>")

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN ${PROJECT_NAME}::PointCloudToLaserScanNode
  EXECUTABLE ${PROJECT_NAME}_node
)

# install
ament_auto_package(
  INSTALL_TO_SHARE
  launch
)