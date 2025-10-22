cmake_minimum_required(VERSION 3.5)
project(rm_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_definitions(-Wall -Werror)
endif()

if (CMAKE_BUILD_TYPE STREQUAL Release)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -flto -march=native")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -flto -march=native")
    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++${CMAKE_CUDA_STANDARD}")
    add_definitions(-DDEBUG=0)
elseif (CMAKE_BUILD_TYPE STREQUAL RelWithDebInfo)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -flto -march=native -g")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -flto -march=native -g")
    set(CUDA_NVCC_FLAGS" ${CUDA_NVCC_FLAGS} -G;-g;-std=c++${CMAKE_CUDA_STANDARD}")
    add_definitions(-DDEBUG=0)
elseif (CMAKE_BUILD_TYPE STREQUAL MinSizeRel)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Os -flto -march=native")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Os -flto -march=native")
    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++${CMAKE_CUDA_STANDARD}")
    add_definitions(-DDEBUG=0)
else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -g")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -g")
    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -Xcompiler -fPIC;-G;-g;-std=c++${CMAKE_CUDA_STANDARD}")
    add_definitions(-DDEBUG=1)
endif ()


# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(rosidl_default_generators REQUIRED)

set(dependencies
        "std_msgs"
        "builtin_interfaces"
        "sensor_msgs"
        "geometry_msgs"
        "visualization_msgs"
        "tf2"
)
file(GLOB_RECURSE rosidl_MESSAGES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} msg/*.msg)
file(GLOB_RECURSE rosidl_SERVICES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} srv/*.srv)
file(GLOB_RECURSE rosidl_ACTIONS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} action/*.action)

rosidl_generate_interfaces(${PROJECT_NAME} ${rosidl_MESSAGES} ${rosidl_SERVICES} ${rosidl_ACTIONS} DEPENDENCIES ${dependencies})
#rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

ament_package()
