cmake_minimum_required(VERSION 3.5)
project(rm_common)

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

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

ament_auto_package(
  INSTALL_TO_SHARE
)