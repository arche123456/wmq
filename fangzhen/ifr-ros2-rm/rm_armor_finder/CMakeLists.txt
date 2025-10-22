cmake_minimum_required(VERSION 3.5)
project(rm_armor_finder LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Export compile commands for clangd


# add_compile_options(
#     "$<$<COMPILE_LANGUAGE:CXX>:-Wall;-Werror>"
#     "$<$<AND:$<CXX_COMPILER_ID:GNU,Clang>,$<CONFIG:Release>>:-O3;-flto;-march=native>"
#     "$<$<AND:$<CXX_COMPILER_ID:GNU,Clang>,$<CONFIG:RelWithDebInfo>>:-O3;-flto;-march=native;-g>"
#     "$<$<AND:$<CXX_COMPILER_ID:GNU,Clang>,$<CONFIG:MinSizeRel>>:-Os;-flto;-march=native>"
#     "$<$<AND:$<CXX_COMPILER_ID:GNU,Clang>,$<CONFIG:Debug>>:-O0;-g>"
# )

# if(CMAKE_BUILD_TYPE STREQUAL "Debug")
#     list(APPEND CUDA_NVCC_FLAGS "-G" "-g")
# endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(OpenCV REQUIRED)

set(USE_MLP OFF)
if (USE_MLP)
find_package(Torch REQUIRED)
endif()


file(GLOB_RECURSE CPP_SOURCE_FILES src/*.*)
ament_auto_add_library(${PROJECT_NAME} SHARED ${CPP_SOURCE_FILES})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
set_target_properties(${PROJECT_NAME} PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_property(TARGET ${PROJECT_NAME} PROPERTY CUDA_ARCHITECTURES 72)

target_compile_definitions(${PROJECT_NAME} PUBLIC "$<$<CONFIG:Debug>:DEBUG=1>")
target_compile_definitions(${PROJECT_NAME} PUBLIC "$<$<NOT:$<CONFIG:Debug>>:DEBUG=0>")
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
if (USE_MLP)
  target_link_libraries(${PROJECT_NAME} torch ${TORCH_LIBRARIES})
  target_compile_definitions(${PROJECT_NAME} PUBLIC USE_MLP=1)
else()
  target_compile_definitions(${PROJECT_NAME} PUBLIC USE_MLP=0)
endif()

set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} --cuda-gpu-arch=sm_72" )
# target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_cuda)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN ${PROJECT_NAME}::ArmorFinder
  EXECUTABLE ${PROJECT_NAME}_node
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN ${PROJECT_NAME}::ArmorSaverNode
  EXECUTABLE ${PROJECT_NAME}_saver
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN ${PROJECT_NAME}::ModelTest
  EXECUTABLE ${PROJECT_NAME}_test
)

# install
ament_auto_package(
  INSTALL_TO_SHARE
  model
  launch
  config
)