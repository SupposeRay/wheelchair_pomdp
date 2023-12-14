cmake_minimum_required(VERSION 3.0.2)
#project(wheelchair_pomdp)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(CUDA_SDK_ROOT_DIR "/usr/local/cuda/samples")



if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_STANDARD_REQUIRED True)
endif()

if(NOT DEFINED CMAKE_CUDA_STANDARD)
  set(CMAKE_CUDA_STANDARD 11)
  set(CMAKE_CUDA_STANDARD_REQUIRED True)
endif()


if (CUDA)
endif()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  cv_bridge
  voronoi_msgs_and_types
)

find_package(OpenCV)

#find_package (Despot CONFIG REQUIRED)

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
# )

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES wheelchair_pomdp
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs cv_bridge voronoi_msgs_and_types
#  DEPENDS system_lib
)

set(Python_FIND_VIRTUALENV "FIRST")
set(Python_FIND_STRATEGY "LOCATION")
set(Python_ROOT_DIR "$ENV{HOME}/miniconda3/envs/proMPs")
find_package (Python 3.6 COMPONENTS Interpreter Development NumPy REQUIRED)
message(${Python_ROOT_DIR})

include_directories(
${catkin_INCLUDE_DIRS}
../../HypDespot/include/
include/
GPU_files/
${Python_INCLUDE_DIRS}

)

# FILE(GLOB_RECURSE CURRENT_INCLUDE include/*.h)
# FILE(GLOB_RECURSE CURRENT_SOURCES src/*.cpp)

add_executable(${PROJECT_NAME}_wheelchair_pomdp
  src/wheelchair_gazebo.cpp
  src/wheelchair_model.cpp
  src/wheelchair_dmp.cpp
  src/main.cpp
  src/param.cpp
  GPU_files/GPU_param.cu
  GPU_files/GPU_wheelchair_model.cu
  GPU_files/GPU_WheelchairLowerBound.cu
  GPU_files/GPU_WheelchairUpperBound.cu
  GPU_files/GPU_Init.cu
)

# add_executable(${PROJECT_NAME}
#   ${CURRENT_SOURCES}
# )

add_dependencies(${PROJECT_NAME}_wheelchair_pomdp
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_wheelchair_pomdp
  #despot
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Python_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

set_target_properties(
			${PROJECT_NAME}_wheelchair_pomdp
                       PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_target_properties(
			${PROJECT_NAME}_wheelchair_pomdp
                       PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_compile_definitions(
  ${PROJECT_NAME}_wheelchair_pomdp  
  PRIVATE KW_EXPORTS
  INTERFACE KW_IMPORTS
)