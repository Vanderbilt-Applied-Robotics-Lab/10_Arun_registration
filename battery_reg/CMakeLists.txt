cmake_minimum_required(VERSION 3.8)
project(battery_reg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

## INCLUDE(S)
include_directories(include)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

# executables
add_executable(battery_reg src/battery_reg.cpp)

# dependencies
ament_target_dependencies(battery_reg rclcpp Eigen3 tf2_ros visualization_msgs geometry_msgs)

target_link_libraries(battery_reg Eigen3::Eigen)

install(TARGETS battery_reg
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

ament_package()
