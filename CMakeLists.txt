cmake_minimum_required(VERSION 3.8)
project(bebop_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Create node executable
add_executable(bebop_control_node src/bebop_control_node.cpp src/DirtyDerivative.cpp)

target_include_directories(bebop_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(bebop_control_node Eigen3::Eigen)
ament_target_dependencies(bebop_control_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  Eigen3
)

# Install executable
install(TARGETS bebop_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY param DESTINATION share/${PROJECT_NAME})

ament_package()
