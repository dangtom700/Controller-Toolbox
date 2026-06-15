cmake_minimum_required(VERSION 3.16...3.31)
project(ctrl_toolbox_ros2 VERSION 1.0.0 LANGUAGES CXX)

# Match controller_toolbox's C++20 standard.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------------------
# ROS 2 dependencies
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

# ---------------------------------------------------------------------------
# Controller Toolbox (installed via DIST-1 cmake --install)
# ---------------------------------------------------------------------------
find_package(ControllerToolbox REQUIRED)

# ---------------------------------------------------------------------------
# Header-only interface library
# ControllerNode<T> is a fully-templated lifecycle node; no .cpp needed.
# ---------------------------------------------------------------------------
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} INTERFACE
  rclcpp rclcpp_lifecycle std_msgs)
target_link_libraries(${PROJECT_NAME} INTERFACE
  ctrl::controller_toolbox)

# ---------------------------------------------------------------------------
# Example executable: PID temperature controller
# ---------------------------------------------------------------------------
add_executable(pid_temperature_node example/pid_temperature_node.cpp)
target_link_libraries(pid_temperature_node ${PROJECT_NAME})

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(DIRECTORY include/
  DESTINATION include)

install(TARGETS ${PROJECT_NAME} pid_temperature_node
  EXPORT ${PROJECT_NAME}Targets
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ctrl_toolbox_ros2::
  DESTINATION lib/cmake/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_lifecycle std_msgs ControllerToolbox)

ament_package()
