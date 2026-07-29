cmake_minimum_required(VERSION 3.8)
project(arm_dashboard)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Required for Qt classes using Q_OBJECT.
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_library(arm_dashboard_panel SHARED
  src/arm_dashboard_panel.cpp
  include/arm_dashboard/arm_dashboard_panel.hpp
)

target_include_directories(arm_dashboard_panel
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(arm_dashboard_panel
  pluginlib
  rclcpp
  rviz_common
  sensor_msgs
  std_msgs
)

target_link_libraries(arm_dashboard_panel
  Qt5::Widgets
)

pluginlib_export_plugin_description_file(
  rviz_common
  plugin_description.xml
)

install(
  TARGETS arm_dashboard_panel
  EXPORT export_arm_dashboard
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_arm_dashboard)
ament_export_dependencies(
  pluginlib
  rclcpp
  rviz_common
  sensor_msgs
  std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
