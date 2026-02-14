cmake_minimum_required(VERSION 3.8)
project(phidgets_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_manager REQUIRED)

include_directories(include)

add_library(phidgets_hardware SHARED
  src/phidgets_stepper_hardware.cpp
)

ament_target_dependencies(phidgets_hardware
  rclcpp
  hardware_interface
  pluginlib
)

# Phidgets (Phidget22) C library + headers
target_link_libraries(phidgets_hardware
  phidget22
)

pluginlib_export_plugin_description_file(hardware_interface phidgets_hardware_plugin.xml)

install(TARGETS phidgets_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY description config launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES phidgets_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install Python teleop script(s)
install(PROGRAMS
  phidgets_hardware/keyboard_teleop.py
  phidgets_hardware/logitech_teleop.py
  phidgets_hardware/ps5_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

