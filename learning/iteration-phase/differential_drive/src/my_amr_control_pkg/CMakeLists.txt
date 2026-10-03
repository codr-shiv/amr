cmake_minimum_required(VERSION 3.10)
project(my_amr_control_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(diff_drive_controller REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(controller_manager REQUIRED)
find_package(sensor_msgs REQUIRED)

# 1. Compile the C++ hardware interface into a shared library
add_library(${PROJECT_NAME} SHARED
  src/my_hw_interface.cpp
)

# 2. Add the include directory to the target
target_include_directories(${PROJECT_NAME} PRIVATE
  include
)

# 3. Link the required ROS 2 dependencies to the library
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  diff_drive_controller
  joint_state_broadcaster
  sensor_msgs
  controller_manager
)

# 4. Export the pluginlib XML file so ros2_control can find it dynamically
pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

# 5. Install the compiled library to the ROS 2 lib destination
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

# 6. Install the config, launch, and urdf directories to the ROS 2 share destination
install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()