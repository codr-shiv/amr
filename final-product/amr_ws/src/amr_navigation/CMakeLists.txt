cmake_minimum_required(VERSION 3.8)
project(amr_navigation)

find_package(ament_cmake REQUIRED)

# Config, launch files and maps -> share/amr_navigation/...
install(
  DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
)

# Helper scripts -> lib/amr_navigation/ (so `ros2 run` finds them)
install(
  PROGRAMS scripts/check_setup.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
