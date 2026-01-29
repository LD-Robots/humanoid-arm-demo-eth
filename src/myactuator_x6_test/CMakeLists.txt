cmake_minimum_required(VERSION 3.10)
project(myactuator_x6_test)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY
  config
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/test_trajectory.py
  scripts/test_simple_position.py
  scripts/test_pid_effort.py
  scripts/test_moveit_ready.py
  scripts/test_speed_position.py
  scripts/test_compliance.py
  scripts/safety_controller.py
  scripts/virtual_wall.py
  scripts/switch_controller.py
  scripts/controller_switcher.py
  scripts/position_mode_manager.py
  scripts/mode_switcher.py
  scripts/set_zero.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
