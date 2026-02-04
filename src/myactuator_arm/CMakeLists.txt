cmake_minimum_required(VERSION 3.8)
project(myactuator_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install description files (URDF, ros2_control, meshes)
install(DIRECTORY
  description
  DESTINATION share/${PROJECT_NAME}
)

# Install robot_arm folder (meshes and URDF)
install(DIRECTORY
  robot_arm
  DESTINATION share/${PROJECT_NAME}
)

# Install robot_arm_6dof folder (meshes and URDF)
install(DIRECTORY
  robot_arm_6dof
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/virtual_wall.py
  scripts/set_zero.py
  scripts/position_mode_manager.py
  scripts/test_joint_motion.py
  scripts/test_position.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
