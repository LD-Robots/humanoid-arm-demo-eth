# Install script for directory: /home/andrei-dragomir/ros2_eth/src/myactuator_x6_test

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/andrei-dragomir/ros2_eth/install/myactuator_x6_test")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/myactuator_x6_test" TYPE DIRECTORY FILES
    "/home/andrei-dragomir/ros2_eth/src/myactuator_x6_test/config"
    "/home/andrei-dragomir/ros2_eth/src/myactuator_x6_test/launch"
    "/home/andrei-dragomir/ros2_eth/src/myactuator_x6_test/urdf"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/myactuator_x6_test" TYPE PROGRAM FILES
    "/home/andrei-dragomir/ros2_eth/src/myactuator_x6_test/scripts/test_trajectory.py"
    "/home/andrei-dragomir/ros2_eth/src/myactuator_x6_test/scripts/test_simple_position.py"
    "/home/andrei-dragomir/ros2_eth/src/myactuator_x6_test/scripts/test_pid_effort.py"
    "/home/andrei-dragomir/ros2_eth/src/myactuator_x6_test/scripts/test_moveit_ready.py"
    "/home/andrei-dragomir/ros2_eth/src/myactuator_x6_test/scripts/test_speed_position.py"
    "/home/andrei-dragomir/ros2_eth/src/myactuator_x6_test/scripts/test_compliance.py"
    "/home/andrei-dragomir/ros2_eth/src/myactuator_x6_test/scripts/safety_controller.py"
    "/home/andrei-dragomir/ros2_eth/src/myactuator_x6_test/scripts/virtual_wall.py"
    "/home/andrei-dragomir/ros2_eth/src/myactuator_x6_test/scripts/switch_controller.py"
    "/home/andrei-dragomir/ros2_eth/src/myactuator_x6_test/scripts/controller_switcher.py"
    "/home/andrei-dragomir/ros2_eth/src/myactuator_x6_test/scripts/position_mode_manager.py"
    "/home/andrei-dragomir/ros2_eth/src/myactuator_x6_test/scripts/mode_switcher.py"
    "/home/andrei-dragomir/ros2_eth/src/myactuator_x6_test/scripts/set_zero.py"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/andrei-dragomir/ros2_eth/build/myactuator_x6_test/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/myactuator_x6_test")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/andrei-dragomir/ros2_eth/build/myactuator_x6_test/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/myactuator_x6_test")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/myactuator_x6_test/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/myactuator_x6_test/environment" TYPE FILE FILES "/home/andrei-dragomir/ros2_eth/build/myactuator_x6_test/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/myactuator_x6_test/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/myactuator_x6_test/environment" TYPE FILE FILES "/home/andrei-dragomir/ros2_eth/build/myactuator_x6_test/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/myactuator_x6_test" TYPE FILE FILES "/home/andrei-dragomir/ros2_eth/build/myactuator_x6_test/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/myactuator_x6_test" TYPE FILE FILES "/home/andrei-dragomir/ros2_eth/build/myactuator_x6_test/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/myactuator_x6_test" TYPE FILE FILES "/home/andrei-dragomir/ros2_eth/build/myactuator_x6_test/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/myactuator_x6_test" TYPE FILE FILES "/home/andrei-dragomir/ros2_eth/build/myactuator_x6_test/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/myactuator_x6_test" TYPE FILE FILES "/home/andrei-dragomir/ros2_eth/build/myactuator_x6_test/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/andrei-dragomir/ros2_eth/build/myactuator_x6_test/ament_cmake_index/share/ament_index/resource_index/packages/myactuator_x6_test")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/myactuator_x6_test/cmake" TYPE FILE FILES
    "/home/andrei-dragomir/ros2_eth/build/myactuator_x6_test/ament_cmake_core/myactuator_x6_testConfig.cmake"
    "/home/andrei-dragomir/ros2_eth/build/myactuator_x6_test/ament_cmake_core/myactuator_x6_testConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/myactuator_x6_test" TYPE FILE FILES "/home/andrei-dragomir/ros2_eth/src/myactuator_x6_test/package.xml")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/andrei-dragomir/ros2_eth/build/myactuator_x6_test/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
if(CMAKE_INSTALL_COMPONENT)
  if(CMAKE_INSTALL_COMPONENT MATCHES "^[a-zA-Z0-9_.+-]+$")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
  else()
    string(MD5 CMAKE_INST_COMP_HASH "${CMAKE_INSTALL_COMPONENT}")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INST_COMP_HASH}.txt")
    unset(CMAKE_INST_COMP_HASH)
  endif()
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/andrei-dragomir/ros2_eth/build/myactuator_x6_test/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
