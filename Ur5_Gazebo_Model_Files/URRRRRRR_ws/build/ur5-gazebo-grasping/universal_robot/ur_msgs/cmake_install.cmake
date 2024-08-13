# Install script for directory: /home/robocupathome/URRRRRRR_ws/src/ur5-gazebo-grasping/universal_robot/ur_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/robocupathome/URRRRRRR_ws/install")
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

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur_msgs/msg" TYPE FILE FILES
    "/home/robocupathome/URRRRRRR_ws/src/ur5-gazebo-grasping/universal_robot/ur_msgs/msg/Analog.msg"
    "/home/robocupathome/URRRRRRR_ws/src/ur5-gazebo-grasping/universal_robot/ur_msgs/msg/Digital.msg"
    "/home/robocupathome/URRRRRRR_ws/src/ur5-gazebo-grasping/universal_robot/ur_msgs/msg/IOStates.msg"
    "/home/robocupathome/URRRRRRR_ws/src/ur5-gazebo-grasping/universal_robot/ur_msgs/msg/RobotStateRTMsg.msg"
    "/home/robocupathome/URRRRRRR_ws/src/ur5-gazebo-grasping/universal_robot/ur_msgs/msg/MasterboardDataMsg.msg"
    "/home/robocupathome/URRRRRRR_ws/src/ur5-gazebo-grasping/universal_robot/ur_msgs/msg/RobotModeDataMsg.msg"
    "/home/robocupathome/URRRRRRR_ws/src/ur5-gazebo-grasping/universal_robot/ur_msgs/msg/ToolDataMsg.msg"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur_msgs/srv" TYPE FILE FILES
    "/home/robocupathome/URRRRRRR_ws/src/ur5-gazebo-grasping/universal_robot/ur_msgs/srv/SetPayload.srv"
    "/home/robocupathome/URRRRRRR_ws/src/ur5-gazebo-grasping/universal_robot/ur_msgs/srv/SetSpeedSliderFraction.srv"
    "/home/robocupathome/URRRRRRR_ws/src/ur5-gazebo-grasping/universal_robot/ur_msgs/srv/SetIO.srv"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur_msgs/cmake" TYPE FILE FILES "/home/robocupathome/URRRRRRR_ws/build/ur5-gazebo-grasping/universal_robot/ur_msgs/catkin_generated/installspace/ur_msgs-msg-paths.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/robocupathome/URRRRRRR_ws/devel/include/ur_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/robocupathome/URRRRRRR_ws/devel/share/roseus/ros/ur_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/robocupathome/URRRRRRR_ws/devel/share/common-lisp/ros/ur_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/robocupathome/URRRRRRR_ws/devel/share/gennodejs/ros/ur_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/robocupathome/URRRRRRR_ws/devel/lib/python3/dist-packages/ur_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/robocupathome/URRRRRRR_ws/devel/lib/python3/dist-packages/ur_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/robocupathome/URRRRRRR_ws/build/ur5-gazebo-grasping/universal_robot/ur_msgs/catkin_generated/installspace/ur_msgs.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur_msgs/cmake" TYPE FILE FILES "/home/robocupathome/URRRRRRR_ws/build/ur5-gazebo-grasping/universal_robot/ur_msgs/catkin_generated/installspace/ur_msgs-msg-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur_msgs/cmake" TYPE FILE FILES
    "/home/robocupathome/URRRRRRR_ws/build/ur5-gazebo-grasping/universal_robot/ur_msgs/catkin_generated/installspace/ur_msgsConfig.cmake"
    "/home/robocupathome/URRRRRRR_ws/build/ur5-gazebo-grasping/universal_robot/ur_msgs/catkin_generated/installspace/ur_msgsConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur_msgs" TYPE FILE FILES "/home/robocupathome/URRRRRRR_ws/src/ur5-gazebo-grasping/universal_robot/ur_msgs/package.xml")
endif()

