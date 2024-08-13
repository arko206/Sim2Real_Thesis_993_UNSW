# Install script for directory: /home/robocupathome/URRRRRRR_ws/src/ur5-gazebo-grasping/ur5-gazebo-grasping/general-message-pkgs/object_msgs

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/object_msgs/msg" TYPE FILE FILES
    "/home/robocupathome/URRRRRRR_ws/src/ur5-gazebo-grasping/ur5-gazebo-grasping/general-message-pkgs/object_msgs/msg/Object.msg"
    "/home/robocupathome/URRRRRRR_ws/src/ur5-gazebo-grasping/ur5-gazebo-grasping/general-message-pkgs/object_msgs/msg/ObjectPose.msg"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/object_msgs/srv" TYPE FILE FILES
    "/home/robocupathome/URRRRRRR_ws/src/ur5-gazebo-grasping/ur5-gazebo-grasping/general-message-pkgs/object_msgs/srv/ObjectInfo.srv"
    "/home/robocupathome/URRRRRRR_ws/src/ur5-gazebo-grasping/ur5-gazebo-grasping/general-message-pkgs/object_msgs/srv/RegisterObject.srv"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/object_msgs/cmake" TYPE FILE FILES "/home/robocupathome/URRRRRRR_ws/build/ur5-gazebo-grasping/ur5-gazebo-grasping/general-message-pkgs/object_msgs/catkin_generated/installspace/object_msgs-msg-paths.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/robocupathome/URRRRRRR_ws/devel/include/object_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/robocupathome/URRRRRRR_ws/devel/share/roseus/ros/object_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/robocupathome/URRRRRRR_ws/devel/share/common-lisp/ros/object_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/robocupathome/URRRRRRR_ws/devel/share/gennodejs/ros/object_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/robocupathome/URRRRRRR_ws/devel/lib/python3/dist-packages/object_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/robocupathome/URRRRRRR_ws/devel/lib/python3/dist-packages/object_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/robocupathome/URRRRRRR_ws/build/ur5-gazebo-grasping/ur5-gazebo-grasping/general-message-pkgs/object_msgs/catkin_generated/installspace/object_msgs.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/object_msgs/cmake" TYPE FILE FILES "/home/robocupathome/URRRRRRR_ws/build/ur5-gazebo-grasping/ur5-gazebo-grasping/general-message-pkgs/object_msgs/catkin_generated/installspace/object_msgs-msg-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/object_msgs/cmake" TYPE FILE FILES
    "/home/robocupathome/URRRRRRR_ws/build/ur5-gazebo-grasping/ur5-gazebo-grasping/general-message-pkgs/object_msgs/catkin_generated/installspace/object_msgsConfig.cmake"
    "/home/robocupathome/URRRRRRR_ws/build/ur5-gazebo-grasping/ur5-gazebo-grasping/general-message-pkgs/object_msgs/catkin_generated/installspace/object_msgsConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/object_msgs" TYPE FILE FILES "/home/robocupathome/URRRRRRR_ws/src/ur5-gazebo-grasping/ur5-gazebo-grasping/general-message-pkgs/object_msgs/package.xml")
endif()

