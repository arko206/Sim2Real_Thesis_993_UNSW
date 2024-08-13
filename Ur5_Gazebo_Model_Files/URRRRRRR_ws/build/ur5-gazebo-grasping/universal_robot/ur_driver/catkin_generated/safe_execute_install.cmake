execute_process(COMMAND "/home/robocupathome/URRRRRRR_ws/build/ur5-gazebo-grasping/universal_robot/ur_driver/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/robocupathome/URRRRRRR_ws/build/ur5-gazebo-grasping/universal_robot/ur_driver/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
