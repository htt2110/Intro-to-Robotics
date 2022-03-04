execute_process(COMMAND "/home/harshit/ros_wkspace_asgn2/build/assignment_cc/urdf_parser_py/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/harshit/ros_wkspace_asgn2/build/assignment_cc/urdf_parser_py/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
