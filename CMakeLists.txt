cmake_minimum_required(VERSION 3.0.2)
project(robolimb)


find_package(catkin REQUIRED COMPONENTS
  std_msgs
)

catkin_package()

catkin_python_setup()

#  catkin_install_python(PROGRAMS
#    src/ros_node.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  )



