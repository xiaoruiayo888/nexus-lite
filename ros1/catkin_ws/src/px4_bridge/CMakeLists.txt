cmake_minimum_required(VERSION 3.0.2)
project(px4_bridge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  mavros_msgs
  px4_bridge_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs sensor_msgs mavros_msgs px4_bridge_msgs
)

catkin_install_python(PROGRAMS
  nodes/bridge_node
  nodes/odom_bridge_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
