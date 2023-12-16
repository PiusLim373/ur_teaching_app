cmake_minimum_required(VERSION 3.1.3)
project(ur5_pap)

find_package(catkin REQUIRED genmsg actionlib_msgs geometry_msgs)
add_action_files(DIRECTORY action)
add_service_files(DIRECTORY srv)
generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs)
catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
