cmake_minimum_required(VERSION 2.8.3)
project(summit_xl_control)


find_package(catkin REQUIRED
    effort_controllers
    joint_state_controller
    velocity_controllers
    twist_mux
    diff_drive_controller    
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
