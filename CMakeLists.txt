cmake_minimum_required(VERSION 2.8.3)
project(car_demo)

set (PRIUSCUP_MAJOR_VERSION 0)
set (PRIUSCUP_MINOR_VERSION 1)
set (PRIUSCUP_PATCH_VERSION 0)
set (PRIUSCUP_VERSION_FULL ${PRIUSCUP_MAJOR_VERSION}.${PRIUSCUP_MINOR_VERSION}.${PRIUSCUP_PATCH_VERSION})

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  message_generation
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  image_geometry
  cv_bridge
  rospy
)

find_package(gazebo 9 REQUIRED)
find_package(ignition-msgs1 REQUIRED)
find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  Control.msg
  LaneCoefficients.msg
  Trajectory.msg
  ControlDebug.msg
  Objects.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
 # INCLUDE_DIRS include
 # LIBRARIES gazebo_radar_plugin
 CATKIN_DEPENDS gazebo_ros message_runtime std_msgs
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_library(PriusHybridPlugin SHARED plugins/PriusHybridPlugin.cc)
target_link_libraries(PriusHybridPlugin ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES})
add_dependencies(PriusHybridPlugin ${catkin_EXPORTED_TARGETS})


catkin_install_python(PROGRAMS nodes/joystick_translator
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS nodes/vehicle_control_gui
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS nodes/lane_detection.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS nodes/lane_detection_loop
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS PriusHybridPlugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

