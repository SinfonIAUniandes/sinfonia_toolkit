cmake_minimum_required(VERSION 2.8.3)
project(robot_toolkit)

set(
  CONVERTERS_SRC
    src/navigation_tools/laser/laser_converter.cpp
    src/navigation_tools/laser/depth_to_laser_converter.cpp
    src/navigation_tools/laser/naoqi_depth2laser_converter.cpp
    src/navigation_tools/laser/laser_merged_converter.cpp
    src/navigation_tools/odom/odom_converter.cpp
    src/navigation_tools/tf/tf_converter.cpp
    src/navigation_tools/path/path_converter.cpp
    src/navigation_tools/robot_pose/robot_pose_converter.cpp
    src/navigation_tools/result/result_event.cpp

    src/vision_tools/camera_converter.cpp
    src/vision_tools/face_detector.cpp

    src/audio_tools/mic/mic_converter.cpp
    src/audio_tools/mic/mic_event.cpp

    src/audio_tools/mic/mic_localization_event.cpp

    src/misc_tools/sonar/sonar_converter.cpp
    src/misc_tools/touch/touch_event.cpp
    src/misc_tools/touch/touch_publisher.cpp

  )
set(
  TOOLS_SRC
    src/tools/robot_description.cpp
  )

set(
  PUBLISHER_SRC

    src/navigation_tools/laser/laser_publisher.cpp
    src/navigation_tools/odom/odom_publisher.cpp
    src/navigation_tools/tf/tf_publisher.cpp
    src/navigation_tools/path/path_publisher.cpp
    src/navigation_tools/robot_pose/robot_pose_publisher.cpp
    src/navigation_tools/result/result_publisher.cpp

    src/vision_tools/camera_publisher.cpp
    src/vision_tools/face_publisher.cpp

    src/audio_tools/mic/mic_publisher.cpp
    src/audio_tools/mic/mic_localization_publisher.cpp
    src/audio_tools/speech_recognition/speech_recognition_event.cpp

    src/misc_tools/sonar/sonar_publisher.cpp
  )

set(
  SUBSCRIBER_SRC
    src/navigation_tools/cmd_vel/cmd_vel_subscriber.cpp
    src/navigation_tools/move_to/move_to.cpp
    src/navigation_tools/navigation_goal/navigation_goal_subscriber.cpp
    src/navigation_tools/robot_pose/robot_pose_subscriber.cpp
    src/navigation_tools/free_zone/free_zone_subscriber.cpp
    src/audio_tools/speech/speech_subscriber.cpp

    src/motion_tools/animation_subscriber.cpp
    src/motion_tools/set_angles_subscriber.cpp

    src/misc_tools/leds_subscriber.cpp
    src/misc_tools/special_settings/special_settings_subscriber.cpp


  )

set(
  SERVICES_SRC

  )

set(
  RECORDER_SRC
  )

set(
  ROBOT_TOOLKIT_SRC
  src/robot_toolkit.cpp
  src/helpers/toolkit_helpers.cpp
  )



add_definitions( -DCATKIN_BUILD )

# CMake file that uses catkin
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  diagnostic_msgs
  diagnostic_updater
  geometry_msgs
  nav_msgs
  image_transport
  kdl_parser
  naoqi_bridge_msgs
  naoqi_libqi
  naoqi_libqicore
  robot_state_publisher
  rosbag_storage
  rosconsole
  rosgraph_msgs
  sensor_msgs
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
  image_geometry
  robot_toolkit_msgs
)

find_package(OpenCV)
find_package(orocos_kdl)

#find_package(qibuild)

#qi_use_lib(ALMATH)
#find_package(almath REQUIRED)
#include_directories(${ALMATH_INCLUDE_DIRS})

add_definitions(-DLIBQI_VERSION=${naoqi_libqi_VERSION_MAJOR}${naoqi_libqi_VERSION_MINOR})

catkin_package(LIBRARIES robot_toolkit_module robot_toolkit)
catkin_package(CATKIN_DEPENDS message_runtime )

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=deprecated-declarations")
endif()

include_directories( include ${catkin_INCLUDE_DIRS})

# create the different libraries
add_library(
  robot_toolkit_module
  SHARED
  src/autoload_registration.cpp
  )
target_link_libraries(robot_toolkit_module ${naoqi_libqi_LIBRARIES})
install(TARGETS robot_toolkit_module DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

add_library(
  robot_toolkit
  SHARED
  ${ROBOT_TOOLKIT_SRC}
  ${CONVERTERS_SRC}
  ${PUBLISHER_SRC}
  ${SUBSCRIBER_SRC}
  ${SERVICES_SRC}
  ${RECORDER_SRC}
  ${TOOLS_SRC}
)
target_link_libraries(
  robot_toolkit
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  )
add_dependencies(robot_toolkit ${catkin_EXPORTED_TARGETS})
install(TARGETS robot_toolkit DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

target_link_libraries(
  robot_toolkit_module
  robot_toolkit
)

# create the binary of the bridge
add_executable( robot_toolkit_node src/main.cpp)
target_link_libraries(
  robot_toolkit_node
  robot_toolkit
  ${catkin_LIBRARIES}
)

install(TARGETS robot_toolkit_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# install the urdf for runtime loading
file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/share" DESTINATION "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/")
install(DIRECTORY share DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}")

# install the launch files
install(DIRECTORY launch DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}")

# make sure there is a file describing a naoqi module
file(WRITE ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_SHARE_DESTINATION}/qi/module/robot_toolkit_module.mod "cpp")
install(FILES ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_SHARE_DESTINATION}/qi/module/robot_toolkit_module.mod DESTINATION ${CATKIN_GLOBAL_SHARE_DESTINATION}/qi/module/)
