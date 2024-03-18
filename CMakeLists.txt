cmake_minimum_required(VERSION 3.0.2)
project(exposure_control)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   std_msgs
   message_generation
   cv_bridge
   image_transport
   sensor_msgs
   dynamic_reconfigure
   nodelet
)

find_package(OpenCV "3.2.0" REQUIRED)

add_message_files(
   FILES
   ExposureRegion.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES exposure_control
   CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
    # include      
    ${PROJECT_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${SPINNAKER_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${LibUnwind_INCLUDE_DIRS}
)

#add_executable(changeParam src/changeParam.cpp)
#target_link_libraries(changeParam ${catkin_LIBRARIES})

add_executable(aaec_exposure_node src/aaec_exposure_node.cpp)
target_link_libraries(aaec_exposure_node ${catkin_LIBRARIES})

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(stag_ros ${catkin_EXPORTED_LIBRARIES})
endif()

install(DIRECTORY param/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/param)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY robots/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/robots)

#install(TARGETS changeParam
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(TARGETS aaec_exposure_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/follow_aruco.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)