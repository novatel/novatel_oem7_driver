cmake_minimum_required(VERSION 2.8.3)
project(novatel_oem7_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Without this line, .deb package build fails due to lpthread linking errors.
set(novatel_oem7_msgs_DIR ${CMAKE_CURRENT_LIST_DIR}/../../install/share/novatel_oem7_msgs/cmake)


set(BUILD_DEPS
  nodelet
  roscpp
  std_msgs
  nav_msgs
  gps_common
  sensor_msgs
  tf
  message_generation
  novatel_oem7_msgs
)


find_package(catkin 			REQUIRED COMPONENTS ${BUILD_DEPS})
find_package(Boost  			REQUIRED COMPONENTS system thread)
find_package(novatel_oem7_msgs  REQUIRED)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJET_NAME}
  CATKIN_DEPENDS nodelet roscpp
)

# Make package available as a macro to C++
add_definitions("-D${PROJECT_NAME}_VERSION=\"${${PROJECT_NAME}_VERSION}\"")

###########
## Build ##
###########

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${novatel_oem7_msgs_INCLUDE_DIRS}
   "../../novatel_oem7/include"
	src
)



## All components are plugins
add_library(${PROJECT_NAME}
   src/oem7_log_nodelet.cpp
   src/oem7_message_nodelet.cpp
   src/oem7_config_nodelet.cpp
   src/oem7_receiver_net.cpp
   src/oem7_receiver_port.cpp
   src/oem7_receiver_file.cpp
   src/oem7_message_decoder.cpp
   src/oem7_message_util.cpp
   src/oem7_ros_messages.cpp
   src/message_handler.cpp
   src/bestpos_handler.cpp
   src/ins_handler.cpp
   src/align_handler.cpp
   src/time_handler.cpp
   src/receiverstatus_handler.cpp
)


add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


FIND_LIBRARY(OEM7_DECODER_LIB liboem7decoder.a "../../novatel_oem7/bin") 
message(STATUS "Linking to Oem7 Decoder at: '${OEM7_DECODER_LIB}'")

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${OEM7_DECODER_LIB}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html


## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} 
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Export plugin header files.
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
   FILES novatel_oem7_driver_nodelets.xml novatel_oem7_driver_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )

install(
   DIRECTORY 
   launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )

install(
   DIRECTORY 
   config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
 

#############
## Testing ##
#############

# Built-in Self Test (BIST)
catkin_install_python(PROGRAMS test/oem7_bist.py test/oem7_message_test.py
 	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
# Instaleld into SHARED vs BINARY. BINARY does not allow BIST to be run using
# rostest novatel_oem7_driver your_launch_file.lauch" syntax.


find_package(rostest REQUIRED)
add_rostest(test/align.test)
add_rostest(test/bestpos.test)
add_rostest(test/ins1.test)
add_rostest(test/ins2.test)
add_rostest(test/rxstatus.test)
add_rostest(test/time.test)



#message("includes:")
#get_property(incdirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
#foreach(dir ${incdirs})
#   message(STATUS "incdir= '${dir}'")
#endforeach()
