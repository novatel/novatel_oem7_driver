cmake_minimum_required(VERSION 3.0.2)
project(novatel_oem7_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(BUILD_DEPS
  nodelet
  roscpp
  std_msgs
  gps_common
  sensor_msgs
  nmea_msgs
  nav_msgs
  tf2_geometry_msgs
  message_generation
  novatel_oem7_msgs
)



find_package(catkin 			REQUIRED COMPONENTS ${BUILD_DEPS})
find_package(Boost  			REQUIRED COMPONENTS system thread)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
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
    "novatel_oem7/include"
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
   src/oem7_debug_file.cpp
   src/message_handler.cpp
   src/bestpos_handler.cpp
   src/ins_handler.cpp
   src/align_handler.cpp
   src/time_handler.cpp
   src/receiverstatus_handler.cpp
   src/nmea_handler.cpp
   src/oem7_imu.cpp
)


add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

message(STATUS "System Host Proc: '${CMAKE_HOST_SYSTEM_PROCESSOR}'")
message(STATUS "System Proc: '${CMAKE_SYSTEM_PROCESSOR}'")
message(STATUS "VOID_P: ${CMAKE_SIZEOF_VOID_P}")

if (CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "armv7l")
    if (CMAKE_SIZEOF_VOID_P EQUAL 8)
        FIND_LIBRARY(OEM7_DECODER_LIB liboem7decoder.a "novatel_oem7/bin/arm64v8") 
    else ()
        FIND_LIBRARY(OEM7_DECODER_LIB liboem7decoder.a "novatel_oem7/bin/arm32v7")
    endif ()
else ()
    if (CMAKE_SIZEOF_VOID_P EQUAL 8)
        FIND_LIBRARY(OEM7_DECODER_LIB liboem7decoder.a "novatel_oem7/bin/amd64") 
    else ()
        FIND_LIBRARY(OEM7_DECODER_LIB liboem7decoder.a "novatel_oem7/bin/i386")
    endif ()
endif ()


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


if (CATKIN_ENABLE_TESTING)
	find_package(rostest REQUIRED)
	add_rostest(test/align.test)
	add_rostest(test/bestpos.test)
	add_rostest(test/ins1.test)
	add_rostest(test/ins2.test)
	add_rostest(test/rxstatus.test)
	add_rostest(test/time.test)
endif()


#message("includes:")
#get_property(incdirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
#foreach(dir ${incdirs})
#   message(STATUS "incdir= '${dir}'")
#endforeach()
