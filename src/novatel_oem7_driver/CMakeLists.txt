cmake_minimum_required(VERSION 3.10)
project(novatel_oem7_driver)


## Compile as C++14, supported on ROS2
set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)

find_package(pluginlib)
find_package(rclcpp REQUIRED) 
find_package(rclcpp_components REQUIRED) 
find_package(std_msgs REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(novatel_oem7_msgs REQUIRED)
find_package(Boost  			REQUIRED)

LIST(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")

find_package(GeographicLib REQUIRED)

# Make package available as a macro to C++
add_definitions("-D${PROJECT_NAME}_VERSION=\"${${PROJECT_NAME}_VERSION}\"")

###########
## Build ##
###########



# Log environment
message(STATUS "System Host Proc: '${CMAKE_HOST_SYSTEM_PROCESSOR}'")
message(STATUS "System Proc: '${CMAKE_SYSTEM_PROCESSOR}'")
message(STATUS "VOID_P: ${CMAKE_SIZEOF_VOID_P}")


set(OEM7_DECODER novatel_oem7_decoder)



include(ExternalProject)
ExternalProject_Add(${OEM7_DECODER}
  GIT_REPOSITORY https://github.com/novatel/novatel_edie
  GIT_TAG origin/dev-ros_install_prefix
  UPDATE_DISCONNECTED TRUE
  PREFIX ${CMAKE_BINARY_DIR}/${OEM7_DECODER}
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_BINARY_DIR} -DVERSION=1.0.0.2
)


set(OEM7_DECODER_DIR ${CMAKE_BINARY_DIR}/${OEM7_DECODER}/src/${OEM7_DECODER})

include_directories(include
    src
    ${OEM7_DECODER_DIR}	
    ${OEM7_DECODER_DIR}/src
    ${CMAKE_BINARY_DIR}/usr/include/novatel/edie           
    ${OEM7_DECODER_DIR}/bin/Novatel/api/
    ${OEM7_DECODER_DIR}/bin/StreamInterface/api/
)

## All components are plugins
add_library(${PROJECT_NAME} SHARED
   src/oem7_receiver_net.cpp
   src/oem7_receiver_port.cpp
   src/oem7_receiver_file.cpp
   src/oem7_message_decoder.cpp
   src/oem7_message_decoder_lib.cpp
   src/oem7_message_util.cpp
   src/oem7_ros_messages.cpp
   src/oem7_debug_file.cpp
   src/bestpos_handler.cpp
   src/ins_handler.cpp
   src/align_handler.cpp
   src/time_handler.cpp
   src/receiverstatus_handler.cpp
   src/nmea_handler.cpp
   src/odometry_handler.cpp
   src/oem7_imu.cpp
)

## All components are plugins
add_executable(${PROJECT_NAME}_exe
   src/oem7_message_node.cpp
   src/oem7_message_util.cpp
   src/oem7_ros_messages.cpp
   src/message_handler.cpp
) 



message("Linking to EDIE at: '${CMAKE_BINARY_DIR}'")

target_link_libraries(${PROJECT_NAME}
   Boost::boost
   ${GeographicLib_LIBRARIES}
   ${CMAKE_BINARY_DIR}/usr/lib/libCommon.a
   ${CMAKE_BINARY_DIR}/usr/lib/libStreamInterface.a
   ${CMAKE_BINARY_DIR}/usr/lib/libNovatel.a
   ${CMAKE_BINARY_DIR}/usr/lib/libCommon.a   
)

add_dependencies(${PROJECT_NAME} ${OEM7_DECODER})

ament_target_dependencies(${PROJECT_NAME} 
	pluginlib
	std_msgs 
	gps_msgs
	sensor_msgs 
	nmea_msgs 	
	nav_msgs
	tf2_geometry_msgs 
	novatel_oem7_msgs
)


ament_target_dependencies(${PROJECT_NAME}_exe 
    rclcpp
    rclcpp_components
	pluginlib
	novatel_oem7_msgs 
)


install(TARGETS
  ${PROJECT_NAME}_exe
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
	SET(TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test)
	find_package(launch_testing_ament_cmake)

 	add_launch_test(
		test/align.test.py
		ARGS "test_name:=align"
	 	ARGS "test_dir:=${TEST_DIR}" 
	 	ARGS "oem7_file_name:=${TEST_DIR}/align.gps"
	 	)
	 	
 	add_launch_test(
		test/bestpos.test.py
		ARGS "test_name:=bestpos"
 		ARGS "test_dir:=${TEST_DIR}" 
 		ARGS "oem7_file_name:=${TEST_DIR}/bestpos.gps"
 	)
 	
 	add_launch_test(
		test/ins1.test.py
		ARGS "test_name:=ins1"
 		ARGS "test_dir:=${TEST_DIR}" 
 		ARGS "oem7_file_name:=${TEST_DIR}/ins1.gps"
 	)
 	
 	add_launch_test(
		test/ins2.test.py
		ARGS "test_name:=ins2"
 		ARGS "test_dir:=${TEST_DIR}" 
 		ARGS "oem7_file_name:=${TEST_DIR}/ins2.gps"
 	)

 	add_launch_test(
		test/rxstatus.test.py
		ARGS "test_name:=rxstatus"
 		ARGS "test_dir:=${TEST_DIR}" 
 		ARGS "oem7_file_name:=${TEST_DIR}/rxstatus.gps"
 	)
 	
 	add_launch_test(
		test/time.test.py
		ARGS "test_name:=time"
 		ARGS "test_dir:=${TEST_DIR}" 
 		ARGS "oem7_file_name:=${TEST_DIR}/time.gps"
 	)
	 	
endif()


#############
## Install ##
#############

ament_export_dependencies(ament_cmake)
ament_export_include_directories(include)
pluginlib_export_plugin_description_file(${PROJECT_NAME} novatel_oem7_driver_plugins.xml)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} 
   ARCHIVE DESTINATION lib 
   LIBRARY DESTINATION lib 
   RUNTIME DESTINATION bin 
)

install(FILES ${PROJECT_NAME}_plugins.xml DESTINATION share)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME} FILES_MATCHING PATTERN "*.yaml")
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} FILES_MATCHING PATTERN "*.py")
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} FILES_MATCHING PATTERN "*.launch")


ament_python_install_package(${PROJECT_NAME})

ament_package()


