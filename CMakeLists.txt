cmake_minimum_required(VERSION 2.8)

# Compiler flags
# TOD0: -std=c++14 will be obsolete when reverting to minimum CMake version 3.2 and
#       using set_property(TARGET ${TARGET_NAME} PROPERTY CXX_STANDARD 14) again.
project(kacanopen C CXX)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra") # -Werror
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra") # -Werror

### settings ###

include(CMakeSettings.txt)

## Driver begin ##

if (DEFINED CAN_DRIVER_NAME)
	message(WARNING "CAN_DRIVER_NAME is deprecated. Use DRIVER instead. Overwriting DRIVER with CAN_DRIVER_NAME (${CAN_DRIVER_NAME}) now...")
	set(DRIVER "${CAN_DRIVER_NAME}")
endif()

foreach(BUILD_DRIVER ${BUILD_DRIVERS})
	# TODO: Replace with CMake 3.3 command IN_LIST
	list(FIND SUPPORTED_DRIVERS "${BUILD_DRIVER}" supported)
	if (${supported} EQUAL -1)
		message(FATAL_ERROR "Driver ${BUILD_DRIVER} does not exist, but you've specified it in BUILD_DRIVERS.")
	endif()
endforeach()

list(FIND SUPPORTED_DRIVERS "${DRIVER}" driver_supported)
if (${driver_supported} EQUAL -1)
	message(FATAL_ERROR "Driver ${DRIVER} does not exist, but you've set it as DRIVER.")
endif()

set(DRIVER_LINK_NAME can_${DRIVER})
message(STATUS "Using CAN driver ${DRIVER_LINK_NAME}")

if(${BUILD_ALL_DRIVERS})
	message(STATUS "Building all available drivers.")
endif()

## Driver end ##

if(${CMAKE_BUILD_TYPE} MATCHES "Release")
	message(STATUS "You are in release mode.")
else()
	if(${EXHAUSTIVE_DEBUGGING})
		message(STATUS "Enabled exhaustive debugging.")
		add_definitions("-DEXHAUSTIVE_DEBUGGING")
	else()
		message(STATUS "Enabled debugging.")
	endif()
endif()



### constants ###

add_definitions("-DSDO_RESPONSE_TIMEOUT_MS=${SDO_RESPONSE_TIMEOUT_MS}")
message(STATUS "SDO response timeout is set to ${SDO_RESPONSE_TIMEOUT_MS}ms")

add_definitions("-DBUSNAME=\"${BUSNAME}\"")
message(STATUS "Busname is set to ${BUSNAME} (only used by examples)")

add_definitions("-DBAUDRATE=${BAUDRATE}")
message(STATUS "Baudrate is set to ${BAUDRATE} (only used by examples)")

add_definitions("-DCONSECUTIVE_SEND_PAUSE_MS=${CONSECUTIVE_SEND_PAUSE_MS}")
message(STATUS "Pause between two consecutively sent CAN frames is set to ${CONSECUTIVE_SEND_PAUSE_MS}ms")


### dependencies ###

find_package(Threads)
find_package(Boost 1.46.1 COMPONENTS system filesystem REQUIRED) 

# Boost required for property_tree / ini_parser
if(NOT(Boost_FOUND))
	message(FATAL_ERROR "Boost > 1.46.1 not found. Boost is required for ini_parser class. This is a header-only library. You can simply download the latest boost package from http://www.boost.org/, extract it into any directory and include it using the following catkin/CMake argument: -DBOOST_ROOT=\"/path/to/boost/\"")
endif()

message(STATUS "Found Boost headers in ${Boost_INCLUDE_DIRS}")
message(STATUS "Found Boost system library: ${Boost_SYSTEM_LIBRARY}")
message(STATUS "Found Boost filesystem library: ${Boost_FILESYSTEM_LIBRARY}")
message(STATUS "Found Threads library: ${CMAKE_THREAD_LIBS_INIT}")


### ROS ###

if (NOT(${NO_ROS}))

	find_package(catkin REQUIRED COMPONENTS
		roscpp
		std_msgs
		sensor_msgs
		# message_generation
	)

	catkin_package(
                INCLUDE_DIRS core/include master/include ros_bridge/include tools/include
                LIBRARIES ${CAN_DRIVER} kacanopen_core kacanopen_master kacanopen_ros_bridge kacanopen_tools
		CATKIN_DEPENDS message_runtime
	)

endif()


### includes ###

include_directories(SYSTEM ${Boost_INCLUDE_DIRS})

if (NOT(${NO_ROS}))
	include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
endif()

include_directories(${CMAKE_CURRENT_LIST_DIR}/core/include)
include_directories(${CMAKE_CURRENT_LIST_DIR}/master/include)
include_directories(${CMAKE_CURRENT_LIST_DIR}/ros_bridge/include)
include_directories(${CMAKE_CURRENT_LIST_DIR}/tools/include)

### Installation ###

# argument 1: targets
# argument 2: if true, install ${CMAKE_CURRENT_LIST_DIR}/include/
# call this from subproject's CMake files
macro(kacanopen_install)
	message(STATUS "Configuring installation for target(s) ${ARGV0}")
	if (${NO_ROS})
		install(TARGETS ${ARGV0}
			LIBRARY DESTINATION lib
			RUNTIME DESTINATION bin
		)
		if (${ARGV1})
			install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include/
				DESTINATION include/kacanopen
			)
		endif()
	else()
		install(TARGETS ${ARGV0}
			LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
			RUNTIME DESTINATION bin
		)
		if (${ARGV1})
			install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/include
				DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
			)
			endif()
	endif()
endmacro()


# Install EDS library
message(STATUS "Configuring installation for EDS library")
add_definitions("-DSHARE_SOURCE_PATH=\"${CMAKE_CURRENT_LIST_DIR}/master/share\"")
add_definitions("-DSHARE_INSTALLED_PATH=\"${CMAKE_INSTALL_PREFIX}/share/kacanopen\"")
message(STATUS "SHARE_SOURCE_PATH set to ${CMAKE_CURRENT_LIST_DIR}/share.")
message(STATUS "SHARE_INSTALLED_PATH set to ${CMAKE_INSTALL_PREFIX}/share/kacanopen.")
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/master/share/
	DESTINATION share/kacanopen
)

### Source code ###

if (NOT(${NO_ROS}))
	add_subdirectory(ros_bridge)
endif()

add_subdirectory(drivers_lgpl)
add_subdirectory(drivers)
add_subdirectory(core)
add_subdirectory(master)
add_subdirectory(examples)
add_subdirectory(tools)


