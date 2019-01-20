cmake_minimum_required(VERSION 2.8)

message(STATUS "Configuring examples")

file(GLOB TARGETS_SRC
	"*.cpp"
	"test/*.cpp"
)

if (NOT(${NO_ROS}))
	#list(REMOVE_ITEM TARGETS_SRC "${CMAKE_CURRENT_LIST_DIR}/ros_*")
	file(GLOB TARGETS_SRC_ROS
		"ros/*.cpp"
	)
	list(APPEND TARGETS_SRC ${TARGETS_SRC_ROS})
endif()

foreach(TARGET_SRC ${TARGETS_SRC})

	get_filename_component(TARGET_NAME ${TARGET_SRC} NAME_WE)
	set(TARGET_NAME "kacanopen_example_${TARGET_NAME}")
	add_executable(${TARGET_NAME} ${TARGET_SRC})
	set_property(TARGET ${TARGET_NAME} PROPERTY CXX_STANDARD 11)
	target_link_libraries(${TARGET_NAME} kacanopen_core)
	target_link_libraries(${TARGET_NAME} kacanopen_master)
        target_link_libraries(${TARGET_NAME} kacanopen_tools)

	if (NOT(${NO_ROS}))
		target_link_libraries(${TARGET_NAME} kacanopen_ros_bridge)
	endif()
	
	list(APPEND KACANOPEN_EXAMPLES_TARGET_NAMES ${TARGET_NAME})

endforeach(TARGET_SRC ${TARGETS_SRC})

# Install
if (${INSTALL_EXAMPLES})
	kacanopen_install("${KACANOPEN_EXAMPLES_TARGET_NAMES}" FALSE)
endif()

