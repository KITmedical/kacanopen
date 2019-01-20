cmake_minimum_required(VERSION 2.8)

message(STATUS "Configuring Tools")

file(GLOB TARGET_SRC
	"src/*.cpp"
)

# Create library
set(TARGET_NAME kacanopen_tools)
add_library(${TARGET_NAME} SHARED ${TARGET_SRC})
set_property(TARGET ${TARGET_NAME} PROPERTY CXX_STANDARD 14)

target_link_libraries(${TARGET_NAME} ${CMAKE_DL_LIBS})
target_link_libraries(${TARGET_NAME} ${CMAKE_THREAD_LIBS_INIT})

# Install
kacanopen_install(${TARGET_NAME} TRUE)
