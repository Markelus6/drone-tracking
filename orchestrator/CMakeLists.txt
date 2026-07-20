cmake_minimum_required(VERSION 3.10)
project(camera_orchestrator)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -Wextra -fPIC")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Shared library / SHM layout used by trackers
add_library(camera_orchestrator SHARED camera_orchestrator.cpp)
target_link_libraries(camera_orchestrator ${OpenCV_LIBS} pthread rt)

# Standalone daemon
add_executable(orch_daemon main.cpp)
target_link_libraries(orch_daemon camera_orchestrator ${OpenCV_LIBS} pthread rt)

# Install
install(TARGETS camera_orchestrator orch_daemon
        LIBRARY DESTINATION /usr/local/lib
        RUNTIME DESTINATION /usr/local/bin)
install(FILES camera_orchestrator.hpp DESTINATION /usr/local/include)
