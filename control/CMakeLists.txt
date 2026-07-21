cmake_minimum_required(VERSION 3.10)
project(drone_control)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../orchestrator)

add_executable(chase_fc
  src/chase_main.cpp
  src/chase_config.cpp
  src/crsf.cpp
  src/msp.cpp
  src/servo_pwm.cpp
  src/uart.cpp
)
target_link_libraries(chase_fc pthread)

add_executable(osd_overlay
  src/osd_overlay_main.cpp
)
target_link_libraries(osd_overlay ${OpenCV_LIBS} pthread rt)

install(TARGETS chase_fc osd_overlay RUNTIME DESTINATION bin)
