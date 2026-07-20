cmake_minimum_required(VERSION 3.10)
project(tracking_fc)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# ncnn installed
set(NCNN_DIR "/root/ncnn-install")
include_directories("${NCNN_DIR}/include")
link_directories("${NCNN_DIR}/lib")

# camera_orchestrator (SHM reader/writer)
set(ORCH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../orchestrator")
include_directories("${ORCH_DIR}")

# NanoTrack
add_executable(nanotrack_fc
  src/main.cpp
  src/nanotrack.cpp
  ${ORCH_DIR}/camera_orchestrator.cpp
)
target_link_libraries(nanotrack_fc
  ${OpenCV_LIBS}
  ncnn
  pthread
  dl
  rt
  gomp
)

# LightTrack
add_executable(lighttrack_fc
  src/lighttrack_main.cpp
  src/lighttrack.cpp
  ${ORCH_DIR}/camera_orchestrator.cpp
)
target_link_libraries(lighttrack_fc
  ${OpenCV_LIBS}
  ncnn
  pthread
  dl
  rt
  gomp
)
