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

set(COMMON_LIBS
  ${OpenCV_LIBS}
  ncnn
  pthread
  dl
  rt
  gomp
)

# NanoTrack (production V1)
add_executable(nanotrack_fc
  src/main.cpp
  src/nanotrack.cpp
  ${ORCH_DIR}/camera_orchestrator.cpp
)
target_link_libraries(nanotrack_fc ${COMMON_LIBS})

# LightTrack
add_executable(lighttrack_fc
  src/lighttrack_main.cpp
  src/lighttrack.cpp
  ${ORCH_DIR}/camera_orchestrator.cpp
)
target_link_libraries(lighttrack_fc ${COMMON_LIBS})

# Unified comparison binary: nanov3 + OpenCV CSRT/KCF/... + DaSiam/GOTURN/CF/TCTrack-lite
add_executable(multitrack_fc
  src/multitrack_main.cpp
  src/tracker_factory.cpp
  src/opencv_legacy_tracker.cpp
  src/opencv_modern_tracker.cpp
  src/cf_ensemble_tracker.cpp
  src/nanotrack.cpp
  src/nanotrack_onnx.cpp
  src/lighttrack.cpp
  ${ORCH_DIR}/camera_orchestrator.cpp
)
target_link_libraries(multitrack_fc ${COMMON_LIBS})

# Unified benchmark binary: runs all trackers on a dataset (no SHM needed)
add_executable(benchmark_fc
  src/benchmark_main.cpp
  src/tracker_benchmark.cpp
  src/tracker_factory.cpp
  src/opencv_legacy_tracker.cpp
  src/opencv_modern_tracker.cpp
  src/cf_ensemble_tracker.cpp
  src/nanotrack.cpp
  src/nanotrack_onnx.cpp
  src/lighttrack.cpp
)
target_link_libraries(benchmark_fc ${COMMON_LIBS})
