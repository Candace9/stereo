cmake_minimum_required(VERSION 3.16)
project(d455_stereo_capture LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs highgui calib3d)
find_package(realsense2 REQUIRED)

add_executable(capture_stereo src/capture_stereo.cpp)

if(TARGET realsense2::realsense2)
  set(REALSENSE_LIB realsense2::realsense2)
elseif(DEFINED realsense2_LIBRARY)
  set(REALSENSE_LIB ${realsense2_LIBRARY})
else()
  set(REALSENSE_LIB realsense2)
endif()

target_include_directories(capture_stereo PRIVATE ${OpenCV_INCLUDE_DIRS})
if(DEFINED realsense2_INCLUDE_DIR)
  target_include_directories(capture_stereo PRIVATE ${realsense2_INCLUDE_DIR})
endif()
target_link_libraries(capture_stereo PRIVATE ${OpenCV_LIBS} ${REALSENSE_LIB})

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(capture_stereo PRIVATE -Wall -Wextra -Wpedantic)
endif()
