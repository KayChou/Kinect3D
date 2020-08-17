cmake_minimum_required(VERSION 3.5)

project(main LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set cmake prefix path to enable cmake to find freenect2
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} $ENV{HOME}/lib/freenect2/lib/cmake/freenect2)
find_package(freenect2 REQUIRED)
INCLUDE_DIRECTORIES( ${freenect2_INCLUDE_DIR} )


FIND_PACKAGE(OpenCV REQUIRED HINTS "$ENV{HOME}/lib/opencv3_4_11/share/OpenCV")

INCLUDE_DIRECTORIES("include")
INCLUDE_DIRECTORIES("/usr/include/libusb-1.0/")


set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(Qt5 COMPONENTS Widgets REQUIRED)

add_executable(main
  main.cpp
  widget.cpp
  # widget.h
  widget.ui
  KinectCapture.cpp FramePacket.cpp PlyIO.cpp utils.cpp calibration.cpp RGBD_FIFO_Process.cpp PLY_Process.cpp
)

target_link_libraries(main ${freenect2_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(main Qt5::Widgets)
target_link_libraries(main Threads::Threads)