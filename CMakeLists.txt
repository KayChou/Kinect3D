cmake_minimum_required(VERSION 3.5)

project(main LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "  ${CMAKE_CXX_FLAGS} -std=c++11 -O3 -fopenmp")

# Set cmake prefix path to enable cmake to find freenect2
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} $ENV{HOME}/lib/freenect2-0.2.1/lib/cmake/freenect2)
find_package(freenect2 REQUIRED)
INCLUDE_DIRECTORIES( ${freenect2_INCLUDE_DIR} )


FIND_PACKAGE(OpenCV REQUIRED HINTS "$ENV{HOME}/lib/opencv3_4_11/share/OpenCV")

INCLUDE_DIRECTORIES("include")
INCLUDE_DIRECTORIES("/usr/include/libusb-1.0/")

set(OpenGL_GL_PREFERENCE LEGACY)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

file(GLOB SOURCES "${PROJECT_SOURCE_DIR}/src/*.cpp")


add_executable(main ${SOURCES})

target_link_libraries(main ${freenect2_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(main Threads::Threads)
target_link_libraries(main glfw)
TARGET_LINK_LIBRARIES(main dl)