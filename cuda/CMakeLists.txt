project(gpu)
cmake_minimum_required(VERSION 3.5)

# packages
find_package(CUDA)

INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/include")

file(GLOB_RECURSE CURRENT_HEADERS  *.h *.hpp *.cuh)
file(GLOB CURRENT_SOURCES  *.cpp *.cu)

source_group("Include" FILES ${CURRENT_HEADERS}) 
source_group("Source" FILES ${CURRENT_SOURCES}) 

# cuda_add_library(gpu STATIC ${CURRENT_HEADERS} ${CURRENT_SOURCES})
cuda_add_library(gpu SHARED ${CURRENT_HEADERS} ${CURRENT_SOURCES})