# Config
cmake_minimum_required(VERSION 2.8)
project(t2)
find_package(OpenCV REQUIRED)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

# t2
add_executable(filter src/filter.cpp)
target_link_libraries(filter ${OpenCV_LIBS})

add_executable(canny src/canny.cpp)
target_link_libraries(canny ${OpenCV_LIBS})