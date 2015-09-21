# Configuracion
cmake_minimum_required(VERSION 2.8)
project(t1)
find_package(OpenCV REQUIRED)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

# t1
add_executable(conv src/src_conv/main_conv.cpp)
target_link_libraries(conv ${OpenCV_LIBS})

add_executable(histeq src/src_histeq/main_histeq.cpp)
target_link_libraries(histeq ${OpenCV_LIBS})

add_executable (median src/src_median/main_median.cpp)
target_link_libraries( median ${OpenCV_LIBS} )