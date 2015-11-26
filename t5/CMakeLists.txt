# Config
cmake_minimum_required(VERSION 2.8)
project(t5)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(
	${PROJECT_SOURCE_DIR}/include
	${EIGEN3_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
)

link_directories(
	${OpenCV_LIBRARY_DIRS}
)


# t3
add_executable(t5 src/main.cpp)
target_link_libraries(t5 ${OpenCV_LIBS})