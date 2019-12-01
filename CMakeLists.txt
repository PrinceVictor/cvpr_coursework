cmake_minimum_required(VERSION 2.8)

project(cvpr_course)
set(CMAKE_BUILD_TYPE Relese)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

MESSAGE(STATUS "This is BINARY dir " ${cvpr_course_BINARY_DIR})
MESSAGE(STATUS "This is SOURCE dir " ${cvpr_course_SOURCE_DIR})

FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${INCLUDE_PATH})
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

set(SRC_PATH src)
set(INC_PATH inc)
ADD_SUBDIRECTORY(${SRC_PATH})

#ADD_LIBRARY(cvpr SHARED ${LIBCVPR_SRC})
INCLUDE_DIRECTORIES(${INC_PATH})
add_executable(camera_calib ${SRC_PATH}/calib.cpp)
target_link_libraries(camera_calib ${OpenCV_LIBS})
