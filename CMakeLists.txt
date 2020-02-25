cmake_minimum_required(VERSION 2.8)

project(cvpr_course)
set(CMAKE_BUILD_TYPE Relese)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

MESSAGE(STATUS "This is BINARY dir " ${cvpr_course_BINARY_DIR})
MESSAGE(STATUS "This is SOURCE dir " ${cvpr_course_SOURCE_DIR})

FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

FIND_PACKAGE(FMT REQUIRED)
INCLUDE_DIRECTORIES(${FMT_INCLUDE_DIRS})

FIND_PACKAGE(OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS"${CMAKE_EXE_LINKER_FLAGS}${OpenMP_EXE_LINKER_FLAGS}")
endif()
#FIND_PACKAGE(Glog REQUIRED)
#INCLUDE_DIRECTORIES(${GLOG_INCLUDE_DIRS})

#set(SRC_PATH src)
#ADD_SUBDIRECTORY(${SRC_PATH})

set(INC_PATH inc)
INCLUDE_DIRECTORIES(${INC_PATH})

AUX_SOURCE_DIRECTORY(src/calib calib_SRC)
SET(Calib_LIBS ${Boost_LIBRARIES}
               ${OpenCV_LIBS}
               glog
               fmt::fmt)

AUX_SOURCE_DIRECTORY(src/KLT KLT_SRC)
SET(KLT_LIBS ${Boost_LIBRARIES}
             ${OpenCV_LIBS}
             glog
             fmt::fmt)

AUX_SOURCE_DIRECTORY(src/laneline_detect laneline_SRC)
SET(laneline_LIBS ${Boost_LIBRARIES}
                  ${OpenCV_LIBS}
                  glog
                  fmt::fmt)

add_executable(camera_calib ${calib_SRC})
target_link_libraries(camera_calib ${Calib_LIBS})

add_executable(klt_node ${KLT_SRC})
target_link_libraries(klt_node ${KLT_LIBS})

add_executable(laneline ${laneline_SRC})
target_link_libraries(laneline ${laneline_LIBS})
