cmake_minimum_required(VERSION 3.16)
project(omip_core LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --- Third-party, non-ROS dependencies -------------------------------------
# lgsm (Lie-group math) and BFL (Bayesian Filtering Library) are vendored
# in thirdparty/ per PORTING_NOTES.md Phase 0/1 — both verified ROS-free.
add_subdirectory(thirdparty/lgsm)
add_subdirectory(thirdparty/bfl)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io registration filters segmentation)
find_package(OpenCV REQUIRED COMPONENTS core imgproc video features2d calib3d)
find_package(Boost REQUIRED COMPONENTS thread)

# --- omip_core library -------------------------------------------------
# Phase 1 exit criterion: build an empty (placeholder) library that links
# against every real dependency (PCL, OpenCV, Eigen, Boost, lgsm, BFL) with
# zero ROS/catkin anywhere in this file. Real estimator sources are added
# stage-by-stage starting in Phase 2.
add_library(omip_core
  src/placeholder.cpp
)

target_include_directories(omip_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(omip_core PUBLIC
  lgsm
  bfl
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  Boost::thread
)

target_include_directories(omip_core PUBLIC ${PCL_INCLUDE_DIRS})
target_compile_definitions(omip_core PUBLIC ${PCL_DEFINITIONS})

enable_testing()
add_subdirectory(tests)
