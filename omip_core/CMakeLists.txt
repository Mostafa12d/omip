cmake_minimum_required(VERSION 3.16)
project(omip_core LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --- Third-party, non-ROS dependencies -------------------------------------
# BFL (Bayesian Filtering Library) is vendored in thirdparty/ per
# PORTING_NOTES.md Phase 0/1 — verified ROS-free. lgsm (Lie-group math) was
# also vendored in Phase 1 but removed in Phase 2: it proved incompatible
# with every modern Eigen version (predates Eigen's evaluator system, see
# PORTING_NOTES.md Phase 2). Its small set of needed operations is
# reimplemented directly against modern Eigen in
# include/omip_core/LieGroup.hpp instead.
add_subdirectory(thirdparty/bfl)
# cnpy (.npy/.npz reader) is used only by the tests/ golden-fixture harness.
add_subdirectory(thirdparty/cnpy)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io registration filters segmentation)
find_package(OpenCV REQUIRED COMPONENTS core imgproc video features2d calib3d)
find_package(Boost REQUIRED COMPONENTS thread)

# --- omip_core library -------------------------------------------------
add_library(omip_core
  # Common infrastructure (omip_common in the original tree)
  src/Feature.cpp
  src/FeaturesDataBase.cpp
  src/OMIPUtils.cpp
  # Phase 2: feature_tracker
  src/feature_tracker/PointFeatureTracker.cpp
  # Phase 3: rb_tracker
  src/rb_tracker/RBFilter.cpp
  src/rb_tracker/StaticEnvironmentFilter.cpp
  src/rb_tracker/MultiRBTracker.cpp
  # Phase 4: joint_tracker
  src/joint_tracker/JointFilter.cpp
  src/joint_tracker/DisconnectedJointFilter.cpp
  src/joint_tracker/RigidJointFilter.cpp
  src/joint_tracker/PrismaticJointFilter.cpp
  src/joint_tracker/RevoluteJointFilter.cpp
  src/joint_tracker/pdf/NonLinearPrismaticMeasurementPdf.cpp
  src/joint_tracker/pdf/NonLinearRevoluteMeasurementPdf.cpp
  src/joint_tracker/JointCombinedFilter.cpp
  src/joint_tracker/MultiJointTracker.cpp
)

target_include_directories(omip_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(omip_core PUBLIC
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

# --- Phase 5: pybind11 bindings ---------------------------------------
# Optional (default ON): a machine without pybind11/Python-dev installed can
# still build and test omip_core itself; only the Python extension module
# is skipped, with a clear status message.
option(OMIP_CORE_BUILD_PYTHON_BINDINGS "Build the pybind11 Python bindings (bindings/)" ON)
if(OMIP_CORE_BUILD_PYTHON_BINDINGS)
  find_package(Python3 COMPONENTS Interpreter Development)
  find_package(pybind11 CONFIG QUIET)
  if(Python3_FOUND AND pybind11_FOUND)
    add_subdirectory(bindings)
  else()
    message(STATUS "OMIP_CORE_BUILD_PYTHON_BINDINGS is ON but Python3 dev headers and/or pybind11 were not found — skipping bindings/. Install pybind11 (e.g. `brew install pybind11`) to enable.")
  endif()
endif()
