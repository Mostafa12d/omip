# BFL (Bayesian Filtering Library, Orocos) — vendored from
# https://github.com/orocos/orocos-bayesian-filtering (unmodified source
# under orocos_bfl/), per the user's decision recorded in PORTING_NOTES.md
# (Open Question Q1). BFL predates and is independent of ROS — verified
# zero ros/tf/catkin references anywhere in orocos_bfl/src during vendoring.
#
# Upstream's own CMake build is an old (CMake 2.2-era) multi-directory
# project that expects to be built standalone (its GLOBAL_ADD_INCLUDE_DIR
# calls use ${CMAKE_SOURCE_DIR}, i.e. the top of *whatever* project it's
# added to, which breaks when vendored as a subdirectory of a larger build).
# Rather than patch upstream's build scripts, this file replaces the build
# orchestration with a flat, explicit source list mirroring exactly what
# upstream's own per-directory CMakeLists.txt GLOBAL_ADD_SRC calls declare
# (see orocos_bfl/src/{wrappers/matrix,wrappers/rng,pdf,model,filter,smoother}/CMakeLists.txt).
# No .h/.cpp source content is modified — this is a build-glue-only change,
# same category of change as ROS build glue stripping elsewhere in this port.
#
# Matrix/RNG backend: "boost" (boost::numeric::ublas + boost::random), the
# same default upstream's own build uses, and consistent with Boost already
# being a hard dependency of the rest of OMIP. The matrix_NEWMAT.* and
# matrix_EIGEN.* files are still compiled (matching upstream's own
# unconditional GLOBAL_ADD_SRC list) but their bodies are entirely
# `#ifdef __MATRIXWRAPPER_{NEWMAT,EIGEN}__`-guarded and compile to empty
# translation units since only __MATRIXWRAPPER_BOOST__ is defined below.

set(BFL_SRC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/orocos_bfl/src")

# Generate wrappers/config.h from upstream's own config.h.in. Must land at
# orocos_bfl/src/wrappers/config.h (not a separate build-dir path) because
# every wrapper source does a relative `#include "../config.h"`, which
# resolves relative to the including file's own directory regardless of
# any -I include path.
set(__MATRIXWRAPPER_NEWMAT__ OFF)
set(__MATRIXWRAPPER_BOOST__ ON)
set(__MATRIXWRAPPER_EIGEN__ OFF)
set(__RNGWRAPPER_BOOST__ ON)
configure_file(
  "${BFL_SRC_ROOT}/wrappers/config.h.in"
  "${BFL_SRC_ROOT}/wrappers/config.h"
  @ONLY
)

find_package(Boost REQUIRED)

add_library(bfl STATIC
  ${BFL_SRC_ROOT}/wrappers/matrix/matrix_wrapper.cpp
  ${BFL_SRC_ROOT}/wrappers/matrix/matrix_BOOST.cpp
  ${BFL_SRC_ROOT}/wrappers/matrix/vector_BOOST.cpp
  ${BFL_SRC_ROOT}/wrappers/matrix/matrix_NEWMAT.cpp
  ${BFL_SRC_ROOT}/wrappers/matrix/vector_NEWMAT.cpp
  ${BFL_SRC_ROOT}/wrappers/matrix/matrix_EIGEN.cpp
  ${BFL_SRC_ROOT}/wrappers/matrix/vector_EIGEN.cpp
  ${BFL_SRC_ROOT}/wrappers/rng/rng.cpp
  ${BFL_SRC_ROOT}/pdf/analyticconditionalgaussian.cpp
  ${BFL_SRC_ROOT}/pdf/conditionalgaussian.cpp
  ${BFL_SRC_ROOT}/pdf/gaussian.cpp
  ${BFL_SRC_ROOT}/pdf/uniform.cpp
  ${BFL_SRC_ROOT}/pdf/conditionalgaussian_additivenoise.cpp
  ${BFL_SRC_ROOT}/pdf/analyticconditionalgaussian_additivenoise.cpp
  ${BFL_SRC_ROOT}/pdf/linearanalyticconditionalgaussian.cpp
  ${BFL_SRC_ROOT}/pdf/filterproposaldensity.cpp
  ${BFL_SRC_ROOT}/pdf/EKF_proposaldensity.cpp
  ${BFL_SRC_ROOT}/pdf/discretepdf.cpp
  ${BFL_SRC_ROOT}/pdf/discreteconditionalpdf.cpp
  ${BFL_SRC_ROOT}/model/analyticmeasurementmodel_gaussianuncertainty.cpp
  ${BFL_SRC_ROOT}/model/analyticsystemmodel_gaussianuncertainty.cpp
  ${BFL_SRC_ROOT}/model/linearanalyticsystemmodel_gaussianuncertainty.cpp
  ${BFL_SRC_ROOT}/model/linearanalyticmeasurementmodel_gaussianuncertainty.cpp
  ${BFL_SRC_ROOT}/model/discretesystemmodel.cpp
  ${BFL_SRC_ROOT}/model/linearanalyticmeasurementmodel_gaussianuncertainty_implicit.cpp
  ${BFL_SRC_ROOT}/filter/iteratedextendedkalmanfilter.cpp
  ${BFL_SRC_ROOT}/filter/kalmanfilter.cpp
  ${BFL_SRC_ROOT}/filter/extendedkalmanfilter.cpp
  ${BFL_SRC_ROOT}/filter/EKparticlefilter.cpp
  ${BFL_SRC_ROOT}/filter/SRiteratedextendedkalmanfilter.cpp
  ${BFL_SRC_ROOT}/filter/innovationCheck.cpp
  ${BFL_SRC_ROOT}/smoother/rauchtungstriebel.cpp
)
target_include_directories(bfl PUBLIC
  $<BUILD_INTERFACE:${BFL_SRC_ROOT}>
)
target_link_libraries(bfl PUBLIC Boost::boost)
target_compile_features(bfl PUBLIC cxx_std_11)
