# lgsm: header-only Lie-group (SE3/SO3, twists, displacements) math library.
# Vendored from the tu-rbo/omip lgsm package. Verified ROS-free in Phase 0
# (PORTING_NOTES.md, section 0.4) — its only ROS coupling was the catkin
# build wrapper, stripped here. No source changes.

find_package(Eigen3 REQUIRED)

add_library(lgsm INTERFACE)
target_include_directories(lgsm INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(lgsm INTERFACE Eigen3::Eigen)
