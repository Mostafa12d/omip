# OMIP ROS-Removal Porting Notes

This log is appended to throughout the port. It records: the type-translation
table (Phase 0), every ROS type replaced and its replacement, every open
question raised to the user and its resolution, and every judgment call made
during the port. See `omip-ros-removal-agent-prompt.md` for the full mission
brief and phase plan.

Status legend: `[ ]` not started, `[~]` in progress / ported-unvalidated, `[x]` done+validated.

---

## Phase 0 — Inventory and type design

Status: **in progress**. Inventory below is complete for `feature_tracker`,
`omip_common`, `rb_tracker`, `joint_tracker`, `lgsm`, and the BFL dependency.
`shape_tracker`/`shape_reconstruction` were scouted at a lighter level since
they are optional Phase 7 work. No code has been written yet.

### 0.1 Package dependency overview (from CMakeLists.txt `find_package(catkin COMPONENTS ...)`)

| Package | catkin components (ROS-only, to be dropped) | Non-ROS deps kept |
|---|---|---|
| `omip_common` | (base catkin) | `bfl` (external, see 0.5), `Boost` (signals, thread) |
| `feature_tracker` | `lgsm`, `dynamic_reconfigure`, `roscpp`, `omip_common`, `omip_msgs`, `std_msgs`, `geometry_msgs`, `sensor_msgs`, `image_transport`, `cv_bridge`, (+camera_info_manager, message_filters transitively) | `Boost` (signals, thread), `OpenCV` (video) |
| `rb_tracker` | catkin ROS components + `omip_common`, `omip_msgs` | `bfl` (unused in practice, see 0.5), `Boost` |
| `joint_tracker` | catkin ROS components + `omip_common`, `omip_msgs` | `bfl` (actually used), `Boost` |
| `shape_tracker` | ROS components + `omip_common`, `omip_msgs` | — |
| `shape_reconstruction` | ROS components + `omip_common`, `omip_msgs`, vendored `vcglib` | `libpointmatcher` (`omip/third_party/libpointmatcherConfig.cmake`) |
| `lgsm` | `catkin` (buildtool only, no ROS components) | `Eigen3` only |

All of `roscpp`, `dynamic_reconfigure`, `image_transport`, `cv_bridge`,
`camera_info_manager`, `message_filters`, `tf`/`tf2`, `pcl_ros`, `rosbag`,
`std_msgs`/`geometry_msgs`/`sensor_msgs`/`visualization_msgs`, and `omip_msgs`
are to be fully removed from `omip_core`. `bfl` is **not** a ROS package per
se (see 0.5) — it needs to be re-sourced as a standalone dependency, not
removed.

### 0.2 `omip_msgs` → `types.hpp` translation table

Fetched directly from `https://github.com/tu-rbo/omip_msgs` (not vendored in
this checkout) since it's a required sibling repo for Phase 0 inventory.

| omip_msgs type | Fields | Plain struct replacement (`omip_core/include/omip_core/types.hpp`) |
|---|---|---|
| `JointMsg` | `parent_rb_id`, `child_rb_id`, `most_likely_joint` (int32); prismatic: `prism_probability`(f32), `prism_position`(Point), `prism_ori_phi/theta`(f64), `prism_ori_cov[4]`(f64), `prism_orientation`(Vector3), `prism_joint_value`(f64); revolute: `rev_probability`(f32), `rev_position`(Point), `rev_position_uncertainty[9]`(f64), `rev_ori_phi/theta`(f64), `rev_ori_cov[4]`(f64), `rev_orientation`(Vector3), `rev_joint_value`(f64); `discon_probability`(f32); `rigid_probability`(f32) | `struct JointModel` — one struct per joint type (`RigidJointModel`, `PrismaticJointModel`, `RevoluteJointModel`, `DisconnectedJointModel`) each with `Eigen::Vector3d position`, `Eigen::Vector3d axis` (unit orientation), `Eigen::Matrix2d orientation_cov` (phi/theta cov, was `[4]`), `Eigen::Matrix3d position_cov` (was `[9]`, revolute only), `double joint_value`, `float probability`. Parent/child ids as `RB_id_t`. |
| `KinematicStructureMsg` | `Header header`; `JointMsg[] kinematic_structure` | `struct KinematicStructure { double timestamp_ns; std::vector<JointModel> joints; }` — this is essentially the `KinematicModel` typedef already used internally (see 0.3), just needs a serializable/plain form for the Python binding boundary. |
| `RigidBodyBrakingEvent` | `Header header`; `int32[] braking_rb_id` | `struct RigidBodyBrakingEvent { double timestamp_ns; std::vector<RB_id_t> braking_rb_ids; }` — **grep shows this message type is never actually referenced anywhere in the ported packages' C++ source** (feature_tracker/rb_tracker/joint_tracker/shape_tracker/shape_reconstruction). Likely dead/unused message from an earlier iteration. Flag: port the struct for completeness but don't wire it into any stage unless a fixture references it. |
| `RigidBodyPoseAndVelMsg` | `rb_id`(int32); `pose_wc`(`geometry_msgs/TwistWithCovariance`) — **note: field is named `pose_wc` but its ROS type is `TwistWithCovariance`, not `PoseWithCovariance`; this looks like a naming inconsistency in the original message, preserved as-is upstream** — see judgment call J1; `velocity_wc`(TwistWithCovariance); `centroid`(Point) | `struct RigidBodyPoseAndVel { RB_id_t rb_id; PoseWithCovariance pose_wc; TwistWithCovariance velocity_wc; Eigen::Vector3d centroid; }` — kept the field name `pose_wc` for auditability even though its C++ type here is really a pose (see J1); see 0.3 for `PoseWithCovariance`/`TwistWithCovariance` struct definitions. |
| `RigidBodyPoseMsg` | `rb_id`(int32); `pose_wc`(`geometry_msgs/PoseWithCovariance`); `centroid`(Point) | `struct RigidBodyPose { RB_id_t rb_id; PoseWithCovariance pose_wc; Eigen::Vector3d centroid; }` |
| `RigidBodyPosesAndVelsMsg` | `Header header`; `RigidBodyPoseAndVelMsg[] rb_poses_and_vels` | `struct RigidBodyPosesAndVels { double timestamp_ns; std::vector<RigidBodyPoseAndVel> rb_poses_and_vels; }` — this is `rbt_state_t`/`ks_measurement_t` in `OMIPTypeDefs.h`, i.e. the **Filter-internal state type for rb_tracker and measurement type for joint_tracker**, not just a wire message. High priority: this struct becomes the actual internal representation, replacing `omip_msgs::RigidBodyPosesAndVelsMsg` used directly inside `MultiRBTracker`/`MultiJointTracker`. |
| `RigidBodyPosesMsg` | `Header header`; `RigidBodyPoseMsg[] rb_poses` | `struct RigidBodyPoses { double timestamp_ns; std::vector<RigidBodyPose> rb_poses; }` |
| `RigidBodyTwistWithCovMsg` | `rb_id`(int32); `twist_wc`(TwistWithCovariance) | `struct RigidBodyTwistWithCov { RB_id_t rb_id; TwistWithCovariance twist_wc; }` |
| `RigidBodyTwistsWithCovMsg` | `Header header`; `RigidBodyTwistWithCovMsg[] twists_wc` | `struct RigidBodyTwistsWithCov { double timestamp_ns; std::vector<RigidBodyTwistWithCov> twists_wc; }` |
| `ShapeModel` | `rb_id`(int32); `rb_shape_model`(`sensor_msgs/PointCloud2`) | `struct ShapeModel { RB_id_t rb_id; PointCloudPCL::Ptr rb_shape_model; }` (Phase 7) |
| `ShapeModels` | `Header header`; `ShapeModel[] rb_shape_models` | `struct ShapeModels { double timestamp_ns; std::vector<ShapeModel> rb_shape_models; }` (Phase 7) |
| `ShapeTrackerState` | `rb_id`(int32); `pose_wc`(TwistWithCovariance, commented-out alt field `RigidBodyPoseMsg pose`); `number_of_points_of_model`(int32); `number_of_points_of_current_pc`(int32); `probabilistic_value`(f32); `fitness_score`(f32) | `struct ShapeTrackerState { RB_id_t rb_id; TwistWithCovariance pose_wc; int num_points_of_model; int num_points_of_current_pc; float probabilistic_value; float fitness_score; }` (Phase 7) |
| `ShapeTrackerStates` | `Header header`; `ShapeTrackerState[] shape_tracker_states` | `struct ShapeTrackerStates { double timestamp_ns; std::vector<ShapeTrackerState> shape_tracker_states; }` (Phase 7) |

### 0.3 Core ROS geometry/sensor type → Eigen/PCL translation table

| ROS type | Where used in Filter-class public interfaces (not just Nodes) | Replacement |
|---|---|---|
| `geometry_msgs::Twist` | `OMIPUtils.{h,cpp}`: `TransformLocation(..., const geometry_msgs::Twist&, ...)`, `GeometryMsgsTwist2EigenTwist`, `EigenTwist2GeometryMsgsTwist`, `ROSTwist2EigenTwist` | Drop entirely — these become plain `Eigen::Twistd` (lgsm) overloads / the ROS-typed overloads are deleted, not translated, since `Eigen::Twistd` already exists and is the "real" internal type. |
| `geometry_msgs::TwistWithCovariance` | `RBFilter::integrateShapeBasedPose`, `RBFilter::getPoseECWithCovariance`/`getVelocityWithCovariance` (return `TwistWithCovariancePtr`), `JointFilter::getPredictedSRB{Pose,DeltaPose,Velocity}WithCovInSensorFrame()` (all 4 joint subtypes), `RigidBodyPoseAndVelMsg::pose_wc`/`velocity_wc`, `RigidBodyTwistWithCovMsg::twist_wc`, `ShapeTrackerState::pose_wc` | `struct TwistWithCovariance { Eigen::Twistd twist; Eigen::Matrix<double,6,6> covariance; }` (row-major layout matching ROS's flattened `float64[36]`, indices `[6*i+j]`) |
| `geometry_msgs::PoseWithCovariance` | `RBFilter::getPoseWithCovariance()` (returns `PoseWithCovariancePtr`), `RigidBodyPoseMsg::pose_wc` | `struct PoseWithCovariance { Eigen::Isometry3d pose; Eigen::Matrix<double,6,6> covariance; }` |
| `geometry_msgs::Pose` / `geometry_msgs::Point` / `geometry_msgs::Vector3` | Various (`JointMsg::prism_position`, `rev_position`, `prism_orientation`, `rev_orientation`, `RigidBodyPoseAndVelMsg::centroid`) | `Eigen::Vector3d` (Point/Vector3), `Eigen::Isometry3d` (Pose) |
| `sensor_msgs::PointCloud2` | `ft_state_ros_t`, `rbt_measurement_ros_t` (Node-layer wire types only — **not** used inside `FeatureTracker`/`PointFeatureTracker`/`MultiRBTracker`'s actual algorithm bodies beyond the Node boundary and `pcl_conversions` timestamp shim) | Not needed at all in `omip_core` — the orchestrator constructs `pcl::PointCloud<...>` directly (e.g. from MuJoCo depth→cloud conversion); no `PointCloud2` serialization step exists in a single-process pipeline. |
| `sensor_msgs::Image` + `cv_bridge::CvImagePtr` | `FeatureTracker`/`PointFeatureTracker`: `ft_measurement_t = std::pair<CvImagePtr,CvImagePtr>`, `setOcclusionMaskImg`, `getRGBImg`/`getDepthImg`/`getTrackedFeaturesImg`/etc. (7+ methods) | `cv_bridge::CvImagePtr` → plain `cv::Mat` (or a small `struct StampedImage { cv::Mat image; double timestamp_ns; }` if the header/stamp is load-bearing — see OPEN QUESTION Q3). `ft_measurement_t` becomes `std::pair<cv::Mat, cv::Mat>` (rgb, depth). |
| `sensor_msgs::CameraInfo` | `FeatureTracker::setCameraInfoMsg(const sensor_msgs::CameraInfo*)`, `ShapeTracker::setCameraInfo`, `ShapeReconstruction::setCameraInfo` | `struct CameraIntrinsics { double fx, fy, cx, cy; int width, height; }` (the only fields actually read are the projection-matrix components: verified by checking `PointFeatureTracker::_image_plane_proj_mat_eigen` construction — full `CameraInfo` (distortion model, ROI, binning) is not used, only the intrinsic matrix). |
| `visualization_msgs::Marker` / `MarkerArray` | `JointFilter::getJointMarkersInRRBFrame()` (pure virtual, all 4 subtypes implement it); consumed only by `MultiJointTrackerNode` | Per the mission brief: **drop from core entirely**. Replace with `struct DebugGeometry { enum Kind { ARROW, SPHERE, TEXT, MESH } kind; Eigen::Vector3d start, end; double scale; std::string text; std::string mesh_uri; Eigen::Vector4d color_rgba; }` and `virtual std::vector<DebugGeometry> getJointDebugGeometryInRRBFrame() const` — a *caller* (MuJoCo viewer, matplotlib) renders it however it likes. Marker mesh_resource `"package://joint_tracker/meshes/cone.stl"` (uncertainty cones, prismatic+revolute) becomes a plain asset path/kind flag in `DebugGeometry`, resolved by the caller, not the core. |
| `std_msgs::Header` (`frame_id` + `stamp`) | Pervasive on Node-published messages; inside Filter classes only as `_state.header.stamp` bookkeeping in a few places | `frame_id` is **not load-bearing inside any Filter class** — verified: every `frame_id` assignment found (`"camera_rgb_optical_frame"`, `"static_environment"`, `"ip/rb"+id`) happens in Node `_publish*()` methods for TF/RViz purposes only, never read back by a Filter to affect computation. Replace with plain `double timestamp_ns` field where a struct needs one; drop `frame_id` entirely from `omip_core` types. The orchestrator (not the core) is responsible for knowing which body/camera a given transform is expressed in, since it owns all frames explicitly (see tf/tf2 row below). |
| ROS logging (`ROS_INFO`/`WARN`/`ERROR`/`DEBUG` [+`_STREAM`/`_NAMED` variants]) | Used throughout for diagnostics; **zero instances found where a log call has control-flow side effects**, except `pdf/NonLinear{Prismatic,Revolute}MeasurementPdf.cpp`'s `dfGet()` invalid-index branch, which logs *and* calls `exit(BFL_ERRMISUSE)` | Small logging shim: `omip_core/include/omip_core/Log.h` with macros `OMIP_INFO(...)`, `OMIP_WARN(...)`, `OMIP_ERROR(...)`, `OMIP_DEBUG(...)` writing to stderr (or wrapping spdlog if added as a dep) so call sites need only an include+macro-name swap. The `exit(BFL_ERRMISUSE)` call sites become a thrown `std::runtime_error` instead of `exit()` (a process-killing `exit()` inside a library is never appropriate — see judgment call J2). |
| `ros::NodeHandle` param reads (`nh.param(...)`, `getROSParameter<T>`, dynamic_reconfigure `.cfg`) | One param struct per stage, all-scalar (`int`/`double`/`bool`/`string`) — full field lists captured in the per-package sections below | `struct FeatureTrackerConfig`, `struct RBTrackerConfig`, `struct JointTrackerConfig` (one per stage) populated from YAML via `yaml-cpp`, loaded once at construction — field lists directly ported from the existing `cfg/*.yaml` files (same names/defaults, see per-package tables below). `feature_tracker`'s and `rb_tracker`'s `dynamic_reconfigure` `.cfg` fields (live-tunable booleans, mostly "publish this debug image y/n") collapse into the same config struct since there's no live-reconfigure server in a single-process library — they just become normal (still-mutable-at-runtime-if-desired) config fields. `joint_tracker_dyn_rec.cfg` is empty (0 bytes) — no fields to port there. |
| `message_filters::TimeSynchronizer` / `sync_policies::ApproximateTime` | `FeatureTrackerNode` (PC+depth+RGB, or depth+RGB "light" sync; queue size 1 for bag mode, 10 for live) | **Confirmed pure infrastructure, not algorithm-relevant**: this exists solely to time-align independently-arriving ROS topics from real sensor hardware/rosbags. In a MuJoCo sim, one physics/render step produces RGB+depth+cloud already perfectly time-aligned in a single call — there is nothing to synchronize. Dropped entirely; the orchestrator just calls `FeatureTracker::step(frame)` once per sim tick with all three already bundled. **Also found:** `rb_tracker` and `joint_tracker` Node headers `#include` `message_filters/subscriber.h` + `time_synchronizer.h` but **never actually instantiate a synchronizer** (plain single-topic `ros::Subscriber` used instead) — those are dead/vestigial includes, not a real behavior to preserve. |
| `tf`/`tf2` (`tf::Transform`, `tf::TransformBroadcaster`, `tf::TransformListener`) | `StaticEnvironmentFilter::estimateDeltaMotion`/`iterativeICP` (public, `tf::Transform&` out-param); `RBFilter.h`/`JointFilter.h`/`JointCombinedFilter.h` all `#include` tf headers and declare `tf::TransformBroadcaster`/`TransformListener` members that are **verified dead** (never called) except the two free functions `Eigen2Tf(Eigen::Matrix4{f,d})` in `StaticEnvironmentFilter.cpp` (pure math, no ROS master interaction) | `tf::Transform` → `Eigen::Isometry3d` everywhere (the `Eigen2Tf` helpers become trivial/unneeded once the type itself is Eigen). All *live* frame transforms found are: (1) camera↔RB pose broadcast (`"camera_rgb_optical_frame"` → `"ip/rb"+id`/`"static_environment"`, TF-broadcast only, not consumed by any Filter) — Node-only, drop; (2) one **dead/commented-out** `"/base_link"`↔`"/camera_rgb_optical_frame"` lookup in `StaticEnvironmentFilter::constrainMotion()`'s never-finished `ROBOT_XY_BASELINK_PLANE` branch — not implemented in the original either, so nothing to port; the orchestrator will own and pass explicit `Eigen::Isometry3d` camera/robot transforms per the mission brief, but there is no *existing* frame-convention behavior in the original code that depends on a live tf tree — everything computed inside the Filters is already expressed directly in the camera/sensor frame. |
| `omip_msgs::*` used as internal Filter state (not just wire format) | `rbt_state_t = ks_measurement_t = omip_msgs::RigidBodyPosesAndVelsMsg` (`OMIPTypeDefs.h`); `MultiJointTracker`'s `_last_rcvd_poses_and_vels`/`_previous_rcvd_poses_and_vels` members typed as `omip_msgs::RigidBodyPosesAndVelsMsgPtr`; `joint_measurement_t = std::pair<omip_msgs::RigidBodyPoseAndVelMsg, omip_msgs::RigidBodyPoseAndVelMsg>` | This is the **single most important finding of Phase 0**, exactly as the mission brief predicted: the ROS message type IS the Filter's belief-state representation, not a translation-boundary artifact. All four typedefs become the plain-struct equivalents from table 0.2 (`RigidBodyPosesAndVels`, etc.) — a type substitution, not a redesign; every field access (`.rb_id`, `.pose_wc.twist.linear.x`, `.covariance[6*i+j]`, ...) maps 1:1 to the same field on the plain struct. |

### 0.4 `lgsm` — verified ROS-free

Read all 15 headers + the `Lgsm` umbrella header in full. Zero `#include`
of any ROS header anywhere in the library; the *only* ROS coupling is the
catkin build wrapper (`find_package(catkin)` + `catkin_package()` in
`CMakeLists.txt`, `buildtool_depend>catkin` in `package.xml`). All real
includes are `Eigen/Core`, `Eigen/Geometry`, `Eigen/StdVector`. **Action:**
vendor as-is into `omip_core/thirdparty/lgsm/`, replace its `CMakeLists.txt`
with a plain one that just does `find_package(Eigen3 REQUIRED)` — no source
changes needed.

### 0.5 BFL (Bayesian Filtering Library) — external dependency, not vendored, not actually ROS-specific

- **Not vendored in this repo.** Referenced only via `omip/third_party/bflConfig.cmake`,
  a catkin-generated package-config stub that resolves BFL relative to
  `$ENV{ROS_ROOT}` (expects the ROS-Indigo-era `ros-indigo-bfl` apt package,
  linking `liborocos-bfl.so` from the ROS install's `lib/`) and additionally
  contains a hardcoded absolute path from the original author's machine
  (`/home/roberto/Libraries/bfl`) — neither is usable outside a ROS install.
- **BFL itself is a standalone, general-purpose C++ library** (Orocos
  Bayesian Filtering Library — Kalman/particle filters, predates and is
  independent of ROS; ROS-Indigo just happened to package a binary of it).
  It is **not** part of core ROS message/transport plumbing, so "removing
  ROS" does not mean removing BFL — it means re-sourcing it as a normal
  standalone C++ dependency (its own upstream repo/build, or a system
  package) instead of resolving it through `$ROS_ROOT`.
- **Actual usage is narrower than the `find_package(bfl REQUIRED)` in three
  packages' CMakeLists.txt suggests:**
  - `joint_tracker`: **real usage.** `PrismaticJointFilter`/`RevoluteJointFilter`
    and their `pdf/NonLinear{Prismatic,Revolute}MeasurementPdf` helper classes
    use `BFL::ExtendedKalmanFilter`, `BFL::LinearAnalyticConditionalGaussian`,
    `BFL::LinearAnalyticSystemModelGaussianUncertainty`,
    `BFL::AnalyticMeasurementModelGaussianUncertainty`, `BFL::Gaussian`,
    `BFL::AnalyticConditionalGaussianAdditiveNoise` (subclassed), and
    `MatrixWrapper::{Matrix,ColumnVector,SymmetricMatrix}` (BFL's own
    linear-algebra types, 1-indexed, distinct from Eigen) throughout the EKF
    predict/correct steps. This is real, load-bearing BFL usage that must be
    preserved.
  - `rb_tracker`: **`using namespace BFL;`/`MatrixWrapper` appear in
    `RBFilter.cpp`/`StaticEnvironmentFilter.cpp` but grep confirms zero actual
    `BFL::`/`MatrixWrapper::` symbols are referenced in either file** — the
    RBT's EKF is hand-rolled with raw Eigen matrices, not BFL. These are dead
    using-directives/includes (there's commented-out legacy code referencing
    `_ekf_velocity`/`_ekf_brake` suggesting an earlier BFL-based
    implementation existed and was later replaced by the Eigen version, but
    the code as it stands today has no real BFL dependency in `rb_tracker`).
  - `omip_common`: `OMIPUtils.h` includes `wrappers/matrix/matrix_wrapper.h`
    for the `MatrixWrapper::{Matrix,ColumnVector}` types used in
    `LocationOfFeature2ColumnVector(Homogeneous)`/`invert3x3Matrix` — real but
    minimal/mechanical usage (just matrix conversion utilities), easily
    portable to plain Eigen instead if desired, or kept on BFL's matrix
    wrapper types if BFL is vendored anyway for joint_tracker's sake.
  - `feature_tracker`, `shape_tracker`, `shape_reconstruction`: **no BFL
    usage found at all.**
- `joint_tracker/src/pdf/` contains two OMIP-authored classes injected into
  the `BFL` namespace (`NonLinearPrismaticMeasurementPdf`,
  `NonLinearRevoluteMeasurementPdf`, subclassing
  `BFL::AnalyticConditionalGaussianAdditiveNoise`) implementing the nonlinear
  joint measurement models (`ExpectedValueGet()`/`dfGet()` analytic
  Jacobian) — these are the two files most critical to port bit-for-bit for
  numerical fidelity of the revolute/prismatic EKFs. Their `.cpp` files
  `#include <ros/ros.h>` **only** for a `ROS_ERROR_STREAM_NAMED` +
  `exit(BFL_ERRMISUSE)` in the invalid-index branch of `dfGet()` — trivial to
  swap for the logging shim + exception (see J2).
- **OPEN QUESTION Q1 (needs your decision before Phase 1 CMake setup):** see
  Open Questions section below — how should BFL be sourced for the ROS-free
  build?

---

## Open questions raised to the user

### Q1 — How should BFL be sourced for the ROS-free build?

BFL is a real, load-bearing dependency for `joint_tracker`'s revolute/prismatic
EKFs (see 0.5), but this repo doesn't vendor it and the existing
`bflConfig.cmake` only resolves it via a ROS install's `$ROS_ROOT`. Options,
roughly in order of fidelity-to-original vs. effort:

- **(a)** Vendor upstream Orocos BFL source directly into
  `omip_core/thirdparty/bfl/` (github.com/orocos/orocos-bayesian-filtering)
  and build it as a plain CMake subproject. Preserves the exact original math
  library bit-for-bit; more build-system work, and BFL's own CMake is old
  (autotools/scons era in places) and may itself need de-ROS-ifying (it may
  have zero ROS coupling since it predates ROS — needs a quick check once we
  fetch it, matching how we verified `lgsm`).
  Highest fidelity, more Phase 1 setup effort.
- **(b)** Install a standalone system/apt build of BFL (if a ROS-independent
  package/build exists for your OS) and just `find_package`/link it normally,
  dropping the ROS-Indigo-specific `bflConfig.cmake` shim.
  Same fidelity as (a) if it's the same upstream version, less setup effort,
  contingent on such a package being available/buildable on your machine.
- **(c)** Reimplement only the two joint EKFs' predict/correct math directly
  with Eigen (no BFL at all), since BFL usage is entirely confined to
  `PrismaticJointFilter`/`RevoluteJointFilter`/their two `pdf/` helper
  classes, and `rb_tracker`'s EKF is already hand-rolled Eigen with no real
  BFL dependency (per 0.5) — meaning the *only* remaining BFL usage anywhere
  in the codebase would be removed, and `omip_core` would have zero BFL
  dependency at all.
  This is explicitly **against ground rule 1 ("preserve the recursive
  estimator logic... must be preserved as closely as possible to the
  original... not a research reimplementation")** unless you tell me
  otherwise, since it changes the exact numerical machinery (BFL's own linear
  algebra / EKF bookkeeping) even if the intended math is equivalent — flagging
  it here only because it is the *simplest* option, not because it's
  recommended.

**My recommendation (not a decision):** (a), vendoring upstream BFL source,
best matches ground rule 4/5 (preserve names, no silent numerical changes) at
acceptable cost, since BFL is small and Phase 1 (build skeleton) is exactly
the phase where this vendoring work belongs. But this is your call — please
confirm before I set up `omip_core/thirdparty/` in Phase 1.

**Resolution:** Confirmed — option (a). User: "This is the nonros package,
do with it as you wish. https://github.com/orocos/orocos-bayesian-filtering"
(2026-07-22). Vendor upstream Orocos BFL source from that repo into
`omip_core/thirdparty/bfl/` in Phase 1; verify its own ROS-freedom the same
way `lgsm` was verified (0.4) before assuming it needs no source changes.

### Q2 — Golden fixture exchange format

The mission brief asks me to agree on a format for the golden reference data
you'll export from the ROS Indigo Docker container (CSV/JSON/npz). Given the
data involved (point clouds with 3D+label fields for feature_tracker,
pose+twist+6x6 covariance matrices for rb_tracker, joint parameters +
covariances for joint_tracker), I'd suggest **npz** (one `.npz` per
stage-per-timestep, or one `.npz` per stage with a leading time/frame-index
axis on every array) since it round-trips arbitrary-shaped float arrays
without a custom parser, and is trivial to read from a C++ test harness via
`cnpy` (small header-only-ish dep) or by having the fixture-generation script
also emit a tiny raw-binary sidecar. CSV/JSON would need per-message-type
custom (de)serialization code in the C++ test harness for comparatively
little benefit. Happy to go with CSV/JSON instead if that's easier on your
Docker-side export script.

**Resolution:** Confirmed — npz (2026-07-22). Test harness in `tests/` will
load `.npz` fixtures (via `cnpy` or equivalent) once they arrive from the
Docker-side export.

### Q3 — Is `std_msgs::Header`'s `frame_id` genuinely unused inside Filter logic, or did I miss a spot?

I traced every `header.frame_id` assignment in `feature_tracker`, `rb_tracker`,
`joint_tracker` and found all of them happen only in `*Node` publish methods
(TF broadcast / RViz), never read back inside a Filter class to affect a
computation. If that matches your understanding of the code, I'll drop
`frame_id` entirely from `omip_core` types (per the mission brief's own
suggestion to check whether it's "load-bearing... or just ROS boilerplate").
Flagging since a wrong guess here would be a silent behavior change that's
easy to miss in review.

**Resolution:** Not yet confirmed by the user. Proceeding on the traced
evidence above (dropping `frame_id` from `omip_core` types) since it's a
low-risk, well-evidenced call and revisiting it later is cheap — but flagging
here so it gets a second look in review rather than silently assuming it's
right.

---

## Judgment calls made so far

- **J1 —** `omip_msgs::RigidBodyPoseAndVelMsg.pose_wc` is typed as
  `geometry_msgs/TwistWithCovariance`, not `PoseWithCovariance`, despite the
  field name — this looks like a naming bug/legacy artifact in the original
  message definition (a `RigidBodyPoseMsg.pose_wc`, by contrast, correctly
  uses `PoseWithCovariance`). Since ground rule 4 says preserve names for
  auditability, I'm keeping the field named `pose_wc` in the ported struct
  even though its C++ type is a twist/pose representation — the point is a
  1:1 audit trail against the original, not a "fixed" name. Called out here
  so it isn't mistaken for a porting mistake later.
- **J2 —** The `exit(BFL_ERRMISUSE)` calls in
  `pdf/NonLinear{Prismatic,Revolute}MeasurementPdf.cpp::dfGet()`'s invalid-index
  branch will become a thrown `std::out_of_range` (or similar) instead of a
  hard process exit, since a library must never unilaterally kill the host
  process. This is a behavior change on an error path only (never hit in
  correct usage — `dfGet(i)` is only called by BFL's own EKF internals with a
  valid conditional-argument index), not on any numerically-meaningful path.
- **J3 —** Marker/visualization code (`visualization_msgs::Marker` in all 4
  `JointFilter` subtypes' `getJointMarkersInRRBFrame()`) is being replaced
  with a generic `DebugGeometry` struct per the mission brief, rather than
  ported faithfully field-by-field — this is presentation-only code with no
  bearing on the estimation math, so "preserve as closely as possible"
  (ground rule 1) is being read as applying to the *estimator*, not the debug
  viz. Flagging since it's still a "replace this ROS-only feature" judgment
  call per ground rule 5.
- **J4 —** Dead/vestigial ROS code identified during inventory (not ported at
  all, since there's nothing to preserve): `rb_tracker`'s unused
  `message_filters`/`visualization_msgs`/`rosbag`/`std_msgs::Float64` includes
  and the never-subscribed `_matthias_refinements_subscriber` /
  never-defined `MatthiasRefinementsCallback` / never-defined
  `RGBDPCCallback` in `MultiRBTrackerNode`; `joint_tracker`'s unused
  `message_filters`/`std_srvs::Empty` includes and dead `tf::TransformBroadcaster _tf_pub`
  member on `JointFilter`; `shape_reconstruction`/`shape_tracker`'s unused
  `dynamic_reconfigure::Server` includes (cfg yamls are read as plain
  rosparam, no autogenerated Config classes are ever instantiated). None of
  this reflects real behavior to reproduce.

---

## Package-level ROS type/param field inventories (supporting detail for 0.1-0.3)

<details>
<summary><b>feature_tracker</b></summary>

- Filter classes: `FeatureTracker` (abstract base, `omip_common/RecursiveEstimatorFilterInterface<ft_state_t, ft_measurement_t>`), `PointFeatureTracker` (concrete impl).
- ROS types leaking into `FeatureTracker`/`PointFeatureTracker` public interface: `sensor_msgs::CameraInfo*` (`setCameraInfoMsg`), `cv_bridge::CvImagePtr` (7+ getters + `ft_measurement_t`), `feature_tracker::FeatureTrackerDynReconfConfig` (`setDynamicReconfigureValues`).
- `ft_state_t` = `FeatureCloudPCLwc::Ptr` (already PCL-native, no change needed) vs. `ft_state_ros_t` = `sensor_msgs::PointCloud2` (Node-only wire type, dropped).
- `cfg/feature_tracker_cfg.yaml` fields (→ `FeatureTrackerConfig`): `ft_type`, `rgb_img_topic`\*, `camera_info_topic`\*, `rgbd_pc_topic`\*, `depth_img_topic`\*, `selfocclusionfilter_img_topic`\*, `selfocclusionfilter_positive`, `number_features`, `min_number_features`, `min_feat_quality`, `min_distance`, `max_distance`, `max_interframe_jump`, `erosion_size_detect`, `erosion_size_track`, `attention_to_motion`, `min_time_to_detect_motion`, `min_depth_difference`, `min_area_size_pixels`, `data_from_bag`\*, `advance_frame_mechanism`\*, `advance_frame_max_wait_time`\*, `advance_frame_min_wait_time`\*, `subscribe_to_pc`\*, `pub_full_pc`\*, `bag_file`\* (fields marked \* are ROS-topic/rosbag-plumbing-only, not consumed by `PointFeatureTracker`'s algorithm — drop in the port, replaced by the orchestrator directly calling `step()`).
- `FeatureTrackerDynReconf.cfg` fields: all `pub_*_img`/`pub_full_pc`/`repub_predicted_feat_locs` booleans (debug-image publish toggles — Node-only, drop) + `advance_frame_min_wait_time` (duplicate of yaml field).
- `message_filters::sync_policies::ApproximateTime`: `FTrackerSyncPolicy` (PointCloud2, Image, Image, queue 1 for bag / 10 for live) and `FTrackerLightSyncPolicy` (Image, Image, queue 10) — pure infra, dropped (see 0.3 table).
- No BFL, no visualization_msgs usage in this package.

</details>

<details>
<summary><b>omip_common</b></summary>

- `RecursiveEstimatorFilterInterface<StateType,MeasurementType>` (`RecursiveEstimatorFilterInterface.h`): **confirmed already 100% ROS-free** (only `<vector>`,`<string>`,`<iostream>`) — this is the base class every Filter derives from and needs zero changes beyond being copied into `omip_core`. This is the good half of the "Filter/Node split" the mission brief warned might not be clean — the split itself is fine, it's the *template type parameters* (`StateType`/`MeasurementType`) instantiated with ROS-msg typedefs that leak ROS in.
- `RecursiveEstimatorNodeInterface<MeasurementTypeROS,StateTypeROS,FilterClass>` (`RecursiveEstimatorNodeInterface.h`): fully ROS-coupled by design (`ros::NodeHandle`, `ros::CallbackQueue`, per-predictor threads, `getROSParameter`) — this whole class is the thing the MuJoCo orchestrator replaces; nothing here gets ported, it gets redesigned as a plain synchronous per-tick call sequence (see mission brief's message_filters guidance — same reasoning applies to this multi-queue/multi-thread design, which exists to let asynchronous ROS topics arrive out of order and interrupt each other; a synchronous sim loop has no such concern).
- `Feature`/`FeaturesDataBase` (`Feature.h/.cpp`, `FeaturesDataBase.h/.cpp`): **confirmed already 100% ROS-free** (boost + STL only) — copy as-is.
- `OMIPTypeDefs.h`: mixes ROS-free typedefs (`ft_state_t`, `RigidBodyShape`, `KinematicModel`) with ROS-msg-typed ones (`rbt_state_t`, `ks_measurement_t`, `joint_measurement_t` — see 0.3's last row, the single most important finding). Also `#include <visualization_msgs/MarkerArray.h>` which is **unused within this header itself** (vestigial include, no `visualization_msgs::` symbol appears) — drop.
- `OMIPUtils.h/.cpp`: `#include <ros/ros.h>` + `<tf/transform_broadcaster.h>` are **effectively vestigial** — the only two real `ROS_ERROR_STREAM` call sites (numeric-instability warnings in the twist/matrix-log code) need only the logging shim swap; no `tf::` symbol is used anywhere in the `.cpp`. Real ROS-typed functions to drop/replace: `TransformLocation(..., const geometry_msgs::Twist&, ...)`, `GeometryMsgsTwist2EigenTwist`, `EigenTwist2GeometryMsgsTwist`, `ROSTwist2EigenTwist` (all have `Eigen::Twistd`-only equivalents already present in the same file, so these are pure-overload removals, not gap-filling). Also `#include "wrappers/matrix/matrix_wrapper.h"` (BFL) for `MatrixWrapper::{Matrix,ColumnVector}` conversion helpers (see 0.5).

</details>

<details>
<summary><b>rb_tracker</b></summary>

- Filter classes: `RBFilter` (per-rigid-body EKF, hand-rolled Eigen, no real BFL despite `using namespace BFL;`), `StaticEnvironmentFilter : public RBFilter` (adds ICP-based static-environment tracking), `MultiRBTracker` (owns the collection of `RBFilter`s, does data association/creation/deletion of RBs).
- ROS types in Filter public interfaces (highest priority list, see 0.3): `RBFilter::setPredictedState(omip_msgs::RigidBodyPoseAndVelMsg)`, `integrateShapeBasedPose(const geometry_msgs::TwistWithCovariance, double)`, `getPoseWithCovariance()`→`geometry_msgs::PoseWithCovariancePtr`, `getPoseECWithCovariance()`/`getVelocityWithCovariance()`→`geometry_msgs::TwistWithCovariancePtr`; `MultiRBTracker::getPosesWithCovariance()`→`vector<PoseWithCovariancePtr>`, `getPosesECWithCovariance()`/`getVelocitiesWithCovariance()`→`vector<TwistWithCovariancePtr>`, `processMeasurementFromShapeTracker(const omip_msgs::ShapeTrackerStates&)`, `setDynamicReconfigureValues(rb_tracker::RBTrackerDynReconfConfig&)`; `StaticEnvironmentFilter::estimateDeltaMotion`/`iterativeICP` (`tf::Transform&` out-params).
- `rbt_state_t` = `rbt_state_ros_t`... no: `rbt_state_t` = `omip_msgs::RigidBodyPosesAndVelsMsg` directly (Filter-internal state = ROS msg type, per 0.3's key finding); `rbt_measurement_t` = `FeatureCloudPCLwc::Ptr` (already clean).
- `cfg/rb_tracker_cfg.yaml`/param reads (all under `rb_tracker/` ns unless noted) → `RBTrackerConfig`: `max_num_rb`, `cam_motion_constraint`, `static_environment_type`, `cov_meas_depth_factor`, `min_cov_meas_{x,y,z}`, `prior_cov_pose`, `prior_cov_vel`, `cov_sys_acc_{tx,ty,tz,rx,ry,rz}`, `min_num_points_in_segment`, `min_probabilistic_value`, `max_fitness_score`, `min_amount_translation_for_new_rb`, `min_amount_rotation_for_new_rb`, `min_num_supporting_feats_to_correct`, `ransac_iterations`, `estimation_error_threshold`, `static_motion_threshold`, `new_rbm_error_threshold`, `max_error_to_reassign_feats`, `supporting_features_threshold`, `min_num_feats_for_new_rb`, `min_num_frames_for_new_rb`, `pub_rb_poses_with_cov`\*, `pub_clustered_pc`\*, `pub_tf`\*, `print_rb_poses`\* (starred = Node-publish-only, drop); cross-package: `/omip/sensor_fps`, `/omip/processing_factor`, `/feature_tracker/number_features`.
- `RBTrackerDynReconf.cfg` fields: `pub_rb_poses_with_cov`, `pub_clustered_pc`, `pub_tf`, `print_rb_poses` (Node-only, drop), `min_feats_new_rb`(default 20), `min_feats_survive_rb`(default 10), `cam_motion_constraint` (enum 0-7: `NoConstraint, RollPitchConstrained, RollPitchZConstrained, TranslationConstrained, TranslationRollYawConstrained, RotationConstrained, FullConstrained, RobotXYPlaneBaselink`, mirrored by `StaticEnvironmentFilter::MotionConstraint` enum — keep this enum, it's algorithm-relevant).
- `message_filters`/`visualization_msgs::MarkerArray` includes in `MultiRBTrackerNode.h` are unused/vestigial (no synchronizer instantiated, no marker ever published) — confirmed dead, not ported.
- tf usage: `_PublishTF()` broadcasts `"camera_rgb_optical_frame"` → `"ip/rb"+id`/`"static_environment"` (Node-only, RViz/debug purpose, not consumed by any Filter); `StaticEnvironmentFilter::Eigen2Tf()` free functions are pure Eigen↔tf::Transform format conversion (no ROS master interaction) feeding into `iterativeICP`'s convergence check (`tf::Transform::getOrigin().length()`/rotation-trace `acos`) — this convergence-check *math* must be preserved (it's algorithm-relevant), just re-expressed directly on `Eigen::Isometry3d` without the `tf::Transform` intermediate.
- Dead code found (not ported, nothing to preserve): unused `RGBDPCCallback`/`MatthiasRefinementsCallback` declarations with no definitions; unused `rosbag`/`geometry_msgs::PoseArray`/`sensor_msgs::Range`/`std_msgs::Float64` includes; commented-out BFL-based `_ekf_velocity`/`_ekf_brake` legacy code in `StaticEnvironmentFilter::constrainMotion()`'s dead `ROBOT_XY_BASELINK_PLANE` branch (references `"/base_link"`/`"/camera_rgb_optical_frame"` tf lookup — never active).
- One stray `getchar()` debug/blocking artifact in `RBFilter::predictState()`'s sanity-check-fail branch — drop, not a real behavior.

</details>

<details>
<summary><b>joint_tracker</b></summary>

Highest-value, highest-risk package. Class inventory (all under `JointFilter` abstract base):

| Class | Joint model | EKF? |
|---|---|---|
| `DisconnectedJointFilter` | unconstrained 6-DoF (probability floor, e.g. constant `_unnormalized_model_probability=0.8`; predictions are random-uniform with huge covariance) | no |
| `RigidJointFilter` | fully constrained, zero relative motion; likelihood from thresholded translation/rotation distance (`_rig_max_translation`, `_rig_max_rotation`) | no |
| `PrismaticJointFilter` | 1-DoF translation along fixed axis; state `[phi,theta,joint_state,joint_velocity]` (`PRISM_STATE_DIM=4`), meas dim 6 (twist) | yes — `BFL::ExtendedKalmanFilter` |
| `RevoluteJointFilter` | 1-DoF rotation about fixed axis+point; state `[phi,theta,px,py,pz,joint_state,joint_velocity]` (`REV_STATE_DIM=7`), meas dim 6, includes explicit ±π/±2π angle-unwrap bookkeeping (`_accumulated_rotation`) | yes — `BFL::ExtendedKalmanFilter` |

`JointCombinedFilter` holds one instance of all 4 above per (parent RB id,
child RB id) pair and does model selection (`getMostProbableJointFilter()`).
`MultiJointTracker` owns a `map<pair<int,int>, JointCombinedFilterPtr>` across
all currently-observed RB pairs.

- ROS types in Filter public interfaces: all 4 concrete `JointFilter`
  subtypes' `getPredictedSRB{Pose,DeltaPose,Velocity}WithCovInSensorFrame()`
  (→`geometry_msgs::TwistWithCovariance`) and
  `getJointMarkersInRRBFrame()` (→`vector<visualization_msgs::Marker>`, see
  J3); `MultiJointTracker`'s `_last_rcvd_poses_and_vels`/
  `_previous_rcvd_poses_and_vels` members typed
  `omip_msgs::RigidBodyPosesAndVelsMsgPtr`; `joint_measurement_t` typedef
  wraps `omip_msgs::RigidBodyPoseAndVelMsg` pairs (see 0.3 last row).
- `JointCombinedFilter.h`'s `ros/ros.h`/`geometry_msgs/*`/`tf/*` includes are
  entirely unused/vestigial (copy-pasted boilerplate header block, no symbol
  from them appears in the class) — drop, nothing to preserve.
- BFL usage (real, see 0.5): `pdf/NonLinearPrismaticMeasurementPdf`,
  `pdf/NonLinearRevoluteMeasurementPdf` implement the analytic nonlinear
  observation model + Jacobian for each joint's EKF — port these two files
  bit-for-bit, they're the numerically critical ones.
- `cfg/joint_tracker_cfg.yaml` fields → `JointTrackerConfig`: `ks_analysis_type`(default 3), `disconnected_ne`(0.1), `clear_rviz_markers`\*(true), `likelihood_sample_num`(100), `min_joint_age_for_ee`(3), `sigma_delta_meas_uncertainty_linear`(0.03), `sigma_delta_meas_uncertainty_angular`(1), `prism_prior_cov_vel`(0.5), `prism_sigma_sys_noise_{phi,theta}`(2.55 each), `prism_sigma_sys_noise_pv`(0.9), `prism_sigma_sys_noise_pvd`(75), `prism_sigma_meas_noise`(0.9), `rev_prior_cov_vel`(1.0), `rev_sigma_sys_noise_{phi,theta}`(2.55 each), `rev_sigma_sys_noise_p{x,y,z}`(0.3 each), `rev_sigma_sys_noise_rv`(5.1), `rev_sigma_sys_noise_rvd`(75), `rev_sigma_meas_noise`(0.05), `rev_min_rot_for_ee`(0.045), `rev_max_joint_distance_for_ee`(0.5), `rig_max_translation`(0.05), `rig_max_rotation`(0.1); cross-package: `/omip/sensor_fps`, `/omip/processing_factor`, `/rb_tracker/min_num_frames_for_new_rb` (starred `clear_rviz_markers` is Node/viz-only, drop). `joint_tracker_dyn_rec.cfg` is empty — nothing to port there.
- tf usage: entirely vestigial/dead (`tf::TransformBroadcaster _tf_pub` member on `JointFilter`, never called; `tf/transform_listener.h` include in `.cpp`, unused). Frame-name literals (`"camera_rgb_optical_frame"`, `"static_environment"`, `"ip/rb"+id`) appear only in dead marker-cleanup code / `#ifdef PUBLISH_PREDICTED_POSE_AS_PWC` blocks (compiled out by default) / Node-side marker publishing — none load-bearing on the estimator.
- `urdf/km1.urdf`, `urdf/km2.urdf~`: **generated output artifacts** (URDF snapshots written by `MultiJointTrackerNode::publishURDF()`, with automatic old→backup rename), not input templates — nothing to port, this is a Node-side debug/export feature (`srv/publish_urdf.srv`, `/joint_tracker/urdf_publisher_srv`) that can be dropped or reimplemented as a plain function call in the orchestrator if URDF export is still wanted later.
- `joint_tracker/meshes/cone.stl`: referenced via marker `mesh_resource` for orientation-uncertainty-cone visualization (prismatic+revolute) — presentation-only, becomes an optional asset path on `DebugGeometry` (see J3), not core estimator logic.
- Logging: all `ROS_*` calls are simple diagnostics except the `pdf/NonLinear*MeasurementPdf.cpp::dfGet()` invalid-index branches, which additionally `exit(BFL_ERRMISUSE)` (see J2).

</details>

<details>
<summary><b>shape_tracker / shape_reconstruction (Phase 7, optional — lighter-weight scouting pass only)</b></summary>

- Both packages' core algorithm classes (`ShapeTracker`, `ShapeReconstruction`, `SRUtils`) bake ROS types directly into their public interfaces even more heavily than rb_tracker/joint_tracker: `ShapeTracker::step(sensor_msgs::PointCloud2ConstPtr, omip_msgs::RigidBodyPoseAndVelMsg, omip_msgs::ShapeTrackerState&)`; `ShapeReconstruction::set{Initial,}FullRGBDPCandRBT(..., const geometry_msgs::TwistWithCovariance&)`, `getShapeModel(omip_msgs::ShapeModelsPtr)`, `PublishMovedModelAndSegment(ros::Time, ..., rosbag::Bag&, bool)`; internal `FrameForSegmentation` struct holds `ros::Time`/`sensor_msgs::Image`/`cv_bridge::CvImagePtr`/`geometry_msgs::TwistWithCovariance` fields directly (ROS types threaded into per-frame algorithm state, not just I/O boundary). `SRUtils::DepthImage2CvImage`/`fillNaNsCBF` also pull in `sensor_msgs`/`cv_bridge`/`ros::Time`.
- Numeric/algorithm parameter setters (`setMinDepthChange`, `setKNNMinRadius`, `setApproxVoxelGridLeafSize`, etc.) are already plain `double`/`int`/`bool` — only the pose/camera-info/publish-oriented methods are ROS-coupled.
- No BFL usage in either package. No `visualization_msgs` usage in either package (debug inspection instead relies on publishing raw PCL clouds as topics + rosbag recording + a pre-built `.rviz` layout + PCL's own standalone `pcl::visualization` viewer).
- `shape_reconstruction/srv/generate_meshes.srv` is a no-argument trigger service ("generate STL meshes now" for all tracked shapes + optional static environment) — trivially becomes a plain method call in the port.
- `dynamic_reconfigure::Server` is included in 4 headers across these packages but **never instantiated** anywhere — all `cfg/*.yaml` fields are read as plain rosparam via `getROSParameter<T>`, not autogenerated dynamic_reconfigure Config classes. Nothing to port for dynamic_reconfigure here.
- Python scripts: `icra16_play_bag.py`, `extract_images.py`, `generate_statistics.py` are rosbag/rospy/cv_bridge-dependent (would need a bag-reading replacement, e.g. the `rosbags` pure-Python library, if ever revisited); `compare_statistics.py`, `plot_statistics.py`, `toyexample.py` are already ROS-free.
- Not pursued further unless explicitly requested after Phase 6, per the mission brief.

</details>

---

## Phase 1 — Build skeleton
Status: **done**. `omip_core` (plain CMake, no catkin) builds and links
locally against real PCL 1.15.1, OpenCV 4.12.0, Eigen3, and Boost 1.90.0
(all via Homebrew) plus vendored `lgsm` and `bfl`, producing an (currently
empty) `libomip_core.a`. Verified zero `find_package(catkin ...)`/`roscpp`/
etc. anywhere in `omip_core/CMakeLists.txt` or the vendored thirdparty
CMakeLists.

What was set up:
- `omip_core/` directory layout matching the mission brief's target repo
  layout (`include/omip_core/{feature_tracker,rb_tracker,joint_tracker,
  shape_tracker,shape_reconstruction}`, `src/...`, `thirdparty/`, `tests/`,
  `bindings/`).
- `thirdparty/lgsm/`: verbatim copy of `lgsm/include/lgsm` (Phase 0 already
  verified zero ROS coupling) + a new plain `CMakeLists.txt` (interface
  library, `find_package(Eigen3)` only, catkin wrapper stripped). No source
  changes.
- `thirdparty/bfl/orocos_bfl/`: verbatim clone of
  `github.com/orocos/orocos-bayesian-filtering` (master, orocos_bfl/
  subdirectory only — `bfl_typekit/` and RTT/`stack.xml` bindings dropped,
  they're Orocos-RTT integration, not needed). Re-grepped the actual cloned
  source for `ros/`, `catkin`, `roscpp`, `tf/`, `tf2` — **zero matches**,
  confirming BFL is genuinely ROS-independent as expected. Matrix/RNG
  backend selected: **boost** (`boost::numeric::ublas` + `boost::random`),
  matching upstream's own default and Boost already being a hard OMIP
  dependency; the `matrix_NEWMAT.*`/`matrix_EIGEN.*` files are still
  compiled (matching upstream's own unconditional source list) but their
  bodies are `#ifdef`-guarded and compile to empty translation units since
  only `__MATRIXWRAPPER_BOOST__`/`__RNGWRAPPER_BOOST__` are defined.
- `thirdparty/bfl/CMakeLists.txt`: a **new** flat CMakeLists.txt (not
  upstream's own) that explicitly lists the exact source files upstream's
  own per-directory `GLOBAL_ADD_SRC` calls declare, because upstream's own
  multi-directory build uses `${CMAKE_SOURCE_DIR}`-relative paths that only
  resolve correctly when BFL is built as the top-level project — vendoring
  it as a subdirectory of a larger build breaks that assumption. This is
  build-orchestration-only; no upstream `.cpp`/`.h` content was changed
  except one line (see judgment call J5 below).
- **J5 (new judgment call):** `thirdparty/bfl/orocos_bfl/src/wrappers/rng/rng.cpp`
  had `#include <../config.h>` (angle brackets) where every other sibling
  file in the same directory structure correctly uses `#include "../config.h"`
  (quotes) for the identical same-tree relative header — this is a
  preexisting upstream bug/typo (angle-bracket includes don't get
  file-relative resolution, so it only ever "worked" upstream if their old
  build happened to add enough extra `-I` search paths to paper over it).
  Fixed the one line to match every other file's correct form. This is a
  preprocessor-include-syntax fix in vendored build-adjacent code, not a
  math/behavior change — flagged here per ground rule 5 anyway since it's a
  vendored-source edit, however trivial.
- `omip_core/CMakeLists.txt`: top-level library target linking
  `lgsm`, `bfl`, `Eigen3::Eigen`, PCL, OpenCV, `Boost::thread`; currently
  builds one placeholder translation unit (`src/placeholder.cpp`), removed
  once Phase 2 adds real ported code.
- Toolchain notes (this machine, macOS/Homebrew): PCL was not installed;
  installing it pulled in VTK/Qt and hit a Homebrew conflict between the
  already-installed `qt` formula (used by this machine's existing opencv/
  pyqt/vtk) and the `qtbase`/`qtsvg` formulae PCL's dependency chain wants.
  Resolved via `brew link --overwrite` on the conflicting formulae per the
  user's explicit go-ahead (2026-07-22) — `qt` itself remains installed,
  only some of its symlinks were overwritten by `qtbase`/`qtsvg`'s. Not an
  OMIP-repo concern, but noted here since it's exactly the kind of
  environment-specific toolchain wrinkle a future from-scratch build on a
  different machine might hit differently (e.g. a Linux box with no prior
  Qt install wouldn't see this at all).
- Not yet configured: `bindings/` (pybind11, Phase 5) and any real
  `include/omip_core/*` headers (Phase 2+) — both empty/placeholder per the
  phase plan's "don't skip ahead" rule.

## Phase 2 — Port `feature_tracker`
Status: **ported, unvalidated, awaiting fixtures**. Builds cleanly (fresh
full rebuild, zero warnings in our own code). Test harness
(`tests/test_feature_tracker.cpp`) runs and passes trivially — reports
"no fixtures found" and exits 0, per the mission's instruction to not
block on missing fixtures.

### 2.1 Common infrastructure ported (`omip_common` → `omip_core` root)

Straight ports, include paths updated to `omip_core/...`, `ROS_ERROR_STREAM*`
→ the new `omip_core/Log.h` shim (`OMIP_*_STREAM[_NAMED]`/`OMIP_*_NAMED`
macros — see that file):
- `Feature.h`/`.cpp`, `FeaturesDataBase.h`/`.cpp` — already 100% ROS-free in
  the original (per Phase 0), unchanged besides the above.
- `RecursiveEstimatorFilterInterface.h` — already 100% ROS-free, byte-for-byte
  unchanged besides the include path. `RecursiveEstimatorNodeInterface.h` is
  **not** ported (by design — it's the ROS Node layer the MuJoCo orchestrator
  replaces, per the mission brief and Phase 0 findings).
- `OMIPUtils.h`/`.cpp` — dropped the `geometry_msgs::Twist`-typed overloads
  (`TransformLocation`, `GeometryMsgsTwist2EigenTwist`,
  `EigenTwist2GeometryMsgsTwist`, `ROSTwist2EigenTwist`) since
  `Eigen::Twistd`-only equivalents already existed side-by-side (pure
  overload removal). Also: `<tr1/random>` (pre-C++11 TR1, unavailable on
  this toolchain) → `<random>`, same Mersenne Twister generator and normal
  distribution, no behavior change (see `sampleNormal`'s new comment).
- `types.hpp` (new) + `OMIPTypeDefs.h` (ported, original filename kept) —
  the full omip_msgs/geometry_msgs translation table from Phase 0 is now
  materialized as real structs (`PoseWithCovariance`, `TwistWithCovariance`,
  `StampedImage`, `CameraIntrinsics`, `RigidBodyPoseAndVel`,
  `RigidBodyPosesAndVels`, `JointModel`, `KinematicStructure`, etc.) — all of
  0.2/0.3's table, not just what `feature_tracker` needs, since these are
  shared plain-data types with no algorithm attached; the Filter classes
  that will use the rb_tracker/joint_tracker ones aren't touched until
  Phases 3/4.

### 2.2 `feature_tracker` ported

- `FeatureTracker.h` (abstract base): `sensor_msgs::CameraInfo*` →
  `const CameraIntrinsics&`; `cv_bridge::CvImagePtr` → `StampedImage::Ptr`;
  `camera_info_manager` include dropped (unused beyond the include);
  **`setDynamicReconfigureValues(...)` removed entirely** — dynamic_reconfigure
  has no ROS-free equivalent, and the one field it actually controlled
  (`pub_predicted_and_past_feats_img`) is now a plain, directly-settable
  `FeatureTrackerConfig` field instead of a live-toggle RPC.
- `FeatureTrackerConfig.h` (new): plain struct replacing
  `feature_tracker_cfg.yaml` + the Filter-relevant subset of
  `FeatureTrackerDynReconfConfig` (topic names, rosbag/advance-frame
  settings, and other Node-only fields from the yaml are not carried over —
  see file comment for the full field-by-field mapping).
- `PointFeatureTracker.h`/`.cpp`: `ros::NodeHandle _node_handle` (used only
  to read ROS params) dropped, replaced by the `FeatureTrackerConfig`
  passed into the constructor; `sensor_msgs::CameraInfo::Ptr` →
  `CameraIntrinsics::Ptr`; `cv_bridge::CvImagePtr` members → `StampedImage::Ptr`;
  `ros::Time`/`ros::Time::now()` → `double` (ns or s as appropriate) via a
  `wallClockNowSeconds()`/`wallClockNowNs()` helper (`std::chrono`).
  `cv_bridge::cvtColor(img, "mono8")` → a small `toMono8()` helper
  reimplementing the specific encoding conversions this codebase actually
  exercises (bgr8/bgra8/rgb8/rgba8/mono8 → mono8) — cv_bridge's own
  `cvtColor` is itself just `cv::cvtColor` plus an encoding-string lookup
  table, so this isn't inventing new logic, just narrowing it to what's used.
  `pcl_conversions::fromPCL(...).toNSec()` (PCL-header-stamp ↔ ROS-time
  conversion) → direct `header.stamp * 1000.0` (PCL's own `uint64_t` stamp
  is already microseconds-since-epoch by PCL's own convention, independent
  of ROS; `fromPCL(...).toNSec()` was just `microseconds * 1000` — same
  arithmetic, no ROS type needed).

**Judgment calls (Phase 2):**
- **J6 —** `_features_file` (an unconditionally-opened `"features.txt"` in
  the constructor) is dropped: grepped the whole original file, confirmed
  it is opened/closed but **never written to** anywhere — dead code with
  zero observable behavior, and a bad pattern for a library besides (opens
  a relative-path file in the caller's cwd unconditionally on construction).
- **J7 —** The two duplicate `ROS_INFO_STREAM_NAMED("...ReadParameters"...)`-adjacent
  duplicate param reads in the original `_ReadParameters()` (`max_distance`
  read twice into the same field) collapsed to a single assignment —
  zero behavior difference (reading the same value into the same variable
  twice vs. once).
- **Open question (not blocking):** the BETA "attention to motion" feature
  (`_attention_to_motion`, disabled by default in the original yaml) gates
  on wall-clock elapsed time (`ros::Time::now()` originally, `std::chrono`
  wall clock now) to decide when to re-check for depth-changing regions.
  In a MuJoCo-driven pipeline that may run faster/slower than real time,
  this wall-clock gate may not be the intended semantics once the
  orchestrator (Phase 6) drives this — flagging for a decision then, since
  it's off by default today and not exercised by anything ported so far.

### 2.3 lgsm removed, replaced with `omip_core/LieGroup.hpp`

Per the user's decision (recorded in the Phase 1 section above) to patch
lgsm's Eigen-evaluator incompatibility, deeper investigation during Phase 2
revealed the gap was far more fundamental than the Phase 1 constructor
conflicts: **lgsm predates Eigen's "evaluator" system entirely** (introduced
in Eigen 3.3; present in both Eigen 3.4.1 and 5.0.1, the only versions
available on this machine). Every lgsm class's *inherited* generic
`MatrixBase` machinery (`operator()`, and by extension most arithmetic)
fails to compile — confirmed by testing that direct access to each class's
own underlying storage (`.get()`) works fine, while anything going through
the generic Eigen expression machinery does not. Given this, **the user
decided to abandon lgsm entirely** rather than keep patching Eigen-internal
compatibility shims (see conversation record) — replaced with:

- `omip_core/include/omip_core/LieGroup.hpp` (new): a small, dependency-free
  header providing `omip::Twistd` (plain class, `Eigen::Matrix<double,6,1>`
  storage, no `MatrixBase` inheritance — sidesteps the whole evaluator
  problem) plus free functions `so3Exp`, `so3Dexp`, `se3Exp`, `se3Log`,
  `se3Adjoint`. `Eigen::Displacementd` (lgsm's SE(3) group element) is
  replaced by plain `Eigen::Isometry3d` (a first-class, fully modern-Eigen-
  supported type) rather than a custom wrapper — no reason to reinvent that
  part when Eigen already provides it.
- **Every formula was extracted from lgsm's own source line-by-line** (not
  re-derived from a textbook) specifically to preserve bit-for-bit numerical
  behavior — verified against `LieGroup_SO3.h`, `LieGroup_SE3.h`,
  `LieAlgebra_so3.h`, `LieAlgebra_se3.h` (all still visible in git history
  under the now-removed `thirdparty/lgsm/`, commit `d119d10`) before the
  directory was deleted. Round-trip-tested (`log(exp(x)) ≈ x`) for pure
  rotation, pure translation, identity, and adjoint-of-identity — all pass.
- **Two important, non-obvious findings while doing this, both deliberately
  preserved rather than "fixed":**
  1. **lgsm was built with `USE_RLAB_LOG_FUNCTION` defined**
     (`lgsm/Lgsm:62`), so the *active* `Displacementd::log()` implementation
     is the numerically-stabilized "RLab" `dexpinv` formula, not the
     simpler sin/cos formula that exists side-by-side in the same function
     guarded out by that macro. `se3Log()` replicates the RLab branch
     specifically.
  2. **lgsm's `se(3)` `exp(precision)` never forwards its `precision`
     argument** to the `so(3)` `exp()`/`dexp()` calls it makes internally —
     those always use their own hardcoded defaults (1e-5, and 1e-6/1e-2)
     regardless of what's passed in. `se3Exp()` replicates this (its
     `precision` parameter is intentionally unused) rather than "fixing" it.
  3. **Round-trip testing surfaced an apparent pre-existing asymmetry in
     lgsm itself**: `se(3)` `exp()` computes translation as
     `dexp(w)ᵀ · v` (note the transpose — confirmed present in the literal
     source), while `log()`'s RLab `dexpinv` is numerically confirmed
     (empirically, via direct matrix comparison) to equal `dexp(w)⁻¹`, not
     `(dexp(w)ᵀ)⁻¹`. Since `dexp(w)` is not symmetric in general (only when
     the angular part is zero), `log(exp(ξ))` does **not** recover `ξ`
     exactly for a general twist in lgsm as originally shipped — only for
     pure-rotation or pure-translation twists (confirmed by the round-trip
     tests: those two pass exactly, the general and near-π cases don't).
     Standard SE(3) theory (e.g. any reference deriving the "left Jacobian")
     has no such transpose, so this looks like a genuine latent bug in the
     original vendor library — but since OMIP's own estimation results were
     computed *with* this quirk baked in, **`se3Exp`/`se3Log` preserve it
     exactly** rather than correcting it, per ground rule 5. Flagging this
     prominently since it's exactly the kind of thing that could otherwise
     look like a porting bug in review — it isn't; it's a faithful
     reproduction of the original's actual (if imperfect) behavior. If
     golden-fixture validation (Phase 3+, wherever `TransformLocation`/
     `Twist2TransformMatrix` with a general twist is exercised) shows
     unexpected discrepancies, revisit this note first.
- `types.hpp`'s `TwistWithCovariance::twist` field type updated from
  `Eigen::Twistd` → `omip::Twistd` accordingly.
- `thirdparty/lgsm/` directory removed entirely (was vendored in Phase 1,
  commit `d119d10`); `add_subdirectory(thirdparty/lgsm)` and the `lgsm`
  link target removed from `omip_core/CMakeLists.txt`.

### 2.4 Other toolchain-compatibility fixes (no behavior change)

- `OMIP_ADD_POINT4D` macro (`OMIPTypeDefs.h`, ported from `omip_common`):
  `EIGEN_ALIGN16` moved from suffix to prefix position on the inner
  anonymous union. Modern Eigen expands this macro to the C++11 `alignas(16)`
  keyword, which must precede a declaration, not follow it — the outer
  struct already used prefix position, only the inner union (unchanged
  since the original ROS-era Eigen) needed the fix.
- `PointFeatureTracker.cpp`: `cvPoint(...)`/`cvPoint2D32f(...)` (OpenCV 1.x C
  API, not available via `opencv2/core.hpp` on OpenCV 4.12) → `cv::Point(...)`/
  `cv::Point2f(...)`. `cvRound(...)` is still available (kept as-is).

### 2.5 Test harness

`tests/test_feature_tracker.cpp` + `tests/fixtures/README.md` document the
proposed `.npz` fixture schema (camera intrinsics + per-frame RGB/depth/
expected feature positions) — **proposed**, not yet confirmed against
whatever the Docker-side export script actually produces; revisit once real
fixtures land. Vendored `cnpy` (MIT, github.com/rogersce/cnpy,
`thirdparty/cnpy/`) to read `.npz`, per the user's Phase 0 decision (Q2).
The harness runs via CTest, reports "ported, unvalidated, awaiting
fixtures," and exits 0 when `tests/fixtures/feature_tracker/` is empty —
confirmed working. `.npz` fixture files themselves are gitignored (generated
data, not source).

## Phase 3 — Port `rb_tracker`
Status: **ported, unvalidated, awaiting fixtures**. Builds cleanly (fresh
rebuild, zero warnings). `tests/test_rb_tracker.cpp` runs via CTest and
passes trivially ("no fixtures found") alongside `test_feature_tracker`.

### 3.1 What was ported

`RBFilter` (per-rigid-body hand-rolled Eigen EKF — confirmed in Phase 0 that
its `using namespace BFL;`/`MatrixWrapper` were vestigial, no real BFL
dependency), `StaticEnvironmentFilter : public RBFilter` (ICP- or EKF-based
static-environment/visual-odometry tracking), and `MultiRBTracker`
(owns the collection of `RBFilter`s: data association, creation/deletion,
predict/correct orchestration). Ported to
`omip_core/{include,src}/rb_tracker/`.

Type replacements (per PORTING_NOTES.md 0.2/0.3, all already designed in
Phase 0/2):
- `omip_msgs::RigidBodyPoseAndVelMsg` → `RigidBodyPoseAndVel`.
- `geometry_msgs::TwistWithCovariance` → `TwistWithCovariance`.
- `geometry_msgs::{Pose,Twist}WithCovariancePtr` → `{Pose,Twist}WithCovariance::Ptr`
  (added `Ptr` typedefs to both structs in `types.hpp` this phase, matching
  the existing `Feature::Ptr`/`CameraIntrinsics::Ptr` pattern).
- `omip_msgs::ShapeTrackerStates` → `ShapeTrackerStates` (already in
  `types.hpp` from Phase 2, unused until now).
- `Eigen::Twistd` (lgsm) → `omip::Twistd` (Phase 2's `LieGroup.hpp`) —
  added two operators `LieGroup.hpp` didn't need until now:
  `Twistd::operator/(double)` and a free `operator*(double, const Twistd&)`
  (scalar-on-the-left), both used pervasively in `rb_tracker`'s velocity
  arithmetic (`twist / dt`, `dt * twist`).
- `tf::Transform` (in `StaticEnvironmentFilter` only) → `Eigen::Isometry3d`
  — the two are mathematically equivalent rigid-transform representations;
  replaced the local `Eigen2Tf()` helper with direct `Eigen::Isometry3d`
  construction from the `Matrix4f`/`Matrix4d` PCL/SVD result, and
  `.getOrigin().length()` / `.getBasis()` trace → `.translation().norm()` /
  `.linear().trace()` for the ICP convergence check.
- `rb_tracker::RBTrackerDynReconfConfig` and `setDynamicReconfigureValues(...)`
  dropped entirely on both `RBFilter`/`MultiRBTracker` — same precedent as
  `feature_tracker` in Phase 2 (no ROS-free equivalent; the 3 fields it
  controlled — `cam_motion_constraint`, `min_feats_new_rb`,
  `min_feats_survive_rb` — are already plain constructor arguments here,
  just no longer live-updatable at runtime via a ROS service).
- `ros::NodeHandle _nh` and `tf::TransformListener _tf_listener` dropped
  from `StaticEnvironmentFilter` (confirmed dead — `_nh` unused, `_tf_listener`
  only referenced inside commented-out code).
- `pcl_conversions::toPCL(ros::Time(...))` (in `MultiRBTracker::getPredictedMeasurement`)
  → direct arithmetic: PCL's own point-cloud `header.stamp` is already
  microseconds-since-epoch by PCL's own convention (independent of ROS), so
  `(measurement_timestamp_ns + loop_period_ns) / 1000.0` reproduces the
  exact same value the ROS round-trip computed, without needing
  `pcl_conversions` or the `ROS_VERSION_MINIMUM`/`pcl_conversions_indigo.h`
  compatibility shim at all.
- `meas_from_st.header.stamp.toNSec()` → `meas_from_st.timestamp_ns`
  (`ShapeTrackerStates` already carries a plain timestamp field).
- Unused includes dropped (confirmed unused, no symbol referenced):
  `unsupported/Eigen/{MatrixFunctions,NonLinearOptimization,NumericalDiff}`,
  `pcl_ros/transforms.h`, `omip_msgs/RigidBodyTwistWithCovMsg.h`.

**Judgment calls (Phase 3):**
- **J9 —** `getchar()` in `RBFilter::predictState()`'s negative-time-interval
  sanity check dropped, same rationale as prior blocking-debug-artifact
  removals (J-series precedent from Phase 2): a library must never block
  on stdin. The accompanying `OMIP_ERROR_STREAM` log call is kept.
- **J10 —** `_really_free_feats_file` (`std::ofstream` member, opened via a
  commented-out `.open()` call in the constructor, never written to
  anywhere) dropped — dead code, same pattern as J6 in Phase 2.
- **J11 —** Several `geometry_msgs`-message field-by-field copies collapsed
  to direct struct/matrix assignment now that the ROS message boundary is
  gone — e.g. `getPoseWithCovariance()`'s quaternion decomposition
  (`orientation.x/y/z/w` from a rotation matrix) is now just
  `pose.pose = Eigen::Isometry3d(_pose)`; covariance `float64[36]` flat-array
  copies (`for i,j: covariance[6*i+j] = ...`) are now single `Eigen::Matrix`
  assignments; `setPredictedState`'s `hypothesis.pose_wc.twist.angular.x/y/z`
  + `.linear.x/y/z` reconstruction into an `Eigen::Twistd` is now a direct
  copy since `hypothesis.pose_wc.twist` is already an `omip::Twistd`. These
  are not behavior changes (identical values end up in identical places),
  just simplifications that fall out naturally once the ROS message
  boundary requiring field-by-field translation no longer exists.
- **`StaticEnvironmentFilter::constrainMotion()`** ported as the same no-op
  it already was in the original: every branch of its motion-constraint
  logic was commented out in the ROS source (confirmed in Phase 0 and
  reconfirmed while reading the full file this phase) — only the
  never-active `ROBOT_XY_BASELINK_PLANE` branch had any real logic (a
  `"/base_link"`↔`"/camera_rgb_optical_frame"` tf lookup), and it was dead
  code too. Not revived.

### 3.2 Test harness

`tests/test_rb_tracker.cpp` + `tests/fixtures/README.md` document the
proposed `.npz` schema (per-frame feature id/xyz measurements in, per-rigid-
body pose/velocity twists out) — **proposed**, not yet confirmed against
the actual Docker-side export. Default `MultiRBTracker` construction
parameters in the harness are matched to `rb_tracker/cfg/rb_tracker_cfg.yaml`'s
documented defaults so the harness reflects the same configuration the
original system would run with absent fixture-specific overrides.

## Phase 4 — Port `joint_tracker`
Status: **ported, unvalidated, awaiting fixtures**. Builds cleanly (fresh
rebuild, zero warnings). `tests/test_joint_tracker.cpp` runs via CTest and
passes trivially ("no fixtures found") alongside the Phase 2/3 harnesses.
Ad hoc runtime smoke-testing (not part of the committed suite — synthetic
prismatic/revolute two-rigid-body trajectories fed through a full
predict/correct loop) confirms the estimator runs to completion, converges
to sane joint-type classifications and joint-state values, and doesn't
crash; see 4.3 for one genuine (pre-existing, not porting-introduced)
numerical edge case this testing turned up.

### 4.1 What was ported

All of `joint_tracker`'s estimator core, to `omip_core/{include,src}/joint_tracker/`:
`JointFilter` (abstract base), `DisconnectedJointFilter`, `RigidJointFilter`,
`PrismaticJointFilter`, `RevoluteJointFilter` (all 4 joint filter subtypes
confirmed present — the Phase 4 exit criterion), their two BFL measurement
models (`pdf/NonLinearPrismaticMeasurementPdf`, `pdf/NonLinearRevoluteMeasurementPdf`),
`JointCombinedFilter` (holds one instance of each of the 4 joint filter
types per RB pair, does model selection), and `MultiJointTracker` (owns the
map of `JointCombinedFilter`s across all observed RB pairs, does data
association/lifecycle). This is the highest-value, highest-risk part of the
port per the mission brief, and was ported as literally as possible —
class/method names, member names, and algorithm structure (including
comments) are preserved verbatim wherever the ROS types being replaced
allow it; the EKF predict/correct math, the Park & Okamura model-likelihood
distance metric, and the joint-type probability bookkeeping are unchanged.

Type replacements (per PORTING_NOTES.md 0.2/0.3, all already designed in
Phase 0, following the Phase 2/3 precedent exactly):
- `geometry_msgs::TwistWithCovariance` → `TwistWithCovariance` (already had
  this in `types.hpp` since Phase 3).
- `Eigen::Twistd` (lgsm) → `omip::Twistd` (Phase 2's `LieGroup.hpp`) —
  `.exp(precision)`/`.log(precision)` member calls → free functions
  `se3Exp(twistd, precision)`/`se3Log(isometry, precision)`;
  `Eigen::Displacementd` → `Eigen::Isometry3d`.
- `visualization_msgs::Marker` → new `omip::DebugGeometry` struct
  (`types.hpp`) — see judgment call J12.
- `ROSTwist2EigenTwist(msg.twist, twistd)` → direct assignment
  `twistd = msg.twist` (the ROS message boundary is gone, `.twist` is
  already an `omip::Twistd`) — same simplification pattern as Phase 3's
  J11, applied throughout `JointFilter::setMeasurement()`/
  `setInitialMeasurement()` and `MultiJointTracker::correctState()`.
  Likewise `covariance[6*i+j]` flat-array double-loops → direct
  `Eigen::Matrix<double,6,6>` assignment, and `.centroid.x/.y/.z` (ROS
  Point-like access) → `.centroid` directly (`Eigen::Vector3d` in this
  port since Phase 0).
- `ros::NodeHandle`/`ros::Publisher`/`tf::TransformBroadcaster _tf_pub` —
  see judgment call J13.
- `omip_msgs::RigidBodyPoseAndVelMsg`/`RigidBodyPosesAndVelsMsg` →
  `RigidBodyPoseAndVel`/`RigidBodyPosesAndVels` (already in `types.hpp`
  since Phase 0/3); `MultiJointTracker`'s `omip_msgs::RigidBodyPosesAndVelsMsgPtr`
  members → `boost::shared_ptr<ks_measurement_t>` (matches how the original
  `.cpp` actually constructed them — `boost::shared_ptr<ks_measurement_t>(new ks_measurement_t(...))`
  — rather than introducing a separate message-specific Ptr typedef).
- `<ros/ros.h>` (only used for `ROS_ERROR_STREAM_NAMED` + `exit(BFL_ERRMISUSE)`
  in both measurement pdfs' `dfGet()` fallback branch, and `ROS_INFO/DEBUG_STREAM_NAMED`
  logging elsewhere) → `Log.h` shim; the `exit()` call specifically — see
  judgment call J14.
- `BOOST_FOREACH` kept as-is (not ROS-specific, matches the Phase 3
  precedent of not modernizing working, already-ROS-free boost usage during
  a literal port).

**Judgment calls (Phase 4):**
- **J12 —** `visualization_msgs::Marker` → new `omip::DebugGeometry` struct
  in `types.hpp` (fields: `ns`, `id`, `type` (`DebugGeometryType` enum:
  Arrow/Sphere/TextViewFacing/MeshResource — the only 4 Marker types
  actually used across all 4 joint filters' debug-geometry methods),
  `action` (Add/Delete), `position`/`orientation` (Eigen, replacing
  `pose.position`/`pose.orientation`), `scale` (`Eigen::Vector3d`), `color`
  (`Eigen::Vector4d`, r/g/b/a), `points` (`std::vector<Eigen::Vector3d>`,
  for ARROW-by-two-points mode), `text`, `mesh_resource`). All 4
  `getJointMarkersInRRBFrame()` methods renamed to
  `getJointDebugGeometryInRRBFrame()` (on the abstract base too) since
  "Marker" is an RViz/ROS-specific term with no meaning in a ROS-free
  library — the geometry payload and its construction logic (including the
  full chi-squared-confidence-ellipse uncertainty-cone math in
  Prismatic/RevoluteJointFilter) is otherwise translated verbatim, just
  with direct Eigen-typed field assignment (e.g.
  `marker.orientation = quaternion` directly) replacing the original's
  field-by-field `.x/.y/.z/.w` copies — same simplification category as
  Phase 3's J11, not a behavior change. `header.frame_id` is dropped
  entirely: every call site across all 4 filters either left it unset or
  explicitly commented it out with a note that the Node layer (unported)
  assigns it per-RB-pair.
- **J13 —** Dropped `tf::TransformBroadcaster _tf_pub` (declared on
  `JointFilter`, confirmed dead — grepped the whole package, no
  `sendTransform()` call anywhere) and every
  `#ifdef PUBLISH_PREDICTED_POSE_AS_PWC` block (one in each of
  `JointFilter::initialize()`, `DisconnectedJointFilter`, `RigidJointFilter`,
  `PrismaticJointFilter`, `RevoluteJointFilter`'s `getPredictedSRBPoseWithCovInSensorFrame()`)
  along with the `ros::NodeHandle _predicted_next_pose_nh`/
  `ros::Publisher _predicted_next_pose_publisher` members — the guarding
  macro was permanently commented out in the original source
  (`//#define PUBLISH_PREDICTED_POSE_AS_PWC 1`), so all of this was dead
  code by construction, same category as Phase 3's J9/J10.
- **J14 —** Both `NonLinearPrismaticMeasurementPdf::dfGet()` and
  `NonLinearRevoluteMeasurementPdf::dfGet()` call `exit(BFL_ERRMISUSE)` in
  their "derivative not implemented for this conditional argument" branch
  (unreachable in practice — the EKF only ever calls `dfGet(0)`). A library
  must never call `exit()` (already established precedent, see the
  getchar()-removal rationale in Phase 3's J9) — replaced with
  `OMIP_ERROR_STREAM_NAMED(...)` (preserving the diagnostic) followed by
  `throw std::runtime_error(...)`. Since this branch is provably
  unreachable given how BFL calls `dfGet()`, this is not a behavior change
  for any real execution path.
- **J15 —** Dropped the unused "Force Torque sensor" measurement-model
  scaffolding in `PrismaticJointFilter::_initializeMeasurementModel()` (a
  local `Gaussian meas_uncertainty_ft_PDF(...)` built and then discarded —
  never assigned to any member, never referenced again; the
  `imu_meas_pdf_`/`imu_meas_model_` lines right below it were already
  commented out in the original). No behavior change: the object was
  provably dead (constructed, never read).
- **J16 —** Dropped an unused `Eigen::Twistd new_twist; ROSTwist2EigenTwist(...)`
  pair of lines inside `MultiJointTracker::correctState()`'s
  already-precomputing (but not-yet-old-enough) branch — `new_twist` is
  computed and never referenced again anywhere in that branch. Same
  confirmed-dead-local-variable category as J15.
- **J17 —** Added a `Twistd operator*(const Eigen::Matrix<double,6,6>&, const Twistd&)`
  free function to `LieGroup.hpp`, needed for the `adjoint * twist` pattern
  in `{Prismatic,Revolute}JointFilter::getPredictedSRB{DeltaPose,Velocity}WithCovInSensorFrame()`.
  Verified against lgsm's actual `Twist(const MatrixBase<OtherDerived>&)`
  constructor (recovered from git history, `thirdparty/lgsm/include/lgsm/Twist.h`
  before its Phase 2 removal): it copies a generic 6-vector expression's
  coefficients directly into storage, with no awareness of any
  angular-first/linear-first semantic split. The new operator replicates
  exactly that — a raw `matrix * coeffs()` multiply — regardless of
  `computeAdjoint()`'s own doc-comment (from the original source) about
  swapping rotation/translation block order for a "translation-first"
  convention; whatever the original code's actual numerical result was
  (correct or not, by whatever convention), this reproduces it bit-for-bit
  rather than "fixing" a potential convention mismatch that isn't this
  port's business to resolve.
- **J18 —** Fixed a pre-existing bug in this port's own `OMIPTypeDefs.h`
  (introduced in Phase 0/2, before `joint_tracker` existed in the port and
  went unnoticed since nothing exercised it): `KinematicModel`
  (`= ks_state_t`) was typedef'd with `std::shared_ptr<JointCombinedFilter>`,
  but `JointCombinedFilterPtr` (`joint_tracker/JointCombinedFilter.h`) —
  which `MultiJointTracker::_reflectState()` directly assigns into
  `_state`/`ks_state_t` — is `boost::shared_ptr<JointCombinedFilter>`. Two
  different smart-pointer types are not assignment-compatible, so this
  would not have compiled once `MultiJointTracker` was wired up. Checked
  the original (pre-port) `omip_common/OMIPTypeDefs.h`: it used
  `boost::shared_ptr` here too, confirming this was an unintentional
  deviation from the original in this port, not a deliberate Phase 0
  choice — fixed to match both the original and `JointCombinedFilterPtr`.
- Minor simplification (not a numerical change): `RigidJointFilter`/
  `PrismaticJointFilter`'s debug-geometry and joint-position code built an
  `Eigen::Affine3d` from an `Eigen::Displacementd`'s `.toHomogeneousMatrix()`
  purely to then do `affine.inverse() * point`; since `Eigen::Isometry3d`
  supports `.inverse()` and `operator*(Vector3d)` directly, the
  Affine3d round-trip is skipped and the `Isometry3d` (already produced by
  `se3Exp()`) is used directly for the same point transform.

### 4.2 Testing

Ad hoc runtime smoke test (two synthetic RB pairs — one purely
translating, one purely rotating about a fixed axis — driven through
~40 frames of `setMeasurement`/`predictState`/`predictMeasurement`/
`correctState`/`estimateJointFiltersProbabilities`, with realistic nonzero
pose/velocity covariances) confirms:
- The estimator runs to completion with no crash, for all 4 joint filter
  types running in parallel inside `JointCombinedFilter` every frame.
- On the purely-translating trajectory, `RigidJointFilter`'s probability
  correctly drops to (and stays at) 0 once the accumulated translation
  exceeds `rig_max_translation` — the "motion memory" design working as
  intended (a rigid-joint hypothesis, once disproved, never recovers).
- On the purely-rotating trajectory, `RevoluteJointFilter`'s EKF state
  converges to finite, sane values (joint angle magnitude tracking the
  synthetic angular velocity) — confirming the revolute EKF's nonlinear
  measurement model, its analytic Jacobian (`NonLinearRevoluteMeasurementPdf`),
  and the `unwrapTwist`/`invertTwist` angle-unwrapping logic all work
  end-to-end after the port.
- **Finding, not a porting bug:** on the purely-translating (zero relative
  rotation) trajectory, `RevoluteJointFilter::initialize()` produces NaN.
  Root cause: `_joint_state = _joint_orientation.norm()` is exactly 0 when
  the initial relative twist's rotational part is exactly `(0,0,0)`, and
  the very next line divides by `pow(_joint_state, 2)` — a 0/0 division
  that is a direct, literal property of the original formula, not
  something introduced by this port (verified by re-reading the ported
  code against the original source side-by-side; the division is
  unchanged). This is exceedingly unlikely to trigger against real
  `rb_tracker` output (floating-point sensor noise means a real relative
  rotation is essentially never *exactly* zero), which is presumably why
  it was never observed against real data in the original ROS system. Not
  fixed, per the mission's "no silent numerical changes" instruction — a
  guard here would be a genuine (if reasonable) algorithmic change, not a
  behavior-preserving port step. Flagged here for the user's awareness;
  worth asking about explicitly if it ever surfaces against real golden
  fixtures.

`tests/test_joint_tracker.cpp` + `tests/fixtures/README.md`'s new
`joint_tracker` section document the proposed `.npz` fixture schema
(per-frame rigid-body pose/velocity/centroid in, per-RB-pair most-probable
joint type + joint state out) — **proposed**, not yet confirmed against the
actual Docker-side export. Default `MultiJointTracker` construction
parameters in the harness are matched to
`joint_tracker/cfg/joint_tracker_cfg.yaml`'s documented defaults (`min_num_frames_for_new_rb`
matched to the value `test_rb_tracker.cpp` already uses, since the original
reads this parameter from `/rb_tracker/...`, shared with `rb_tracker`'s own
config). Same "ported, unvalidated, awaiting fixtures" behavior as
Phases 2/3 until fixtures land.

### 4.3 BFL vendoring

No changes needed to `thirdparty/bfl/CMakeLists.txt` — every BFL source
`joint_tracker` needs (`filter/extendedkalmanfilter.cpp`,
`pdf/linearanalyticconditionalgaussian.cpp`,
`pdf/analyticconditionalgaussian_additivenoise.cpp`,
`model/linearanalyticsystemmodel_gaussianuncertainty.cpp`,
`model/analyticmeasurementmodel_gaussianuncertainty.cpp`, `pdf/gaussian.cpp`)
was already vendored and compiled in Phase 1, confirmed by a clean build.
One toolchain-only addition (no behavior change, same category as prior
`#include` fixes): explicit `#include <boost/math/special_functions/round.hpp>`
in `RigidJointFilter.cpp`/`PrismaticJointFilter.cpp`/`RevoluteJointFilter.cpp`
for `boost::math::round()` — the original relied on this being pulled in
transitively via some ROS-era header chain that no longer exists once the
ROS includes are gone.

## Phase 5 — pybind11 bindings
Status: **done**. `import omip_core` works from Python and round-trips a
synthetic frame through all 3 ported stages end-to-end
(`bindings/test_bindings_smoke.py`, wired into CTest as `test_bindings_smoke`,
passing) — the Phase 5 exit criterion.

### 5.1 New dependency

Installed `pybind11` via Homebrew (`brew install pybind11`, 3.0.4) — not
previously a project dependency. Low-risk, standard, reversible
(`brew uninstall pybind11`); needed to do this phase's actual work at all.

### 5.2 What was bound, and what wasn't

Per the mission brief's own instruction ("keep the binding surface small
and stable — avoid exposing every internal class"), only the 3 top-level
stage classes and the plain data types that cross their public interfaces
are exposed in `bindings/omip_core_py.cpp`:

- **Classes:** `PointFeatureTracker`, `MultiRBTracker`, `MultiJointTracker` —
  constructors, the `setMeasurement`/`predictState`/`predictMeasurement`/
  `correctState`/`getState` cycle each already has, and every individual
  `set*` configuration method (mechanical 1:1 binding, same names, matching
  "preserve names" — no new config-struct abstraction invented for
  `MultiRBTracker`/`MultiJointTracker` since their real C++ constructor/
  setter shape already matches the original Filter classes' own API).
- **Plain data types (`types.hpp`):** `Twistd`, `CameraIntrinsics`,
  `StampedImage`, `TwistWithCovariance`, `RigidBodyPoseAndVel`,
  `RigidBodyPosesAndVels`, `JointType`, `JointModel`, `KinematicStructure`,
  `FeatureTrackerConfig`. `Eigen::Vector3d`/`Matrix2d`/`Matrix3d`/
  `Matrix<double,6,6>` fields convert to/from numpy arrays automatically via
  `pybind11/eigen.h` — no manual glue needed for any of them.
- **`FeatureCloudPCLwc`** (`ft_state_t == rbt_measurement_t`, the type
  flowing feature_tracker → rb_tracker) is bound **opaquely** — just enough
  to hold and pass it through Python (`py::class_<..., FeatureCloudPCLwc::Ptr>`
  + a `.size()` accessor), no per-point inspection exposed. This was free:
  confirmed (`pcl/memory.h` in the installed PCL 1.15) that
  `pcl::shared_ptr` is `std::shared_ptr`, matching pybind11's default
  holder with zero extra conversion code.
- **`RigidBodyPosesAndVels`** is *also* `rbt_state_t == ks_measurement_t` —
  rb_tracker's output is already joint_tracker's input type verbatim, so
  no conversion glue was needed between those two stages either.
- **Deliberately NOT bound:** `Feature`, `FeaturesDataBase`, `RBFilter`,
  `StaticEnvironmentFilter`, `JointFilter` and its 4 concrete subtypes,
  `JointCombinedFilter`, anything from BFL, `PoseWithCovariance`/
  `Eigen::Isometry3d` (not needed — `RigidBodyPoseAndVel` uses
  `TwistWithCovariance`, not `PoseWithCovariance`), and
  `MultiJointTracker::getState()` itself (returns the internal
  `KinematicModel`, a map of `JointCombinedFilterPtr` — see 5.3).
- `make_stamped_image_rgb(numpy HxWx3 uint8, timestamp_ns)` /
  `make_stamped_image_depth(numpy HxW float32, timestamp_ns)`: free
  functions building a `StampedImage` from a numpy array, matching the
  exact `encoding` conventions (`"bgr8"`/`"32FC1"`) and construction
  pattern already used in `tests/test_feature_tracker.cpp`'s
  `loadRgb`/`loadDepth` helpers.

### 5.3 `MultiJointTracker::getKinematicStructure()` — new method

`MultiJointTracker::getState()` returns the internal `KinematicModel`
(`std::map<std::pair<int,int>, JointCombinedFilterPtr>`) — an internal
class, not something Python should ever see per the "small stable surface"
instruction. Added a **new** method, `getKinematicStructure() const ->
KinematicStructure`, to `MultiJointTracker` (declared in
`joint_tracker/MultiJointTracker.h`, implemented in `MultiJointTracker.cpp`)
that does the conversion — this is *not* in the original Filter class; it
is a direct, field-for-field port of what the original ROS Node layer did
in `MultiJointTrackerNode::_generateKinematicStructureMessage()` (building
an `omip_msgs::KinematicStructureMsg` from `getState()`), moved onto the
Filter as "a plain class method the orchestrator can call directly" — the
exact pattern the mission brief's own Phase 2 section names as the correct
way to carry over non-ROS-specific Node-layer logic.

Verified field-for-field against the original Node method: for every
tracked RB pair, pulls each of the 4 joint filters' probability
(`rigid_probability`, `discon_probability`, `rev_probability`,
`prism_probability`), the prismatic and revolute filters' position/
orientation/orientation-covariance/joint-value, and the combined filter's
`getMostProbableJointFilter()->getJointFilterType()`.

- **J19 —** `JointFilterType` (joint_tracker's internal C++ enum:
  `RIGID_JOINT=0, PRISMATIC_JOINT=1, REVOLUTE_JOINT=2, DISCONNECTED_JOINT=3`)
  and `JointType` (`types.hpp`'s plain-data enum for `JointModel`, designed
  in Phase 0 before `joint_tracker` existed in this port:
  `Disconnected=0, Rigid=1, Prismatic=2, Revolute=3`) turned out to use
  *different* integer orderings for the same 4 concepts. The original
  Node code did a raw `(int)getJointFilterType()` cast (there was no
  `JointType`-equivalent enum to map to on the ROS side — `most_likely_joint`
  was a bare `int32` in `omip_msgs::JointMsg`). Since `JointType` is this
  port's own Phase-0-designed replacement with no ROS wire-format
  constraint to preserve, a raw `static_cast<JointType>(getJointFilterType())`
  would have silently mismapped (e.g. `RIGID_JOINT`(0) → `JointType::Disconnected`(0)).
  Added an explicit `toJointType()` switch instead (in `MultiJointTracker.cpp`,
  anonymous namespace) so the two independently-meaningful integer
  encodings don't get conflated.
- Confirmed (by re-reading the original Node method) that it never sets
  `joint_msg.rev_position_uncertainty` despite the field existing on the
  message — a pre-existing gap in the original Node layer, not something
  introduced here. `getKinematicStructure()` reproduces this exactly:
  `JointModel::rev_position_uncertainty` is left at its default-constructed
  zero value, not fixed.

### 5.4 CMake / build wiring

- `omip_core/bindings/CMakeLists.txt`: `pybind11_add_module(omip_core_python omip_core_py.cpp)`
  — the CMake *target* is named `omip_core_python` (not `omip_core`, which
  the C++ static library target already owns), but
  `set_target_properties(... OUTPUT_NAME omip_core LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/python)`
  makes the compiled artifact `python/omip_core.cpython-<abi>.so` —
  loadable as `import omip_core`. (First attempt nested it one directory
  too deep — `python/omip_core/omip_core.<abi>.so` — which made Python
  treat `python/omip_core/` as an empty implicit namespace package instead
  of finding the extension module inside it; fixed by not nesting.)
- Top-level `CMakeLists.txt`: new `OMIP_CORE_BUILD_PYTHON_BINDINGS` option
  (default `ON`), gated on `find_package(Python3 ...)` + `find_package(pybind11 CONFIG QUIET)`
  both succeeding, so a machine without Python dev headers or pybind11 can
  still configure/build/test the C++-only parts of `omip_core` — confirmed
  by a from-scratch `cmake ..` with no hints, which found a working
  Homebrew Python3.14 automatically and cleanly skipped the (numpy-requiring)
  Python smoke test with a status message rather than failing, since numpy
  wasn't installed for that particular interpreter.
- `bindings/test_bindings_smoke.py` wired into CTest as `test_bindings_smoke`,
  same "don't fail the whole suite over an optional piece" philosophy:
  skipped (not failed) at configure time if `import numpy` fails for
  whatever `Python3_EXECUTABLE` CMake found.

## Phase 6 — `omip_mujoco_wrapper`
Status: **done**, exit criterion met. `examples/run_demo.py` runs standalone
(no ROS installed anywhere on the machine), renders a synthetic MuJoCo
scene, feeds it through the full ported pipeline via `omip_core`'s Phase 5
bindings, and prints a joint type + parameter estimate for the demo object
— confirmed by actually running it (not just reading the code): it
correctly and confidently identifies the demo's sliding drawer as a
**prismatic** joint (probability ≈0.80 vs. 0.20 for disconnected, ≈0 for
rigid/revolute) with a plausible (same-order-of-magnitude, not exact)
displacement estimate.

### 6.1 What was built

New package `omip_mujoco_wrapper/` (separate from `omip_core`, per the
mission brief's layout), thin on purpose:
- `omip_mujoco_wrapper/driver.py` — `MuJoCoDriver`: owns the
  `mujoco.MjModel`/`MjData`/`Renderer`, exposes `render_rgbd()` (RGB+depth
  numpy arrays) and `camera_intrinsics()`. The only module that imports
  `mujoco` — per the mission brief, `omip_core` must stay simulator-free
  and this wrapper must stay ROS-porting-free, so MuJoCo-specific code is
  quarantined to this one file.
- `omip_mujoco_wrapper/orchestrator.py` — `OmipOrchestrator`: constructs
  the 3 `omip_core` stage objects (parameters matching the same cfg-yaml
  defaults used throughout Phases 2-5's test harnesses) and exposes
  `process_frame(rgb_bgr, depth, timestamp_ns)`, which runs the
  `setMeasurement`/`predictState`/`predictMeasurement`/`correctState`
  sequence through all 3 stages and returns the rigid-body state +
  kinematic structure. Deliberately MuJoCo-agnostic — takes plain arrays,
  so the same orchestrator works against any RGB-D source (a real depth
  camera, once available — see the conversation history around
  RealSense/LiDAR-phone options).
- `omip_mujoco_wrapper/scene_utils.py` — builds the demo MJCF scene and a
  `linear_ramp()` helper for scripting the interaction (directly
  prescribing the joint's qpos over time, per the mission brief's
  "scripted...actuated joint motion" option — no physical
  actuator/controller needed since this isn't testing MuJoCo's dynamics).
- `examples/run_demo.py` — wires the three together, runs 150 simulated
  frames of a drawer sliding open, prints the final joint estimate.
- `pyproject.toml` + `README.md`: minimal `setuptools` packaging;
  `omip_core` is deliberately **not** listed as a pip dependency (nothing
  publishes it yet — see 6.4) — `omip_mujoco_wrapper/__init__.py` instead
  auto-adds `omip_core/build/python` to `sys.path` if `omip_core` isn't
  already importable.
- New dev-only dependency: a `.venv` at the repo root with `mujoco`,
  `numpy` installed (not committed; `.gitignore`'d).

### 6.2 Demo scene: a drawer, not the hinged door tried first

The mission brief's Phase 6 description suggests "e.g. a hinged door or
drawer" — a hinged door (revolute joint) was tried first, since it was the
more visually obvious "articulated object" example. It was set aside after
substantial tuning effort for a concrete reason, not aesthetics:

- Once a second rigid body (the door) was reliably detected and a joint
  filter initialized, `RevoluteJointFilter`'s probability stayed
  essentially zero (`rev_prob` sometimes exactly `0.000`, sometimes ~1e-3)
  across many parameter sweeps (system-noise covariances, rotation speed/
  range, RANSAC thresholds, rigid-body-tracker thresholds, MuJoCo depth
  clip planes) — `PrismaticJointFilter` or `DisconnectedJointFilter` won
  the model-selection race instead, even though the true motion was purely
  rotational. This was investigated as a possible **porting** bug first
  (same rigor as Phase 4's degenerate-input finding), but the ported
  `RevoluteJointFilter`/`NonLinearRevoluteMeasurementPdf` code itself was
  re-checked against PORTING_NOTES.md's Phase 4 record and found
  unchanged/correct; the issue is EKF **convergence/calibration** for this
  specific synthetic trajectory (large-angle rotation over a short,
  compressed demo timeline, plus whatever depth-precision limits MuJoCo's
  offscreen renderer has), not a mistranslated formula.
- The same scene/pipeline, switched to a **drawer** (prismatic joint,
  simpler linear motion, no large-angle EKF linearization or spherical-
  coordinate axis-fitting involved) converged cleanly and repeatably on
  the first real attempt: stable rigid-body tracking (no spurious
  loss/recreation of the tracked body) and a confidently-correct
  PRISMATIC classification.
- Since the mission brief explicitly names "a hinged door **or drawer**"
  as an acceptable example object, switching is a legitimate choice, not a
  workaround — but the hinged-door gap is real and worth recording:
  **getting a revolute demo to classify correctly is flagged as a
  reasonable, still-open follow-up**, not silently swept under the rug.
  (See `omip_mujoco_wrapper/README.md`'s "Known limitations" for the
  user-facing version of this note.)

### 6.3 Two MuJoCo-specific rendering findings (not omip_core issues)

Both discovered empirically while getting the demo scene to actually
produce trackable features and valid depth — neither is a porting
decision, both are just MuJoCo/rendering facts worth recording so they
don't need re-discovering:

- **`builtin="checker"` 2D textures don't tile correctly on `box` geom
  faces in this MuJoCo version** (verified: renders as either a flat solid
  color or 1-axis stripes depending on `texrepeat`/`texuniform`, never a
  real 2-axis checkerboard — confirmed separately that the *same* texture
  on a `plane` geom renders a correct checkerboard). Since `goodFeaturesToTrack`
  needs real 2D texture variation to find corners, and MuJoCo requires
  `plane` geoms to live in static bodies (can't attach one to the moving
  drawer), the drawer and cabinet faces are built as a literal grid of
  small alternating-color `box` geoms (a "physical" checkerboard) instead
  of a single textured box — see `scene_utils._tile_grid_xml()`. This
  sidesteps the texture-mapping limitation entirely and is arguably more
  robust (real geometric edges, not a texture that could render
  inconsistently).
- **Empty background renders at a very large ("far-plane") depth value**
  (tens of meters, in a scene where everything real is 1-3m away) unless
  every pixel in frame is covered by real, finite geometry. The demo scene
  adds a floor and backdrop wall (both `plane` geoms, sized to fill the
  whole camera view) purely so no pixel ever reports a bogus depth reading
  that could corrupt feature triangulation. Also tightened MuJoCo's
  `<visual><map znear=".." zfar=".."/>` to roughly the scene's real depth
  range (0.3-6m instead of MuJoCo's much larger defaults) for better
  z-buffer precision in that range.

### 6.4 Known follow-ups (not blocking, recorded for later)

- Hinged-door (revolute) demo classification (6.2).
- `omip_core` has no build backend that compiles its C++/pybind11 side as
  part of `pip install` — it's wired into `omip_mujoco_wrapper` via a
  `sys.path` bootstrap (`omip_mujoco_wrapper/__init__.py`) pointing at the
  locally-built `omip_core/build/python`, not a real packaging story. Fine
  for this mission's scope (a single-machine, build-it-yourself research
  pipeline) but would need a proper `scikit-build-core`-based build (or
  similar) to be pip-installable/distributable.
- The prismatic joint's estimated displacement (`prism_joint_value`)
  tracks the true slide distance in the right order of magnitude and
  direction but isn't a tight numerical match (e.g. ≈0.29m estimated vs.
  0.40m true at the end of the demo) — plausible given no golden fixtures
  ever validated the estimator's absolute numerical accuracy (Phases 2-4
  are all still "ported, unvalidated, awaiting fixtures") and given the
  demo's synthetic camera/noise parameters were tuned for stable
  classification, not for parameter-estimation accuracy. Not investigated
  further this phase — flagged for whenever real golden fixtures (or a
  more careful demo-parameter calibration pass) become available.

## Phase 7 — `shape_tracker` / `shape_reconstruction` (optional)
Status: not started; not pursued unless requested after Phase 6.
