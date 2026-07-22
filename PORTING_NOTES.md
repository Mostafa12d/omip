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

**Resolution:** _pending._

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

**Resolution:** _pending._

### Q3 — Is `std_msgs::Header`'s `frame_id` genuinely unused inside Filter logic, or did I miss a spot?

I traced every `header.frame_id` assignment in `feature_tracker`, `rb_tracker`,
`joint_tracker` and found all of them happen only in `*Node` publish methods
(TF broadcast / RViz), never read back inside a Filter class to affect a
computation. If that matches your understanding of the code, I'll drop
`frame_id` entirely from `omip_core` types (per the mission brief's own
suggestion to check whether it's "load-bearing... or just ROS boilerplate").
Flagging since a wrong guess here would be a silent behavior change that's
easy to miss in review.

**Resolution:** _pending._

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
Status: not started.

## Phase 2 — Port `feature_tracker`
Status: not started.

## Phase 3 — Port `rb_tracker`
Status: not started.

## Phase 4 — Port `joint_tracker`
Status: not started.

## Phase 5 — pybind11 bindings
Status: not started.

## Phase 6 — `omip_mujoco_wrapper`
Status: not started.

## Phase 7 — `shape_tracker` / `shape_reconstruction` (optional)
Status: not started; not pursued unless requested after Phase 6.
