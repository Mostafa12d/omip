// Plain structs replacing omip_msgs and the ROS geometry_msgs/sensor_msgs
// types that leaked into Filter-class public interfaces in the original
// codebase. See PORTING_NOTES.md sections 0.2/0.3 for the full translation
// table and rationale behind each replacement.
#ifndef OMIP_CORE_TYPES_HPP_
#define OMIP_CORE_TYPES_HPP_

#include <array>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

#include "omip_core/LieGroup.hpp"

namespace omip
{

typedef long int RB_id_t;
typedef long int Joint_id_t;

// --- geometry_msgs replacements ---------------------------------------

// Replaces geometry_msgs::PoseWithCovariance.
struct PoseWithCovariance
{
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
};

// Replaces geometry_msgs::TwistWithCovariance.
struct TwistWithCovariance
{
    Twistd twist;
    Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
};

// --- sensor_msgs replacements ------------------------------------------

// Replaces sensor_msgs::Image + cv_bridge::CvImage. `encoding` preserves
// the original ROS/OpenCV encoding string (e.g. "bgr8", "mono8", "32FC1",
// "16UC1") since several ported algorithms branch on it explicitly.
struct StampedImage
{
    cv::Mat image;
    std::string encoding;
    double timestamp_ns = 0.0;

    typedef std::shared_ptr<StampedImage> Ptr;
};

// Replaces sensor_msgs::CameraInfo. Only the fields actually read anywhere
// in the ported Filter classes are kept (the intrinsic matrix K and
// projection matrix P, plus image size) — no distortion model/ROI/binning,
// which the original code never used outside the Node layer.
struct CameraIntrinsics
{
    int width = 0;
    int height = 0;
    // Row-major 3x3 intrinsic matrix, same layout as sensor_msgs::CameraInfo::K.
    std::array<double, 9> K{};
    // Row-major 3x4 projection matrix, same layout as sensor_msgs::CameraInfo::P.
    std::array<double, 12> P{};

    typedef std::shared_ptr<CameraIntrinsics> Ptr;
};

// --- omip_msgs replacements ---------------------------------------------

// Replaces omip_msgs::RigidBodyPoseAndVelMsg. Field kept named `pose_wc`
// even though it (like the original message) holds a twist/pose hybrid
// representation — see PORTING_NOTES.md judgment call J1.
struct RigidBodyPoseAndVel
{
    RB_id_t rb_id = 0;
    TwistWithCovariance pose_wc;
    TwistWithCovariance velocity_wc;
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
};

// Replaces omip_msgs::RigidBodyPoseMsg.
struct RigidBodyPose
{
    RB_id_t rb_id = 0;
    PoseWithCovariance pose_wc;
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
};

// Replaces omip_msgs::RigidBodyPosesAndVelsMsg. This is `rbt_state_t` /
// `ks_measurement_t` in OMIPTypeDefs.h — the Filter-internal state of
// rb_tracker and measurement of joint_tracker, not just a wire message.
struct RigidBodyPosesAndVels
{
    double timestamp_ns = 0.0;
    std::vector<RigidBodyPoseAndVel> rb_poses_and_vels;
};

// Replaces omip_msgs::RigidBodyPosesMsg.
struct RigidBodyPoses
{
    double timestamp_ns = 0.0;
    std::vector<RigidBodyPose> rb_poses;
};

// Replaces omip_msgs::RigidBodyTwistWithCovMsg.
struct RigidBodyTwistWithCov
{
    RB_id_t rb_id = 0;
    TwistWithCovariance twist_wc;
};

// Replaces omip_msgs::RigidBodyTwistsWithCovMsg.
struct RigidBodyTwistsWithCov
{
    double timestamp_ns = 0.0;
    std::vector<RigidBodyTwistWithCov> twists_wc;
};

// Replaces omip_msgs::RigidBodyBrakingEvent. Not referenced anywhere in
// the ported packages' C++ source (see PORTING_NOTES.md 0.2) — kept for
// completeness only.
struct RigidBodyBrakingEvent
{
    double timestamp_ns = 0.0;
    std::vector<RB_id_t> braking_rb_ids;
};

// Joint model kind, replacing JointMsg::most_likely_joint plus the
// disconnected/rigid/prismatic/revolute discriminator used throughout
// joint_tracker.
enum class JointType
{
    Disconnected = 0,
    Rigid = 1,
    Prismatic = 2,
    Revolute = 3
};

// Replaces omip_msgs::JointMsg. One struct covering every joint type's
// fields (mirroring the original single-message-many-fields layout) since
// JointCombinedFilter always evaluates all 4 models together.
struct JointModel
{
    RB_id_t parent_rb_id = 0;
    RB_id_t child_rb_id = 0;
    JointType most_likely_joint = JointType::Disconnected;

    // Prismatic joint
    float prism_probability = 0.f;
    Eigen::Vector3d prism_position = Eigen::Vector3d::Zero();
    double prism_ori_phi = 0.0;
    double prism_ori_theta = 0.0;
    Eigen::Matrix2d prism_ori_cov = Eigen::Matrix2d::Zero();
    Eigen::Vector3d prism_orientation = Eigen::Vector3d::Zero();
    double prism_joint_value = 0.0;

    // Revolute joint
    float rev_probability = 0.f;
    Eigen::Vector3d rev_position = Eigen::Vector3d::Zero();
    Eigen::Matrix3d rev_position_uncertainty = Eigen::Matrix3d::Zero();
    double rev_ori_phi = 0.0;
    double rev_ori_theta = 0.0;
    Eigen::Matrix2d rev_ori_cov = Eigen::Matrix2d::Zero();
    Eigen::Vector3d rev_orientation = Eigen::Vector3d::Zero();
    double rev_joint_value = 0.0;

    // Disconnected joint
    float discon_probability = 0.f;

    // Rigid joint
    float rigid_probability = 0.f;
};

// Replaces omip_msgs::KinematicStructureMsg.
struct KinematicStructure
{
    double timestamp_ns = 0.0;
    std::vector<JointModel> joints;
};

// Measurement type for the combined joint filter and all joint filters;
// replaces the original std::pair<omip_msgs::RigidBodyPoseAndVelMsg,
// omip_msgs::RigidBodyPoseAndVelMsg>.
typedef std::pair<RigidBodyPoseAndVel, RigidBodyPoseAndVel> joint_measurement_t;

// --- Phase 7 (shape_tracker/shape_reconstruction) placeholders ----------
// Declared now (plain structs only, no Filter code ported yet) since they
// are simple and already fully specified by the Phase 0 translation table;
// the Filter classes that use them are not touched until Phase 7.

struct ShapeTrackerState
{
    RB_id_t rb_id = 0;
    TwistWithCovariance pose_wc;
    int num_points_of_model = 0;
    int num_points_of_current_pc = 0;
    float probabilistic_value = 0.f;
    float fitness_score = 0.f;
};

struct ShapeTrackerStates
{
    double timestamp_ns = 0.0;
    std::vector<ShapeTrackerState> shape_tracker_states;
};

} // namespace omip

#endif // OMIP_CORE_TYPES_HPP_
