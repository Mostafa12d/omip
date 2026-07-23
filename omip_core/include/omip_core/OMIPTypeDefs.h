/*
 * OMIPTypeDefs.h
 *
 *      Author: roberto
 *
 * This is a modified implementation of the method for online estimation of kinematic structures described in our paper
 * "Online Interactive Perception of Articulated Objects with Multi-Level Recursive Estimation Based on Task-Specific Priors"
 * (Martín-Martín and Brock, 2014).
 *
 * Ported to a ROS-free library — see PORTING_NOTES.md, section 0.3 (last
 * row) for the rationale: the ROS message typedefs here (rbt_state_t,
 * ks_measurement_t, joint_measurement_t) were the Filter classes' actual
 * internal state/measurement representation, not just wire types. The
 * *_ros_t Node-only wire typedefs (ft_measurement_ros_t, ft_state_ros_t,
 * rbt_measurement_ros_t, rbt_state_ros_t, ks_measurement_ros_t,
 * ks_state_ros_t) are dropped entirely: there is no ROS wire format in a
 * single-process library, and the mission's orchestrator calls stages
 * directly with the Filter-side types below.
 *
 * OMIP_ADD_POINT4D: moved EIGEN_ALIGN16 from suffix to prefix position on
 * the inner anonymous union (`EIGEN_ALIGN16 union {...};` instead of
 * `union {...} EIGEN_ALIGN16;`) — modern Eigen expands this macro to the
 * C++11 `alignas(16)` keyword, which must precede a declaration, not
 * follow it (the outer struct below already used prefix position). Same
 * category of toolchain-compatibility fix as elsewhere in PORTING_NOTES.md
 * (BFL's rng.cpp, lgsm) — no behavior change, just updated syntax.
 */

#ifndef OMIPTYPEDEFS_H_
#define OMIPTYPEDEFS_H_

#include <map>
#include <utility>

#include <boost/shared_ptr.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "omip_core/types.hpp"

#define OMIP_ADD_POINT4D \
  EIGEN_ALIGN16 union { \
    float data[4]; \
    struct { \
      float x; \
      float y; \
      float z; \
    }; \
  }; \
  inline Eigen::Map<Eigen::Vector3f> getVector3fMap () { return (Eigen::Vector3f::Map (data)); } \
  inline const Eigen::Map<const Eigen::Vector3f> getVector3fMap () const { return (Eigen::Vector3f::Map (data)); } \
  inline Eigen::Map<Eigen::Vector4f, Eigen::Aligned> getVector4fMap () { return (Eigen::Vector4f::MapAligned (data)); } \
  inline const Eigen::Map<const Eigen::Vector4f, Eigen::Aligned> getVector4fMap () const { return (Eigen::Vector4f::MapAligned (data)); } \
  inline Eigen::Map<Eigen::Array3f> getArray3fMap () { return (Eigen::Array3f::Map (data)); } \
  inline const Eigen::Map<const Eigen::Array3f> getArray3fMap () const { return (Eigen::Array3f::Map (data)); } \
  inline Eigen::Map<Eigen::Array4f, Eigen::Aligned> getArray4fMap () { return (Eigen::Array4f::MapAligned (data)); } \
  inline const Eigen::Map<const Eigen::Array4f, Eigen::Aligned> getArray4fMap () const { return (Eigen::Array4f::MapAligned (data)); }

namespace omip
{

//Forward declaration
class JointFilter;

//Forward declaration
class JointCombinedFilter;

typedef pcl::PointXYZ PointPCL;
typedef pcl::PointXYZL FeaturePCL;

struct EIGEN_ALIGN16 _FeaturePCLwc
{
  OMIP_ADD_POINT4D // This adds the members x,y,z which can also be accessed using the point (which is float[4])
  uint32_t label;
  float covariance[9];
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct FeaturePCLwc : public _FeaturePCLwc
{
  inline FeaturePCLwc ()
  {
    x = y = z = 0.0f;
    data[3] = 1.0f;
    label = 0;
    covariance[0] = 1.0f;
    covariance[1] = 0.0f;
    covariance[2] = 0.0f;
    covariance[3] = 0.0f;
    covariance[4] = 1.0f;
    covariance[5] = 0.0f;
    covariance[6] = 0.0f;
    covariance[7] = 0.0f;
    covariance[8] = 1.0f;
  }
};

typedef pcl::PointCloud<PointPCL> PointCloudPCLNoColor;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudPCL;
typedef pcl::PointCloud<FeaturePCL> FeatureCloudPCL;
typedef pcl::PointCloud<FeaturePCLwc> FeatureCloudPCLwc;

typedef pcl::PointCloud<PointPCL> RigidBodyShape;

// boost::shared_ptr (not std::shared_ptr): must match JointCombinedFilterPtr
// (joint_tracker/JointCombinedFilter.h), which MultiJointTracker's
// joint_combined_filters_map is built from and directly assigns into
// ks_state_t/_state (see MultiJointTracker::_reflectState(), Phase 4).
// Originally this file (Phase 0/2, before joint_tracker existed in this
// port) had this as std::shared_ptr — an unintentional deviation from the
// original ROS source (which used boost::shared_ptr here too) that went
// unnoticed because nothing exercised KinematicModel/ks_state_t until now.
typedef std::map<std::pair<int, int>, boost::shared_ptr<JointCombinedFilter> > KinematicModel;

// Feature Tracker FILTER MEASUREMENT type. The first element is the RGB
// image. The second element is the depth image. Was
// std::pair<cv_bridge::CvImagePtr, cv_bridge::CvImagePtr> — kept as a pair
// of shared pointers (StampedImage::Ptr) to match, rather than by-value,
// since call sites (e.g. PointFeatureTracker::_ProcessRGBImg) assign these
// straight into StampedImage::Ptr members.
typedef std::pair<StampedImage::Ptr, StampedImage::Ptr> ft_measurement_t;

// Feature Tracker FILTER STATE type
typedef FeatureCloudPCLwc::Ptr ft_state_t;

// Rigid Body Tracker FILTER MEASUREMENT type (= ft_state_t).
typedef FeatureCloudPCLwc::Ptr rbt_measurement_t;

// Rigid Body Tracker FILTER STATE type. Was omip_msgs::RigidBodyPosesAndVelsMsg.
typedef RigidBodyPosesAndVels rbt_state_t;

// Kinematic Model Tracker FILTER MEASUREMENT type. Was omip_msgs::RigidBodyPosesAndVelsMsg.
typedef RigidBodyPosesAndVels ks_measurement_t;

// Kinematic Model Tracker FILTER STATE type
typedef KinematicModel ks_state_t;

enum shape_model_selector_t
{
    BASED_ON_DEPTH = 1,
    BASED_ON_COLOR = 2,
    BASED_ON_EXT_DEPTH = 3,
    BASED_ON_EXT_COLOR = 4,
    BASED_ON_EXT_DEPTH_AND_COLOR = 5
};

enum static_environment_tracker_t
{
    STATIC_ENVIRONMENT_EKF_TRACKER = 1,
    STATIC_ENVIRONMENT_ICP_TRACKER = 2
};

}

POINT_CLOUD_REGISTER_POINT_STRUCT(omip::FeaturePCLwc,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (uint32_t, label, label)
                                  (float[9], covariance, covariance))


#endif /* OMIPTYPEDEFS_H_ */
