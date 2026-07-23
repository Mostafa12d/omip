// pybind11 bindings for omip_core (Phase 5 of the ROS-removal port — see
// PORTING_NOTES.md's Phase 5 section).
//
// Deliberately small and stable: only the 3 top-level stage classes
// (PointFeatureTracker, MultiRBTracker, MultiJointTracker) and the plain
// data types that cross their public interfaces are exposed. Everything
// "internal" to a stage — Feature, FeaturesDataBase, RBFilter,
// StaticEnvironmentFilter, JointFilter and its 4 concrete subtypes,
// JointCombinedFilter, all of BFL — stays C++-only, exactly as the mission
// brief's Phase 5 instructions call for ("avoid exposing every internal
// class").
//
// `FeatureCloudPCLwc` (the type flowing feature_tracker -> rb_tracker,
// `ft_state_t == rbt_measurement_t`) is bound opaquely: Python holds and
// passes it straight through without needing to inspect its contents,
// since PCL's `pcl::shared_ptr` is `std::shared_ptr` as of PCL 1.15
// (verified against the installed `pcl/memory.h`), matching pybind11's
// default holder with no extra glue needed.
//
// `MultiRBTracker::getState()` and `MultiJointTracker::setMeasurement()`
// both use the plain `RigidBodyPosesAndVels` struct directly
// (`rbt_state_t == ks_measurement_t`), so no conversion is needed between
// those two stages either.
//
// `MultiJointTracker::getState()` (returns the internal `KinematicModel` —
// a map of `JointCombinedFilterPtr`) is intentionally NOT bound; instead
// `MultiJointTracker::getKinematicStructure()` (added this phase, see
// MultiJointTracker.h/.cpp) is bound, which converts to the plain
// `KinematicStructure`/`JointModel` types already designed in Phase 0 for
// exactly this purpose.
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <opencv2/core/core.hpp>

#include "omip_core/types.hpp"
#include "omip_core/OMIPTypeDefs.h"
#include "omip_core/feature_tracker/FeatureTrackerConfig.h"
#include "omip_core/feature_tracker/PointFeatureTracker.h"
#include "omip_core/rb_tracker/MultiRBTracker.h"
#include "omip_core/joint_tracker/MultiJointTracker.h"

namespace py = pybind11;
using namespace omip;

namespace
{

// Builds a StampedImage from an HxWx3 uint8 numpy array (matches the "bgr8"
// encoding convention used throughout the ported code, e.g.
// tests/test_feature_tracker.cpp's loadRgb()).
StampedImage::Ptr makeStampedImageRGB(py::array_t<uint8_t, py::array::c_style | py::array::forcecast> rgb,
                                       double timestamp_ns)
{
    if (rgb.ndim() != 3 || rgb.shape(2) != 3)
        throw std::runtime_error("make_stamped_image_rgb: expected an HxWx3 uint8 array");

    const int rows = static_cast<int>(rgb.shape(0));
    const int cols = static_cast<int>(rgb.shape(1));

    StampedImage::Ptr img(new StampedImage());
    img->encoding = "bgr8";
    img->timestamp_ns = timestamp_ns;
    img->image = cv::Mat(rows, cols, CV_8UC3, const_cast<uint8_t*>(rgb.data())).clone();
    return img;
}

// Builds a StampedImage from an HxW float32 numpy array (matches the
// "32FC1" encoding convention, e.g. tests/test_feature_tracker.cpp's
// loadDepth()).
StampedImage::Ptr makeStampedImageDepth(py::array_t<float, py::array::c_style | py::array::forcecast> depth,
                                         double timestamp_ns)
{
    if (depth.ndim() != 2)
        throw std::runtime_error("make_stamped_image_depth: expected an HxW float32 array");

    const int rows = static_cast<int>(depth.shape(0));
    const int cols = static_cast<int>(depth.shape(1));

    StampedImage::Ptr img(new StampedImage());
    img->encoding = "32FC1";
    img->timestamp_ns = timestamp_ns;
    img->image = cv::Mat(rows, cols, CV_32FC1, const_cast<float*>(depth.data())).clone();
    return img;
}

} // namespace

PYBIND11_MODULE(omip_core, m)
{
    m.doc() = "ROS-free C++ port of the OMIP kinematic-structure estimation pipeline "
              "(feature_tracker -> rb_tracker -> joint_tracker). See PORTING_NOTES.md.";

    // --- Twistd (LieGroup.hpp) -------------------------------------------
    py::class_<Twistd>(m, "Twistd")
        .def(py::init<>())
        .def(py::init<double, double, double, double, double, double>(),
             py::arg("rx"), py::arg("ry"), py::arg("rz"),
             py::arg("vx"), py::arg("vy"), py::arg("vz"))
        .def_property("rx", [](const Twistd& t) { return t.rx(); }, [](Twistd& t, double v) { t.rx() = v; })
        .def_property("ry", [](const Twistd& t) { return t.ry(); }, [](Twistd& t, double v) { t.ry() = v; })
        .def_property("rz", [](const Twistd& t) { return t.rz(); }, [](Twistd& t, double v) { t.rz() = v; })
        .def_property("vx", [](const Twistd& t) { return t.vx(); }, [](Twistd& t, double v) { t.vx() = v; })
        .def_property("vy", [](const Twistd& t) { return t.vy(); }, [](Twistd& t, double v) { t.vy() = v; })
        .def_property("vz", [](const Twistd& t) { return t.vz(); }, [](Twistd& t, double v) { t.vz() = v; })
        .def_property_readonly("coeffs", [](const Twistd& t) { return t.coeffs(); })
        .def("__repr__", [](const Twistd& t) {
            return "Twistd(rx=" + std::to_string(t.rx()) + ", ry=" + std::to_string(t.ry())
                + ", rz=" + std::to_string(t.rz()) + ", vx=" + std::to_string(t.vx())
                + ", vy=" + std::to_string(t.vy()) + ", vz=" + std::to_string(t.vz()) + ")";
        });

    // --- Plain geometry/sensor structs (types.hpp) -----------------------
    py::class_<CameraIntrinsics>(m, "CameraIntrinsics")
        .def(py::init<>())
        .def_readwrite("width", &CameraIntrinsics::width)
        .def_readwrite("height", &CameraIntrinsics::height)
        .def_readwrite("K", &CameraIntrinsics::K)
        .def_readwrite("P", &CameraIntrinsics::P);

    py::class_<StampedImage, StampedImage::Ptr>(m, "StampedImage")
        .def(py::init<>())
        .def_readwrite("encoding", &StampedImage::encoding)
        .def_readwrite("timestamp_ns", &StampedImage::timestamp_ns);

    m.def("make_stamped_image_rgb", &makeStampedImageRGB, py::arg("rgb"), py::arg("timestamp_ns"),
          "Build a StampedImage (encoding='bgr8') from an HxWx3 uint8 array.");
    m.def("make_stamped_image_depth", &makeStampedImageDepth, py::arg("depth"), py::arg("timestamp_ns"),
          "Build a StampedImage (encoding='32FC1') from an HxW float32 array.");

    py::class_<TwistWithCovariance>(m, "TwistWithCovariance")
        .def(py::init<>())
        .def_readwrite("twist", &TwistWithCovariance::twist)
        .def_readwrite("covariance", &TwistWithCovariance::covariance);

    py::class_<RigidBodyPoseAndVel>(m, "RigidBodyPoseAndVel")
        .def(py::init<>())
        .def_readwrite("rb_id", &RigidBodyPoseAndVel::rb_id)
        .def_readwrite("pose_wc", &RigidBodyPoseAndVel::pose_wc)
        .def_readwrite("velocity_wc", &RigidBodyPoseAndVel::velocity_wc)
        .def_readwrite("centroid", &RigidBodyPoseAndVel::centroid);

    py::class_<RigidBodyPosesAndVels>(m, "RigidBodyPosesAndVels")
        .def(py::init<>())
        .def_readwrite("timestamp_ns", &RigidBodyPosesAndVels::timestamp_ns)
        .def_readwrite("rb_poses_and_vels", &RigidBodyPosesAndVels::rb_poses_and_vels);

    py::enum_<JointType>(m, "JointType")
        .value("Disconnected", JointType::Disconnected)
        .value("Rigid", JointType::Rigid)
        .value("Prismatic", JointType::Prismatic)
        .value("Revolute", JointType::Revolute);

    py::class_<JointModel>(m, "JointModel")
        .def(py::init<>())
        .def_readwrite("parent_rb_id", &JointModel::parent_rb_id)
        .def_readwrite("child_rb_id", &JointModel::child_rb_id)
        .def_readwrite("most_likely_joint", &JointModel::most_likely_joint)
        .def_readwrite("prism_probability", &JointModel::prism_probability)
        .def_readwrite("prism_position", &JointModel::prism_position)
        .def_readwrite("prism_ori_phi", &JointModel::prism_ori_phi)
        .def_readwrite("prism_ori_theta", &JointModel::prism_ori_theta)
        .def_readwrite("prism_ori_cov", &JointModel::prism_ori_cov)
        .def_readwrite("prism_orientation", &JointModel::prism_orientation)
        .def_readwrite("prism_joint_value", &JointModel::prism_joint_value)
        .def_readwrite("rev_probability", &JointModel::rev_probability)
        .def_readwrite("rev_position", &JointModel::rev_position)
        .def_readwrite("rev_position_uncertainty", &JointModel::rev_position_uncertainty)
        .def_readwrite("rev_ori_phi", &JointModel::rev_ori_phi)
        .def_readwrite("rev_ori_theta", &JointModel::rev_ori_theta)
        .def_readwrite("rev_ori_cov", &JointModel::rev_ori_cov)
        .def_readwrite("rev_orientation", &JointModel::rev_orientation)
        .def_readwrite("rev_joint_value", &JointModel::rev_joint_value)
        .def_readwrite("discon_probability", &JointModel::discon_probability)
        .def_readwrite("rigid_probability", &JointModel::rigid_probability);

    py::class_<KinematicStructure>(m, "KinematicStructure")
        .def(py::init<>())
        .def_readwrite("timestamp_ns", &KinematicStructure::timestamp_ns)
        .def_readwrite("joints", &KinematicStructure::joints);

    // --- feature_tracker ---------------------------------------------------
    py::class_<FeatureTrackerConfig>(m, "FeatureTrackerConfig")
        .def(py::init<>())
        .def_readwrite("number_features", &FeatureTrackerConfig::number_features)
        .def_readwrite("min_number_features", &FeatureTrackerConfig::min_number_features)
        .def_readwrite("min_feat_quality", &FeatureTrackerConfig::min_feat_quality)
        .def_readwrite("min_distance", &FeatureTrackerConfig::min_distance)
        .def_readwrite("max_distance", &FeatureTrackerConfig::max_distance)
        .def_readwrite("max_interframe_jump", &FeatureTrackerConfig::max_interframe_jump)
        .def_readwrite("erosion_size_detect", &FeatureTrackerConfig::erosion_size_detect)
        .def_readwrite("erosion_size_track", &FeatureTrackerConfig::erosion_size_track)
        .def_readwrite("attention_to_motion", &FeatureTrackerConfig::attention_to_motion)
        .def_readwrite("min_time_to_detect_motion", &FeatureTrackerConfig::min_time_to_detect_motion)
        .def_readwrite("min_depth_difference", &FeatureTrackerConfig::min_depth_difference)
        .def_readwrite("min_area_size_pixels", &FeatureTrackerConfig::min_area_size_pixels)
        .def_readwrite("pub_tracked_feats_with_pred_mask_img", &FeatureTrackerConfig::pub_tracked_feats_with_pred_mask_img)
        .def_readwrite("pub_tracked_feats_img", &FeatureTrackerConfig::pub_tracked_feats_img)
        .def_readwrite("pub_predicted_and_past_feats_img", &FeatureTrackerConfig::pub_predicted_and_past_feats_img)
        .def_readwrite("sensor_fps", &FeatureTrackerConfig::sensor_fps)
        .def_readwrite("processing_factor", &FeatureTrackerConfig::processing_factor);

    py::class_<FeatureCloudPCLwc, FeatureCloudPCLwc::Ptr>(m, "FeatureCloud")
        .def(py::init<>())
        .def("size", &FeatureCloudPCLwc::size);

    py::class_<PointFeatureTracker>(m, "PointFeatureTracker")
        .def(py::init<double, const FeatureTrackerConfig&, bool, std::string>(),
             py::arg("loop_period_ns"), py::arg("config"),
             py::arg("using_pc") = false, py::arg("ft_ns") = std::string(""))
        .def("setCameraInfoMsg", &PointFeatureTracker::setCameraInfoMsg, py::arg("camera_info"))
        .def("setMeasurement", [](PointFeatureTracker& self, StampedImage::Ptr rgb, StampedImage::Ptr depth, double timestamp_ns)
             { self.setMeasurement(ft_measurement_t(rgb, depth), timestamp_ns); },
             py::arg("rgb"), py::arg("depth"), py::arg("timestamp_ns"))
        .def("predictState", &PointFeatureTracker::predictState, py::arg("time_interval_ns"))
        .def("predictMeasurement", &PointFeatureTracker::predictMeasurement)
        .def("correctState", &PointFeatureTracker::correctState)
        .def("getState", &PointFeatureTracker::getState);

    // --- rb_tracker ---------------------------------------------------------
    py::enum_<static_environment_tracker_t>(m, "StaticEnvironmentTrackerType")
        .value("EKF", STATIC_ENVIRONMENT_EKF_TRACKER)
        .value("ICP", STATIC_ENVIRONMENT_ICP_TRACKER);

    py::class_<MultiRBTracker>(m, "MultiRBTracker")
        .def(py::init<int, double, int, double, double, double, double, int, int, int, int, static_environment_tracker_t>(),
             py::arg("max_num_rb"), py::arg("loop_period_ns"), py::arg("ransac_iterations"),
             py::arg("estimation_error_threshold"), py::arg("static_motion_threshold"),
             py::arg("new_rbm_error_threshold"), py::arg("max_error_to_reassign_feats"),
             py::arg("supporting_features_threshold"), py::arg("min_num_feats_for_new_rb"),
             py::arg("min_num_frames_for_new_rb"), py::arg("initial_cam_motion_constraint"),
             py::arg("static_environment_tracker_type"))
        .def("Init", &MultiRBTracker::Init)
        .def("setMeasurement", &MultiRBTracker::setMeasurement, py::arg("acquired_measurement"), py::arg("measurement_timestamp"))
        .def("predictState", &MultiRBTracker::predictState, py::arg("time_interval_ns"))
        .def("predictMeasurement", &MultiRBTracker::predictMeasurement)
        .def("correctState", &MultiRBTracker::correctState)
        .def("ReflectState", &MultiRBTracker::ReflectState)
        .def("getState", &MultiRBTracker::getState)
        .def("setPriorCovariancePose", &MultiRBTracker::setPriorCovariancePose)
        .def("setPriorCovarianceVelocity", &MultiRBTracker::setPriorCovarianceVelocity)
        .def("setMinCovarianceMeasurementX", &MultiRBTracker::setMinCovarianceMeasurementX)
        .def("setMinCovarianceMeasurementY", &MultiRBTracker::setMinCovarianceMeasurementY)
        .def("setMinCovarianceMeasurementZ", &MultiRBTracker::setMinCovarianceMeasurementZ)
        .def("setMeasurementDepthFactor", &MultiRBTracker::setMeasurementDepthFactor)
        .def("setCovarianceSystemAccelerationTx", &MultiRBTracker::setCovarianceSystemAccelerationTx)
        .def("setCovarianceSystemAccelerationTy", &MultiRBTracker::setCovarianceSystemAccelerationTy)
        .def("setCovarianceSystemAccelerationTz", &MultiRBTracker::setCovarianceSystemAccelerationTz)
        .def("setCovarianceSystemAccelerationRx", &MultiRBTracker::setCovarianceSystemAccelerationRx)
        .def("setCovarianceSystemAccelerationRy", &MultiRBTracker::setCovarianceSystemAccelerationRy)
        .def("setCovarianceSystemAccelerationRz", &MultiRBTracker::setCovarianceSystemAccelerationRz)
        .def("setNumberOfTrackedFeatures", &MultiRBTracker::setNumberOfTrackedFeatures)
        .def("setMinNumPointsInSegment", &MultiRBTracker::setMinNumPointsInSegment)
        .def("setMinProbabilisticValue", &MultiRBTracker::setMinProbabilisticValue)
        .def("setMaxFitnessScore", &MultiRBTracker::setMaxFitnessScore)
        .def("setMinAmountTranslationForNewRB", &MultiRBTracker::setMinAmountTranslationForNewRB)
        .def("setMinAmountRotationForNewRB", &MultiRBTracker::setMinAmountRotationForNewRB)
        .def("setMinNumberOfSupportingFeaturesToCorrectPredictedState", &MultiRBTracker::setMinNumberOfSupportingFeaturesToCorrectPredictedState);

    // --- joint_tracker --------------------------------------------------
    py::enum_<ks_analysis_t>(m, "KsAnalysisType")
        .value("MOVING_BODIES_TO_STATIC_ENV", MOVING_BODIES_TO_STATIC_ENV)
        .value("BETWEEN_MOVING_BODIES", BETWEEN_MOVING_BODIES)
        .value("FULL_ANALYSIS", FULL_ANALYSIS);

    py::class_<MultiJointTracker>(m, "MultiJointTracker")
        .def(py::init<double, ks_analysis_t, double>(),
             py::arg("loop_period_ns"), py::arg("ks_analysis_type"), py::arg("dj_ne"))
        .def("setMeasurement", &MultiJointTracker::setMeasurement, py::arg("poses_and_vels"), py::arg("measurement_timestamp_ns"))
        .def("predictState", &MultiJointTracker::predictState, py::arg("time_interval_ns"))
        .def("predictMeasurement", &MultiJointTracker::predictMeasurement)
        .def("correctState", &MultiJointTracker::correctState)
        .def("estimateJointFiltersProbabilities", &MultiJointTracker::estimateJointFiltersProbabilities)
        .def("getKinematicStructure", &MultiJointTracker::getKinematicStructure)
        .def("setNumSamplesLikelihoodEstimation", &MultiJointTracker::setNumSamplesLikelihoodEstimation)
        .def("setSigmaDeltaMeasurementUncertaintyLinear", &MultiJointTracker::setSigmaDeltaMeasurementUncertaintyLinear)
        .def("setSigmaDeltaMeasurementUncertaintyAngular", &MultiJointTracker::setSigmaDeltaMeasurementUncertaintyAngular)
        .def("setPrismaticPriorCovarianceVelocity", &MultiJointTracker::setPrismaticPriorCovarianceVelocity)
        .def("setPrismaticSigmaSystemNoisePhi", &MultiJointTracker::setPrismaticSigmaSystemNoisePhi)
        .def("setPrismaticSigmaSystemNoiseTheta", &MultiJointTracker::setPrismaticSigmaSystemNoiseTheta)
        .def("setPrismaticSigmaSystemNoisePV", &MultiJointTracker::setPrismaticSigmaSystemNoisePV)
        .def("setPrismaticSigmaSystemNoisePVd", &MultiJointTracker::setPrismaticSigmaSystemNoisePVd)
        .def("setPrismaticSigmaMeasurementNoise", &MultiJointTracker::setPrismaticSigmaMeasurementNoise)
        .def("setRevolutePriorCovarianceVelocity", &MultiJointTracker::setRevolutePriorCovarianceVelocity)
        .def("setRevoluteSigmaSystemNoisePhi", &MultiJointTracker::setRevoluteSigmaSystemNoisePhi)
        .def("setRevoluteSigmaSystemNoiseTheta", &MultiJointTracker::setRevoluteSigmaSystemNoiseTheta)
        .def("setRevoluteSigmaSystemNoisePx", &MultiJointTracker::setRevoluteSigmaSystemNoisePx)
        .def("setRevoluteSigmaSystemNoisePy", &MultiJointTracker::setRevoluteSigmaSystemNoisePy)
        .def("setRevoluteSigmaSystemNoisePz", &MultiJointTracker::setRevoluteSigmaSystemNoisePz)
        .def("setRevoluteSigmaSystemNoiseRV", &MultiJointTracker::setRevoluteSigmaSystemNoiseRV)
        .def("setRevoluteSigmaSystemNoiseRVd", &MultiJointTracker::setRevoluteSigmaSystemNoiseRVd)
        .def("setRevoluteSigmaMeasurementNoise", &MultiJointTracker::setRevoluteSigmaMeasurementNoise)
        .def("setRevoluteMinimumRotForEstimation", &MultiJointTracker::setRevoluteMinimumRotForEstimation)
        .def("setRevoluteMaximumJointDistanceForEstimation", &MultiJointTracker::setRevoluteMaximumJointDistanceForEstimation)
        .def("setMinimumJointAgeForEE", &MultiJointTracker::setMinimumJointAgeForEE)
        .def("setRigidMaxTranslation", &MultiJointTracker::setRigidMaxTranslation)
        .def("setRigidMaxRotation", &MultiJointTracker::setRigidMaxRotation)
        .def("setMinimumNumFramesForNewRB", &MultiJointTracker::setMinimumNumFramesForNewRB);
}
