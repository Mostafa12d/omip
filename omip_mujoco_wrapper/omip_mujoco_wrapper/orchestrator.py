"""Calls the 3 ported omip_core stages (feature_tracker -> rb_tracker ->
joint_tracker) in sequence, once per frame.

This module is MuJoCo-agnostic on purpose: it only takes plain RGB/depth
numpy arrays + an omip_core.CameraIntrinsics + a timestamp, so the same
orchestrator would work unchanged with frames from a real depth camera
(RealSense, LiDAR phone, ...) instead of MuJoCo — see the driver/orchestrator
split described in the mission brief's target layout.

Default construction parameters below match the ones already validated
across Phases 2-5 (test_rb_tracker.cpp / test_joint_tracker.cpp / the
Phase 5 bindings smoke test), which themselves match the original ROS
packages' documented cfg-yaml defaults — see PORTING_NOTES.md.
"""
from __future__ import annotations

import dataclasses

import omip_core as oc


@dataclasses.dataclass
class FrameResult:
    rigid_bodies: "oc.RigidBodyPosesAndVels"
    kinematic_structure: "oc.KinematicStructure"


class OmipOrchestrator:
    def __init__(self, loop_period_ns: float, number_features: int = 200):
        ft_config = oc.FeatureTrackerConfig()
        ft_config.number_features = number_features
        ft_config.min_number_features = 0
        self.feature_tracker = oc.PointFeatureTracker(loop_period_ns, ft_config)

        self.rb_tracker = oc.MultiRBTracker(
            10000,           # max_num_rb
            loop_period_ns,
            1000,            # ransac_iterations
            0.05,            # estimation_error_threshold
            0.02,            # static_motion_threshold
            0.03,            # new_rbm_error_threshold
            0.05,            # max_error_to_reassign_feats
            5,               # supporting_features_threshold
            20,              # min_num_feats_for_new_rb
            20,              # min_num_frames_for_new_rb
            6,               # initial_cam_motion_constraint
            oc.StaticEnvironmentTrackerType.EKF,
        )
        self.rb_tracker.setMinCovarianceMeasurementX(0.03)
        self.rb_tracker.setMinCovarianceMeasurementY(0.03)
        self.rb_tracker.setMinCovarianceMeasurementZ(0.03)
        self.rb_tracker.setMeasurementDepthFactor(100.0)
        self.rb_tracker.setCovarianceSystemAccelerationTx(0.02)
        self.rb_tracker.setCovarianceSystemAccelerationTy(0.02)
        self.rb_tracker.setCovarianceSystemAccelerationTz(0.02)
        self.rb_tracker.setCovarianceSystemAccelerationRx(0.2)
        self.rb_tracker.setCovarianceSystemAccelerationRy(0.2)
        self.rb_tracker.setCovarianceSystemAccelerationRz(0.2)
        self.rb_tracker.setPriorCovariancePose(0.05)
        self.rb_tracker.setPriorCovarianceVelocity(0.1)
        self.rb_tracker.setMinNumPointsInSegment(300)
        self.rb_tracker.setMinProbabilisticValue(0.9)
        self.rb_tracker.setMaxFitnessScore(0.005)
        self.rb_tracker.setMinAmountTranslationForNewRB(0.03)
        self.rb_tracker.setMinAmountRotationForNewRB(6.0)
        self.rb_tracker.setMinNumberOfSupportingFeaturesToCorrectPredictedState(5)
        self.rb_tracker.setNumberOfTrackedFeatures(number_features)
        self.rb_tracker.Init()

        self.joint_tracker = oc.MultiJointTracker(loop_period_ns, oc.KsAnalysisType.FULL_ANALYSIS, 0.1)
        self.joint_tracker.setNumSamplesLikelihoodEstimation(100)
        self.joint_tracker.setMinimumJointAgeForEE(3)
        self.joint_tracker.setSigmaDeltaMeasurementUncertaintyLinear(0.03)
        self.joint_tracker.setSigmaDeltaMeasurementUncertaintyAngular(1)
        self.joint_tracker.setPrismaticPriorCovarianceVelocity(0.5)
        self.joint_tracker.setPrismaticSigmaSystemNoisePhi(2.55)
        self.joint_tracker.setPrismaticSigmaSystemNoiseTheta(2.55)
        self.joint_tracker.setPrismaticSigmaSystemNoisePV(0.9)
        self.joint_tracker.setPrismaticSigmaSystemNoisePVd(75)
        self.joint_tracker.setPrismaticSigmaMeasurementNoise(0.9)
        self.joint_tracker.setRevolutePriorCovarianceVelocity(1.0)
        self.joint_tracker.setRevoluteSigmaSystemNoisePhi(2.55)
        self.joint_tracker.setRevoluteSigmaSystemNoiseTheta(2.55)
        self.joint_tracker.setRevoluteSigmaSystemNoisePx(0.3)
        self.joint_tracker.setRevoluteSigmaSystemNoisePy(0.3)
        self.joint_tracker.setRevoluteSigmaSystemNoisePz(0.3)
        self.joint_tracker.setRevoluteSigmaSystemNoiseRV(5.1)
        self.joint_tracker.setRevoluteSigmaSystemNoiseRVd(75)
        self.joint_tracker.setRevoluteSigmaMeasurementNoise(0.05)
        self.joint_tracker.setRevoluteMinimumRotForEstimation(0.045)
        self.joint_tracker.setRevoluteMaximumJointDistanceForEstimation(0.5)
        self.joint_tracker.setRigidMaxTranslation(0.05)
        self.joint_tracker.setRigidMaxRotation(0.1)
        self.joint_tracker.setMinimumNumFramesForNewRB(20)

        self.loop_period_ns = loop_period_ns

    def set_camera_info(self, intrinsics: "oc.CameraIntrinsics") -> None:
        self.feature_tracker.setCameraInfoMsg(intrinsics)

    def process_frame(self, rgb_bgr, depth, timestamp_ns: float) -> FrameResult:
        """Runs one predict/correct cycle through all 3 stages, feeding
        rb_tracker's output directly as joint_tracker's input (they share
        the same plain RigidBodyPosesAndVels type — see
        omip_core/bindings/omip_core_py.cpp's module docstring)."""
        rgb_img = oc.make_stamped_image_rgb(rgb_bgr, timestamp_ns)
        depth_img = oc.make_stamped_image_depth(depth, timestamp_ns)

        self.feature_tracker.setMeasurement(rgb_img, depth_img, timestamp_ns)
        self.feature_tracker.predictState(self.loop_period_ns)
        self.feature_tracker.predictMeasurement()
        self.feature_tracker.correctState()
        features = self.feature_tracker.getState()

        self.rb_tracker.setMeasurement(features, timestamp_ns)
        self.rb_tracker.predictState(self.loop_period_ns)
        self.rb_tracker.predictMeasurement()
        self.rb_tracker.correctState()
        self.rb_tracker.ReflectState()
        rb_state = self.rb_tracker.getState()

        self.joint_tracker.setMeasurement(rb_state, timestamp_ns)
        self.joint_tracker.predictState(self.loop_period_ns)
        self.joint_tracker.predictMeasurement()
        self.joint_tracker.correctState()
        self.joint_tracker.estimateJointFiltersProbabilities()
        kinematic_structure = self.joint_tracker.getKinematicStructure()

        return FrameResult(rigid_bodies=rb_state, kinematic_structure=kinematic_structure)
