"""Phase 5 exit-criterion smoke test: `import omip_core` and round-trip a
synthetic frame through all 3 ported stages (feature_tracker -> rb_tracker
-> joint_tracker). See PORTING_NOTES.md's Phase 5 section.

This is not a numerical-correctness test (that's the golden-fixture C++
harnesses in tests/) — it only exercises the Python binding surface itself:
construction, the setMeasurement/predictState/predictMeasurement/
correctState call sequence, and that the plain-data outputs
(RigidBodyPosesAndVels, KinematicStructure) come back as well-formed Python
objects with no crash.
"""
import sys

import numpy as np

import omip_core as oc


def make_base_texture(width, height, seed):
    # Textured RGB image (goodFeaturesToTrack needs corners/gradients to
    # find); oversized so later frames can shift a window across it and
    # give the feature tracker real frame-to-frame correspondences to
    # track, rather than an unrelated random image every frame.
    rng = np.random.default_rng(seed)
    return rng.integers(0, 256, size=(height + 20, width + 20, 3), dtype=np.uint8)


def make_synthetic_frame(base_texture, width, height, frame_idx, depth_m=1.0):
    shift = frame_idx  # pixels of horizontal pan per frame
    rgb = base_texture[0:height, shift:shift + width].copy()
    depth = np.full((height, width), depth_m, dtype=np.float32)
    return rgb, depth


def main():
    width, height = 320, 240
    loop_period_ns = 1e9 / 30.0

    fx = fy = 500.0
    cx, cy = width / 2.0, height / 2.0
    intrinsics = oc.CameraIntrinsics()
    intrinsics.width = width
    intrinsics.height = height
    intrinsics.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    intrinsics.P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

    ft_config = oc.FeatureTrackerConfig()
    ft_config.number_features = 100
    ft_config.min_number_features = 0

    feature_tracker = oc.PointFeatureTracker(loop_period_ns, ft_config)
    feature_tracker.setCameraInfoMsg(intrinsics)

    rb_tracker = oc.MultiRBTracker(
        10000,          # max_num_rb
        loop_period_ns,
        500,            # ransac_iterations
        0.03,           # estimation_error_threshold
        0.03,           # static_motion_threshold
        0.006,          # new_rbm_error_threshold
        0.02,           # max_error_to_reassign_feats
        5,              # supporting_features_threshold
        20,             # min_num_feats_for_new_rb
        20,             # min_num_frames_for_new_rb
        6,              # initial_cam_motion_constraint
        oc.StaticEnvironmentTrackerType.EKF,
    )
    rb_tracker.setMinCovarianceMeasurementX(0.03)
    rb_tracker.setMinCovarianceMeasurementY(0.03)
    rb_tracker.setMinCovarianceMeasurementZ(0.03)
    rb_tracker.setMeasurementDepthFactor(100.0)
    rb_tracker.setCovarianceSystemAccelerationTx(0.02)
    rb_tracker.setCovarianceSystemAccelerationTy(0.02)
    rb_tracker.setCovarianceSystemAccelerationTz(0.02)
    rb_tracker.setCovarianceSystemAccelerationRx(0.2)
    rb_tracker.setCovarianceSystemAccelerationRy(0.2)
    rb_tracker.setCovarianceSystemAccelerationRz(0.2)
    rb_tracker.setPriorCovariancePose(0.05)
    rb_tracker.setPriorCovarianceVelocity(0.1)
    rb_tracker.setMinNumPointsInSegment(300)
    rb_tracker.setMinProbabilisticValue(0.9)
    rb_tracker.setMaxFitnessScore(0.005)
    rb_tracker.setMinAmountTranslationForNewRB(0.03)
    rb_tracker.setMinAmountRotationForNewRB(6.0)
    rb_tracker.setMinNumberOfSupportingFeaturesToCorrectPredictedState(8)
    rb_tracker.setNumberOfTrackedFeatures(ft_config.number_features)
    rb_tracker.Init()

    joint_tracker = oc.MultiJointTracker(loop_period_ns, oc.KsAnalysisType.FULL_ANALYSIS, 0.1)
    joint_tracker.setNumSamplesLikelihoodEstimation(100)
    joint_tracker.setMinimumJointAgeForEE(3)
    joint_tracker.setSigmaDeltaMeasurementUncertaintyLinear(0.03)
    joint_tracker.setSigmaDeltaMeasurementUncertaintyAngular(1)
    joint_tracker.setPrismaticPriorCovarianceVelocity(0.5)
    joint_tracker.setPrismaticSigmaSystemNoisePhi(2.55)
    joint_tracker.setPrismaticSigmaSystemNoiseTheta(2.55)
    joint_tracker.setPrismaticSigmaSystemNoisePV(0.9)
    joint_tracker.setPrismaticSigmaSystemNoisePVd(75)
    joint_tracker.setPrismaticSigmaMeasurementNoise(0.9)
    joint_tracker.setRevolutePriorCovarianceVelocity(1.0)
    joint_tracker.setRevoluteSigmaSystemNoisePhi(2.55)
    joint_tracker.setRevoluteSigmaSystemNoiseTheta(2.55)
    joint_tracker.setRevoluteSigmaSystemNoisePx(0.3)
    joint_tracker.setRevoluteSigmaSystemNoisePy(0.3)
    joint_tracker.setRevoluteSigmaSystemNoisePz(0.3)
    joint_tracker.setRevoluteSigmaSystemNoiseRV(5.1)
    joint_tracker.setRevoluteSigmaSystemNoiseRVd(75)
    joint_tracker.setRevoluteSigmaMeasurementNoise(0.05)
    joint_tracker.setRevoluteMinimumRotForEstimation(0.045)
    joint_tracker.setRevoluteMaximumJointDistanceForEstimation(0.5)
    joint_tracker.setRigidMaxTranslation(0.05)
    joint_tracker.setRigidMaxRotation(0.1)
    joint_tracker.setMinimumNumFramesForNewRB(20)

    num_frames = 5
    base_texture = make_base_texture(width, height, seed=0)
    rb_state = None
    kinematic_structure = None
    for frame_idx in range(num_frames):
        timestamp_ns = frame_idx * loop_period_ns
        rgb, depth = make_synthetic_frame(base_texture, width, height, frame_idx)

        rgb_img = oc.make_stamped_image_rgb(rgb, timestamp_ns)
        depth_img = oc.make_stamped_image_depth(depth, timestamp_ns)

        feature_tracker.setMeasurement(rgb_img, depth_img, timestamp_ns)
        feature_tracker.predictState(loop_period_ns)
        feature_tracker.predictMeasurement()
        feature_tracker.correctState()
        features = feature_tracker.getState()
        assert isinstance(features, oc.FeatureCloud)

        rb_tracker.setMeasurement(features, timestamp_ns)
        rb_tracker.predictState(loop_period_ns)
        rb_tracker.predictMeasurement()
        rb_tracker.correctState()
        rb_tracker.ReflectState()
        rb_state = rb_tracker.getState()
        assert isinstance(rb_state, oc.RigidBodyPosesAndVels)

        joint_tracker.setMeasurement(rb_state, timestamp_ns)
        joint_tracker.predictState(loop_period_ns)
        joint_tracker.predictMeasurement()
        joint_tracker.correctState()
        joint_tracker.estimateJointFiltersProbabilities()
        kinematic_structure = joint_tracker.getKinematicStructure()
        assert isinstance(kinematic_structure, oc.KinematicStructure)

        print(f"frame {frame_idx}: features={features.size()} "
              f"rigid_bodies={len(rb_state.rb_poses_and_vels)} "
              f"joints={len(kinematic_structure.joints)}")

    print("OK: round-tripped a synthetic frame through feature_tracker -> "
          "rb_tracker -> joint_tracker with no crash.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
