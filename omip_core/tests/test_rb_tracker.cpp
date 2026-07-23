// Golden-fixture test harness for rb_tracker (MultiRBTracker). See
// test_feature_tracker.cpp for the general pattern this follows; per
// PORTING_NOTES.md's Validation Protocol, this reports "ported,
// unvalidated, awaiting fixtures" and exits successfully until real
// fixtures land at tests/fixtures/rb_tracker/.
//
// Expected fixture layout (proposed; confirm with the user once real
// fixtures are exported — see PORTING_NOTES.md open question):
//   tests/fixtures/rb_tracker/<sequence_name>/
//     frame_0000.npz, frame_0001.npz, ... — one per timestep, keys:
//         "feature_ids"     (N,)  int64   — matches FeatureCloudPCLwc labels
//         "feature_xyz"     (N,3) float64 — matches FeatureCloudPCLwc x,y,z
//         "timestamp_ns"    (1,)  float64
//         "expected_rb_ids"          (M,)   int64
//         "expected_pose_twist"      (M,6)  float64 [rx,ry,rz,vx,vy,vz]
//         "expected_velocity_twist"  (M,6)  float64 [rx,ry,rz,vx,vy,vz]
//   (M includes the static-environment body, rb_id 0.)
//
// Default MultiRBTracker parameters below match rb_tracker/cfg/rb_tracker_cfg.yaml
// (the original ROS package's defaults) so the harness reproduces the same
// configuration the golden fixtures would have been generated with, absent
// fixture-specific overrides.
#include "omip_core/rb_tracker/MultiRBTracker.h"

#include <cnpy.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace omip;

namespace
{

constexpr double kRelativeTolerance = 1e-4;

bool nearlyEqual(double actual, double expected, double rel_tol)
{
    return std::fabs(actual - expected) <= rel_tol * std::max(1.0, std::fabs(expected));
}

MultiRBTracker* makeTracker(double loop_period_ns)
{
    MultiRBTracker* tracker = new MultiRBTracker(
        /*max_num_rb=*/10000,
        loop_period_ns,
        /*ransac_iterations=*/500,
        /*estimation_error_threshold=*/0.03,
        /*static_motion_threshold=*/0.03,
        /*new_rbm_error_threshold=*/0.006,
        /*max_error_to_reassign_feats=*/0.02,
        /*supporting_features_threshold=*/5,
        /*min_num_feats_for_new_rb=*/20,
        /*min_num_frames_for_new_rb=*/20,
        /*initial_cam_motion_constraint=*/6,
        /*static_environment_tracker_type=*/STATIC_ENVIRONMENT_EKF_TRACKER);

    tracker->setMinCovarianceMeasurementX(0.03);
    tracker->setMinCovarianceMeasurementY(0.03);
    tracker->setMinCovarianceMeasurementZ(0.03);
    tracker->setMeasurementDepthFactor(100.0);
    tracker->setCovarianceSystemAccelerationTx(0.02);
    tracker->setCovarianceSystemAccelerationTy(0.02);
    tracker->setCovarianceSystemAccelerationTz(0.02);
    tracker->setCovarianceSystemAccelerationRx(0.2);
    tracker->setCovarianceSystemAccelerationRy(0.2);
    tracker->setCovarianceSystemAccelerationRz(0.2);
    tracker->setPriorCovariancePose(0.05);
    tracker->setPriorCovarianceVelocity(0.1);
    tracker->setMinNumPointsInSegment(300);
    tracker->setMinProbabilisticValue(0.9);
    tracker->setMaxFitnessScore(0.005);
    tracker->setMinAmountTranslationForNewRB(0.03);
    tracker->setMinAmountRotationForNewRB(6.0);
    tracker->setMinNumberOfSupportingFeaturesToCorrectPredictedState(8);
    tracker->setNumberOfTrackedFeatures(200); // matches feature_tracker's default number_features
    tracker->Init();
    return tracker;
}

rbt_measurement_t loadMeasurement(const cnpy::npz_t& npz)
{
    const cnpy::NpyArray& ids_arr = npz.at("feature_ids");
    const cnpy::NpyArray& xyz_arr = npz.at("feature_xyz");
    const int64_t* ids = ids_arr.data<int64_t>();
    const double* xyz = xyz_arr.data<double>();
    const size_t n = ids_arr.shape[0];

    FeatureCloudPCLwc::Ptr cloud(new FeatureCloudPCLwc());
    cloud->points.resize(n);
    for (size_t i = 0; i < n; ++i)
    {
        cloud->points[i].label = static_cast<uint32_t>(ids[i]);
        cloud->points[i].x = xyz[3 * i + 0];
        cloud->points[i].y = xyz[3 * i + 1];
        cloud->points[i].z = xyz[3 * i + 2];
    }
    return cloud;
}

bool compareState(const rbt_state_t& state, const cnpy::npz_t& npz, int frame_idx)
{
    const cnpy::NpyArray& ids_arr = npz.at("expected_rb_ids");
    const cnpy::NpyArray& pose_arr = npz.at("expected_pose_twist");
    const cnpy::NpyArray& vel_arr = npz.at("expected_velocity_twist");
    const int64_t* expected_ids = ids_arr.data<int64_t>();
    const double* expected_pose = pose_arr.data<double>();
    const double* expected_vel = vel_arr.data<double>();
    const size_t n_expected = ids_arr.shape[0];

    bool all_ok = true;
    for (size_t i = 0; i < n_expected; ++i)
    {
        const int64_t expected_id = expected_ids[i];
        bool found = false;
        for (const auto& rb : state.rb_poses_and_vels)
        {
            if (rb.rb_id != expected_id)
                continue;
            found = true;
            const double* ep = expected_pose + 6 * i;
            const double* ev = expected_vel + 6 * i;
            const bool pose_ok = nearlyEqual(rb.pose_wc.twist.rx(), ep[0], kRelativeTolerance)
                && nearlyEqual(rb.pose_wc.twist.ry(), ep[1], kRelativeTolerance)
                && nearlyEqual(rb.pose_wc.twist.rz(), ep[2], kRelativeTolerance)
                && nearlyEqual(rb.pose_wc.twist.vx(), ep[3], kRelativeTolerance)
                && nearlyEqual(rb.pose_wc.twist.vy(), ep[4], kRelativeTolerance)
                && nearlyEqual(rb.pose_wc.twist.vz(), ep[5], kRelativeTolerance);
            const bool vel_ok = nearlyEqual(rb.velocity_wc.twist.rx(), ev[0], kRelativeTolerance)
                && nearlyEqual(rb.velocity_wc.twist.ry(), ev[1], kRelativeTolerance)
                && nearlyEqual(rb.velocity_wc.twist.rz(), ev[2], kRelativeTolerance)
                && nearlyEqual(rb.velocity_wc.twist.vx(), ev[3], kRelativeTolerance)
                && nearlyEqual(rb.velocity_wc.twist.vy(), ev[4], kRelativeTolerance)
                && nearlyEqual(rb.velocity_wc.twist.vz(), ev[5], kRelativeTolerance);
            if (!pose_ok || !vel_ok)
            {
                std::cerr << "[frame " << frame_idx << "] rigid body id " << expected_id << " mismatch\n";
                all_ok = false;
            }
            break;
        }
        if (!found)
        {
            std::cerr << "[frame " << frame_idx << "] expected rigid body id " << expected_id
                       << " not found in tracker state\n";
            all_ok = false;
        }
    }
    return all_ok;
}

int runSequence(const fs::path& sequence_dir)
{
    std::vector<fs::path> frame_files;
    for (const auto& entry : fs::directory_iterator(sequence_dir))
    {
        if (entry.path().filename().string().rfind("frame_", 0) == 0)
            frame_files.push_back(entry.path());
    }
    std::sort(frame_files.begin(), frame_files.end());

    if (frame_files.empty())
    {
        std::cerr << "No frame_*.npz files in " << sequence_dir << "\n";
        return 1;
    }

    const double loop_period_ns = 1e9 / 30.0;
    std::unique_ptr<MultiRBTracker> tracker(makeTracker(loop_period_ns));

    bool all_ok = true;
    double previous_timestamp_ns = 0.0;
    for (size_t frame_idx = 0; frame_idx < frame_files.size(); ++frame_idx)
    {
        cnpy::npz_t npz = cnpy::npz_load(frame_files[frame_idx].string());
        const double timestamp_ns = *npz.at("timestamp_ns").data<double>();
        const double time_interval_ns = (frame_idx == 0) ? loop_period_ns : (timestamp_ns - previous_timestamp_ns);

        tracker->setMeasurement(loadMeasurement(npz), timestamp_ns);
        tracker->predictState(time_interval_ns);
        tracker->predictMeasurement();
        tracker->correctState();
        tracker->ReflectState();

        if (!compareState(tracker->getState(), npz, static_cast<int>(frame_idx)))
            all_ok = false;

        previous_timestamp_ns = timestamp_ns;
    }
    return all_ok ? 0 : 1;
}

} // namespace

int main()
{
    const fs::path fixtures_root = fs::path(OMIP_CORE_TEST_FIXTURES_DIR) / "rb_tracker";

    if (!fs::exists(fixtures_root) || fs::directory_iterator(fixtures_root) == fs::directory_iterator())
    {
        std::cout << "[test_rb_tracker] No golden fixtures found at " << fixtures_root << ".\n"
                  << "[test_rb_tracker] rb_tracker is PORTED, UNVALIDATED, AWAITING FIXTURES "
                     "(see PORTING_NOTES.md Phase 3). Not a failure.\n";
        return 0;
    }

    bool all_ok = true;
    for (const auto& entry : fs::directory_iterator(fixtures_root))
    {
        if (!entry.is_directory())
            continue;
        std::cout << "[test_rb_tracker] Running sequence: " << entry.path().filename() << "\n";
        if (runSequence(entry.path()) != 0)
            all_ok = false;
    }
    return all_ok ? 0 : 1;
}
