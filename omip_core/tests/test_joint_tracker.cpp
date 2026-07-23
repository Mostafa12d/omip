// Golden-fixture test harness for joint_tracker (MultiJointTracker). See
// test_feature_tracker.cpp / test_rb_tracker.cpp for the general pattern
// this follows; per PORTING_NOTES.md's Validation Protocol, this reports
// "ported, unvalidated, awaiting fixtures" and exits successfully until
// real fixtures land at tests/fixtures/joint_tracker/.
//
// Expected fixture layout (proposed; confirm with the user once real
// fixtures are exported — see PORTING_NOTES.md open question):
//   tests/fixtures/joint_tracker/<sequence_name>/
//     frame_0000.npz, frame_0001.npz, ... — one per timestep, keys:
//         "rb_ids"              (M,)   int64   — matches rb_tracker's output
//         "pose_twist"          (M,6)  float64  [rx,ry,rz,vx,vy,vz]
//         "velocity_twist"      (M,6)  float64  [rx,ry,rz,vx,vy,vz]
//         "centroid"            (M,3)  float64
//         "timestamp_ns"        (1,)   float64
//         "expected_rrb_ids"        (K,) int64
//         "expected_srb_ids"        (K,) int64
//         "expected_joint_type"     (K,) int64 — JointFilterType enum value:
//                                    RIGID_JOINT=0, PRISMATIC_JOINT=1,
//                                    REVOLUTE_JOINT=2, DISCONNECTED_JOINT=3
//         "expected_joint_state"    (K,) float64
//   (M includes the static-environment body, rb_id 0; K is the number of
//   rrb/srb pairs the original produced a joint estimate for at that frame.)
//
// Default MultiJointTracker parameters below match
// joint_tracker/cfg/joint_tracker_cfg.yaml (the original ROS package's
// defaults) so the harness reproduces the same configuration the golden
// fixtures would have been generated with, absent fixture-specific
// overrides. `min_num_frames_for_new_rb` is shared with rb_tracker
// (originally read from the ROS param `/rb_tracker/min_num_frames_for_new_rb`,
// see MultiJointTrackerNode.cpp) — matched here to the same value
// test_rb_tracker.cpp uses (20).
#include "omip_core/joint_tracker/MultiJointTracker.h"

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

MultiJointTracker* makeTracker(double loop_period_ns)
{
    MultiJointTracker* tracker = new MultiJointTracker(loop_period_ns, FULL_ANALYSIS, /*dj_ne=*/0.1);

    tracker->setNumSamplesLikelihoodEstimation(100);
    tracker->setMinimumJointAgeForEE(3);
    tracker->setSigmaDeltaMeasurementUncertaintyLinear(0.03);
    tracker->setSigmaDeltaMeasurementUncertaintyAngular(1);

    tracker->setPrismaticPriorCovarianceVelocity(0.5);
    tracker->setPrismaticSigmaSystemNoisePhi(2.55);
    tracker->setPrismaticSigmaSystemNoiseTheta(2.55);
    tracker->setPrismaticSigmaSystemNoisePV(0.9);
    tracker->setPrismaticSigmaSystemNoisePVd(75);
    tracker->setPrismaticSigmaMeasurementNoise(0.9);

    tracker->setRevolutePriorCovarianceVelocity(1.0);
    tracker->setRevoluteSigmaSystemNoisePhi(2.55);
    tracker->setRevoluteSigmaSystemNoiseTheta(2.55);
    tracker->setRevoluteSigmaSystemNoisePx(0.3);
    tracker->setRevoluteSigmaSystemNoisePy(0.3);
    tracker->setRevoluteSigmaSystemNoisePz(0.3);
    tracker->setRevoluteSigmaSystemNoiseRV(5.1);
    tracker->setRevoluteSigmaSystemNoiseRVd(75);
    tracker->setRevoluteSigmaMeasurementNoise(0.05);
    tracker->setRevoluteMinimumRotForEstimation(0.045);
    tracker->setRevoluteMaximumJointDistanceForEstimation(0.5);

    tracker->setRigidMaxTranslation(0.05);
    tracker->setRigidMaxRotation(0.1);

    tracker->setMinimumNumFramesForNewRB(20);

    return tracker;
}

ks_measurement_t loadMeasurement(const cnpy::npz_t& npz, double timestamp_ns)
{
    const cnpy::NpyArray& ids_arr = npz.at("rb_ids");
    const cnpy::NpyArray& pose_arr = npz.at("pose_twist");
    const cnpy::NpyArray& vel_arr = npz.at("velocity_twist");
    const cnpy::NpyArray& centroid_arr = npz.at("centroid");
    const int64_t* ids = ids_arr.data<int64_t>();
    const double* pose = pose_arr.data<double>();
    const double* vel = vel_arr.data<double>();
    const double* centroid = centroid_arr.data<double>();
    const size_t n = ids_arr.shape[0];

    ks_measurement_t measurement;
    measurement.timestamp_ns = timestamp_ns;
    measurement.rb_poses_and_vels.resize(n);
    for (size_t i = 0; i < n; ++i)
    {
        RigidBodyPoseAndVel& rb = measurement.rb_poses_and_vels[i];
        rb.rb_id = ids[i];
        rb.pose_wc.twist = Twistd(pose[6 * i + 0], pose[6 * i + 1], pose[6 * i + 2],
                                   pose[6 * i + 3], pose[6 * i + 4], pose[6 * i + 5]);
        rb.velocity_wc.twist = Twistd(vel[6 * i + 0], vel[6 * i + 1], vel[6 * i + 2],
                                       vel[6 * i + 3], vel[6 * i + 4], vel[6 * i + 5]);
        rb.centroid = Eigen::Vector3d(centroid[3 * i + 0], centroid[3 * i + 1], centroid[3 * i + 2]);
    }
    return measurement;
}

bool compareState(const ks_state_t& state, const cnpy::npz_t& npz, int frame_idx)
{
    const cnpy::NpyArray& rrb_arr = npz.at("expected_rrb_ids");
    const cnpy::NpyArray& srb_arr = npz.at("expected_srb_ids");
    const cnpy::NpyArray& type_arr = npz.at("expected_joint_type");
    const cnpy::NpyArray& state_arr = npz.at("expected_joint_state");
    const int64_t* expected_rrb = rrb_arr.data<int64_t>();
    const int64_t* expected_srb = srb_arr.data<int64_t>();
    const int64_t* expected_type = type_arr.data<int64_t>();
    const double* expected_state = state_arr.data<double>();
    const size_t n_expected = rrb_arr.shape[0];

    bool all_ok = true;
    for (size_t i = 0; i < n_expected; ++i)
    {
        const std::pair<int, int> ids(static_cast<int>(expected_rrb[i]), static_cast<int>(expected_srb[i]));
        ks_state_t::const_iterator it = state.find(ids);
        if (it == state.end())
        {
            std::cerr << "[frame " << frame_idx << "] expected joint (" << ids.first << "," << ids.second
                       << ") not found in tracker state\n";
            all_ok = false;
            continue;
        }

        JointFilterPtr most_probable = it->second->getMostProbableJointFilter();
        const int actual_type = static_cast<int>(most_probable->getJointFilterType());
        const bool type_ok = actual_type == static_cast<int>(expected_type[i]);
        const bool state_ok = nearlyEqual(most_probable->getJointState(), expected_state[i], kRelativeTolerance);
        if (!type_ok || !state_ok)
        {
            std::cerr << "[frame " << frame_idx << "] joint (" << ids.first << "," << ids.second << ") mismatch: "
                       << "got type=" << actual_type << " state=" << most_probable->getJointState()
                       << " expected type=" << expected_type[i] << " state=" << expected_state[i] << "\n";
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
    std::unique_ptr<MultiJointTracker> tracker(makeTracker(loop_period_ns));

    bool all_ok = true;
    double previous_timestamp_ns = 0.0;
    for (size_t frame_idx = 0; frame_idx < frame_files.size(); ++frame_idx)
    {
        cnpy::npz_t npz = cnpy::npz_load(frame_files[frame_idx].string());
        const double timestamp_ns = *npz.at("timestamp_ns").data<double>();
        const double time_interval_ns = (frame_idx == 0) ? loop_period_ns : (timestamp_ns - previous_timestamp_ns);

        tracker->setMeasurement(loadMeasurement(npz, timestamp_ns), timestamp_ns);
        tracker->predictState(time_interval_ns);
        tracker->predictMeasurement();
        tracker->correctState();
        tracker->estimateJointFiltersProbabilities();

        if (!compareState(tracker->getState(), npz, static_cast<int>(frame_idx)))
            all_ok = false;

        previous_timestamp_ns = timestamp_ns;
    }
    return all_ok ? 0 : 1;
}

} // namespace

int main()
{
    const fs::path fixtures_root = fs::path(OMIP_CORE_TEST_FIXTURES_DIR) / "joint_tracker";

    if (!fs::exists(fixtures_root) || fs::directory_iterator(fixtures_root) == fs::directory_iterator())
    {
        std::cout << "[test_joint_tracker] No golden fixtures found at " << fixtures_root << ".\n"
                  << "[test_joint_tracker] joint_tracker is PORTED, UNVALIDATED, AWAITING FIXTURES "
                     "(see PORTING_NOTES.md Phase 4). Not a failure.\n";
        return 0;
    }

    bool all_ok = true;
    for (const auto& entry : fs::directory_iterator(fixtures_root))
    {
        if (!entry.is_directory())
            continue;
        std::cout << "[test_joint_tracker] Running sequence: " << entry.path().filename() << "\n";
        if (runSequence(entry.path()) != 0)
            all_ok = false;
    }
    return all_ok ? 0 : 1;
}
