// Golden-fixture test harness for feature_tracker (PointFeatureTracker).
// Per PORTING_NOTES.md's Validation Protocol: if no fixtures have arrived
// yet from the user's parallel ROS Indigo Docker run, this reports the
// stage as "ported, unvalidated, awaiting fixtures" and exits successfully
// rather than failing — it does not block the port. Once fixtures land at
// tests/fixtures/feature_tracker/, this harness runs the real numerical
// comparison automatically.
//
// Expected fixture layout (proposed; confirm with the user once real
// fixtures are exported — see PORTING_NOTES.md open question):
//   tests/fixtures/feature_tracker/<sequence_name>/
//     camera_info.npz  — keys: "K" (9,) float64, "P" (12,) float64,
//                         "width" (1,) int32, "height" (1,) int32
//     frame_0000.npz, frame_0001.npz, ... — one per timestep, keys:
//         "rgb"            (H,W,3) uint8, encoding assumed bgr8
//         "depth"          (H,W)   float32, encoding assumed 32FC1
//         "timestamp_ns"   (1,)    float64
//         "expected_ids"   (N,)    int64   — reference feature Ids after
//                                            this frame's correctState()
//         "expected_xyz"   (N,3)   float64 — reference 3D locations,
//                                            row-matched to expected_ids
#include "omip_core/feature_tracker/PointFeatureTracker.h"

#include <cnpy.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
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

CameraIntrinsics loadCameraInfo(const fs::path& path)
{
    cnpy::npz_t npz = cnpy::npz_load(path.string());
    CameraIntrinsics intr;
    std::copy_n(npz.at("K").data<double>(), 9, intr.K.begin());
    std::copy_n(npz.at("P").data<double>(), 12, intr.P.begin());
    intr.width = *npz.at("width").data<int32_t>();
    intr.height = *npz.at("height").data<int32_t>();
    return intr;
}

StampedImage::Ptr loadRgb(const cnpy::npz_t& npz, double timestamp_ns)
{
    const cnpy::NpyArray& arr = npz.at("rgb");
    const int rows = static_cast<int>(arr.shape[0]);
    const int cols = static_cast<int>(arr.shape[1]);
    StampedImage::Ptr img(new StampedImage());
    img->encoding = "bgr8";
    img->timestamp_ns = timestamp_ns;
    img->image = cv::Mat(rows, cols, CV_8UC3, const_cast<unsigned char*>(arr.data<unsigned char>())).clone();
    return img;
}

StampedImage::Ptr loadDepth(const cnpy::npz_t& npz, double timestamp_ns)
{
    const cnpy::NpyArray& arr = npz.at("depth");
    const int rows = static_cast<int>(arr.shape[0]);
    const int cols = static_cast<int>(arr.shape[1]);
    StampedImage::Ptr img(new StampedImage());
    img->encoding = "32FC1";
    img->timestamp_ns = timestamp_ns;
    img->image = cv::Mat(rows, cols, CV_32FC1, const_cast<float*>(arr.data<float>())).clone();
    return img;
}

// Returns false (and prints per-point diffs) if any expected feature
// location isn't matched within tolerance by the filter's current state.
bool compareState(const ft_state_t& state, const cnpy::npz_t& npz, int frame_idx)
{
    const cnpy::NpyArray& ids_arr = npz.at("expected_ids");
    const cnpy::NpyArray& xyz_arr = npz.at("expected_xyz");
    const int64_t* expected_ids = ids_arr.data<int64_t>();
    const double* expected_xyz = xyz_arr.data<double>();
    const size_t n_expected = ids_arr.shape[0];

    bool all_ok = true;
    for (size_t i = 0; i < n_expected; ++i)
    {
        const int64_t expected_id = expected_ids[i];
        const double ex = expected_xyz[3 * i + 0];
        const double ey = expected_xyz[3 * i + 1];
        const double ez = expected_xyz[3 * i + 2];

        bool found = false;
        for (const auto& p : state->points)
        {
            if (static_cast<int64_t>(p.label) != expected_id)
                continue;
            found = true;
            const bool ok = nearlyEqual(p.x, ex, kRelativeTolerance)
                && nearlyEqual(p.y, ey, kRelativeTolerance)
                && nearlyEqual(p.z, ez, kRelativeTolerance);
            if (!ok)
            {
                std::cerr << "[frame " << frame_idx << "] feature id " << expected_id
                          << " mismatch: got (" << p.x << "," << p.y << "," << p.z
                          << ") expected (" << ex << "," << ey << "," << ez << ")\n";
                all_ok = false;
            }
            break;
        }
        if (!found)
        {
            std::cerr << "[frame " << frame_idx << "] expected feature id " << expected_id
                       << " not found in filter state\n";
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

    FeatureTrackerConfig config;
    config.number_features = 200;
    config.min_number_features = 0;

    PointFeatureTracker tracker(/*loop_period_ns=*/1e9 / 30.0, config);
    tracker.setCameraInfoMsg(loadCameraInfo(sequence_dir / "camera_info.npz"));

    bool all_ok = true;
    for (size_t frame_idx = 0; frame_idx < frame_files.size(); ++frame_idx)
    {
        cnpy::npz_t npz = cnpy::npz_load(frame_files[frame_idx].string());
        const double timestamp_ns = *npz.at("timestamp_ns").data<double>();

        tracker.setMeasurement(ft_measurement_t(loadRgb(npz, timestamp_ns), loadDepth(npz, timestamp_ns)), timestamp_ns);
        tracker.predictState(1e9 / 30.0);
        tracker.predictMeasurement();
        tracker.correctState();

        if (!compareState(tracker.getState(), npz, static_cast<int>(frame_idx)))
            all_ok = false;
    }
    return all_ok ? 0 : 1;
}

} // namespace

int main()
{
    const fs::path fixtures_root = fs::path(OMIP_CORE_TEST_FIXTURES_DIR) / "feature_tracker";

    if (!fs::exists(fixtures_root) || fs::directory_iterator(fixtures_root) == fs::directory_iterator())
    {
        std::cout << "[test_feature_tracker] No golden fixtures found at " << fixtures_root << ".\n"
                  << "[test_feature_tracker] feature_tracker is PORTED, UNVALIDATED, AWAITING FIXTURES "
                     "(see PORTING_NOTES.md Phase 2). Not a failure.\n";
        return 0;
    }

    bool all_ok = true;
    for (const auto& entry : fs::directory_iterator(fixtures_root))
    {
        if (!entry.is_directory())
            continue;
        std::cout << "[test_feature_tracker] Running sequence: " << entry.path().filename() << "\n";
        if (runSequence(entry.path()) != 0)
            all_ok = false;
    }
    return all_ok ? 0 : 1;
}
