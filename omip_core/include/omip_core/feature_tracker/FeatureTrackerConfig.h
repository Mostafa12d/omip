// Plain config struct replacing feature_tracker_cfg.yaml (read via ROS
// param server / getROSParameter<T>) and the small subset of
// FeatureTrackerDynReconfConfig fields that affected the Filter itself.
// See PORTING_NOTES.md Phase 2 section for the field-by-field mapping;
// fields that were purely ROS-topic/rosbag plumbing (rgb_img_topic,
// data_from_bag, bag_file, subscribe_to_pc, advance_frame_*, ...) are not
// carried over — the orchestrator drives the Filter directly instead of
// through topics.
#ifndef OMIP_CORE_FEATURE_TRACKER_CONFIG_H_
#define OMIP_CORE_FEATURE_TRACKER_CONFIG_H_

namespace omip
{

struct FeatureTrackerConfig
{
    // Number of feature points to track.
    int number_features = 200;
    // Minimum number of feature points that have to be detected/tracked.
    int min_number_features = 0;
    // Minimum feature quality (see cv::goodFeaturesToTrack).
    double min_feat_quality = 0.005;
    // Region-of-interest depth bounds; negative disables the bound.
    double min_distance = -1.0;
    double max_distance = -1.0;
    // Maximum allowed 3D motion of a feature between consecutive frames.
    double max_interframe_jump = 0.04;
    // Morphological erosion sizes for the detecting/tracking masks.
    int erosion_size_detect = 2;
    int erosion_size_track = 1;
    // BETA in the original: focus new-feature detection on depth-changing
    // areas. See PORTING_NOTES.md Phase 2 open question re: wall-clock vs.
    // simulation-time gating for min_time_to_detect_motion.
    bool attention_to_motion = false;
    double min_time_to_detect_motion = 1.0;
    double min_depth_difference = 0.02;
    int min_area_size_pixels = 3000;
    // Debug-image getters honor these flags exactly like the original
    // publish-toggle parameters did (they just gate computation now,
    // instead of gating a ROS publish).
    bool pub_tracked_feats_with_pred_mask_img = false;
    bool pub_tracked_feats_img = false;
    bool pub_predicted_and_past_feats_img = false;

    // Used to compute the recursive-estimator loop period, identically to
    // the original: loop_period_ns = 1e9 / (sensor_fps / processing_factor).
    double sensor_fps = 30.0;
    int processing_factor = 1;
};

} // namespace omip

#endif // OMIP_CORE_FEATURE_TRACKER_CONFIG_H_
