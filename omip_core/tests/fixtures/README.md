# Golden fixtures

Per PORTING_NOTES.md's Validation Protocol, golden reference data is
exported from the original ROS Indigo Docker pipeline and dropped here,
incrementally, stage by stage. Format: `.npz` (confirmed with the user,
see PORTING_NOTES.md Open Question Q2).

`.npz` fixture files themselves are gitignored (see `.gitignore` in this
directory) since they're generated data, not source — regenerate them from
the ROS Docker pipeline rather than committing them.

## feature_tracker

Expected layout (proposed by the port; confirm against whatever the
Docker-side export script actually produces once available):

```
tests/fixtures/feature_tracker/<sequence_name>/
  camera_info.npz    keys: "K" (9,) float64, "P" (12,) float64,
                      "width" (1,) int32, "height" (1,) int32
  frame_0000.npz, frame_0001.npz, ...
    keys: "rgb" (H,W,3) uint8 [bgr8], "depth" (H,W) float32 [32FC1],
          "timestamp_ns" (1,) float64,
          "expected_ids" (N,) int64, "expected_xyz" (N,3) float64
          — reference feature Ids/3D locations after that frame's
          correctState(), from the original PointFeatureTracker.
```

See `tests/test_feature_tracker.cpp` for the loader/comparison code. Until
a `<sequence_name>/` directory exists here, that test reports
"ported, unvalidated, awaiting fixtures" and passes trivially — this is
expected, not a bug.
