#!/usr/bin/env python3
"""End-to-end Phase 6 demo: MuJoCo scene (a drawer sliding out of a
cabinet) -> omip_mujoco_wrapper's driver + orchestrator -> omip_core's
ported feature_tracker/rb_tracker/joint_tracker pipeline -> a joint type
and parameter estimate, printed at the end.

Runs standalone. No ROS anywhere. See PORTING_NOTES.md's Phase 6 section
for how the scene/parameters were arrived at, and README.md at the
omip_mujoco_wrapper package root for how to set up an environment to run
this (mujoco + numpy, and omip_core built per Phases 0-5).
"""
import sys

from omip_mujoco_wrapper.driver import MuJoCoDriver
from omip_mujoco_wrapper.orchestrator import OmipOrchestrator
from omip_mujoco_wrapper.scene_utils import DrawerSceneConfig, build_drawer_scene_xml, linear_ramp


def main():
    fps = 30.0
    loop_period_ns = 1e9 / fps

    scene_config = DrawerSceneConfig()
    xml = build_drawer_scene_xml(scene_config)
    driver = MuJoCoDriver(xml, camera_name="cam0", width=scene_config.width, height=scene_config.height)

    orchestrator = OmipOrchestrator(loop_period_ns, number_features=200)
    orchestrator.set_camera_info(driver.camera_intrinsics())

    num_frames = 150
    open_start_frame, open_end_frame = 10, 140

    print(f"Running {num_frames} frames ({num_frames / fps:.1f}s simulated) of a drawer "
          f"sliding open by {scene_config.slide_range_m} m...")

    last_result = None
    for frame_idx in range(num_frames):
        distance = linear_ramp(frame_idx, open_start_frame, open_end_frame, 0.0, scene_config.slide_range_m)
        driver.set_joint_qpos("slide", distance)
        driver.forward()

        rgb_bgr, depth = driver.render_rgbd()
        timestamp_ns = frame_idx * loop_period_ns

        last_result = orchestrator.process_frame(rgb_bgr, depth, timestamp_ns)

        if frame_idx % 10 == 0 or frame_idx == num_frames - 1:
            rb_ids = [rb.rb_id for rb in last_result.rigid_bodies.rb_poses_and_vels]
            print(f"  frame {frame_idx:3d}/{num_frames}: true_distance={distance:.3f} m, "
                  f"tracked rigid bodies={rb_ids}, joints={len(last_result.kinematic_structure.joints)}")

    print("\n=== Final kinematic structure estimate ===")
    if last_result is None or not last_result.kinematic_structure.joints:
        print("No joint was estimated — the drawer's rigid body was never separated "
              "from the static environment. See PORTING_NOTES.md Phase 6 for tuning notes.")
        return 1

    for joint in last_result.kinematic_structure.joints:
        print(f"Joint (parent_rb={joint.parent_rb_id}, child_rb={joint.child_rb_id}):")
        print(f"  most likely type: {joint.most_likely_joint}")
        print(f"  prismatic:  probability={joint.prism_probability:.3f}  joint_value={joint.prism_joint_value:.3f} m")
        print(f"  revolute:   probability={joint.rev_probability:.3f}  joint_value={joint.rev_joint_value:.3f} rad")
        print(f"  rigid:      probability={joint.rigid_probability:.3f}")
        print(f"  disconnected: probability={joint.discon_probability:.3f}")

    print(f"\n(Ground truth: a prismatic/sliding joint, {scene_config.slide_range_m} m of travel.)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
