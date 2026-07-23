"""Thin MuJoCo driver: owns the MjModel/MjData/Renderer, steps physics,
and renders synchronized RGB + depth + camera intrinsics per frame.

All MuJoCo-specific code lives here, per the mission brief's separation of
concerns: `omip_core` must stay simulator-agnostic, and `orchestrator.py`
(which calls into omip_core) must stay MuJoCo-agnostic. This is the one
module allowed to know about both worlds — it hands plain numpy arrays +
an `omip_core.CameraIntrinsics` to its caller, nothing MuJoCo-specific
leaks past it.
"""
from __future__ import annotations

import math

import mujoco
import numpy as np

import omip_core as oc


class MuJoCoDriver:
    def __init__(self, xml_string: str, camera_name: str, width: int = 320, height: int = 240):
        self.model = mujoco.MjModel.from_xml_string(xml_string)
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(self.model, height=height, width=width)
        self.camera_name = camera_name
        self.width = width
        self.height = height

    def joint_qpos_address(self, joint_name: str) -> int:
        return self.model.joint(joint_name).qposadr[0]

    def set_joint_qpos(self, joint_name: str, value: float) -> None:
        self.data.qpos[self.joint_qpos_address(joint_name)] = value

    def forward(self) -> None:
        """Recompute all derived quantities (kinematics, etc.) from the
        current qpos — this demo scripts joint positions directly rather
        than running MuJoCo's dynamics integrator, so mj_forward (not
        mj_step) is what's needed each frame."""
        mujoco.mj_forward(self.model, self.data)

    def camera_intrinsics(self) -> oc.CameraIntrinsics:
        """Pinhole intrinsics derived from the named camera's vertical FOV,
        matching the standard formula used for MuJoCo cameras (isotropic
        pixels, so fx == fy): fy = height / (2 * tan(fovy/2))."""
        cam_id = self.model.camera(self.camera_name).id
        fovy_deg = self.model.cam_fovy[cam_id]
        fy = self.height / (2.0 * math.tan(math.radians(fovy_deg) / 2.0))
        fx = fy
        cx, cy = self.width / 2.0, self.height / 2.0

        intrinsics = oc.CameraIntrinsics()
        intrinsics.width = self.width
        intrinsics.height = self.height
        intrinsics.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        intrinsics.P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return intrinsics

    def render_rgbd(self):
        """Renders the current state from `self.camera_name`.

        Returns (bgr, depth): `bgr` is an (H,W,3) uint8 array in BGR
        channel order (MuJoCo's renderer returns RGB; omip_core's
        PointFeatureTracker expects "bgr8", matching the ROS convention the
        original pipeline was built against — see
        omip_core/bindings/omip_core_py.cpp's make_stamped_image_rgb), and
        `depth` is an (H,W) float32 array in meters.
        """
        self.renderer.update_scene(self.data, camera=self.camera_name)
        rgb = self.renderer.render()

        self.renderer.enable_depth_rendering()
        self.renderer.update_scene(self.data, camera=self.camera_name)
        depth = self.renderer.render()
        self.renderer.disable_depth_rendering()

        bgr = np.ascontiguousarray(rgb[:, :, ::-1])
        return bgr, depth
