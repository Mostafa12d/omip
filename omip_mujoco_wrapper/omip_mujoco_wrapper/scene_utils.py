"""Helpers for building and scripting the demo MuJoCo scene.

This module only knows about MJCF/geometry — it doesn't import omip_core or
know anything about kinematic-structure estimation. Its job is purely to
give `driver.py`/`examples/run_demo.py` a scene to render and a way to
script an interaction (moving the drawer over time), per the mission
brief's suggested layout ("helpers for scripting interactions/actuation").

Design note on the tile-grid checkerboard (`_tile_grid_xml`): MuJoCo's
built-in procedural textures (`builtin="checker"`) render correctly on
`plane` geoms but only vary along a single axis on `box` geoms in this
version of MuJoCo (verified empirically — texrepeat/texuniform combinations
were tried and none produced a 2-axis checker on a box face), and `plane`
geoms cannot be attached to a body with a joint (MuJoCo requires planes to
be static). Since the drawer needs to (a) move and (b) have real visual
texture for `cv::goodFeaturesToTrack` to find corners on, it's built out of
a literal grid of small alternating-color box geoms instead of a single
textured box — this sidesteps the texture-mapping limitation entirely and
is, if anything, more reliable (real geometric edges between tiles, not a
texture that could be filtered/blurred away).
"""
from __future__ import annotations

import dataclasses


def _tile_grid_xml(name_prefix, half_width, half_height, n=6,
                    color_a=(0.1, 0.4, 0.1), color_b=(0.6, 0.9, 0.6), thickness=0.01):
    """Emit MJCF <geom> tags for an n x n checkerboard of tiles spanning
    [-half_width, half_width] x [-half_height, half_height] in the parent
    body's local X/Z plane (normal along local Y)."""
    tile_w = (2 * half_width) / n
    tile_h = (2 * half_height) / n
    parts = []
    for i in range(n):
        for j in range(n):
            cx = -half_width + tile_w * (i + 0.5)
            cz = -half_height + tile_h * (j + 0.5)
            color = color_a if (i + j) % 2 == 0 else color_b
            parts.append(
                f'<geom name="{name_prefix}_{i}_{j}" type="box" '
                f'size="{tile_w / 2 * 0.98:.5f} {thickness} {tile_h / 2 * 0.98:.5f}" '
                f'pos="{cx:.5f} 0 {cz:.5f}" rgba="{color[0]} {color[1]} {color[2]} 1" '
                f'contype="0" conaffinity="0"/>'
            )
    return "\n      ".join(parts)


@dataclasses.dataclass
class DrawerSceneConfig:
    """Tunable parameters for the cabinet+drawer demo scene. Defaults are
    the values found (by iterating against the real omip_core pipeline, not
    guessed) to give: stable rigid-body tracking of the drawer (no
    spurious loss/recreation) and confident, correct PRISMATIC joint
    classification. See PORTING_NOTES.md's Phase 6 section for the tuning
    story, including why a hinged door (the mission brief's other suggested
    example) was tried first and set aside — its revolute EKF didn't
    converge reliably within the time available, whereas the drawer's
    prismatic case did, cleanly and repeatably.
    """
    width: int = 320
    height: int = 240
    fovy_deg: float = 45.0
    cabinet_half_w: float = 0.6
    cabinet_half_h: float = 0.4
    cabinet_tiles: int = 6
    drawer_half_w: float = 0.35
    drawer_half_h: float = 0.3
    drawer_tiles: int = 5
    slide_range_m: float = 0.4
    slide_damping: float = 2.0


def build_drawer_scene_xml(config: DrawerSceneConfig = DrawerSceneConfig()) -> str:
    """Returns an MJCF XML string for a static "cabinet" body and a "drawer"
    body that slides out of it along a single prismatic (slide) joint,
    named "slide". The camera is named "cam0".
    """
    cabinet_tiles = _tile_grid_xml(
        "cab", config.cabinet_half_w, config.cabinet_half_h, n=config.cabinet_tiles,
        color_a=(0.1, 0.4, 0.1), color_b=(0.6, 0.9, 0.6))
    drawer_tiles = _tile_grid_xml(
        "drw", config.drawer_half_w, config.drawer_half_h, n=config.drawer_tiles,
        color_a=(0.5, 0.1, 0.1), color_b=(0.9, 0.5, 0.5))

    return f"""
<mujoco>
  <visual>
    <headlight ambient="0.4 0.4 0.4" diffuse="0.6 0.6 0.6"/>
    <!-- Tightened near/far clip planes around the scene's actual depth
         range (rather than MuJoCo's much larger defaults) for better
         z-buffer precision — see PORTING_NOTES.md Phase 6. -->
    <map znear="0.3" zfar="6"/>
  </visual>
  <asset>
    <texture name="floor_tex" type="2d" builtin="checker" rgb1="0.25 0.25 0.28" rgb2="0.35 0.35 0.38" width="300" height="300"/>
    <material name="floor_mat" texture="floor_tex" texrepeat="6 6" texuniform="true" reflectance="0.0"/>
  </asset>
  <worldbody>
    <light pos="0 -1 3" dir="0 0.2 -1" diffuse="1 1 1"/>
    <camera name="cam0" pos="0 -1.3 0.4" xyaxes="1 0 0 0 0.2 1" fovy="{config.fovy_deg}"/>
    <!-- Floor and backwall are real (finite) planes filling the whole
         camera view, so every pixel gets a finite, valid depth reading —
         without these, empty background renders at MuJoCo's far-plane
         depth (tens of meters), which would corrupt feature triangulation. -->
    <geom name="floor" type="plane" size="3 3 0.1" pos="0 0.5 -0.5" material="floor_mat"/>
    <geom name="backwall" type="plane" size="3 3 0.1" pos="0 1.3 2.5" euler="90 0 0" material="floor_mat"/>
    <body name="cabinet" pos="0 0.6 0">
      {cabinet_tiles}
      <body name="drawer" pos="0 -0.05 -0.05">
        <joint name="slide" type="slide" axis="1 0 0" range="0 {config.slide_range_m}" damping="{config.slide_damping}"/>
        {drawer_tiles}
      </body>
    </body>
  </worldbody>
</mujoco>
"""


def linear_ramp(frame_idx: int, start_frame: int, end_frame: int, start_val: float, end_val: float) -> float:
    """Scripted-interaction helper: linearly ramps a joint's qpos from
    `start_val` (for frame_idx <= start_frame) to `end_val` (for frame_idx
    >= end_frame), matching the mission brief's "scripted...actuated joint
    motion" option for driving the demo object (as opposed to a physically
    simulated actuator/controller — unnecessary here since we're directly
    prescribing kinematics, not testing MuJoCo's actuation stack).
    """
    if frame_idx <= start_frame:
        return start_val
    if frame_idx >= end_frame:
        return end_val
    t = (frame_idx - start_frame) / (end_frame - start_frame)
    return start_val + t * (end_val - start_val)
