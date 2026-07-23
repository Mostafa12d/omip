# omip_mujoco_wrapper

A thin, separate Python package that drives [`omip_core`](../omip_core)
(the ROS-free port of OMIP's kinematic-structure estimation pipeline) from
MuJoCo. See the repo root's `PORTING_NOTES.md` for the full porting story;
this package is Phase 6.

- `omip_mujoco_wrapper/driver.py` — steps MuJoCo, renders synchronized
  RGB + depth + camera intrinsics per frame. The only module that knows
  about MuJoCo.
- `omip_mujoco_wrapper/orchestrator.py` — calls omip_core's 3 ported stages
  (feature_tracker → rb_tracker → joint_tracker) in sequence per frame.
  Only knows about omip_core's plain Python API — MuJoCo-agnostic, so the
  same orchestrator would work with frames from a real depth camera
  instead.
- `omip_mujoco_wrapper/scene_utils.py` — builds the demo MJCF scene (a
  drawer sliding out of a cabinet) and scripts its motion.
- `examples/run_demo.py` — ties the three together and prints a joint
  type/parameter estimate at the end.

## Setup

1. Build `omip_core` first (from the repo root):

   ```
   cd omip_core
   mkdir -p build && cd build
   cmake ..
   cmake --build . -j
   ```

   This needs `pybind11` and Python dev headers available to CMake (see
   `omip_core/CMakeLists.txt`'s `OMIP_CORE_BUILD_PYTHON_BINDINGS` option) —
   e.g. `brew install pybind11` on macOS. If they're missing, CMake skips
   the Python bindings with a status message rather than failing the whole
   build.

2. Create a virtual environment and install this package's dependencies:

   ```
   python3 -m venv .venv
   source .venv/bin/activate
   pip install mujoco numpy
   pip install -e omip_mujoco_wrapper   # or just: pip install mujoco numpy
   ```

   `omip_mujoco_wrapper/__init__.py` automatically adds
   `omip_core/build/python` to `sys.path` if `omip_core` isn't already
   importable, so no manual path wiring is needed as long as you built
   `omip_core` in-tree per step 1.

3. Run the demo:

   ```
   python omip_mujoco_wrapper/examples/run_demo.py
   ```

   It renders a synthetic drawer sliding out of a cabinet, feeds the
   frames through the full pipeline, and prints the estimated joint type
   and parameters at the end — no ROS installed anywhere.

## Known limitations (see PORTING_NOTES.md Phase 6 for details)

- The demo scene is a sliding drawer (prismatic joint), not the hinged
  door originally tried first — the revolute joint filter's EKF didn't
  converge reliably for a hinge within the tuning time available in this
  phase; the prismatic/drawer case did, cleanly and repeatably. Getting a
  hinged-door demo to classify correctly is a reasonable follow-up.
- `omip_core` isn't a pip-installable dependency yet (no build backend
  compiles its C++ side as part of `pip install`) — see step 1 above.
