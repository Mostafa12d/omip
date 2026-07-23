"""omip_mujoco_wrapper: a thin, separate Python package that drives
omip_core from MuJoCo. Per the mission brief: this package must never
contain ROS-porting logic (that's all in omip_core) and omip_core must
never contain MuJoCo/simulation-specific code — this package is the only
place the two are wired together.

`omip_core` is a locally-built pybind11 extension (Phase 5), not yet
published as an installable wheel (no build backend builds its C++ side as
part of `pip install` — see PORTING_NOTES.md's Phase 6 section for this
being flagged as a known follow-up, not something this phase needed to
solve). Until that exists, importing `omip_core` requires its build output
directory on `sys.path`. This bootstrap does that automatically for the
common case (this repo's own `omip_core/build/python`, built via the
instructions in the top-level README/PORTING_NOTES), so `import
omip_mujoco_wrapper` "just works" for anyone who already built omip_core
per Phases 0-5 — without requiring every script to repeat the same
sys.path fiddling, and without silently masking a real ImportError if
omip_core truly isn't built anywhere findable.
"""
import importlib.util
import pathlib
import sys

if importlib.util.find_spec("omip_core") is None:
    _repo_root = pathlib.Path(__file__).resolve().parents[2]
    _candidate = _repo_root / "omip_core" / "build" / "python"
    if _candidate.is_dir():
        sys.path.insert(0, str(_candidate))
