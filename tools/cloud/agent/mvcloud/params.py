"""Pipeline parameters: the patch an agent applies, and the digest that
shows the shards of a job ran with the same parameters.
"""

import copy
import hashlib
import json

# Keys that name local paths or the run's own range, not the computation
k_volatile_keys = frozenset({"out_dir", "outdir", "feedback", "replay_dir", "start_frame"})


# --------------------------------------------------------------- strip_volatile
def strip_volatile(o):
    """A copy of `o` without the volatile keys, at any depth."""
    if isinstance(o, dict):
        return {k: strip_volatile(v) for k, v in o.items() if k not in k_volatile_keys}
    if isinstance(o, list):
        return [strip_volatile(v) for v in o]
    return o


# --------------------------------------------------------- canonical_params_md5
def canonical_params_md5(task_params):
    """md5 of the `task_params` object, independent of key order,
    whitespace and the volatile keys."""
    s = json.dumps(strip_volatile(task_params), sort_keys=True, separators=(",", ":"))
    return hashlib.md5(s.encode("utf-8")).hexdigest()


# -------------------------------------------------------------- params_file_md5
def params_file_md5(fname):
    """canonical_params_md5 of a params.json that a pipeline run wrote."""
    with open(fname) as f:
        return canonical_params_md5(json.load(f)["task_params"])


# ----------------------------------------------------------------- patch_params
def patch_params(params, *, pose_mode, replay_dir=None):
    """A copy of `params` (a params.json object) set up for `pose_mode`:
    "replay" reads the 2D poses from `replay_dir`; "hyperpose" runs the pose
    network and writes the poses it finds next to the outputs."""
    out = copy.deepcopy(params)
    op = out["task_params"]["frame_params"]["pose_skeleton_init"]["op_params"]
    if pose_mode == "replay":
        if not replay_dir:
            raise ValueError("pose_mode 'replay' needs a replay directory")
        op["replay_dir"] = str(replay_dir)
    elif pose_mode == "hyperpose":
        op["replay_dir"] = ""
        op["feedback"] = True
        op["use_hyperpose"] = True
    else:
        raise ValueError(f"unknown pose_mode '{pose_mode}'")
    return out
