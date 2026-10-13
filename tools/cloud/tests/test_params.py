"""params.canonical_params_md5 and params.patch_params."""

import copy
import json
import os
import tempfile
import unittest

import support  # noqa: F401  (sys.path)
from mvcloud.params import canonical_params_md5, params_file_md5, patch_params

k_params = {
    "task_params": {
        "frame_params": {"pose_skeleton_init": {"op_params": {
            "feedback": False, "replay_dir": "/a", "use_hyperpose": True}}},
        "tracklet_params": {"max_frames_per_tracklet": 10, "out_dir": "/x"},
        "post_process": {"duration_median": 3.0, "list": [{"start_frame": 4, "v": 1}]},
    }
}


class TestParams(unittest.TestCase):
    def test_stable_under_order_and_whitespace(self):
        a = k_params["task_params"]
        b = json.loads(json.dumps(a, indent=4, sort_keys=True))
        reordered = dict(reversed(list(b.items())))
        self.assertEqual(canonical_params_md5(a), canonical_params_md5(reordered))

    def test_ignores_volatile_keys_at_any_depth(self):
        a = copy.deepcopy(k_params["task_params"])
        b = copy.deepcopy(a)
        b["frame_params"]["pose_skeleton_init"]["op_params"].update(replay_dir="/b", feedback=True)
        b["tracklet_params"]["out_dir"] = "/y"
        b["post_process"]["list"][0]["start_frame"] = 99
        b["outdir"] = "/z"
        self.assertEqual(canonical_params_md5(a), canonical_params_md5(b))

    def test_changes_with_k(self):
        a = copy.deepcopy(k_params["task_params"])
        b = copy.deepcopy(a)
        b["tracklet_params"]["max_frames_per_tracklet"] = 20
        self.assertNotEqual(canonical_params_md5(a), canonical_params_md5(b))

    def test_file_digest_uses_task_params(self):
        with tempfile.TemporaryDirectory() as d:
            fname = os.path.join(d, "params.json")
            with open(fname, "w") as f:
                json.dump({**k_params, "config": {"start_frame": 7}}, f)
            self.assertEqual(params_file_md5(fname), canonical_params_md5(k_params["task_params"]))

    def test_patch_replay(self):
        p = patch_params(k_params, pose_mode="replay", replay_dir="/poses")
        op = p["task_params"]["frame_params"]["pose_skeleton_init"]["op_params"]
        self.assertEqual(op["replay_dir"], "/poses")
        self.assertEqual(k_params["task_params"]["frame_params"]["pose_skeleton_init"]
                         ["op_params"]["replay_dir"], "/a")  # a copy
        with self.assertRaises(ValueError):
            patch_params(k_params, pose_mode="replay", replay_dir=None)

    def test_patch_hyperpose(self):
        p = patch_params(k_params, pose_mode="hyperpose")
        op = p["task_params"]["frame_params"]["pose_skeleton_init"]["op_params"]
        self.assertEqual((op["replay_dir"], op["feedback"], op["use_hyperpose"]), ("", True, True))
        with self.assertRaises(ValueError):
            patch_params(k_params, pose_mode="openpose")


if __name__ == "__main__":
    unittest.main()
