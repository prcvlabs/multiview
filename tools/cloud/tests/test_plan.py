"""plan.plan_shards and plan.wall_budget_s. The cases and their results are
also in fixtures/plan-golden.json, for the Worker's planner to replay;
MVCLOUD_UPDATE_GOLDEN=1 rewrites it."""

import dataclasses
import json
import os
import unittest

import support  # noqa: F401  (sys.path)
from mvcloud.plan import PlanError, ShardSpec, plan_shards, wall_budget_s

k_golden = os.path.join(support.k_tests_dir, "fixtures", "plan-golden.json")
k_cases = [
    (499, 0, 499, 250, 0, 10),
    (499, 0, 499, 100, 0, 10),
    (101, 0, 101, 100, 0, 10),
    (499, 0, 499, 250, 30, 10),
    (149, 0, 149, 80, 0, 10),
    (120, 0, 120, 60, 0, 10),
    (120, 0, 120, 120, 0, 10),
    (499, 300, 199, 300, 0, 10),
    (149, 0, 150, 80, 0, 10),
    (499, 5, 100, 50, 0, 10),
    (499, 0, 95, 50, 0, 10),
    (499, 0, 1, 50, 0, 10),
    (499, 0, 100, 1, 0, 10),
    (499, 0, 100, 50, -1, 10),
    (499, 0, 100, 50, 0, 0),
]


def run_case(case):
    try:
        return [dataclasses.asdict(s) for s in plan_shards(*case)]
    except PlanError as e:
        return {"error": str(e)}


class TestPlan(unittest.TestCase):
    def ranges(self, *case):
        return [(s.start, s.start + s.n) for s in plan_shards(*case)]

    def test_tiles(self):
        self.assertEqual(self.ranges(499, 0, 499, 250, 0, 10), [(0, 250), (250, 499)])
        shards = plan_shards(499, 0, 499, 100, 0, 10)
        self.assertEqual(len(shards), 5)
        self.assertEqual(shards[-1].n, 99)
        self.assertEqual(self.ranges(120, 0, 120, 60, 0, 10), [(0, 60), (60, 120)])

    def test_short_tail_is_folded(self):
        self.assertEqual(plan_shards(101, 0, 101, 100, 0, 10),
                         [ShardSpec(idx=0, start=0, n=101, keep_start=0, keep_n=101)])

    def test_overlap(self):
        s = plan_shards(499, 0, 499, 250, 30, 10)
        self.assertEqual((s[0].start, s[0].n, s[0].keep_start, s[0].keep_n), (0, 250, 0, 250))
        self.assertEqual((s[1].start, s[1].start + s[1].n), (220, 499))
        self.assertEqual((s[1].keep_start, s[1].keep_start + s[1].keep_n), (250, 499))

    def test_end_is_the_scene_end(self):
        self.assertEqual(self.ranges(149, 0, 149, 80, 0, 10), [(0, 80), (80, 149)])
        self.assertEqual(self.ranges(499, 300, 199, 300, 0, 10), [(300, 499)])

    def test_rejects(self):
        for case in [(149, 0, 150, 80, 0, 10),  # past the 149-frame scene, although 150 % 10 == 0
                     (499, 5, 100, 50, 0, 10),  # unaligned start
                     (499, 0, 95, 50, 0, 10),  # unaligned end before the scene end
                     (499, 0, 1, 50, 0, 10),  # one frame
                     (499, 0, 100, 1, 0, 10),  # shard_frames < 2
                     (499, 0, 100, 50, -1, 10),
                     (499, 0, 100, 50, 0, 0)]:
            with self.subTest(case=case), self.assertRaises(PlanError):
                plan_shards(*case)

    def test_keeps_tile_the_range(self):
        for total, start, n, f in [(499, 0, 499, 7), (3150, 600, 2550, 300), (149, 0, 149, 3)]:
            keeps = [(s.keep_start, s.keep_n) for s in plan_shards(total, start, n, f, 0, 1)]
            frames = [t for k0, kn in keeps for t in range(k0, k0 + kn)]
            self.assertEqual(frames, list(range(start, start + n)))
            self.assertTrue(all(kn >= 2 for _, kn in keeps))

    def test_wall_budget(self):
        self.assertEqual(wall_budget_s("loc", 300, "replay"), 3180)
        self.assertEqual(wall_budget_s("loc", 300, "hyperpose"), 3300)
        self.assertEqual(wall_budget_s("tracks", 3150, "replay"), 9630)

    def test_golden(self):
        golden = [{"case": list(c), "result": run_case(c)} for c in k_cases]
        if os.environ.get("MVCLOUD_UPDATE_GOLDEN") == "1":
            with open(k_golden, "w") as f:
                f.write("[\n" + ",\n".join(json.dumps(x) for x in golden) + "\n]\n")
        with open(k_golden) as f:
            self.assertEqual(json.load(f), golden)


if __name__ == "__main__":
    unittest.main()
