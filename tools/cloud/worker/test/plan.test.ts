// planShards replays the cases of tests/fixtures/plan-golden.json, which
// test_plan.py (the agent's planner) writes: the two planners agree on every
// shard and on every error message.

import { readFileSync } from "node:fs";
import { describe, expect, it } from "vitest";

import { PlanError, planShards, wallBudgetS, type ShardSpec } from "../src/plan";

type Case = [number, number, number, number, number, number];
const golden = JSON.parse(
  readFileSync(new URL("../../tests/fixtures/plan-golden.json", import.meta.url), "utf8"),
) as { case: Case; result: ShardSpec[] | { error: string } }[];

function run(c: Case): ShardSpec[] | { error: string } {
  try {
    return planShards(...c);
  } catch (e) {
    if (e instanceof PlanError) return { error: e.message };
    throw e;
  }
}

describe("planShards", () => {
  it("has the golden cases", () => {
    expect(golden.length).toBeGreaterThanOrEqual(15);
  });
  for (const g of golden) {
    it(`plans ${JSON.stringify(g.case)} as the Python planner does`, () => {
      expect(run(g.case)).toEqual(g.result);
    });
  }

  it("tiles the range with keeps of at least 2 frames", () => {
    for (const [total, start, n, f] of [
      [499, 0, 499, 7],
      [3150, 600, 2550, 300],
      [149, 0, 149, 3],
    ]) {
      const frames = planShards(total, start, n, f, 0, 1).flatMap((s) =>
        Array.from({ length: s.keep_n }, (_, i) => s.keep_start + i),
      );
      expect(frames).toEqual(Array.from({ length: n }, (_, i) => start + i));
    }
  });
});

describe("wallBudgetS", () => {
  it("mirrors plan.wall_budget_s", () => {
    expect(wallBudgetS("loc", 300, "replay")).toBe(3180);
    expect(wallBudgetS("loc", 300, "hyperpose")).toBe(3300);
    expect(wallBudgetS("tracks", 3150, "replay")).toBe(9630);
  });
});
