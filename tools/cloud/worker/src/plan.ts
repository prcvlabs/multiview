// Shard planning: the loc shards that tile a job's frame range. The same
// rules, and the same error messages, as tools/cloud/agent/mvcloud/plan.py;
// test/plan.test.ts replays tests/fixtures/plan-golden.json, which
// test_plan.py writes.
//
// A job covers the scene frames [start, start + n). The tracks job runs once
// over that whole range, so the range must be aligned to K, the frames per
// tracklet (`pipeline --require-aligned` checks it again); loc shards need
// no alignment. Every loc shard keeps [S_i, S_i + N_i); with an overlap O (a
// calibration tool), shard i > 0 also computes the O frames before S_i.

export class PlanError extends Error {
  constructor(message: string) {
    super(message);
    this.name = "PlanError";
  }
}

export interface ShardSpec {
  idx: number;
  start: number; // first computed frame (--start)
  n: number; // computed frames (--n-frames)
  keep_start: number; // first kept frame
  keep_n: number; // kept frames
}

export type ShardKind = "loc" | "tracks";
export type PoseMode = "replay" | "hyperpose";

// ----------------------------------------------------------------- checkRange
// Throws PlanError unless [start, start + n) is a valid tracks-job range of a
// scene with `total` frames (shard_alignment_error in shard-range.cpp).
export function checkRange(total: number, start: number, n: number, K: number): void {
  const end = start + n;
  if (K <= 0) throw new PlanError(`K must be positive, got ${K}`);
  if (n < 2) throw new PlanError(`at least 2 frames required, got n_frames ${n}`);
  if (start < 0 || end > total)
    throw new PlanError(`frames [${start}, ${end}) are outside the scene's ${total} frames`);
  if (start % K !== 0) throw new PlanError(`start frame ${start} is not a multiple of K = ${K}`);
  if (end % K !== 0 && end !== total)
    throw new PlanError(
      `end frame ${end} is neither a multiple of K = ${K} nor the scene's last frame ${total}`,
    );
}

// ----------------------------------------------------------------- planShards
// The loc shards of the job range [start, start + n). A last shard of fewer
// than 2 frames (the pipeline refuses those) is folded into the one before.
export function planShards(
  total: number,
  start: number,
  n: number,
  shardFrames: number,
  overlap: number,
  K: number,
): ShardSpec[] {
  checkRange(total, start, n, K);
  if (shardFrames < 2) throw new PlanError(`shard_frames must be at least 2, got ${shardFrames}`);
  if (overlap < 0) throw new PlanError(`overlap must not be negative, got ${overlap}`);

  const end = start + n;
  const keeps: [number, number][] = [];
  for (let s = start; s < end; s += shardFrames) {
    const keepN = Math.min(shardFrames, end - s);
    if (keepN < 2 && keeps.length > 0) keeps[keeps.length - 1][1] += keepN;
    else keeps.push([s, keepN]);
  }

  return keeps.map(([s, keepN], idx) => {
    const o = idx === 0 ? 0 : Math.min(overlap, s);
    return { idx, start: s - o, n: keepN + o, keep_start: s, keep_n: keepN };
  });
}

// ---------------------------------------------------------------- wallBudgetS
// The wall-clock budget of one attempt (plan.wall_budget_s): the measured
// 1.3-2.3 s per loc frame and ~1 s per tracks frame, with a large margin. A
// HyperPose loc shard may also build its TensorRT engine.
export function wallBudgetS(kind: ShardKind, nFrames: number, poseMode: PoseMode): number {
  if (kind === "loc") return 180 + 10 * nFrames + (poseMode === "hyperpose" ? 120 : 0);
  if (kind === "tracks") return 180 + 3 * nFrames;
  throw new PlanError(`unknown shard kind '${kind}'`);
}
