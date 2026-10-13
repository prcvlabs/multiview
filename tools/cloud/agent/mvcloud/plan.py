"""Shard planning: the loc shards that tile a job's frame range.

A job covers the scene frames [start, start + n). The tracks job runs once
over that whole range, so the range must be aligned to K, the frames per
tracklet (`pipeline --require-aligned` checks it again); loc shards need no
alignment. Every loc shard keeps [S_i, S_i + N_i); with an overlap O (a
calibration tool), shard i > 0 also computes the O frames before S_i.
"""

from dataclasses import dataclass


class PlanError(ValueError):
    """The job's frame range or shard size cannot be planned."""


# -------------------------------------------------------------------- ShardSpec
@dataclass(frozen=True)
class ShardSpec:
    idx: int
    start: int  # first computed frame (--start)
    n: int  # computed frames (--n-frames)
    keep_start: int  # first kept frame
    keep_n: int  # kept frames


# ------------------------------------------------------------------ check_range
def check_range(total, start, n, K):
    """Raises PlanError unless [start, start + n) is a valid tracks-job range
    of a scene with `total` frames: the same rules as shard_alignment_error
    in the C++ (shard-range.cpp)."""
    end = start + n
    if K <= 0:
        raise PlanError(f"K must be positive, got {K}")
    if n < 2:
        raise PlanError(f"at least 2 frames required, got n_frames {n}")
    if start < 0 or end > total:
        raise PlanError(f"frames [{start}, {end}) are outside the scene's {total} frames")
    if start % K != 0:
        raise PlanError(f"start frame {start} is not a multiple of K = {K}")
    if end % K != 0 and end != total:
        raise PlanError(f"end frame {end} is neither a multiple of K = {K} "
                        f"nor the scene's last frame {total}")


# ------------------------------------------------------------------ plan_shards
def plan_shards(total, start, n, shard_frames, overlap, K):
    """The loc shards of the job range [start, start + n). A last shard of
    fewer than 2 frames (the pipeline refuses those) is folded into the one
    before it."""
    check_range(total, start, n, K)
    if shard_frames < 2:
        raise PlanError(f"shard_frames must be at least 2, got {shard_frames}")
    if overlap < 0:
        raise PlanError(f"overlap must not be negative, got {overlap}")

    end = start + n
    keeps = []
    for s in range(start, end, shard_frames):
        keep_n = min(shard_frames, end - s)
        if keep_n < 2 and keeps:
            prev_start, prev_n = keeps.pop()
            keeps.append((prev_start, prev_n + keep_n))
        else:
            keeps.append((s, keep_n))

    shards = []
    for idx, (s, keep_n) in enumerate(keeps):
        o = 0 if idx == 0 else min(overlap, s)
        shards.append(ShardSpec(idx=idx, start=s - o, n=keep_n + o, keep_start=s, keep_n=keep_n))
    return shards


# ---------------------------------------------------------------- wall_budget_s
def wall_budget_s(kind, n_frames, pose_mode):
    """The wall-clock budget of one attempt, from the measured 1.3-2.3 s per
    loc frame and ~1 s per tracks frame, with a large margin. A HyperPose
    loc shard may also build its TensorRT engine."""
    if kind == "loc":
        return 180 + 10 * n_frames + (120 if pose_mode == "hyperpose" else 0)
    if kind == "tracks":
        return 180 + 3 * n_frames
    raise PlanError(f"unknown shard kind '{kind}'")
