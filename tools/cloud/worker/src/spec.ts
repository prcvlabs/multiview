// The job spec of POST /v1/jobs: defaults (those of fsbackend.k_job_defaults
// in the agent), validation and the plan check, before anything is stored.

import { PlanError, planShards, type ShardKind } from "./plan";
import type { JobSpec } from "./types";

export type JobSpecIn = Omit<JobSpec, "inputs">;

const k_defaults = {
  start_frame: 0,
  n_frames: null as number | null, // null: to the end of the scene
  shard_frames: 300,
  overlap_frames: 0,
  still_warmup: 0,
  no_stereo: true,
  pose_mode: "replay",
  pose_classifier: "pose-svm-lite_v0",
  target_fps: 0.0,
  extra_cli_args: [] as string[],
  max_attempts: 3,
  lease_seconds: 1800,
  heartbeat_s: 60,
  max_wall_s: { loc: null, tracks: null },
  progress_timeout_s: 900,
  retry_delay_s: 30,
  wait_seconds: 300,
  poses_key: null,
  gt_key: null,
  videos: [] as { name: string; key: string }[],
};

// ------------------------------------------------------------------ isBlobKey
// A key the blob routes accept: relative, no empty, "." or ".." segment.
export function isBlobKey(key: string): boolean {
  if (key.length === 0 || key.length > 1024) return false;
  return key.split("/").every((s) => s.length > 0 && s !== "." && s !== "..");
}

// ---------------------------------------------------------------- integer_
function integer_(spec: Record<string, unknown>, key: string, min: number): number {
  const x = spec[key];
  if (typeof x !== "number" || !Number.isInteger(x) || x < min)
    throw new PlanError(`'${key}' must be an integer >= ${min}, got ${JSON.stringify(x)}`);
  return x;
}

// ---------------------------------------------------------------- string_
function string_(spec: Record<string, unknown>, key: string): string {
  const x = spec[key];
  if (typeof x !== "string" || x.length === 0)
    throw new PlanError(`'${key}' must be a non-empty string, got ${JSON.stringify(x)}`);
  return x;
}

// --------------------------------------------------------------- key_
function key_(spec: Record<string, unknown>, key: string): string {
  const x = string_(spec, key);
  if (!isBlobKey(x)) throw new PlanError(`'${key}' is not a valid object key: '${x}'`);
  return x;
}

// ---------------------------------------------------------------- normalizeSpec
// The spec with its defaults filled in; throws PlanError when a field is
// missing or invalid, or the range cannot be planned.
export function normalizeSpec(body: unknown): JobSpecIn {
  if (typeof body !== "object" || body === null || Array.isArray(body))
    throw new PlanError("the job spec must be a json object");
  const s: Record<string, unknown> = { ...k_defaults, ...(body as Record<string, unknown>) };
  if (s.n_frames === null || s.n_frames === undefined)
    s.n_frames = (s.total_frames as number) - (s.start_frame as number);

  const scene_id = string_(s, "scene_id");
  if (!/^[A-Za-z0-9_.-]+$/.test(scene_id))
    throw new PlanError(`'scene_id' may hold letters, digits and _ . - only: '${scene_id}'`);
  const total_frames = integer_(s, "total_frames", 1);
  const K = integer_(s, "K", 1);
  const n_cameras = integer_(s, "n_cameras", 1);
  const start_frame = integer_(s, "start_frame", 0);
  const n_frames = integer_(s, "n_frames", 0);
  const shard_frames = integer_(s, "shard_frames", 0);
  const overlap_frames = integer_(s, "overlap_frames", 0);
  const still_warmup = integer_(s, "still_warmup", 0);
  if (s.pose_mode !== "replay" && s.pose_mode !== "hyperpose")
    throw new PlanError(`unknown pose_mode '${String(s.pose_mode)}'`);
  if (s.pose_mode === "replay" && !s.poses_key)
    throw new PlanError("pose_mode 'replay' needs 'poses_key' (a tar of the 2D poses)");
  if (!Array.isArray(s.videos) || s.videos.length === 0)
    throw new PlanError("'videos' must list the manifest's videos as {name, key}");
  const videos = (s.videos as Record<string, unknown>[]).map((v) => ({
    name: string_(v, "name"),
    key: key_(v, "key"),
  }));
  const extra = s.extra_cli_args;
  const strings = (x: unknown) => Array.isArray(x) && x.every((a) => typeof a === "string");
  if (
    !strings(extra) &&
    !(
      typeof extra === "object" &&
      extra !== null &&
      Object.entries(extra).every(([k, v]) => (k === "loc" || k === "tracks") && strings(v))
    )
  )
    throw new PlanError("'extra_cli_args' must be a list of strings or {loc: [...], tracks: [...]}");
  const wall = (s.max_wall_s ?? {}) as Record<string, unknown>;
  const budget = (k: ShardKind) => {
    const x = wall[k];
    if (x === null || x === undefined) return null;
    if (typeof x !== "number" || !Number.isInteger(x) || x <= 0)
      throw new PlanError(`'max_wall_s.${k}' must be a positive integer or null`);
    return x;
  };

  const spec: JobSpecIn = {
    scene_id,
    manifest_key: key_(s, "manifest_key"),
    params_key: key_(s, "params_key"),
    poses_key: s.poses_key ? key_(s, "poses_key") : null,
    gt_key: s.gt_key ? key_(s, "gt_key") : null,
    videos,
    total_frames,
    K,
    n_cameras,
    start_frame,
    n_frames,
    shard_frames,
    overlap_frames,
    still_warmup,
    no_stereo: s.no_stereo === true,
    pose_mode: s.pose_mode,
    pose_classifier: string_(s, "pose_classifier"),
    target_fps: typeof s.target_fps === "number" ? s.target_fps : 0.0,
    extra_cli_args: extra as JobSpecIn["extra_cli_args"],
    max_attempts: integer_(s, "max_attempts", 1),
    lease_seconds: integer_(s, "lease_seconds", 5),
    heartbeat_s: integer_(s, "heartbeat_s", 1),
    max_wall_s: { loc: budget("loc"), tracks: budget("tracks") },
    progress_timeout_s: integer_(s, "progress_timeout_s", 1),
    retry_delay_s: integer_(s, "retry_delay_s", 0),
    wait_seconds: integer_(s, "wait_seconds", 1),
  };
  planShards(total_frames, start_frame, n_frames, shard_frames, overlap_frames, K);
  return spec;
}

// ------------------------------------------------------------- extraArgs
export function extraArgs(spec: JobSpec, kind: ShardKind): string[] {
  const x = spec.extra_cli_args;
  return Array.isArray(x) ? [...x] : [...(x[kind] ?? [])];
}

// ------------------------------------------------------------- outputNames
// The products of a shard and which of them must exist for `complete`.
export function outputNames(kind: ShardKind, hasGt: boolean): { name: string; required: boolean }[] {
  const loc = ["localization.data", "params.json", "run-object.json", "stdout.log", "poses.tar", "done.json"];
  const tracks = ["pipeline-output.json", "params.json", "score.json", "overlap.json", "stdout.log", "done.json"];
  const required = new Set(
    kind === "loc"
      ? ["localization.data", "params.json", "stdout.log", "done.json"]
      : ["pipeline-output.json", "params.json", "stdout.log", "done.json", ...(hasGt ? ["score.json"] : [])],
  );
  return (kind === "loc" ? loc : tracks).map((name) => ({ name, required: required.has(name) }));
}
