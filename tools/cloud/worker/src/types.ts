// Types shared by the Worker's modules: the bindings, the job spec as it is
// stored in D1, the rows, and the lease an agent receives (the TypedDict
// control.Lease of the agent).

import type { PoseMode, ShardKind } from "./plan";

export interface Env {
  DB: D1Database;
  BUCKET: R2Bucket;
  SHARDS: Queue<ShardMessage>;
  JOB_WORKFLOW: Workflow<JobParams>;
  BLOB_MODE: string; // "proxy" (presigned URLs are milestone M8, not built)
  MULTIPART_PART_BYTES: string; // outputs larger than this go up in parts
  MVCLOUD_ADMIN_TOKEN: string; // submit, cancel, status page, any blob
  MVCLOUD_WORKER_TOKEN: string; // leases and the blobs of the caller's lease
}

export interface JobParams {
  jobId: string;
}

// Dispatch only: D1 is the authority, so a stale or duplicate message cannot
// claim (the claim is conditional on the attempt).
export interface ShardMessage {
  v: 1;
  job_id: string;
  shard_id: string;
  attempt: number;
}

// One input of every shard of a job: an object in the bucket, verified by
// the agent against its size and md5 (null: unknown, not verified).
export interface JobInput {
  name: string; // the file name in the attempt's in/ directory
  key: string;
  kind: "file" | "tar"; // a tar (the replayed 2D poses) is extracted once
  bytes: number | null;
  md5: string | null;
  cache: "scene" | "job" | null;
}

export interface JobSpec {
  scene_id: string;
  manifest_key: string;
  params_key: string;
  poses_key: string | null; // a tar of pose-2d_<frame>_<sensor>.json (replay)
  gt_key: string | null; // ground truth: the tracks job scores against it
  videos: { name: string; key: string }[];
  total_frames: number;
  K: number;
  n_cameras: number;
  start_frame: number;
  n_frames: number;
  shard_frames: number;
  overlap_frames: number;
  still_warmup: number;
  no_stereo: boolean;
  pose_mode: PoseMode;
  pose_classifier: string;
  target_fps: number;
  extra_cli_args: string[] | { loc?: string[]; tracks?: string[] };
  max_attempts: number;
  lease_seconds: number;
  heartbeat_s: number;
  max_wall_s: { loc: number | null; tracks: number | null };
  progress_timeout_s: number;
  retry_delay_s: number; // the n-th retry waits min(600, retry_delay_s * 2^(n-1))
  wait_seconds: number; // the Workflow's sweep period
  inputs: JobInput[]; // resolved at submit
}

export type JobState = "planned" | "running" | "merging" | "done" | "failed" | "cancelled";
export type ShardState = "blocked" | "pending" | "leased" | "failed" | "done" | "dead" | "cancelled";

export interface JobRow {
  id: string;
  scene_id: string;
  spec_json: string;
  state: JobState;
  workflow_id: string | null;
  created_at: number;
  updated_at: number;
  error: string | null;
  summary_json: string | null;
}

export interface ShardRow {
  job_id: string;
  id: string;
  kind: ShardKind;
  seq: number;
  start_frame: number;
  n_frames: number;
  keep_start: number;
  keep_n: number;
  max_wall_s: number;
  lease_seconds: number;
  state: ShardState;
  attempt: number;
  max_attempts: number;
  not_before: number;
  worker_id: string | null;
  lease_token: string | null;
  lease_started_at: number | null;
  lease_expires_at: number | null;
  run_started_at: number | null;
  last_heartbeat_at: number | null;
  progress_json: string | null;
  output_prefix: string | null;
  done_json: string | null;
  params_md5: string | null;
  error: string | null;
  created_at: number;
  updated_at: number;
}

export interface AttemptRow {
  job_id: string;
  shard_id: string;
  attempt: number;
  worker_id: string | null;
  started_at: number;
  finished_at: number | null;
  outcome: string | null;
  error: string | null;
  wall_s: number | null;
}

export interface WorkerInfo {
  worker_id: string;
  gpu?: string;
  version?: string;
}

export interface LeaseInput {
  name: string;
  url: string;
  kind: "file" | "tar";
  bytes: number | null;
  md5: string | null;
  cache: "scene" | "job" | null;
}

export interface MultipartSpec {
  init_url: string;
  part_url: string;
  complete_url: string;
  abort_url: string;
  part_bytes: number;
}

export interface LeaseOutput {
  name: string;
  url: string;
  required: boolean;
  multipart: MultipartSpec | null;
}

export interface Lease {
  shard_id: string;
  job_id: string;
  attempt: number;
  lease_token: string;
  lease_expires_at: number;
  lease_seconds: number;
  heartbeat_s: number;
  max_wall_s: number;
  progress_timeout_s: number;
  kind: ShardKind;
  scene_id: string;
  total_frames: number;
  n_cameras: number;
  output_prefix: string;
  cli: {
    start_frame: number;
    n_frames: number;
    K: number;
    no_stereo: boolean;
    no_still: boolean;
    pose_mode: PoseMode;
    pose_classifier: string | null;
    still_warmup: number;
    require_aligned: boolean;
    target_fps: number;
    extra_args: string[];
  };
  inputs: LeaseInput[];
  outputs: LeaseOutput[];
  keep: { name: string; start: number; n: number; computed_start: number; computed_n: number }[];
  params_md5: string | null;
}

// done.json as the agent writes it (only the fields the Worker reads).
export interface DoneRecord {
  outputs: { name: string; bytes: number; md5: string }[];
  params_md5?: string | null;
  wall_s?: number;
  n_tracks?: number;
  score?: unknown;
  [key: string]: unknown;
}
