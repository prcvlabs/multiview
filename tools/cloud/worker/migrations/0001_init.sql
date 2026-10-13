-- mvcloud control-plane state (D1). Times are seconds since the epoch, as
-- REAL: the clock of the agent's leases (control.Lease.lease_expires_at).
--
-- Every state change of a shard is one conditional UPDATE whose WHERE clause
-- names the state (and, for an agent's call, the lease token) it expects;
-- the Worker checks meta.changes, so a late or duplicate call changes
-- nothing.

-- A job: one scene range, its spec (with the inputs resolved to R2 keys,
-- sizes and md5s) and the Workflow instance that drives it.
--   planned -> running (loc shards) -> merging (tracks job) -> done
--   any of those -> failed | cancelled
CREATE TABLE jobs (
  id TEXT PRIMARY KEY,
  scene_id TEXT NOT NULL,
  spec_json TEXT NOT NULL,
  state TEXT NOT NULL CHECK (state IN ('planned', 'running', 'merging', 'done', 'failed', 'cancelled')),
  workflow_id TEXT,
  created_at REAL NOT NULL,
  updated_at REAL NOT NULL,
  error TEXT,
  summary_json TEXT
);

-- A shard: a loc shard (loc-0000, ...) or the job's tracks job (tracks).
--   blocked (the tracks job, until every loc shard is done) -> pending
--   pending -> leased -> done | failed (retryable; pending again after
--   not_before) | dead (out of attempts, or fatal) ; pending | leased ->
--   cancelled
-- run_started_at is the Worker's time of the first heartbeat after the
-- agent took the GPU lock: the wall-clock budget counts from there.
CREATE TABLE shards (
  job_id TEXT NOT NULL REFERENCES jobs (id),
  id TEXT NOT NULL,
  kind TEXT NOT NULL CHECK (kind IN ('loc', 'tracks')),
  seq INTEGER NOT NULL,
  start_frame INTEGER NOT NULL,
  n_frames INTEGER NOT NULL,
  keep_start INTEGER NOT NULL,
  keep_n INTEGER NOT NULL,
  max_wall_s INTEGER NOT NULL,
  lease_seconds INTEGER NOT NULL,
  state TEXT NOT NULL CHECK (state IN ('blocked', 'pending', 'leased', 'failed', 'done', 'dead', 'cancelled')),
  attempt INTEGER NOT NULL DEFAULT 0,
  max_attempts INTEGER NOT NULL DEFAULT 3,
  not_before REAL NOT NULL DEFAULT 0,
  worker_id TEXT,
  lease_token TEXT,
  lease_started_at REAL,
  lease_expires_at REAL,
  run_started_at REAL,
  last_heartbeat_at REAL,
  progress_json TEXT,
  output_prefix TEXT,
  done_json TEXT,
  params_md5 TEXT,
  error TEXT,
  created_at REAL NOT NULL,
  updated_at REAL NOT NULL,
  PRIMARY KEY (job_id, id),
  UNIQUE (job_id, kind, seq)
);
CREATE INDEX shards_state_kind ON shards (state, kind);
CREATE INDEX shards_lease_token ON shards (lease_token);

-- One row per lease handed out. outcome: done, retry, timeout, fatal,
-- stopped, expired, cancelled.
CREATE TABLE attempts (
  job_id TEXT NOT NULL,
  shard_id TEXT NOT NULL,
  attempt INTEGER NOT NULL,
  worker_id TEXT,
  started_at REAL NOT NULL,
  finished_at REAL,
  outcome TEXT,
  error TEXT,
  wall_s REAL,
  PRIMARY KEY (job_id, shard_id, attempt),
  FOREIGN KEY (job_id, shard_id) REFERENCES shards (job_id, id)
);

CREATE TABLE workers (
  id TEXT PRIMARY KEY,
  gpu TEXT,
  version TEXT,
  first_seen_at REAL NOT NULL,
  last_seen_at REAL NOT NULL,
  current_shard TEXT,
  shards_done INTEGER NOT NULL DEFAULT 0
);
