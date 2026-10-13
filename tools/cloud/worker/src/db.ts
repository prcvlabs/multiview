// The D1 state of jobs, shards, attempts and workers, and the lease protocol
// on it (section 6.8 of the plan; fsbackend.FsControl is the same protocol on
// a directory).
//
// Agents move a shard pending -> leased -> done | failed | dead; the Workflow
// (the only re-enqueuer) moves failed and expired shards back to pending; the
// last loc shard's complete (or the Workflow) moves the tracks job from
// blocked to pending; cancel moves pending and leased shards to cancelled.
// Every one of those changes is a conditional UPDATE whose WHERE clause
// names the state and, for an agent's call, the lease token it expects,
// checked through meta.changes. Nothing here imports a
// Workers runtime module, so test/db.test.ts runs it on node:sqlite.

import { planShards, wallBudgetS, type ShardKind } from "./plan";
import { extraArgs, outputNames } from "./spec";
import type {
  AttemptRow,
  DoneRecord,
  JobRow,
  JobSpec,
  Lease,
  ShardMessage,
  ShardRow,
  WorkerInfo,
} from "./types";

// The outputs that are uploaded in parts when they are larger than one part.
const k_multipart_outputs = new Set(["localization.data", "poses.tar"]);
const k_retry_delay_max_s = 600;

// ------------------------------------------------------------------------- Job
export interface Job {
  row: JobRow;
  spec: JobSpec;
}

// --------------------------------------------------------------- attemptPrefix
// Every attempt writes under its own prefix; the shard row records the
// winner's, and finalize deletes the others.
export function attemptPrefix(jobId: string, shardId: string, attempt: number): string {
  return `jobs/${jobId}/shards/${shardId}/a${attempt}`;
}

// ------------------------------------------------------------------- encodeKey
export function encodeKey(key: string): string {
  return key.split("/").map(encodeURIComponent).join("/");
}

// --------------------------------------------------------------------- blobUrl
export function blobUrl(origin: string, key: string): string {
  return `${origin}/v1/blobs/${encodeKey(key)}`;
}

function locId_(idx: number): string {
  return `loc-${String(idx).padStart(4, "0")}`;
}

function newToken_(): string {
  return crypto.randomUUID().replaceAll("-", "");
}

// --------------------------------------------------------------------- loadJob
export async function loadJob(db: D1Database, jobId: string): Promise<Job | null> {
  const row = await db.prepare("SELECT * FROM jobs WHERE id = ?").bind(jobId).first<JobRow>();
  return row ? { row, spec: JSON.parse(row.spec_json) as JobSpec } : null;
}

// -------------------------------------------------------------------- getShard
export async function getShard(db: D1Database, jobId: string, shardId: string): Promise<ShardRow | null> {
  return db.prepare("SELECT * FROM shards WHERE job_id = ? AND id = ?").bind(jobId, shardId).first<ShardRow>();
}

// ------------------------------------------------------------------- insertJob
// Inserts the job unless a job of that id exists (a retried POST /v1/jobs
// names the same id). Returns the row that holds the id: ours, or the one
// that was there.
export async function insertJob(db: D1Database, jobId: string, spec: JobSpec, now: number): Promise<JobRow> {
  await db
    .prepare(
      `INSERT INTO jobs (id, scene_id, spec_json, state, created_at, updated_at) VALUES (?, ?, ?, 'planned', ?, ?)
       ON CONFLICT (id) DO NOTHING`,
    )
    .bind(jobId, spec.scene_id, JSON.stringify(spec), now, now)
    .run();
  const job = await loadJob(db, jobId);
  if (!job) throw new Error(`job '${jobId}' vanished after its insert`);
  return job.row;
}

// --------------------------------------------------------------------- planJob
// Inserts the job's loc shards (pending) and its tracks job (blocked) and
// moves the job to running. Idempotent: the Workflow step may run twice.
// Returns the number of loc shards.
export async function planJob(db: D1Database, jobId: string, now: number): Promise<number> {
  const job = await loadJob(db, jobId);
  if (!job) throw new Error(`no job '${jobId}'`);
  const s = job.spec;
  const shards = planShards(s.total_frames, s.start_frame, s.n_frames, s.shard_frames, s.overlap_frames, s.K);
  const insert = db.prepare(
    `INSERT OR IGNORE INTO shards (job_id, id, kind, seq, start_frame, n_frames, keep_start, keep_n,
       max_wall_s, lease_seconds, state, max_attempts, created_at, updated_at)
     VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)`,
  );
  const budget = (kind: ShardKind, n: number) => s.max_wall_s[kind] ?? wallBudgetS(kind, n, s.pose_mode);
  const statements = shards.map((x) =>
    insert.bind(jobId, locId_(x.idx), "loc", x.idx, x.start, x.n, x.keep_start, x.keep_n, budget("loc", x.n),
      s.lease_seconds, "pending", s.max_attempts, now, now),
  );
  statements.push(
    insert.bind(jobId, "tracks", "tracks", 0, s.start_frame, s.n_frames, s.start_frame, s.n_frames,
      budget("tracks", s.n_frames), s.lease_seconds, "blocked", s.max_attempts, now, now),
    db.prepare("UPDATE jobs SET state = 'running', updated_at = ? WHERE id = ? AND state = 'planned'").bind(now, jobId),
  );
  await db.batch(statements);
  return shards.length;
}

// ------------------------------------------------------------- pendingMessages
// One queue message per pending shard of `kind`.
export async function pendingMessages(db: D1Database, jobId: string, kind: ShardKind): Promise<ShardMessage[]> {
  const { results } = await db
    .prepare("SELECT id, attempt FROM shards WHERE job_id = ? AND kind = ? AND state = 'pending' ORDER BY seq")
    .bind(jobId, kind)
    .all<{ id: string; attempt: number }>();
  return results.map((r) => ({ v: 1, job_id: jobId, shard_id: r.id, attempt: r.attempt }));
}

// ----------------------------------------------------------------- seenWorker_
function seenWorker_(db: D1Database, worker: WorkerInfo, now: number): D1PreparedStatement {
  return db
    .prepare(
      `INSERT INTO workers (id, gpu, version, first_seen_at, last_seen_at) VALUES (?, ?, ?, ?, ?)
       ON CONFLICT (id) DO UPDATE SET gpu = excluded.gpu, version = excluded.version,
         last_seen_at = excluded.last_seen_at`,
    )
    .bind(worker.worker_id, worker.gpu ?? null, worker.version ?? null, now, now);
}

// ------------------------------------------------------------------ claimShard
// Leases attempt `attempt + 1` of the shard if it is still pending at
// `attempt` (so a stale queue message cannot claim) and its job is live.
// Returns the leased row, or null.
export async function claimShard(
  db: D1Database,
  jobId: string,
  shardId: string,
  attempt: number,
  worker: WorkerInfo,
  now: number,
): Promise<ShardRow | null> {
  const token = newToken_();
  const results = await db.batch([
    db
      .prepare(
        `UPDATE shards SET state = 'leased', attempt = attempt + 1, worker_id = ?, lease_token = ?,
           lease_started_at = ?, lease_expires_at = ? + lease_seconds, run_started_at = NULL,
           last_heartbeat_at = NULL, progress_json = NULL, output_prefix = NULL, done_json = NULL,
           error = NULL, updated_at = ?
         WHERE job_id = ? AND id = ? AND state = 'pending' AND attempt = ? AND not_before <= ?
           AND EXISTS (SELECT 1 FROM jobs WHERE id = ? AND state IN ('running', 'merging'))`,
      )
      .bind(worker.worker_id, token, now, now, now, jobId, shardId, attempt, now, jobId),
    db
      .prepare(
        `INSERT OR REPLACE INTO attempts (job_id, shard_id, attempt, worker_id, started_at)
         SELECT job_id, id, attempt, worker_id, lease_started_at FROM shards
         WHERE job_id = ? AND id = ? AND lease_token = ? AND state = 'leased'`,
      )
      .bind(jobId, shardId, token),
    seenWorker_(db, worker, now),
    db
      .prepare(
        `UPDATE workers SET current_shard = ? WHERE id = ?
           AND EXISTS (SELECT 1 FROM shards WHERE job_id = ? AND id = ? AND lease_token = ?)`,
      )
      .bind(`${jobId}/${shardId}`, worker.worker_id, jobId, shardId, token),
  ]);
  if (results[0].meta.changes !== 1) return null;
  return getShard(db, jobId, shardId);
}

// ------------------------------------------------------------------- pullShard
// The /v1/pull dispatcher: leases the oldest live job's next pending shard
// (loc shards first; jobs of `sceneAffinity` first). Returns null when there
// is none.
//
// An agent pulls only when it holds no lease, so a lease of this worker that
// has not heartbeated yet is one whose reply was lost: a retried pull gets
// it back instead of leasing a second shard. (A lease that has heartbeated
// belongs to a run that is going on or crashed; it expires as usual.)
export async function pullShard(
  db: D1Database,
  worker: WorkerInfo,
  sceneAffinity: string | null,
  now: number,
): Promise<ShardRow | null> {
  const held = await db
    .prepare(
      `SELECT * FROM shards WHERE worker_id = ? AND state = 'leased' AND last_heartbeat_at IS NULL
         AND lease_expires_at >= ?
       ORDER BY lease_started_at DESC LIMIT 1`,
    )
    .bind(worker.worker_id, now)
    .first<ShardRow>();
  if (held) {
    await seenWorker_(db, worker, now).run();
    return held;
  }
  const { results } = await db
    .prepare(
      `SELECT s.job_id, s.id, s.attempt FROM shards s JOIN jobs j ON j.id = s.job_id
       WHERE s.state = 'pending' AND s.not_before <= ? AND j.state IN ('running', 'merging')
       ORDER BY (j.scene_id = ?) DESC, j.created_at, j.id, s.kind = 'tracks', s.seq LIMIT 3`,
    )
    .bind(now, sceneAffinity ?? "")
    .all<{ job_id: string; id: string; attempt: number }>();
  for (const r of results) {
    const row = await claimShard(db, r.job_id, r.id, r.attempt, worker, now);
    if (row) return row;
  }
  await seenWorker_(db, worker, now).run();
  return null;
}

// ------------------------------------------------------------------ buildLease
// The lease of a leased shard: its command line, its inputs and where its
// outputs go, as URLs of the Worker's blob proxy under `origin`.
export async function buildLease(
  db: D1Database,
  row: ShardRow,
  spec: JobSpec,
  origin: string,
  partBytes: number,
): Promise<Lease> {
  const loc = row.kind === "loc";
  const prefix = attemptPrefix(row.job_id, row.id, row.attempt);
  const toInput = (i: JobSpec["inputs"][number]) => ({
    name: i.name,
    url: blobUrl(origin, i.key),
    kind: i.kind,
    bytes: i.bytes,
    md5: i.md5,
    cache: i.cache,
  });

  let inputs = spec.inputs.filter((i) => (loc ? i.name !== "ground-truth.json" : i.kind === "file")).map(toInput);
  const keep: Lease["keep"] = [];
  let paramsMd5: string | null = null;
  if (!loc) {
    const { results } = await db
      .prepare("SELECT * FROM shards WHERE job_id = ? AND kind = 'loc' AND state = 'done' ORDER BY seq")
      .bind(row.job_id)
      .all<ShardRow>();
    for (const x of results) {
      const done = JSON.parse(x.done_json ?? "{}") as DoneRecord;
      const env = done.outputs.find((o) => o.name === "localization.data");
      if (!env || !x.output_prefix) throw new Error(`${row.job_id}/${x.id}: done without an envelope`);
      const name = `${x.id}.data`;
      inputs = [
        ...inputs,
        {
          name,
          url: blobUrl(origin, `${x.output_prefix}/localization.data`),
          kind: "file",
          bytes: env.bytes,
          md5: env.md5,
          cache: "job",
        },
      ];
      keep.push({ name, start: x.keep_start, n: x.keep_n, computed_start: x.start_frame, computed_n: x.n_frames });
      paramsMd5 = paramsMd5 ?? x.params_md5;
    }
  }

  const outputs = outputNames(row.kind, spec.gt_key !== null).map(({ name, required }) => {
    const url = blobUrl(origin, `${prefix}/${name}`);
    const multipart =
      partBytes > 0 && k_multipart_outputs.has(name)
        ? { init_url: `${url}?uploads`, part_url: url, complete_url: url, abort_url: url, part_bytes: partBytes }
        : null;
    return { name, url, required, multipart };
  });

  return {
    shard_id: row.id,
    job_id: row.job_id,
    attempt: row.attempt,
    lease_token: row.lease_token ?? "",
    lease_expires_at: row.lease_expires_at ?? 0,
    lease_seconds: row.lease_seconds,
    heartbeat_s: spec.heartbeat_s,
    max_wall_s: row.max_wall_s,
    progress_timeout_s: spec.progress_timeout_s,
    kind: row.kind,
    scene_id: spec.scene_id,
    total_frames: spec.total_frames,
    n_cameras: spec.n_cameras,
    output_prefix: prefix,
    cli: {
      start_frame: row.start_frame,
      n_frames: row.n_frames,
      K: spec.K,
      no_stereo: spec.no_stereo,
      no_still: loc && spec.no_stereo,
      pose_mode: spec.pose_mode,
      pose_classifier: loc ? null : spec.pose_classifier,
      still_warmup: loc && !spec.no_stereo ? spec.still_warmup : 0,
      require_aligned: !loc,
      target_fps: spec.target_fps,
      extra_args: extraArgs(spec, row.kind),
    },
    inputs,
    outputs,
    keep,
    params_md5: paramsMd5,
  };
}

// ----------------------------------------------------------------- leasePrefix
// The prefix the holder of `token` may write to, or null if the token is not
// a current lease.
export async function leasePrefix(db: D1Database, token: string): Promise<string | null> {
  const row = await db
    .prepare("SELECT job_id, id, attempt FROM shards WHERE lease_token = ? AND state = 'leased'")
    .bind(token)
    .first<{ job_id: string; id: string; attempt: number }>();
  return row ? attemptPrefix(row.job_id, row.id, row.attempt) : null;
}

// -------------------------------------------------------------------- mayWrite
// Whether the holder of the lease token `token` (the x-mvcloud-lease header;
// empty without one) may write the blob `key`: null if it may, else the
// refusal, 409 without a current lease and 403 outside its attempt's prefix.
export async function mayWrite(
  db: D1Database,
  token: string,
  key: string,
): Promise<{ status: number; error: string } | null> {
  const prefix = token.length > 0 ? await leasePrefix(db, token) : null;
  if (prefix === null) return { status: 409, error: "no current lease (x-mvcloud-lease)" };
  if (!key.startsWith(`${prefix}/`)) return { status: 403, error: `the lease may write under ${prefix}/ only` };
  return null;
}

export type Refusal = { ok: false; state: string; reason: string };

// -------------------------------------------------------------------- refusal_
function refusal_(row: ShardRow | null, token: string): Refusal {
  if (!row) return { ok: false, state: "missing", reason: "no such shard" };
  if (row.state !== "leased") return { ok: false, state: row.state, reason: `the shard is ${row.state}` };
  if (row.lease_token !== token) return { ok: false, state: row.state, reason: "the lease was superseded" };
  return { ok: false, state: row.state, reason: "wall-clock budget exceeded" };
}

// -------------------------------------------------------------- heartbeatShard
// Extends the lease. Refused unless the lease is current and, once the run
// has started (the agent reports run_started_at when it holds the GPU lock;
// the Worker records its own time of that first report), it is within
// 1.5 x max_wall_s of that start. Waiting for the GPU does not count.
export async function heartbeatShard(
  db: D1Database,
  jobId: string,
  shardId: string,
  token: string,
  progress: Record<string, unknown> | null,
  now: number,
): Promise<{ ok: true; lease_expires_at: number } | Refusal> {
  const running = progress !== null && typeof progress.run_started_at === "number" ? 1 : 0;
  const r = await db
    .prepare(
      `UPDATE shards SET lease_expires_at = ? + lease_seconds, last_heartbeat_at = ?, progress_json = ?,
         run_started_at = COALESCE(run_started_at, CASE WHEN ? = 1 THEN ? END), updated_at = ?
       WHERE job_id = ? AND id = ? AND state = 'leased' AND lease_token = ?
         AND (run_started_at IS NULL OR ? <= run_started_at + 1.5 * max_wall_s)
       RETURNING lease_expires_at`,
    )
    .bind(now, now, JSON.stringify(progress ?? {}), running, now, now, jobId, shardId, token, now)
    .first<{ lease_expires_at: number }>();
  if (r) return { ok: true, lease_expires_at: r.lease_expires_at };
  return refusal_(await getShard(db, jobId, shardId), token);
}

export type HeadSize = (key: string) => Promise<number | null>;

export interface Outcome {
  status: number;
  body: Record<string, unknown>;
  wake: boolean; // the shard's kind is finished, or the shard died
}

// --------------------------------------------------------------- kindFinished_
async function kindFinished_(db: D1Database, jobId: string, kind: ShardKind): Promise<boolean> {
  const t = await tally(db, jobId, kind);
  return t.done === t.total || t.dead > 0;
}

// --------------------------------------------------------------- completeShard
// Marks the shard done once every output that done.json lists (and every
// required one) is in the bucket under the attempt's prefix with its size.
// 200 also for a cancelled shard and for a repeated call of the same lease;
// 409 for a lease that is not current; 422 for missing outputs. A loc shard
// whose params_md5 differs from the other loc shards' is dead (fatal). The
// last loc shard promotes the tracks job in the same batch, so the job moves
// on even if its Workflow instance is gone.
export async function completeShard(
  db: D1Database,
  jobId: string,
  shardId: string,
  token: string,
  done: DoneRecord,
  headSize: HeadSize,
  now: number,
): Promise<Outcome> {
  const row = await getShard(db, jobId, shardId);
  if (row?.state === "cancelled") return { status: 200, body: { state: "cancelled", noop: true }, wake: false };
  if (row?.state === "done" && row.lease_token === token)
    return { status: 200, body: { state: "done", noop: true }, wake: false };
  if (!row || row.state !== "leased" || row.lease_token !== token) {
    const r = refusal_(row, token);
    return { status: 409, body: { state: r.state, reason: r.reason }, wake: false };
  }
  if (!Array.isArray(done?.outputs)) return { status: 422, body: { error: "done.outputs is missing" }, wake: false };

  const job = await loadJob(db, jobId);
  const prefix = attemptPrefix(jobId, shardId, row.attempt);
  const listed = new Set(done.outputs.map((o) => o.name));
  for (const o of outputNames(row.kind, job?.spec.gt_key != null)) {
    if (o.required && o.name !== "done.json" && !listed.has(o.name))
      return { status: 422, body: { error: `required output ${o.name} is not in done.json` }, wake: false };
  }
  for (const o of [...done.outputs, { name: "done.json", bytes: null }]) {
    const size = await headSize(`${prefix}/${o.name}`);
    if (size === null || (o.bytes !== null && size !== o.bytes))
      return {
        status: 422,
        body: { error: `output ${o.name} is missing or has ${size} bytes, not ${o.bytes}` },
        wake: false,
      };
  }

  let state = "done";
  let outcome = "done";
  let error: string | null = null;
  const paramsMd5 = done.params_md5 ?? null;
  if (row.kind === "loc") {
    const { results } = await db
      .prepare(
        "SELECT DISTINCT params_md5 FROM shards WHERE job_id = ? AND kind = 'loc' AND state = 'done' AND id <> ?",
      )
      .bind(jobId, shardId)
      .all<{ params_md5: string | null }>();
    const others = results.map((r) => r.params_md5);
    if (others.length > 0 && (others.length > 1 || others[0] !== paramsMd5)) {
      state = "dead";
      outcome = "fatal";
      error = `params_md5 ${paramsMd5} differs from the other loc shards' ${JSON.stringify(others)}`;
    }
  }

  const results = await db.batch([
    db
      .prepare(
        `UPDATE shards SET state = ?, output_prefix = ?, done_json = ?, params_md5 = ?, error = ?, updated_at = ?
         WHERE job_id = ? AND id = ? AND state = 'leased' AND lease_token = ?`,
      )
      .bind(state, prefix, JSON.stringify(done), paramsMd5, error, now, jobId, shardId, token),
    db
      .prepare(
        `UPDATE attempts SET finished_at = ?, outcome = ?, error = ?, wall_s = ?
         WHERE job_id = ? AND shard_id = ? AND attempt = ? AND finished_at IS NULL`,
      )
      .bind(now, outcome, error, typeof done.wall_s === "number" ? done.wall_s : null, jobId, shardId, row.attempt),
    db
      .prepare(
        `UPDATE workers SET current_shard = NULL, last_seen_at = ?,
           shards_done = shards_done + (CASE WHEN ? = 'done' THEN 1 ELSE 0 END) WHERE id = ?`,
      )
      .bind(now, state, row.worker_id),
    ...(row.kind === "loc" ? promoteStatements_(db, jobId, now) : []),
  ]);
  if (results[0].meta.changes !== 1) {
    const r = refusal_(await getShard(db, jobId, shardId), token);
    return { status: 409, body: { state: r.state, reason: r.reason }, wake: false };
  }
  return { status: 200, body: { state, error }, wake: state === "dead" || (await kindFinished_(db, jobId, row.kind)) };
}

// ------------------------------------------------------------------- failShard
// Ends the attempt: a retryable failure with attempts left makes the shard
// failed (pending again after min(600, retry_delay_s * 2^(attempt-1)) s, when
// the Workflow sweeps), anything else dead. A no-op (200) for a lease that
// is not current or a cancelled shard.
export async function failShard(
  db: D1Database,
  jobId: string,
  shardId: string,
  token: string,
  error: string,
  retryable: boolean,
  outcome: string | null,
  now: number,
): Promise<Outcome> {
  const row = await getShard(db, jobId, shardId);
  if (!row || row.state !== "leased" || row.lease_token !== token)
    return { status: 200, body: { state: row?.state ?? "missing", noop: true }, wake: false };
  const job = await loadJob(db, jobId);
  const again = retryable && row.attempt < row.max_attempts;
  const state = again ? "failed" : "dead";
  const delay = Math.min(k_retry_delay_max_s, (job?.spec.retry_delay_s ?? 30) * 2 ** (row.attempt - 1));
  const results = await db.batch([
    db
      .prepare(
        `UPDATE shards SET state = ?, not_before = ?, lease_token = NULL, error = ?, updated_at = ?
         WHERE job_id = ? AND id = ? AND state = 'leased' AND lease_token = ?`,
      )
      .bind(state, now + delay, error, now, jobId, shardId, token),
    db
      .prepare(
        `UPDATE attempts SET finished_at = ?, outcome = ?, error = ?
         WHERE job_id = ? AND shard_id = ? AND attempt = ? AND finished_at IS NULL`,
      )
      .bind(now, outcome ?? (retryable ? "retry" : "fatal"), error, jobId, shardId, row.attempt),
    db.prepare("UPDATE workers SET current_shard = NULL, last_seen_at = ? WHERE id = ?").bind(now, row.worker_id),
  ]);
  if (results[0].meta.changes !== 1) return { status: 200, body: { state: row.state, noop: true }, wake: false };
  return { status: 200, body: { state }, wake: state === "dead" };
}

// ------------------------------------------------------------- stopStatements_
// Ends the running attempts of the job and cancels its unfinished shards, so
// the next heartbeat of a running one is refused (409) and its agent kills
// the run. cancelJob and failJob batch them with the job's own UPDATE.
function stopStatements_(db: D1Database, jobId: string, why: string, now: number): D1PreparedStatement[] {
  return [
    db
      .prepare(
        `UPDATE attempts SET finished_at = ?, outcome = 'cancelled', error = ?
         WHERE finished_at IS NULL AND (job_id, shard_id, attempt) IN
           (SELECT job_id, id, attempt FROM shards WHERE job_id = ? AND state = 'leased')`,
      )
      .bind(now, why, jobId),
    db
      .prepare(
        `UPDATE shards SET state = 'cancelled', lease_token = NULL, error = ?, updated_at = ?
         WHERE job_id = ? AND state IN ('blocked', 'pending', 'leased', 'failed')`,
      )
      .bind(why, now, jobId),
  ];
}

// ------------------------------------------------------------------- cancelJob
// Every unfinished shard becomes cancelled, and the job unless it has ended.
// Returns false if there is no such job.
export async function cancelJob(db: D1Database, jobId: string, now: number): Promise<boolean> {
  if (!(await loadJob(db, jobId))) return false;
  await db.batch([
    ...stopStatements_(db, jobId, "job cancelled", now),
    db
      .prepare(
        `UPDATE jobs SET state = 'cancelled', updated_at = ?
         WHERE id = ? AND state IN ('planned', 'running', 'merging')`,
      )
      .bind(now, jobId),
  ]);
  return true;
}

// --------------------------------------------------------------------- failJob
// The job failed (a dead shard, correlated failures, a lost Workflow): its
// job row says why and its unfinished shards are cancelled so running agents
// stop, in one batch, so nobody sees it cancelled first. Returns false if
// the job had already ended.
export async function failJob(db: D1Database, jobId: string, error: string, now: number): Promise<boolean> {
  const results = await db.batch([
    ...stopStatements_(db, jobId, `job failed: ${error}`, now),
    db
      .prepare(
        `UPDATE jobs SET state = 'failed', error = ?, updated_at = ?
         WHERE id = ? AND state IN ('planned', 'running', 'merging')`,
      )
      .bind(error, now, jobId),
  ]);
  return results[results.length - 1].meta.changes === 1;
}

// ------------------------------------------------------------- failStrandedJob
// A live job whose Workflow instance errored or was terminated has nothing
// left to sweep or finalize it: it fails, so that agents stop leasing its
// shards. `wf` is the instance's status. Returns true if the job failed.
export async function failStrandedJob(
  db: D1Database,
  jobId: string,
  wf: { status?: string; error?: { message?: string } | string },
  now: number,
): Promise<boolean> {
  if (wf.status !== "errored" && wf.status !== "terminated") return false;
  const why = typeof wf.error === "string" ? wf.error : wf.error?.message;
  return failJob(db, jobId, `the Workflow instance is ${wf.status}${why ? `: ${why}` : ""}`, now);
}

// -------------------------------------------------------------------- sweepJob
// Expires leases whose holder stopped heartbeating (pending again with
// attempt + 1 to come, or dead when out of attempts) and makes failed shards
// whose delay has passed pending. Returns a queue message per shard that
// became pending.
export async function sweepJob(db: D1Database, jobId: string, now: number): Promise<ShardMessage[]> {
  const messages: ShardMessage[] = [];
  const expired = await db
    .prepare("SELECT * FROM shards WHERE job_id = ? AND state = 'leased' AND lease_expires_at < ?")
    .bind(jobId, now)
    .all<ShardRow>();
  for (const r of expired.results) {
    const state = r.attempt < r.max_attempts ? "pending" : "dead";
    const results = await db.batch([
      db
        .prepare(
          `UPDATE shards SET state = ?, lease_token = NULL, worker_id = NULL, error = 'lease expired', updated_at = ?
           WHERE job_id = ? AND id = ? AND state = 'leased' AND lease_token = ?`,
        )
        .bind(state, now, jobId, r.id, r.lease_token),
      db
        .prepare(
          `UPDATE attempts SET finished_at = ?, outcome = 'expired', error = 'lease expired'
           WHERE job_id = ? AND shard_id = ? AND attempt = ? AND finished_at IS NULL`,
        )
        .bind(now, jobId, r.id, r.attempt),
      db.prepare("UPDATE workers SET current_shard = NULL WHERE id = ? AND current_shard = ?")
        .bind(r.worker_id, `${jobId}/${r.id}`),
    ]);
    if (results[0].meta.changes === 1 && state === "pending")
      messages.push({ v: 1, job_id: jobId, shard_id: r.id, attempt: r.attempt });
  }

  const ready = await db
    .prepare("SELECT id, attempt FROM shards WHERE job_id = ? AND state = 'failed' AND not_before <= ?")
    .bind(jobId, now)
    .all<{ id: string; attempt: number }>();
  for (const r of ready.results) {
    const res = await db
      .prepare(
        `UPDATE shards SET state = 'pending', updated_at = ?
         WHERE job_id = ? AND id = ? AND state = 'failed' AND attempt = ?`,
      )
      .bind(now, jobId, r.id, r.attempt)
      .run();
    if (res.meta.changes === 1) messages.push({ v: 1, job_id: jobId, shard_id: r.id, attempt: r.attempt });
  }
  return messages;
}

// ---------------------------------------------------------- promoteStatements_
// The tracks job becomes pending, and the job merging, once every loc shard
// is done with the same params_md5 (two loc shards that complete at once can
// both pass completeShard's check; sweepAndTally fails such a job).
function promoteStatements_(db: D1Database, jobId: string, now: number): D1PreparedStatement[] {
  const ready = `NOT EXISTS (SELECT 1 FROM shards WHERE job_id = ? AND kind = 'loc' AND state <> 'done')
    AND (SELECT COUNT(DISTINCT COALESCE(params_md5, '')) FROM shards WHERE job_id = ? AND kind = 'loc') = 1`;
  return [
    db
      .prepare(
        `UPDATE shards SET state = 'pending', updated_at = ?
         WHERE job_id = ? AND kind = 'tracks' AND state = 'blocked' AND ${ready}`,
      )
      .bind(now, jobId, jobId, jobId),
    db
      .prepare(`UPDATE jobs SET state = 'merging', updated_at = ? WHERE id = ? AND state = 'running' AND ${ready}`)
      .bind(now, jobId, jobId, jobId),
  ];
}

// --------------------------------------------------------------- promoteTracks
// Promotes the tracks job if it is ready (completeShard of the last loc shard
// has normally done it). Returns its queue message if it is pending.
export async function promoteTracks(db: D1Database, jobId: string, now: number): Promise<ShardMessage[]> {
  await db.batch(promoteStatements_(db, jobId, now));
  return pendingMessages(db, jobId, "tracks");
}

export interface Tally {
  total: number;
  done: number;
  leased: number;
  failed: number;
  dead: number;
  cancelled: number;
  dead_error: string | null;
  params_md5s: number; // distinct params_md5 of the done shards
}

// ----------------------------------------------------------------------- tally
export async function tally(db: D1Database, jobId: string, kind: ShardKind): Promise<Tally> {
  const { results } = await db
    .prepare(
      `SELECT state, COUNT(*) AS n, MAX(CASE WHEN state = 'dead' THEN id || ': ' || COALESCE(error, '') END) AS err,
         COUNT(DISTINCT COALESCE(params_md5, '')) AS md5s
       FROM shards WHERE job_id = ? AND kind = ? GROUP BY state`,
    )
    .bind(jobId, kind)
    .all<{ state: string; n: number; err: string | null; md5s: number }>();
  const n = (state: string) => results.find((r) => r.state === state)?.n ?? 0;
  return {
    total: results.reduce((sum, r) => sum + r.n, 0),
    done: n("done"),
    leased: n("leased"),
    failed: n("failed"),
    dead: n("dead"),
    cancelled: n("cancelled"),
    dead_error: results.find((r) => r.state === "dead")?.err ?? null,
    params_md5s: results.find((r) => r.state === "done")?.md5s ?? 0,
  };
}

// ----------------------------------------------------------- correlatedFailure
// When the job's last three finished attempts are retryable failures of
// three different shards with the same class (outcome and the error up to
// its first ':'), the cause is not the shards: the job is paused as failed.
// Returns that class, or null.
export async function correlatedFailure(db: D1Database, jobId: string): Promise<string | null> {
  const { results } = await db
    .prepare(
      `SELECT shard_id, outcome, error FROM attempts WHERE job_id = ? AND finished_at IS NOT NULL
       ORDER BY finished_at DESC, rowid DESC LIMIT 3`,
    )
    .bind(jobId)
    .all<{ shard_id: string; outcome: string; error: string | null }>();
  if (results.length < 3 || !results.every((a) => a.outcome === "retry" || a.outcome === "timeout")) return null;
  const classes = new Set(results.map((a) => `${a.outcome}: ${(a.error ?? "").split(":")[0].trim()}`));
  const shards = new Set(results.map((a) => a.shard_id));
  return classes.size === 1 && shards.size === 3 ? [...classes][0] : null;
}

// ------------------------------------------------------------------- finishJob
export async function finishJob(db: D1Database, jobId: string, summary: unknown, now: number): Promise<void> {
  await db
    .prepare(
      `UPDATE jobs SET state = 'done', summary_json = ?, updated_at = ?
       WHERE id = ? AND state IN ('running', 'merging')`,
    )
    .bind(JSON.stringify(summary), now, jobId)
    .run();
}

// -------------------------------------------------------------------- shardsOf
export async function shardsOf(db: D1Database, jobId: string): Promise<ShardRow[]> {
  const { results } = await db
    .prepare("SELECT * FROM shards WHERE job_id = ? ORDER BY kind <> 'loc', seq")
    .bind(jobId)
    .all<ShardRow>();
  return results;
}

// ------------------------------------------------------------------- jobStatus
// The job with its shards, each with its attempts: what GET /v1/jobs/{id},
// the status page and the agent's `status` show.
export async function jobStatus(db: D1Database, jobId: string): Promise<Record<string, unknown> | null> {
  const job = await loadJob(db, jobId);
  if (!job) return null;
  const { results: attempts } = await db
    .prepare("SELECT * FROM attempts WHERE job_id = ? ORDER BY shard_id, attempt")
    .bind(jobId)
    .all<AttemptRow>();
  const { spec_json: _, summary_json, ...row } = job.row;
  return {
    ...row,
    spec: job.spec,
    summary: summary_json ? JSON.parse(summary_json) : null,
    shards: (await shardsOf(db, jobId)).map(({ done_json, progress_json, lease_token: __, ...r }) => ({
      ...r,
      done: done_json ? JSON.parse(done_json) : null,
      progress: progress_json ? JSON.parse(progress_json) : null,
      attempts: attempts
        .filter((a) => a.shard_id === r.id)
        .map(({ job_id: ___, shard_id: ____, ...a }) => a),
    })),
  };
}

// ------------------------------------------------------------------ liveJobIds
// The jobs that have not ended (planned, running or merging).
export async function liveJobIds(db: D1Database): Promise<string[]> {
  const { results } = await db
    .prepare("SELECT id FROM jobs WHERE state IN ('planned', 'running', 'merging') ORDER BY created_at")
    .all<{ id: string }>();
  return results.map((r) => r.id);
}

// -------------------------------------------------------------------- listJobs
export async function listJobs(db: D1Database, limit = 100): Promise<Record<string, unknown>[]> {
  const { results } = await db
    .prepare(
      `SELECT j.id, j.scene_id, j.state, j.created_at, j.updated_at, j.error,
         (SELECT json_group_object(state, n) FROM
            (SELECT state, COUNT(*) AS n FROM shards WHERE job_id = j.id GROUP BY state)) AS shard_states
       FROM jobs j ORDER BY j.created_at DESC, j.id LIMIT ?`,
    )
    .bind(limit)
    .all<Record<string, unknown>>();
  return results.map((r) => ({ ...r, shard_states: JSON.parse((r.shard_states as string) ?? "{}") }));
}

// ----------------------------------------------------------------- listWorkers
export async function listWorkers(db: D1Database): Promise<Record<string, unknown>[]> {
  const { results } = await db.prepare("SELECT * FROM workers ORDER BY last_seen_at DESC").all();
  return results;
}
