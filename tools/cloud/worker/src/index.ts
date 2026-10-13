// mvcloud: the control plane of cloud-distributed multiview (issue #14).
//
// The API is the agent's only contract (tools/cloud/agent/mvcloud/
// httpcontrol.py); D1 holds the state, R2 the blobs, one Workflow instance
// per job drives it, and the Queue is dispatch only. Every route needs
// `Authorization: Bearer <token>`: the admin token for job writes, the
// status page and any blob write, the worker token (or the admin token) for
// the rest. A worker may write blobs only under its current lease's prefix,
// named by the x-mvcloud-lease header.
//
//   POST /v1/jobs                   job spec -> {job_id, workflow_id, shards}       (admin)
//   GET  /v1/jobs, /v1/jobs/{id}    the jobs; one job with shards, attempts, Workflow status
//   POST /v1/jobs/{id}/cancel                                                         (admin)
//   POST /v1/pull                   {worker_id, gpu, version, scene_affinity?} -> lease | 204
//   POST /v1/claim                  {job_id, shard_id, attempt, worker_id, ...} -> lease | 409
//   POST /v1/heartbeat              {job_id, shard_id, lease_token, progress} -> {lease_expires_at} | 409
//   POST /v1/complete               {job_id, shard_id, lease_token, done} -> 200 | 409 | 422
//   POST /v1/fail                   {job_id, shard_id, lease_token, error, retryable, outcome?} -> 200
//   GET  /v1/workers
//   /v1/blobs/<key>, GET /v1/blobs?prefix=   the blob proxy (blobs.ts)
//   GET  /v1/health                 no token
//   GET  /[?job=<id>][&token=<admin>]       the status page

import { headObject, json, listBlobs, readBlob, writeBlob } from "./blobs";
import {
  buildLease,
  cancelJob,
  claimShard,
  completeShard,
  failShard,
  heartbeatShard,
  insertJob,
  jobStatus,
  leasePrefix,
  listJobs,
  listWorkers,
  loadJob,
  pullShard,
  type Outcome,
} from "./db";
import { wakeWorkflow } from "./job-workflow";
import { PlanError, planShards } from "./plan";
import { normalizeSpec } from "./spec";
import { renderJob, renderJobs } from "./status";
import type { Env, JobInput, JobSpec, ShardMessage, ShardRow, WorkerInfo } from "./types";

export { JobWorkflow } from "./job-workflow";

type Role = "admin" | "worker" | null;

function now_(): number {
  return Date.now() / 1000;
}

// ---------------------------------------------------------------- sameToken_
function sameToken_(a: string, b: string | undefined): boolean {
  if (!b) return false; // an unset token admits nobody
  const x = new TextEncoder().encode(a);
  const y = new TextEncoder().encode(b);
  return x.byteLength === y.byteLength && crypto.subtle.timingSafeEqual(x, y);
}

// ---------------------------------------------------------------------- role_
function role_(request: Request, env: Env): Role {
  const auth = request.headers.get("authorization") ?? "";
  const token = auth.startsWith("Bearer ") ? auth.slice(7) : new URL(request.url).searchParams.get("token") ?? "";
  if (token.length === 0) return null;
  if (sameToken_(token, env.MVCLOUD_ADMIN_TOKEN)) return "admin";
  if (sameToken_(token, env.MVCLOUD_WORKER_TOKEN)) return "worker";
  return null;
}

// ---------------------------------------------------------------- readJson_
async function readJson_(request: Request): Promise<Record<string, unknown>> {
  try {
    const body = await request.json();
    if (typeof body === "object" && body !== null && !Array.isArray(body)) return body as Record<string, unknown>;
  } catch {
    // fall through
  }
  throw new PlanError("the request body must be a json object");
}

// ------------------------------------------------------------------- field_
function field_(body: Record<string, unknown>, key: string): string {
  const x = body[key];
  if (typeof x !== "string" || x.length === 0) throw new PlanError(`'${key}' must be a non-empty string`);
  return x;
}

// ------------------------------------------------------------------ worker_
function worker_(body: Record<string, unknown>): WorkerInfo {
  return {
    worker_id: field_(body, "worker_id"),
    gpu: typeof body.gpu === "string" ? body.gpu : undefined,
    version: typeof body.version === "string" ? body.version : undefined,
  };
}

// -------------------------------------------------------------- partBytes_
function partBytes_(env: Env): number {
  const n = Number(env.MULTIPART_PART_BYTES ?? "0");
  return Number.isFinite(n) && n > 0 ? Math.floor(n) : 0;
}

// ------------------------------------------------------------------ lease_
async function lease_(env: Env, row: ShardRow, origin: string): Promise<Response> {
  const job = await loadJob(env.DB, row.job_id);
  if (!job) return json(500, { error: `no job '${row.job_id}'` });
  return json(200, await buildLease(env.DB, row, job.spec, origin, partBytes_(env)));
}

// -------------------------------------------------------------- outcome_
function outcome_(env: Env, ctx: ExecutionContext, jobId: string, kind: string, o: Outcome): Response {
  if (o.wake) ctx.waitUntil(wakeWorkflow(env, jobId, kind));
  return json(o.status, o.body);
}

// --------------------------------------------------------------- newJobId_
function newJobId_(sceneId: string): string {
  const t = new Date().toISOString().replace(/[-:]/g, "").slice(0, 15);
  return `${sceneId}-${t}-${crypto.randomUUID().slice(0, 6)}`;
}

// ------------------------------------------------------------ resolveInputs_
// Every input of the job's shards, with its size and md5 from the bucket.
// Throws PlanError naming the keys that are missing.
async function resolveInputs_(env: Env, spec: Omit<JobSpec, "inputs">): Promise<JobInput[]> {
  const wanted: Omit<JobInput, "bytes" | "md5">[] = [
    { name: "manifest.json", key: spec.manifest_key, kind: "file", cache: "scene" },
    ...spec.videos.map((v) => ({ name: v.name, key: v.key, kind: "file" as const, cache: "scene" as const })),
    { name: "params-in.json", key: spec.params_key, kind: "file", cache: "scene" },
  ];
  if (spec.poses_key) wanted.push({ name: "poses.tar", key: spec.poses_key, kind: "tar", cache: "scene" });
  if (spec.gt_key) wanted.push({ name: "ground-truth.json", key: spec.gt_key, kind: "file", cache: "scene" });
  const heads = await Promise.all(wanted.map((w) => headObject(env.BUCKET, w.key)));
  const missing = wanted.filter((_, i) => heads[i] === null).map((w) => w.key);
  if (missing.length > 0) throw new PlanError(`missing objects: ${missing.join(", ")}`);
  return wanted.map((w, i) => ({ ...w, bytes: heads[i]!.bytes, md5: heads[i]!.md5 }));
}

// ----------------------------------------------------------------- createJob_
async function createJob_(request: Request, env: Env): Promise<Response> {
  const specIn = normalizeSpec(await readJson_(request));
  const spec: JobSpec = { ...specIn, inputs: await resolveInputs_(env, specIn) };
  const jobId = newJobId_(spec.scene_id);
  const now = now_();
  await insertJob(env.DB, jobId, spec, now);
  try {
    const instance = await env.JOB_WORKFLOW.create({ id: jobId, params: { jobId } });
    await env.DB.prepare("UPDATE jobs SET workflow_id = ? WHERE id = ?").bind(instance.id, jobId).run();
  } catch (e) {
    await env.DB.prepare("UPDATE jobs SET state = 'failed', error = ?, updated_at = ? WHERE id = ?")
      .bind(`could not start the Workflow: ${String(e)}`, now_(), jobId)
      .run();
    return json(500, { error: `could not start the Workflow: ${String(e)}`, job_id: jobId });
  }
  const shards = planShards(spec.total_frames, spec.start_frame, spec.n_frames, spec.shard_frames,
    spec.overlap_frames, spec.K);
  return json(201, { job_id: jobId, workflow_id: jobId, shards });
}

// ------------------------------------------------------------ workflowStatus_
async function workflowStatus_(env: Env, jobId: string): Promise<unknown> {
  try {
    return await (await env.JOB_WORKFLOW.get(jobId)).status();
  } catch (e) {
    return { status: "unknown", error: String(e) };
  }
}

// --------------------------------------------------------------- blobRoute_
async function blobRoute_(request: Request, env: Env, role: Role, key: string): Promise<Response> {
  if (request.method === "GET" || request.method === "HEAD") return readBlob(request, env.BUCKET, key);
  if (role !== "admin") {
    const token = request.headers.get("x-mvcloud-lease") ?? "";
    const prefix = token ? await leasePrefix(env.DB, token) : null;
    if (prefix === null) return json(409, { error: "no current lease (x-mvcloud-lease)" });
    if (!key.startsWith(`${prefix}/`)) return json(403, { error: `the lease may write under ${prefix}/ only` });
  }
  return writeBlob(request, env.BUCKET, key);
}

// ---------------------------------------------------------------------- route
async function route(request: Request, env: Env, ctx: ExecutionContext): Promise<Response> {
  const url = new URL(request.url);
  const path = url.pathname;
  const method = request.method;
  if (path === "/v1/health") return json(200, { ok: true });

  const role = role_(request, env);
  if (role === null) return json(401, { error: "a valid bearer token is required" });

  if (path === "/" && method === "GET") {
    if (role !== "admin") return json(403, { error: "the status page needs the admin token" });
    const jobId = url.searchParams.get("job");
    const headers = { "content-type": "text/html; charset=utf-8" };
    if (jobId === null)
      return new Response(renderJobs(await listJobs(env.DB), url.searchParams.get("token") ?? ""), { headers });
    const job = await jobStatus(env.DB, jobId);
    if (!job) return json(404, { error: `no job '${jobId}'` });
    const wf = (await workflowStatus_(env, jobId)) as { status?: string };
    return new Response(renderJob(job, wf.status ?? "unknown"), { headers });
  }

  if (path === "/v1/blobs" && method === "GET") return listBlobs(env.BUCKET, url.searchParams.get("prefix") ?? "");
  if (path.startsWith("/v1/blobs/")) {
    const key = path.slice("/v1/blobs/".length).split("/").map(decodeURIComponent).join("/");
    return blobRoute_(request, env, role, key);
  }

  if (path === "/v1/jobs" && method === "POST") {
    if (role !== "admin") return json(403, { error: "submitting a job needs the admin token" });
    return createJob_(request, env);
  }
  if (path === "/v1/jobs" && method === "GET") return json(200, { jobs: await listJobs(env.DB) });
  const m = /^\/v1\/jobs\/([^/]+)(\/cancel)?$/.exec(path);
  if (m) {
    const jobId = decodeURIComponent(m[1]);
    if (m[2] && method === "POST") {
      if (role !== "admin") return json(403, { error: "cancelling a job needs the admin token" });
      if (!(await cancelJob(env.DB, jobId, now_()))) return json(404, { error: `no job '${jobId}'` });
      ctx.waitUntil(wakeWorkflow(env, jobId, "cancel"));
      return json(200, { job_id: jobId, state: "cancelled" });
    }
    if (!m[2] && method === "GET") {
      const job = await jobStatus(env.DB, jobId);
      if (!job) return json(404, { error: `no job '${jobId}'` });
      return json(200, { ...job, workflow: await workflowStatus_(env, jobId) });
    }
  }
  if (path === "/v1/workers" && method === "GET") return json(200, { workers: await listWorkers(env.DB) });

  if (method !== "POST") return json(404, { error: `no route ${method} ${path}` });
  const body = await readJson_(request);
  if (path === "/v1/pull") {
    const affinity = typeof body.scene_affinity === "string" ? body.scene_affinity : null;
    const row = await pullShard(env.DB, worker_(body), affinity, now_());
    return row ? lease_(env, row, url.origin) : new Response(null, { status: 204 });
  }
  const jobId = field_(body, "job_id");
  const shardId = field_(body, "shard_id");
  if (path === "/v1/claim") {
    const attempt = Number(body.attempt);
    const row = await claimShard(env.DB, jobId, shardId, attempt, worker_(body), now_());
    if (row) return lease_(env, row, url.origin);
    const cur = await env.DB.prepare("SELECT state, attempt FROM shards WHERE job_id = ? AND id = ?")
      .bind(jobId, shardId)
      .first<{ state: string; attempt: number }>();
    return json(409, { state: cur?.state ?? "missing", attempt: cur?.attempt ?? null, reason: "not claimable" });
  }
  const token = field_(body, "lease_token");
  if (path === "/v1/heartbeat") {
    const progress = typeof body.progress === "object" ? (body.progress as Record<string, unknown>) : null;
    const r = await heartbeatShard(env.DB, jobId, shardId, token, progress, now_());
    return r.ok ? json(200, { lease_expires_at: r.lease_expires_at }) : json(409, { state: r.state, reason: r.reason });
  }
  const kind = shardId === "tracks" ? "tracks" : "loc";
  if (path === "/v1/complete") {
    const headSize = async (key: string) => (await headObject(env.BUCKET, key))?.bytes ?? null;
    const done = body.done as Parameters<typeof completeShard>[4];
    return outcome_(env, ctx, jobId, kind, await completeShard(env.DB, jobId, shardId, token, done, headSize, now_()));
  }
  if (path === "/v1/fail") {
    const outcome = typeof body.outcome === "string" ? body.outcome : null;
    const error = typeof body.error === "string" ? body.error : "";
    return outcome_(env, ctx, jobId, kind,
      await failShard(env.DB, jobId, shardId, token, error, body.retryable === true, outcome, now_()));
  }
  return json(404, { error: `no route ${method} ${path}` });
}

export default {
  async fetch(request, env, ctx): Promise<Response> {
    try {
      return await route(request, env, ctx);
    } catch (e) {
      if (e instanceof PlanError) return json(422, { error: e.message });
      console.error(e);
      return json(500, { error: String(e) });
    }
  },

  // The push consumer of the local environment: messages are dispatch hints
  // and /v1/pull reads D1, so they are acknowledged and logged. Production
  // uses an HTTP pull consumer (milestone M8) and has no push consumer.
  async queue(batch, _env): Promise<void> {
    for (const m of batch.messages) {
      console.log(`queue: ${JSON.stringify(m.body)}`);
      m.ack();
    }
  },
} satisfies ExportedHandler<Env, ShardMessage>;
