// The status page (GET /): the jobs with a bar of their shards' states, and
// with ?job=<id> one job's shards, attempts, progress and score. Rendered on
// the server from D1; no script.

const k_state_colours: Record<string, string> = {
  done: "#2e7d32",
  leased: "#1565c0",
  pending: "#9e9e9e",
  blocked: "#cfd8dc",
  failed: "#ef6c00",
  dead: "#c62828",
  cancelled: "#6d4c41",
};

// ---------------------------------------------------------------------- esc_
function esc_(x: unknown): string {
  return String(x ?? "")
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;");
}

// --------------------------------------------------------------------- time_
function time_(t: unknown): string {
  return typeof t === "number" ? new Date(t * 1000).toISOString().replace("T", " ").slice(0, 19) : "";
}

// ---------------------------------------------------------------------- bar_
function bar_(states: Record<string, number>): string {
  const total = Object.values(states).reduce((a, b) => a + b, 0) || 1;
  const cells = Object.entries(states)
    .map(
      ([s, n]) =>
        `<span title="${esc_(s)}: ${n}" style="display:inline-block;height:12px;width:${(160 * n) / total}px;` +
        `background:${k_state_colours[s] ?? "#000"}"></span>`,
    )
    .join("");
  const label = Object.entries(states)
    .map(([s, n]) => `${n} ${esc_(s)}`)
    .join(", ");
  return `${cells} <small>${label}</small>`;
}

// ---------------------------------------------------------------------- page_
function page_(title: string, body: string): string {
  return `<!doctype html><html><head><meta charset="utf-8"><title>${esc_(title)}</title>
<style>body{font:14px system-ui,sans-serif;margin:1.5em}table{border-collapse:collapse}
td,th{padding:3px 8px;border-bottom:1px solid #ddd;text-align:left;vertical-align:top}
code{font-size:12px}</style></head><body><h1>${esc_(title)}</h1>${body}</body></html>`;
}

// ---------------------------------------------------------------- renderJobs
export function renderJobs(jobs: Record<string, unknown>[], token: string): string {
  const q = token ? `&token=${encodeURIComponent(token)}` : "";
  const rows = jobs
    .map(
      (j) =>
        `<tr><td><a href="/?job=${encodeURIComponent(String(j.id))}${q}"><code>${esc_(j.id)}</code></a></td>` +
        `<td>${esc_(j.scene_id)}</td><td>${esc_(j.state)}</td><td>${time_(j.created_at)}</td>` +
        `<td>${bar_(j.shard_states as Record<string, number>)}</td><td>${esc_(j.error)}</td></tr>`,
    )
    .join("");
  return page_(
    "mvcloud jobs",
    `<table><tr><th>job</th><th>scene</th><th>state</th><th>created (UTC)</th><th>shards</th><th>error</th></tr>${rows}</table>`,
  );
}

// ----------------------------------------------------------------- renderJob
export function renderJob(job: Record<string, unknown>, workflow: string): string {
  const shards = job.shards as Record<string, unknown>[];
  const rows = shards
    .map((s) => {
      const attempts = (s.attempts as Record<string, unknown>[])
        .map((a) => `a${a.attempt}: ${esc_(a.outcome ?? "running")} ${a.wall_s ? `${Number(a.wall_s).toFixed(1)} s` : ""}` +
          `${a.error ? ` <small>(${esc_(a.error)})</small>` : ""}`)
        .join("<br>");
      const progress = s.progress as Record<string, unknown> | null;
      return (
        `<tr><td>${esc_(s.id)}</td><td>[${s.start_frame}, ${Number(s.start_frame) + Number(s.n_frames)})</td>` +
        `<td style="color:${k_state_colours[String(s.state)] ?? "#000"}">${esc_(s.state)}</td>` +
        `<td>${esc_(s.worker_id)}</td><td>${attempts}</td><td><code>${esc_(progress?.line ?? "")}</code></td></tr>`
      );
    })
    .join("");
  const summary = job.summary as Record<string, unknown> | null;
  const score = summary?.score ? `<h2>Score</h2><pre>${esc_(JSON.stringify(summary.score, null, 1))}</pre>` : "";
  return page_(
    `job ${String(job.id)}`,
    `<p>scene ${esc_(job.scene_id)}; state <b>${esc_(job.state)}</b>; Workflow ${esc_(workflow)}` +
      `${job.error ? `; error ${esc_(job.error)}` : ""}</p>` +
      `<table><tr><th>shard</th><th>frames</th><th>state</th><th>worker</th><th>attempts</th><th>progress</th></tr>` +
      `${rows}</table>${score}`,
  );
}
