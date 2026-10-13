#!/usr/bin/env python3
"""The scenarios of e2e_wrangler.sh that need no GPU: an agent with the fake
multiview-cli (fake_multiview_cli.py, MVCLOUD_FAKE_MODE=loc=sleep) against
the Worker at URL, in this process.

  wrangler_scenarios.py cancel  URL OUT_DIR
      A synthetic job's loc shard runs (sleeps); POST /v1/jobs/{id}/cancel.
      The agent's next heartbeat is refused, it kills the child's session
      within heartbeat_s / 3 + kill_grace (+1 s), and fail is a no-op: the
      job is cancelled, no shard is leased, the Workflow is complete.
  wrangler_scenarios.py timeout URL OUT_DIR
      max_wall_s.loc = 4, max_attempts 2: both attempts end as `timeout:
      wall`, the shard is dead, the job failed, the Workflow errored.

Tokens: $MVCLOUD_ADMIN_TOKEN and $MVCLOUD_WORKER_TOKEN. Exit status 0 when
every assertion holds.
"""

import logging
import os
import sys
import threading
import time

import support
from e2e_check import load_job, report, wait_for
from mvcloud import agent
from mvcloud.httpcontrol import HttpControl

k_heartbeat_s = 3
k_kill_grace_s = 2.0


# ----------------------------------------------------------------------- alive_
def alive_(pids_file):
    """The pids the fake recorded that are still running."""
    if not os.path.isfile(pids_file):
        return []
    alive = []
    for pid in {int(x) for x in open(pids_file).read().split()}:
        try:
            os.kill(pid, 0)
            alive.append(pid)
        except ProcessLookupError:
            pass
    return alive


# --------------------------------------------------------------------- Scenario
class Scenario:
    """A synthetic job on the Worker and an agent running it in a thread."""

    def __init__(self, base, out, name, **spec):
        self.base = base
        self.d = os.path.join(out, name)
        os.makedirs(self.d, exist_ok=True)
        self.pids = os.path.join(self.d, "pids.txt")
        os.environ["MVCLOUD_FAKE_MODE"] = "loc=sleep"
        os.environ["MVCLOUD_FAKE_PIDS"] = self.pids
        paths = support.make_scene(os.path.join(self.d, "scene"))
        job = support.make_spec(paths, scene_id=f"synthetic-{name}", n_frames=20, shard_frames=20,
                                heartbeat_s=k_heartbeat_s, wait_seconds=2, **spec)
        self.admin = HttpControl(base, os.environ["MVCLOUD_ADMIN_TOKEN"])
        self.job_id = self.admin.submit(job)
        print(f"   {name}: job {self.job_id}")
        self.agent = agent.Agent(HttpControl(base, os.environ["MVCLOUD_WORKER_TOKEN"]),
                                 cli_bin=support.fake_cli(os.path.join(self.d, "bin")),
                                 cache_dir=os.path.join(self.d, "cache"),
                                 work_dir=os.path.join(self.d, "work"),
                                 gpu_lock=os.path.join(self.d, "gpu.lock"), mem="",
                                 kill_grace_s=k_kill_grace_s, poll_s=1.0,
                                 worker_id=f"fake-{name}")
        self.thread = threading.Thread(target=self.agent.run, name=f"agent-{name}")
        self.thread.start()

    def job(self):
        return load_job(self.base, self.job_id)

    def shard(self, shard_id):
        return next(r for r in self.job()["shards"] if r["id"] == shard_id)

    def stop(self):
        self.agent.stop("scenario over")
        self.thread.join()


# ----------------------------------------------------------------------- cancel
def cancel(base, out):
    s = Scenario(base, out, "cancel")
    try:
        running = wait_for("loc-0000 running", 60, lambda: (s.shard("loc-0000").get("progress") or {})
                           .get("run_started_at") and alive_(s.pids))
        ok = report(bool(running), "loc-0000 is running (the fake sleeps)")
        t0 = time.time()
        s.admin.cancel(s.job_id)
        gone = wait_for("the child's session gone", 30, lambda: not alive_(s.pids))
        dt = time.time() - t0
        limit = k_heartbeat_s / 3 + k_kill_grace_s + 1.0
        ok &= report(bool(gone) and dt <= limit, f"the child was killed {dt:.1f} s after cancel (<= {limit:.1f} s)")
        wf = wait_for("Workflow complete", 30, lambda: (s.job().get("workflow") or {}).get("status") == "complete")
        job = s.job()
        ok &= report(job["state"] == "cancelled", f"job {job['state']}")
        ok &= report(all(r["state"] != "leased" for r in job["shards"]),
                     f"no leased shard: {[(r['id'], r['state']) for r in job['shards']]}")
        ok &= report(bool(wf), f"Workflow {job['workflow'].get('status')}, output {job['workflow'].get('output')}")
        attempts = s.shard("loc-0000")["attempts"]
        ok &= report([a["outcome"] for a in attempts] == ["cancelled"], f"loc-0000 attempts {attempts}")
        return ok
    finally:
        s.stop()


# ---------------------------------------------------------------------- timeout
def timeout(base, out):
    s = Scenario(base, out, "timeout", max_wall_s={"loc": 4, "tracks": None}, max_attempts=2,
                 retry_delay_s=0)
    try:
        state = wait_for("job ended", 120, lambda: (lambda st: st if st in ("done", "failed", "cancelled")
                                                     else None)(s.job()["state"]))
        ok = report(state == "failed", f"job {state}")
        r = s.shard("loc-0000")
        outcomes = [a["outcome"] for a in r["attempts"]]
        ok &= report(r["state"] == "dead" and outcomes == ["timeout", "timeout"],
                     f"loc-0000 {r['state']}, attempts {outcomes}")
        ok &= report(all((a["error"] or "").startswith("timeout: wall") for a in r["attempts"]),
                     f"errors {[a['error'] for a in r['attempts']]}")
        wf = wait_for("Workflow errored", 30, lambda: (s.job().get("workflow") or {}).get("status") == "errored")
        ok &= report(bool(wf), f"Workflow {s.job()['workflow'].get('status')}: {s.job()['workflow'].get('error')}")
        ok &= report(not alive_(s.pids), "no fake process survives")
        return ok
    finally:
        s.stop()


if __name__ == "__main__":
    os.makedirs(sys.argv[3], exist_ok=True)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s mvcloud %(levelname)s %(message)s",
                        filename=os.path.join(sys.argv[3], f"{sys.argv[1]}-agent.log"))
    scenario = {"cancel": cancel, "timeout": timeout}[sys.argv[1]]
    sys.exit(0 if scenario(sys.argv[2], sys.argv[3]) else 1)
