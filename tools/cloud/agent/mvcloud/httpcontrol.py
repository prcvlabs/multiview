"""The Worker API backend (tools/cloud/worker): the Control protocol over
HTTP, with the /v1/pull dispatcher. `submit` uploads a job's local inputs
(manifest, videos, params, the replayed poses as a tar, ground truth) to
the bucket through the Worker's blob proxy, content-addressed under
scenes/<scene_id>/<md5>/<name>, and posts the job with their keys.

Inputs and outputs whose URLs are the Worker's own get the bearer token,
and outputs the lease token (x-mvcloud-lease), when a lease arrives; any
other URL (a presigned one) is used as it is.
"""

import json
import logging
import os
import re
import shutil
import tarfile
import tempfile
import time
import urllib.error
import urllib.request
import uuid

from . import storage
from .control import Conflict, HeartbeatResult

log = logging.getLogger("mvcloud")

k_timeout_s = 60
k_attempts = 5
k_backoff_s = 1.0
k_single_put_max = 64 << 20  # larger scene files go up in parts
k_part_bytes = 64 << 20


class HttpError(Exception):
    """The Worker refused a request or could not be reached."""


# -------------------------------------------------------------------- tar_poses
def tar_poses(src, dest):
    """The pose-2d_*.json files of the directory `src` as an uncompressed tar
    `dest` that depends on their names and contents only (sorted, no
    times, no owners), so the same poses always have the same md5."""
    names = sorted(f for f in os.listdir(src) if f.startswith("pose-2d_") and f.endswith(".json"))
    if not names:
        raise ValueError(f"no pose-2d_*.json in '{src}'")
    with tarfile.open(dest, "w", format=tarfile.PAX_FORMAT) as t:
        for name in names:
            info = t.gettarinfo(os.path.join(src, name), arcname=name)
            info.mtime = 0
            info.uid = info.gid = 0
            info.uname = info.gname = ""
            info.mode = 0o644
            with open(os.path.join(src, name), "rb") as f:
                t.addfile(info, f)
    return len(names)


# ------------------------------------------------------------------- new_job_id
def new_job_id(scene_id):
    """A job id as the Worker would make it (<scene>-<UTC time>-<random>),
    chosen here so that a retried POST /v1/jobs names the same job; it is
    also a Workflow instance id, hence the character set."""
    t = time.strftime("%Y%m%dT%H%M%S", time.gmtime())
    return f"{re.sub(r'[^A-Za-z0-9_-]', '_', scene_id)[:70]}-{t}-{uuid.uuid4().hex[:6]}"


# ------------------------------------------------------------------ HttpControl
class HttpControl:
    """Control backend on the Worker at `base` (http://127.0.0.1:8787 under
    `wrangler dev`), authenticated by `token`: the worker token for an
    agent, the admin token for submit and cancel."""

    def __init__(self, base, token):
        if not token:
            raise ValueError("the Worker API needs a token (--token, MVCLOUD_WORKER_TOKEN or MVCLOUD_ADMIN_TOKEN)")
        self.base = base.rstrip("/")
        self.token = token

    def auth_(self):
        return {"Authorization": f"Bearer {self.token}"}

    # -------------------------------------------------------------------- call_
    def call_(self, method, path, body=None):
        """One request; returns (status, decoded json body or None). Raises
        HttpError when the Worker cannot be reached."""
        data = None if body is None else json.dumps(body).encode()
        headers = {**self.auth_(), "Content-Type": "application/json"}
        request = urllib.request.Request(self.base + path, data=data, headers=headers, method=method)
        try:
            with urllib.request.urlopen(request, timeout=k_timeout_s) as r:
                status, raw = r.status, r.read()
        except urllib.error.HTTPError as e:
            status, raw = e.code, e.read()
        except (OSError, urllib.error.URLError) as e:
            raise HttpError(f"{method} {path}: {e}") from e
        try:
            return status, (json.loads(raw) if raw else None)
        except ValueError:
            return status, {"error": raw[:500].decode(errors="replace")}

    # ---------------------------------------------------------------- retrying_
    def retrying_(self, method, path, body=None):
        """call_ with retries of 5xx answers and unreachable Workers. Only
        for calls that the Worker makes idempotent: a repeated complete or
        fail of the same lease is a no-op, a repeated pull returns the lease
        it handed out if its reply was lost (one that has not heartbeated
        yet), and a repeated POST /v1/jobs names the same job_id, so it
        returns the job the first one created."""
        for i in range(k_attempts):
            try:
                status, obj = self.call_(method, path, body)
                if status < 500:
                    return status, obj
                problem = f"{method} {path}: HTTP {status} {obj}"
            except HttpError as e:
                problem = str(e)
            if i + 1 == k_attempts:
                raise HttpError(problem)
            log.warning(f"{problem}; retrying")
            time.sleep(min(30.0, k_backoff_s * 2**i))

    # --------------------------------------------------------------- authorize_
    def authorize_(self, lease):
        """Adds the tokens to the lease's URLs on this Worker."""
        own = lambda url: url.startswith(self.base + "/")
        for i in lease["inputs"]:
            if own(i["url"]):
                i["headers"] = self.auth_()
        for o in lease["outputs"]:
            if own(o["url"]):
                o["headers"] = {**self.auth_(), "x-mvcloud-lease": lease["lease_token"]}
        return lease

    def ids_(self, lease):
        return {"job_id": lease["job_id"], "shard_id": lease["shard_id"],
                "lease_token": lease["lease_token"]}

    # --------------------------------------------------------------------- pull
    def pull(self, worker):
        """A lease, or None when there is no work or the Worker cannot be
        reached (the agent polls again)."""
        try:
            status, obj = self.retrying_("POST", "/v1/pull", dict(worker))
        except HttpError as e:
            log.warning(f"pull: {e}")
            return None
        if status == 204:
            return None
        if status != 200:
            log.warning(f"pull: HTTP {status} {obj}")
            return None
        return self.authorize_(obj)

    # ---------------------------------------------------------------- heartbeat
    def heartbeat(self, lease, progress):
        try:
            status, obj = self.call_("POST", "/v1/heartbeat", {**self.ids_(lease), "progress": progress})
        except HttpError as e:
            log.warning(f"heartbeat: {e}")
            return HeartbeatResult.TRANSIENT
        if status == 200:
            return HeartbeatResult.OK
        if status == 409:
            log.warning(f"heartbeat refused: {obj}")
            return HeartbeatResult.LOST
        log.warning(f"heartbeat: HTTP {status} {obj}")
        return HeartbeatResult.TRANSIENT

    # ----------------------------------------------------------------- complete
    def complete(self, lease, done):
        try:
            status, obj = self.retrying_("POST", "/v1/complete", {**self.ids_(lease), "done": done})
        except HttpError as e:
            raise Conflict(f"complete: {e}") from e
        if status != 200:
            raise Conflict(f"complete: HTTP {status} {obj}")

    # --------------------------------------------------------------------- fail
    def fail(self, lease, error, retryable, outcome=None):
        body = {**self.ids_(lease), "error": error, "retryable": bool(retryable), "outcome": outcome}
        try:
            status, obj = self.retrying_("POST", "/v1/fail", body)
            if status != 200:
                log.warning(f"fail: HTTP {status} {obj}")
        except HttpError as e:  # the lease expires on its own
            log.warning(f"fail: {e}")

    # ------------------------------------------------------------------ upload_
    def upload_(self, path, scene_id, name):
        """Puts the local file `path` at scenes/<scene_id>/<md5>/<name>
        unless an object of its size is there; returns the key."""
        md5 = storage.file_md5(path)
        key = f"scenes/{scene_id}/{md5}/{name}"
        url = f"{self.base}/v1/blobs/{key}"
        size = os.path.getsize(path)
        headers = {**self.auth_(), "x-mvcloud-md5": md5}
        if storage.head(url, headers=self.auth_()) == size:
            log.info(f"{key}: present")
            return key
        if size > k_single_put_max:
            spec = {"init_url": f"{url}?uploads", "part_url": url, "complete_url": url, "abort_url": url,
                    "part_bytes": k_part_bytes}
            storage.put_multipart(spec, path, headers=headers)
        else:
            storage.put(url, path, headers=headers)
        log.info(f"{key}: uploaded {size} bytes")
        return key

    # ------------------------------------------------------------------- submit
    def submit(self, spec):
        """Uploads the inputs of the job `spec` (the fs:// job format, with
        local paths and the scene's numbers) and creates the job. Returns
        its id; raises ValueError when the Worker refuses it."""
        s = {k: v for k, v in spec.items() if k not in ("manifest", "params", "poses", "gt")}
        scene = spec["scene_id"]
        s.setdefault("job_id", new_job_id(scene))
        manifest = os.path.abspath(spec["manifest"])
        s["manifest_key"] = self.upload_(manifest, scene, "manifest.json")
        with open(manifest) as f:
            videos = [v["file"] for v in json.load(f).get("videos", [])]
        s["videos"] = [{"name": v, "key": self.upload_(os.path.join(os.path.dirname(manifest), v), scene, v)}
                       for v in videos]
        s["params_key"] = self.upload_(spec["params"], scene, "params.json")
        if spec.get("poses"):
            if os.path.isdir(spec["poses"]):
                tmp = tempfile.mkdtemp(prefix="mvcloud-poses-")
                try:
                    tar = os.path.join(tmp, "poses.tar")
                    n = tar_poses(spec["poses"], tar)
                    log.info(f"poses: {n} files from {spec['poses']}")
                    s["poses_key"] = self.upload_(tar, scene, "poses.tar")
                finally:
                    shutil.rmtree(tmp, ignore_errors=True)
            else:
                s["poses_key"] = self.upload_(spec["poses"], scene, "poses.tar")
        if spec.get("gt"):
            s["gt_key"] = self.upload_(spec["gt"], scene, "ground-truth.json")
        status, obj = self.retrying_("POST", "/v1/jobs", s)
        if status != 201:
            raise ValueError(f"the Worker refused the job: HTTP {status} {(obj or {}).get('error', obj)}")
        return obj["job_id"]

    # ------------------------------------------------------------------- status
    def status(self, job_id=None):
        """The job (or every job, without shards) as the Worker reports it."""
        status, obj = self.retrying_("GET", f"/v1/jobs/{job_id}" if job_id else "/v1/jobs")
        if status == 404:
            raise KeyError(f"no job '{job_id}'")
        if status != 200:
            raise HttpError(f"status: HTTP {status} {obj}")
        return [obj] if job_id else obj["jobs"]

    # ------------------------------------------------------------------- cancel
    def cancel(self, job_id):
        status, obj = self.retrying_("POST", f"/v1/jobs/{job_id}/cancel")
        if status == 404:
            raise KeyError(f"no job '{job_id}'")
        if status != 200:
            raise HttpError(f"cancel: HTTP {status} {obj}")

    def sweep(self, verify=False):
        raise ValueError("the Worker's Workflow sweeps its jobs; `sweep` is for fs:// roots")
