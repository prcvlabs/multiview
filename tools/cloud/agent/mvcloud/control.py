"""The control-plane interface: what an agent needs from a backend.

A backend hands out leases on shards, extends them on heartbeats, and
records how each attempt ended. The filesystem backend (fsbackend.FsControl)
implements it for one machine and the tests; httpcontrol.HttpControl is the
same calls on the Worker API (tools/cloud/worker).

Times are seconds since the epoch.
"""

import enum
from typing import Literal, Protocol, TypedDict


class Lease(TypedDict):
    shard_id: str  # "loc-0003" or "tracks", unique within the job
    job_id: str
    attempt: int  # 1 for the first attempt
    lease_token: str
    lease_expires_at: float
    lease_seconds: int  # each heartbeat extends the lease by this much
    heartbeat_s: int
    max_wall_s: int  # from the GPU lock to the child's exit
    progress_timeout_s: int  # longest gap between two progress lines
    kind: Literal["loc", "tracks"]
    scene_id: str
    total_frames: int
    n_cameras: int
    cli: dict  # start_frame, n_frames, K, no_stereo, no_still, pose_mode,
    #            pose_classifier, still_warmup, require_aligned, target_fps,
    #            extra_args
    inputs: list  # [{name, url, kind: "file"|"dir"|"tar", bytes, md5, cache}]
    outputs: list  # [{name, url, required}]
    keep: list  # tracks only: [{name, start, n}] per loc envelope, in order
    params_md5: str | None  # tracks only: the loc shards' parameter digest


class WorkerInfo(TypedDict):
    worker_id: str
    gpu: str
    version: str


class HeartbeatResult(enum.Enum):
    OK = "ok"
    LOST = "lost"  # the lease is gone: expired, re-leased or cancelled
    TRANSIENT = "transient"  # no answer; retry until the lease would expire


class Conflict(Exception):
    """The lease is not the shard's current lease."""


# ---------------------------------------------------------------------- Control
class Control(Protocol):
    def pull(self, worker: WorkerInfo) -> Lease | None:
        """Leases the next runnable shard, or returns None."""

    def heartbeat(self, lease: Lease, progress: dict) -> HeartbeatResult:
        """Extends the lease."""

    def complete(self, lease: Lease, done: dict) -> None:
        """Marks the shard done once its outputs are in place. A no-op for a
        cancelled shard or a repeated call; raises Conflict for a lease that
        is no longer current."""

    def fail(self, lease: Lease, error: str, retryable: bool, outcome: str | None = None) -> None:
        """Ends the attempt; `outcome` (retry, timeout, fatal, stopped) is
        recorded with it. A no-op for a cancelled shard or a lease that is
        no longer current."""


# ----------------------------------------------------------------- open_control
def open_control(url, token=None):
    """The backend that `url` names: fs:///path/to/root, or the Worker API
    at http(s)://host[:port] with `token` (bearer)."""
    if url.startswith("fs://"):
        from .fsbackend import FsControl

        return FsControl(url[len("fs://"):])
    if url.startswith(("http://", "https://")):
        from .httpcontrol import HttpControl

        return HttpControl(url, token)
    raise ValueError(f"unsupported control url '{url}' (fs:// or http(s)://)")
