"""Cloud-distributed multiview (issue #14): the agent that runs shards.

A job splits one scene's frame range into loc shards, each a
`multiview-cli pipeline --save-loc-data --no-tracks` run over its frames,
and one tracks job that runs the tracker over all their envelopes
(`--load-loc-data` repeated). Modules:

  control    the backend interface (Lease, Control)
  fsbackend  a backend in a directory (leases, heartbeats, attempts, done.json)
  storage    fetch (size and md5 verified) and put of blobs
  procs      the GPU lock, child sessions, kills, the memory cap
  runner     one shard: the command line, the watchdog, the exit-code taxonomy
  plan       the shards of a job range
  params     the params patch and the params digest
  scene      `multiview-cli scene_info`
  agent      the main loop and the heartbeat thread
  cli        python3 -m mvcloud {agent,submit,status,plan,sweep,cancel}

Python 3.12 standard library only.
"""
