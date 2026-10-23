# hybridnets_receiver

ROS1 NUC-side receiver for the GPU-board HybridNets stream.

```bash
roslaunch hybridnets_receiver receiver.launch
```

The package receives H.264/RTP on UDP 5000 and validated JSON on UDP 6000.
By default, it publishes the original video to `/camera/remote_image_raw` and
`/dev/video0`, and validated inference JSON to `/hybridnets/result_json`.
`/hybridnets/observations_json` independently carries admitted fresh observations,
including no-road, low-confidence and control-confirmation frames, with their
per-frame receiver diagnostics. The overlay and matched debug MP4/JSONL use this
observation stream; only the control JSON is withheld until visual confirmation.
Frequency/confidence metrics continue updating during visual failures. Packets
with rejected credentials, bad schema, stale timestamps or invalid ordering are
not observations and remain rejection diagnostics only. All three bag profiles
record observations as an optional topic.
The debug overlay is only generated when requested:

```bash
roslaunch hybridnets_receiver receiver.launch show_overlay:=true
```

For a debug topic without an automatic display window, use
`start_overlay:=true show_overlay:=false`. Neither the receiver nor its debug
overlay publishes vehicle control commands. The separate `camera_right_path`
node in `osm_map_handler` projects valid boundaries into map coordinates and
provides a refined reference path to NeuPAN when camera-right control is
explicitly enabled. See the workspace-level
`REALTIME_HYBRIDNETS_GUIDE.md` for network setup and acceptance checks.

The 2026-10-23 binding protocol supersedes data-driven session takeover:
GPU owns a process lock and exclusive management UDP 6001. NUC generates a fresh
`receiver_instance_id`, challenges that fixed endpoint with a random nonce, and
confirms a `binding_ticket` (lease ID) tied to producer/camera identity. Replies
expire after one monotonic second; leases last 5 seconds with conservative local
expiry and periodic renewal. Only authorized post-bind captures receive a ticket.
Expired or mismatched tickets and unseen producer IDs cannot nominate a session.
An explicitly changed/not-ready authority reply revokes the old binding; mere
foreign data or a lost query cannot replace a still-valid one. A NUC restart may
wait up to the old lease's 5-second expiry before binding again.
Allow UDP 6001 between 192.168.10.21 and 192.168.10.20; video 5000 and result 6000
remain unchanged. Single-direction legacy senders cannot automatically take over.
`stream_streak` tracks data health separately from `valid_streak`; candidate frames
are diagnostic only until three valid fresh frames share one binding. Only then
is the JSON submitted to the path node. No-road results still diagnose the link.
The packet/health diagnostics expose binding state and `submitted_to_path`.
Existing map-cache TTL, localization, historical capture TF and OSM fallback stay unchanged.

Control-mode confirmation requires three consecutive valid frames in the same
session. Sessions combine the sender, `camera_time_epoch`, and preferably a
per-process `producer_session_id` (or `session_id`). GPU now generates a UUID at
each inference-node startup and includes it in every JSON result. Camera reconnects
independently change `camera_time_epoch`. Both identities and the new lease plus
receiver instance identify the accepted stream; UDP source ports are not an authority. New sessions
reset confirmation; retired sessions, duplicate/out-of-order frames and expired
packets cannot refresh the active heartbeat. Retired identities are diagnostic
history bounded to 1024 entries; history exhaustion cannot veto a currently
challenged valid lease or permanently lock out the authoritative stream.
The path node also checks confirmation for the JSON frame itself, rather than
trusting an independently delivered Boolean alone. Inspect
`/hybridnets/timing_diagnostics` for session, forwarding and rejection details;
the three vehicle bag profiles record this optional topic.

The receiver and path diagnostics expose `producer_session_id` and
`receiver_session_id` explicitly. Once an explicit producer identity has been
seen, identity-less packets cannot downgrade that session protection. The
receiver fails closed on capture ages beyond its age limit, even when the
GPU-reported source age is small; a clock difference over 60 seconds must not
bypass the capture-age check. Keep both boards time-synchronized.
Duplicate/out-of-order datagrams also leave an unexpired confirmation streak
and valid state unchanged, without extending the heartbeat. A timeout still
invalidates the state, and a new invalid observation still resets confirmation.
UUIDs distinguish processes but do not authenticate senders; retain the sender
IP restriction and isolated network. Retired-session history is diagnostic only;
eligibility is now decided by the active challenged lease, not first data arrival.

After this source update, restart the existing ROS1 receiver and camera path
refinement nodes in their normal runtime. Do not launch a duplicate receiver
on UDP 6000. Validate GPU restart, camera unplug/replug, and network disconnect
while stationary before enabling motion; automated tests are not field acceptance.

The overlay keeps a short image buffer and matches each result by
`capture_timestamp_ns`, `source_age_ms`, and the configured RTP delay. The
displayed `sync=...ms` is the distance to the selected buffered frame, not a
measurement of the true network delay. Tune `overlay_video_delay_sec` with a
visible motion/flash test. This is still an estimate because the H.264 stream carries no application frame ID; exact
matching requires the GPU to generate the overlay from the inference input
frame or to transport a common frame ID with both video and JSON.

New GPU results distinguish all measured `*_boundary_segments_px` from the
quality-approved `*_valid_segments_px`. The debug overlay draws measured
segments as dim thin lines and valid segments as bright thick lines. Control
code must check the side's `*_boundary_valid` flag and fit only its valid
segments.

To save one GPU-side video replay as a matched MP4/JSONL pair without opening
a window:

```bash
roslaunch hybridnets_receiver receiver.launch \
  start_overlay:=true show_overlay:=false record_debug_mp4:=true
```

The first `/hybridnets/debug_image` frame starts same-stem `.mp4` and `.jsonl`
files under `/home/nuc/lxw/neupan_ws/videos/hybridnets_debug/`. Each JSONL row
contains the result matched to a recorded debug frame plus `mp4_frame_index`
and `mp4_time_sec`. The pair closes after three seconds without frames;
another replay starts a new pair. Sparse matched frames are held according to
their ROS timestamps so the MP4 keeps wall-clock duration without duplicating
JSON rows. Set
`debug_mp4_output_dir`, `debug_mp4_fps`, or `debug_mp4_idle_timeout_sec` as
launch arguments if needed. When the receiver is already running, start only
the recorder with `rosrun hybridnets_receiver debug_mp4_recorder.py` instead
of opening a second UDP receiver.
