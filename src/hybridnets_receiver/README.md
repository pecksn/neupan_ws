# hybridnets_receiver

ROS1 NUC-side receiver for the GPU-board HybridNets stream.

```bash
roslaunch hybridnets_receiver receiver.launch
```

The package receives H.264/RTP on UDP 5000 and validated JSON on UDP 6000.
By default, it publishes the original video to `/camera/remote_image_raw` and
`/dev/video0`, and validated inference JSON to `/hybridnets/result_json`.
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

Control-mode confirmation requires three consecutive valid frames in the same
session. Sessions combine the sender, `camera_time_epoch`, and preferably a
per-process `producer_session_id` (or `session_id`). GPU now generates a UUID at
each inference-node startup and includes it in every JSON result. Camera reconnects
independently change `camera_time_epoch`. Legacy GPU packets without
a producer ID use the UDP source endpoint as a compatibility fallback; a restart
reusing both endpoint and epoch cannot be distinguished reliably. New sessions
reset confirmation; retired sessions, duplicate/out-of-order frames and expired
packets cannot refresh the active heartbeat. Retired identities are retained
up to 1024 entries; exhaustion fails closed until the receiver is restarted.
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
IP restriction and isolated network. Retired-session protection applies to
sessions already observed by this receiver, not arbitrary unseen senders.

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
