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
overlay publishes vehicle control commands. The NeuPAN planner does not yet
subscribe to `/hybridnets/result_json`; a separate metric fusion and path
integration step is needed. See the workspace-level
`REALTIME_HYBRIDNETS_GUIDE.md` for network setup and acceptance checks.

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
