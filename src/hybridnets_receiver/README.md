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
