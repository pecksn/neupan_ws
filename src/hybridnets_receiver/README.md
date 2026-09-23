# hybridnets_receiver

ROS1 NUC-side receiver for the GPU-board HybridNets stream.

```bash
roslaunch hybridnets_receiver receiver.launch
```

The package receives H.264/RTP on UDP 5000 and validated JSON on UDP 6000.
It deliberately does not command the vehicle. See the workspace-level
`REALTIME_HYBRIDNETS_GUIDE.md` for network setup and acceptance checks.
