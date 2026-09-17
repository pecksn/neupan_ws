# ROS 1 USB Camera3

This catkin package starts `/dev/video0` at 1920x1080, 30 FPS, MJPEG. Images
are rotated by 180 degrees before being published on `/camera/image_raw`.
`/dev/video1` is camera metadata and must not be used for images.

## Build

```bash
cd /home/nuc/lxw/neupan_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## Start

```bash
roslaunch usb_camera camera.launch preview:=true
```

The image and camera-info topics are `/camera/image_raw` and
`/camera/camera_info`. Headless operation uses `preview:=false` (the default).

## Calibrate

After entering the Neupan ROS 1 container, activate the `lxw` Conda environment
so that `cameracalibrator.py` can import OpenCV:

```bash
conda activate lxw
source /home/nuc/lxw/neupan_ws/devel/setup.bash
```

The calibration launch file starts the camera automatically, so only one
command is needed:

```bash
roslaunch usb_camera calibrate.launch
```

The defaults are 8x12 internal corners and a measured square side length of
20 mm (`0.020` metres). If 8x12 describes the number of squares instead, use
`size:=7x11` because the calibration argument is the number of internal
corners:

```bash
roslaunch usb_camera calibrate.launch size:=7x11 square:=0.020
```

Move and tilt the board until the X, Y, Size and Skew bars turn green. Press
CALIBRATE, wait for the result, then press SAVE and COMMIT. The calibration is
committed to `/home/nuc/.ros/camera_info/usb_camera.yaml`. Copy the result to
`config/usb_camera.yaml` to keep the calibration with this project; the camera
launcher loads that project copy automatically.
