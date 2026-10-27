# 视频独立自启，JSON随NeuPAN启动

- `neupan-ros-master.service`：只维持共享ROS1 Master，不启动雷达、底盘或规划。
- `neupan-remote-video.service`：只接收GPU的H.264/RTP UDP 5000，发布`/camera/remote_image_raw`、`/camera/remote_camera_info`，写入`/dev/video0`；无JSON、绘图、录像或弹窗。
- `full_system_run.launch`：默认`enable_hybridnets_video:=false`，仍启动原JSON接收及按配置启用的靠右逻辑。GPU命令与5000/6000端口不变。
- `receiver.launch`同样默认`start_video:=false`，避免人工测试重复抢占自启节点。没有启用自启服务时，可显式`start_video:=true`。

本机2026-10-27已安装并启用两项服务，收到1920×1080 BGR原图；单独重启视频服务后画面恢复，Master PID不变且没有重复接收进程，未启动JSON或车辆节点。构建通过，全量538项中530通过、8项可选跳过；尚未重启整机验证开机流程。

登录NUC图形桌面后自动打开“原视频接收服务”终端，不再用对话框：最多等待60秒确认服务、容器和节点，然后显示检查结果、服务状态、实时日志及停止/启动命令。按Ctrl+C退出日志跟随后进入宿主机命令行；关窗口不停止服务。终端不显示摄像头画面，不启动控制，检查通过不等于GPU已推流。纯SSH不登录图形桌面时不会弹终端，后台服务照常运行；配置在`~/.config/autostart/neupan-video-status.desktop`。可在“启动应用程序”禁用“原视频接收服务终端”，只取消弹终端，不取消视频服务。

实际收流状态由接收节点按有效解码、尺寸正确且已发布的原图计数，使用单调时钟，不订阅/拷贝第二份大图：

- `视频接收成功 | 接收帧率=xx.x FPS`：每秒更新接收/ROS发布帧率，不是GPU推理帧率或虚拟摄像头配置的30 FPS。
- `等待推流`：本次接收节点启动后尚无有效帧，每5秒提示；端口已打开不算接收成功。
- `视频断流`：收到过有效帧后，连续3秒无有效帧（`receiver.yaml`的`stream_timeout_sec`）；状态变化立即提示，持续断流每5秒提醒。

恢复有效帧后自动切回成功；网络断流仍继续监听，不额外增加“无帧强制重建”策略。原有进程/解码器/虚拟摄像头写入错误的自动恢复保留。

## 服务安装与查看（NUC宿主机，不在容器内）

```bash
cd /home/nuc/lxw/neupan_ws
sudo install -m 0644 src/hybridnets_receiver/systemd/neupan-ros-master.service /etc/systemd/system/neupan-ros-master.service
sudo install -m 0644 src/hybridnets_receiver/systemd/neupan-remote-video.service /etc/systemd/system/neupan-remote-video.service
sudo systemctl daemon-reload
sudo systemctl enable --now neupan-ros-master.service neupan-remote-video.service
systemctl status neupan-remote-video.service --no-pager
```

```bash
journalctl -u neupan-remote-video.service -n 30 --no-pager
```

服务会自动启动现有`Neupan`容器并加载Noetic/devel环境，不需要菜单按键或激活conda。复用现有`v4l2loopback`开机配置，不覆盖其他虚拟摄像头。无GPU推流时等待，不代表已有画面；端口/同名节点已有使用者时只等待，不杀原进程。Master丢失/更换后重新连接，子进程退出自动重试。

关闭NeuPAN不会关闭服务拥有的Master和视频。若服务启动时已有外部Master，只借用、不接管；外部Master退出时服务才创建自己的Master，因此首次交接可能短暂断流。不要在行驶中停止/重启共享Master，也不要用`rosnode kill -a`关闭整车。

如需停用视频自启，先停稳再在宿主机执行`sudo systemctl disable --now neupan-remote-video.service`；Master仍可保留。之后整车可显式增加`enable_hybridnets_video:=true`，或人工`roslaunch hybridnets_receiver video_only.launch`，二者只选一个视频接收者。
