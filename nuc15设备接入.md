# nuc15 设备接入

## 当前接口

| 设备 | nuc15 主机接口 / 别名 | 配置 |
|---|---|---|
| MID360 | `enp86s0`，连接名 `雷达-MID360` | 主机 `192.168.1.100/24`，雷达 `192.168.1.183`；无默认网关 |
| Jetson | `enxec1ac30270b6`，连接名 `GPU板通信` | NUC `192.168.1.10/24`，Jetson `192.168.1.20/24`；无默认网关 |
| G90 RTK | `/dev/wheeltec_gnss` → `ttyACM0` | `115200`；按序列号 `5B0A021611` 固定 |
| 底盘 CH340 | `/dev/robot_base` → `ttyUSB0` | `115200`；按 USB ID `1a86:7523` 固定 |
| 5G 上网连接 | `enxec1ac30270b0` | 保留 DHCP、网关和 DNS；当前地址 `10.2.111.61` 可能变化 |

雷达及 Jetson 网络连接均自动启用，不占用默认路由；5G/Wi-Fi 上网配置未更改。

两个独立网口使用相同 `192.168.1.0/24` 网段：NetworkManager 已持久化 `.20/32` 到 Jetson 网口、`.183/32` 到雷达网口的主机路由。主机 `/etc/sysctl.d/99-neupan-multinic.conf` 为这两个接口设置 `arp_ignore=1`、`arp_announce=2`，避免其他网口代答 ARP。源配置为 `src/hybridnets_receiver/config/nuc15_multinic_sysctl.conf`。接收 JSON 的 `allowed_sender_ip` 已改为 `192.168.1.20`，管理握手仍使用 UDP 6001。

## 主机与 ROS1 容器路径

主机工作区为 `/home/nuc15/lxw/neupan_ws`；Docker 将 `/home/nuc15` 映射为 `/home/nuc`，因此容器中仍使用 `/home/nuc/lxw/neupan_ws`。不要全局把 ROS 配置中的 `/home/nuc` 改为 `/home/nuc15`。

NUC 主机输入 `Neupan`、按 `e` 后执行：

```bash
cd /home/nuc/lxw/neupan_ws
conda activate lxw
source /opt/ros/noetic/setup.bash
source /home/nuc/lxw/neupan_ws/devel/setup.bash
roslaunch neupan_ros full_system_run.launch outdoor_mode:=true
```

已使用 NetworkManager 保存固定地址，无需每次执行 `ifconfig`。单独修改 launch 和串口规则无需编译；本次未启动实车导航。

已通过 `Neupan → e → 1` 进入 lxw 环境完成工作区编译。迁移遗留的 `OMPL_LIBRARIES` 缓存指向不存在的 `/opt/ros/noetic/lib/x86_64-linux-gnu/libompl.so`；使用下列命令重新查找为 `/usr/lib/x86_64-linux-gnu/libompl.so`，不修改业务源码：

```bash
catkin_make -j4 -l4 --cmake-args -UOMPL_LIBRARIES
# 后续正常编译
catkin_make -j4 -l4
```

## 串口规则

规则源文件：`src/robot_base_driver/config/99-neupan-serial.rules`，已安装到主机 `/etc/udev/rules.d/99-neupan-serial.rules`。

CH340 使用内核自带 `ch341` 驱动，无需下载安装。此前串口消失是 BRLTTY 抢占 `1a86:7523`；主机 `/etc/udev/rules.d/85-brltty.rules` 已覆盖系统规则，仅禁用该 ID 的误识别条目，其他盲文设备规则保留。已停止本次误启动的 `brltty-udev.service` 并重新绑定驱动。

CH340 无唯一序列号，目前规则要求只连接一只 `1a86:7523`。以后接入多只相同适配器，应按 USB 物理端口分别匹配，避免别名冲突。

## 原视频开机自启动

nuc15 已安装并启用 `neupan-ros-master.service` 和 `neupan-remote-video.service`。开机自动启动 Neupan 容器、共享 ROS Master、UDP 5000 原视频接收，发布 `/camera/remote_image_raw` 和 `/camera/remote_camera_info`，并写入虚拟摄像头 `/dev/video0`。只启动原视频接收，不启动雷达、底盘、导航或 JSON 处理；JSON 接收仍随 NeuPAN 全系统启动。

登录桌面后打开“原视频接收服务”状态终端，不弹摄像头画面。桌面入口已适配主机 `/home/nuc15` 路径，后台服务仍使用容器内 `/home/nuc` 路径。关状态窗口不停止后台接收。

主机查看或控制服务：

```bash
systemctl status neupan-ros-master.service neupan-remote-video.service --no-pager
journalctl -u neupan-remote-video.service -n 20 -f
# 暂停接收（不关闭共享 Master）
sudo systemctl stop neupan-remote-video.service
# 恢复接收
sudo systemctl start neupan-remote-video.service
```

全系统默认不重复启动原视频节点；不要同时手动启动第二个 UDP 5000 接收器。服务“运行中”只表示在监听，必须看到“视频接收成功”和非零 FPS 才算真正收到视频。

## 网络与串口检查命令

```bash
ip -br -4 addr
ip route get 192.168.1.183
ip route get 192.168.1.20
ping -c 3 192.168.1.183
ping -c 3 192.168.1.20
ls -l /dev/wheeltec_gnss /dev/robot_base
```

2026-11-03 接入验证：雷达 `.183` 和 Jetson `.20` 均可达，有线 SSH 已验证；容器通过两个固定串口别名成功读取 G90 数据和底盘 `MOTOR` / `ODO` 数据。RTK 当时 GGA quality=0（未定位）。5G 接口已获取 DHCP，指定该接口访问外网 HTTPS 返回 200。

## 给 Jetson 共享外网

已启用 `neupan-jetson-network.service` 和 `neupan-jetson-dns.service`，开机提供转发、NAT 和 DNS，不提供 DHCP、不改变雷达网口。NUC 内置 Wi-Fi 路由 metric=100，5G=600；Wi-Fi 断开后使用 5G，不保证 Wi-Fi 保持连接但外网故障时自动切换。

Jetson 的 `nuc-link` 保留 `192.168.1.20/24`，网关和 DNS 均为 `192.168.1.10`，有线路由 metric=100。已从 Jetson 强制走 `eno1` 访问外网 HTTPS 返回 200。

USB 拔插后曾出现 NetworkManager 向已不存在接口同步 DNS 失败，系统解析服务没有可用上游，导致 RustDesk 域名解析失败。已将 `src/hybridnets_receiver/config/nuc15_resolved_dns.conf` 安装为 `/etc/systemd/resolved.conf.d/60-neupan-dns.conf`，保留原有 `/etc/resolv.conf` 链接，提供系统级 DNS `223.5.5.5`、`1.1.1.1`。修复后域名解析及 RustDesk 服务器 TCP 21116 连接验证通过。

USB 无线网卡为 UGREEN AIC8800DC，已从驱动 U 盘模式切换为无线模式，编译安装当前 `6.12.111-0612111-generic` 内核驱动并成功连接 `chishane`。接口 `wlxec1ac304a39a`，连接名 `USB-WiFi-chishane`；保留内置 Wi-Fi，不强制切换现有连接。后续检查时该 USB 设备已不在 USB 列表，需要插回才能使用。

驱动源码及内核头文件保留在 `/home/nuc15/lxw/hardware_drivers/`，构建镜像为 `neupan-aic-build:6.12.111-20261103`；未更换系统内核。源码来自 https://github.com/Kiborgik/aic8800dc-linux-patched ，提交 `aad6a095dc795975bb16a291fc7b22025348115b`。更换内核后需重新编译驱动；模式切换规则安装在 `/etc/udev/rules.d/99-neupan-aic-wifi.rules`。

视频自启动服务已运行并通过节点检查，尚未重启整机验收。当前没有图像或推理 JSON：Jetson 的既有 `hybridnets-stream.service` 正在运行并发送到 `.10:5000/6000`，但内核记录 Camera3 于 15:16:31 USB 断开，检查时 USB 列表及 `/dev/video*` 均无摄像头。需要在 Jetson 侧恢复摄像头 USB 识别；其监督程序会自动重连。不要把服务 active 或 ping 成功当作已收到视频。
