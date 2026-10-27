#!/bin/bash
# Run as desktop user after graphical login, never as a root system service.
set -u

video_ready() {
    /usr/bin/systemctl is-active --quiet neupan-ros-master.service &&
    /usr/bin/systemctl is-active --quiet neupan-remote-video.service &&
    test "$(/usr/bin/timeout 3 /usr/bin/docker inspect --format '{{.State.Running}}' Neupan 2>/dev/null)" = true &&
    /usr/bin/python3 - <<'PY'
import socket
import sys
import xmlrpc.client
socket.setdefaulttimeout(1.)
try:
    master = xmlrpc.client.ServerProxy('http://127.0.0.1:11311')
    code, _, uri = master.lookupNode('/video_startup_notice', '/rtp_camera_receiver')
    if code != 1:
        sys.exit(1)
    code, _, _ = xmlrpc.client.ServerProxy(uri).getPid('/video_startup_notice')
    sys.exit(0 if code == 1 else 1)
except (OSError, ValueError, xmlrpc.client.Error):
    sys.exit(1)
PY
}

# Headless check for deployment verification; never opens a window.
if [[ "${1:-}" == --check ]]; then
    video_ready
    exit $?
fi

# Open a terminal, not a camera window or a Zenity dialog.
if [[ "${1:-}" != --terminal ]]; then
    exec /usr/bin/gnome-terminal --window --title="原视频接收服务" -- \
        /bin/bash "$0" --terminal
fi

# Prevent simultaneous duplicate status terminals.
exec 9>"${XDG_RUNTIME_DIR:-/run/user/$(id -u)}/neupan-video-terminal.lock"
/usr/bin/flock -n 9 || exit 0
printf '%s\n' '正在检查原视频接收服务（最多等待60秒）……'
video_notice_deadline=$((SECONDS + 60))
video_notice_ready=false
while (( SECONDS < video_notice_deadline )); do
    if video_ready; then
        video_notice_ready=true
        break
    fi
    sleep 2
done

if $video_notice_ready; then
    printf '\033[32m%s\033[0m\n' '检查通过：Neupan容器、ROS Master、原视频接收节点已运行。'
else
    printf '\033[33m%s\033[0m\n' '启动尚未确认，请检查下面的服务状态及日志。'
fi
printf '%s\n' \
    '视频话题：/camera/remote_image_raw   虚拟摄像头：/dev/video0' \
    '此服务不启动NeuPAN、底盘或JSON处理；节点已运行不代表GPU已经推流。' \
    '实际收流看下方日志：视频接收成功＋接收帧率 / 等待推流 / 视频断流。' \
    '首次无帧为等待推流；收到过帧后连续3秒无帧为断流；恢复来帧后自动变为成功。' \
    '' \
    '操作说明（以下命令都在NUC宿主机执行，不进入容器）：' \
    '  关闭本窗口：点右上角X，只关闭日志窗口，视频服务继续运行。' \
    '  Ctrl+C：停止跟随日志，进入可输入命令的终端；不会停止视频服务。' \
    '  暂停原视频：sudo systemctl stop neupan-remote-video.service' \
    '  重新启动：  sudo systemctl start neupan-remote-video.service' \
    '  取消视频开机自启并立即停止：sudo systemctl disable --now neupan-remote-video.service' \
    '  恢复视频开机自启并立即启动：sudo systemctl enable --now neupan-remote-video.service' \
    '  取消此终端自动弹出：在“启动应用程序”中禁用“原视频接收服务终端”。' \
    '  停止视频前请先停车；不要在行驶中停止共享Master或容器。' \
    ''
/usr/bin/systemctl status neupan-ros-master.service neupan-remote-video.service --no-pager -n 3
printf '\n%s\n' '现在实时显示服务日志；按Ctrl+C后可输入上面的关闭/启动命令。'
trap ':' INT
/usr/bin/journalctl -u neupan-ros-master.service -u neupan-remote-video.service -n 15 -f --no-pager
trap - INT
printf '\n%s\n' '日志跟随已退出，视频服务未停止。现在可以输入systemctl命令；exit只关闭窗口。'
exec /bin/bash --noprofile --norc -i
