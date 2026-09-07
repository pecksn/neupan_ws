#!/usr/bin/env python3
"""Start rosbag record from a reusable YAML profile and guard disk space."""

import argparse
import datetime
import json
import os
import shutil
import signal
import socket
import subprocess
import sys

import rospkg
import rospy
import yaml


class BagRecorder:
    def __init__(self, config_path):
        self.config_path = os.path.abspath(os.path.expanduser(config_path))
        self.config = self._load_config(self.config_path)
        self.process = None
        self.stopping = False

        output_override = rospy.get_param("~output_directory_override", "").strip()
        prefix_override = rospy.get_param("~bag_prefix_override", "").strip()
        output_dir = output_override or str(self.config.get("output_directory", "~/rosbags"))
        bag_prefix = prefix_override or str(self.config.get("bag_prefix", "vehicle_test"))
        self.output_dir = os.path.abspath(os.path.expanduser(output_dir))
        self.bag_prefix = self._safe_name(bag_prefix)
        self.min_free_gb = max(0.0, float(self.config.get("min_free_space_gb", 5.0)))
        self.check_period = max(1.0, float(self.config.get("disk_check_period_sec", 5.0)))

        os.makedirs(self.output_dir, exist_ok=True)
        self._check_initial_space()
        self.output_prefix = os.path.join(
            self.output_dir,
            "{}_{}".format(self.bag_prefix, datetime.datetime.now().strftime("%Y%m%d_%H%M%S")),
        )
        self.command = self._build_command()
        self._write_manifest()

    @staticmethod
    def _load_config(path):
        if not os.path.isfile(path):
            raise RuntimeError("配置文件不存在: {}".format(path))
        with open(path, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        if not isinstance(data, dict):
            raise RuntimeError("配置文件顶层必须是 YAML 字典")
        return data

    @staticmethod
    def _safe_name(value):
        safe = "".join(char if char.isalnum() or char in "-_" else "_" for char in value)
        return safe.strip("_") or "vehicle_test"

    def _free_gb(self):
        return shutil.disk_usage(self.output_dir).free / float(1024 ** 3)

    def _check_initial_space(self):
        free_gb = self._free_gb()
        if self.min_free_gb > 0.0 and free_gb <= self.min_free_gb:
            raise RuntimeError(
                "磁盘剩余 {:.2f} GB，不高于保护阈值 {:.2f} GB".format(free_gb, self.min_free_gb)
            )

    def _build_command(self):
        command = ["rosbag", "record", "-O", self.output_prefix]
        compression = str(self.config.get("compression", "lz4")).lower()
        if compression == "lz4":
            command.append("--lz4")
        elif compression == "bz2":
            command.append("--bz2")
        elif compression != "none":
            raise RuntimeError("compression 只能是 none、bz2 或 lz4")

        split_size = int(self.config.get("split_size_mb", 0))
        if split_size > 0:
            command.extend(["--split", "--size={}".format(split_size)])
        command.append("--buffsize={}".format(max(1, int(self.config.get("buffer_size_mb", 256)))))
        command.append("--chunksize={}".format(max(1, int(self.config.get("chunk_size_kb", 768)))))
        if bool(self.config.get("tcp_nodelay", True)):
            command.append("--tcpnodelay")

        if bool(self.config.get("record_all", False)):
            command.append("-a")
            exclude = str(self.config.get("exclude_regex", "")).strip()
            if exclude:
                command.extend(["--exclude", exclude])
        else:
            topics = self.config.get("topics", [])
            if not isinstance(topics, list) or not topics:
                raise RuntimeError("record_all=false 时 topics 必须是非空列表")
            command.extend(str(topic).strip() for topic in topics if str(topic).strip())
        return command

    def _write_manifest(self):
        manifest = {
            "created_at": datetime.datetime.now().isoformat(),
            "hostname": socket.gethostname(),
            "config_file": self.config_path,
            "output_prefix": self.output_prefix,
            "command": self.command,
            "config": self.config,
        }
        with open(self.output_prefix + ".json", "w", encoding="utf-8") as stream:
            json.dump(manifest, stream, ensure_ascii=False, indent=2, sort_keys=True)

    def start(self):
        rospy.loginfo("rosbag 输出前缀: %s", self.output_prefix)
        rospy.loginfo("磁盘剩余 %.2f GB；停止阈值 %.2f GB", self._free_gb(), self.min_free_gb)
        rospy.loginfo("执行: %s", " ".join(self.command))
        self.process = subprocess.Popen(self.command, preexec_fn=os.setsid)
        rospy.Timer(rospy.Duration(self.check_period), self._monitor)

    def _monitor(self, _event):
        if self.process is None or self.stopping:
            return
        return_code = self.process.poll()
        if return_code is not None:
            rospy.logerr("rosbag record 意外退出，返回码 %d", return_code)
            rospy.signal_shutdown("rosbag record exited")
            return
        free_gb = self._free_gb()
        if self.min_free_gb > 0.0 and free_gb <= self.min_free_gb:
            rospy.logerr("磁盘仅剩 %.2f GB，正在安全停止录制", free_gb)
            rospy.signal_shutdown("low disk space")

    def stop(self):
        if self.stopping:
            return
        self.stopping = True
        if self.process is not None and self.process.poll() is None:
            rospy.loginfo("正在关闭 rosbag，请等待索引写入完成……")
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
                self.process.wait(timeout=20.0)
            except subprocess.TimeoutExpired:
                rospy.logwarn("rosbag 关闭超时，发送 SIGTERM")
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                self.process.wait(timeout=5.0)
        rospy.loginfo("录制结束: %s", self.output_prefix)


def default_config_path():
    package_dir = rospkg.RosPack().get_path("vehicle_bag_recorder")
    return os.path.join(package_dir, "config", "lidar_curb.yaml")


def main():
    rospy.init_node("vehicle_bag_recorder", disable_signals=True)
    parser = argparse.ArgumentParser(description="按 YAML 配置启动 rosbag record")
    parser.add_argument("--config", default=None, help="录包 YAML 配置文件")
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    recorder = None
    try:
        recorder = BagRecorder(args.config or default_config_path())
        recorder.start()

        def shutdown_handler(_signum, _frame):
            rospy.signal_shutdown("received signal")

        signal.signal(signal.SIGINT, shutdown_handler)
        signal.signal(signal.SIGTERM, shutdown_handler)
        while not rospy.is_shutdown():
            rospy.sleep(0.2)
    except Exception as exc:
        rospy.logfatal("无法启动录包: %s", exc)
        return 1
    finally:
        if recorder is not None:
            recorder.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
