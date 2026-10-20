#!/usr/bin/env python3
"""Start rosbag record from a reusable YAML profile and guard disk space."""

import argparse
import datetime
import glob
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
from datetime import timedelta, timezone

import rospkg
import rospy
import yaml


class BagRecorder:
    def __init__(self, config_path):
        self.config_path = os.path.abspath(os.path.expanduser(config_path))
        self.config = self._load_config(self.config_path)
        self.process = None
        self.stopping = False
        self.stop_finished = False
        self.inventory_saved = False
        self.stop_reason = 'stop_requested'
        self.stop_details = None

        output_override = rospy.get_param("~output_directory_override", "").strip()
        prefix_override = rospy.get_param("~bag_prefix_override", "").strip()
        output_dir = output_override or str(self.config.get("output_directory", "~/rosbags"))
        bag_prefix = prefix_override or str(self.config.get("bag_prefix", "vehicle_test"))
        self.output_dir = os.path.abspath(os.path.expanduser(output_dir))
        self.bag_prefix = self._safe_name(bag_prefix)
        self.min_free_gb = max(0.0, float(self.config.get("min_free_space_gb", 5.0)))
        self.check_period = max(1.0, float(self.config.get("disk_check_period_sec", 5.0)))
        self.timezone_offset_hours = float(self.config.get("timezone_offset_hours", 8.0))
        self.local_timezone = timezone(timedelta(hours=self.timezone_offset_hours))
        self.use_session_subdirectory = bool(
            self.config.get(
                "session_subdirectory",
                self.config.get("daily_subdirectory", True),
            )
        )
        if self.use_session_subdirectory:
            session_name = "rosbag{}".format(
                datetime.datetime.now(self.local_timezone).strftime("%Y%m%d_%H%M%S")
            )
            self.output_dir = os.path.join(self.output_dir, session_name)
        required_topics = self.config.get("required_topics", [])
        if not isinstance(required_topics, list):
            raise RuntimeError("required_topics 必须是列表")
        self.required_topics = [
            str(topic).strip() for topic in required_topics if str(topic).strip()
        ]
        self.seen_required_topics = set()

        os.makedirs(self.output_dir, exist_ok=True)
        self._check_initial_space()
        self.output_prefix = os.path.join(
            self.output_dir,
            "{}_{}".format(
                self.bag_prefix,
                datetime.datetime.now(self.local_timezone).strftime("%Y%m%d_%H%M%S"),
            ),
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
        self.manifest = {
            "created_at": datetime.datetime.now(self.local_timezone).isoformat(),
            "timezone_offset_hours": self.timezone_offset_hours,
            "hostname": socket.gethostname(),
            "config_file": self.config_path,
            "output_prefix": self.output_prefix,
            "command": self.command,
            "config": self.config,
        }
        self._save_manifest()

    def _save_manifest(self):
        # Keep the original manifest intact if a shutdown update fails.
        path = self.output_prefix + ".json"
        temporary = path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            json.dump(self.manifest, stream, ensure_ascii=False, indent=2, sort_keys=True)
        os.replace(temporary, path)

    def _record_bag_inventory(self):
        """Read finalized bag indexes only; no image/point-cloud deserialization."""
        counts, files, errors = {}, [], []
        paths = sorted(glob.glob(self.output_prefix + "*.bag"))
        active = sorted(glob.glob(self.output_prefix + "*.bag.active"))
        try:
            if paths:
                import rosbag
                for path in paths:
                    try:
                        with rosbag.Bag(path, 'r') as bag:
                            topics = bag.get_type_and_topic_info().topics
                            entries = {name: {'message_count': info.message_count,
                                              'type': info.msg_type}
                                       for name, info in topics.items()}
                        files.append({'path': path, 'topics': entries})
                        for name, entry in entries.items():
                            counts[name] = counts.get(name, 0) + entry['message_count']
                    except Exception as exc:
                        errors.append({'path': path, 'error': str(exc)})
        except Exception as exc:
            errors.append({'error': str(exc)})
        return_code, process_running = None, False
        try:
            if self.process is not None:
                return_code = self.process.poll()
                process_running = return_code is None
        except Exception as exc:
            process_running = None
            errors.append({'stage': 'process_poll', 'error': str(exc)})
        verified = bool(paths) and not errors and not active and process_running is False
        missing = [name for name in self.required_topics if counts.get(name, 0) == 0]
        self.manifest['recording_result'] = {
            'closed_at': datetime.datetime.now(self.local_timezone).isoformat(),
            'return_code': return_code, 'process_running': process_running,
            'shutdown': self.stop_details,
            'index_verification_complete': verified,
            'files': files, 'unfinalized_files': active, 'inspection_errors': errors,
            'topic_message_counts': counts,
            'required_topics_without_messages': missing,
            'all_required_topics_recorded': verified and not missing,
            'required_topics_seen_publishers': sorted(self.seen_required_topics),
        }
        self._save_manifest()
        if not verified:
            rospy.logwarn('录包消息核验未完成：请检查 recording_result（进程仍运行、无文件、未完成索引或读取失败）')
        elif missing:
            rospy.logerr('bag索引确认以下必录话题没有消息: %s', ', '.join(missing))
        else:
            rospy.loginfo('bag索引核验完成：所有必录话题均有消息，计数已写入录包清单')

    def start(self):
        rospy.loginfo("rosbag 输出前缀: %s", self.output_prefix)
        rospy.loginfo("磁盘剩余 %.2f GB；停止阈值 %.2f GB", self._free_gb(), self.min_free_gb)
        rospy.loginfo("执行: %s", " ".join(self.command))
        process_env = os.environ.copy()
        process_env["TZ"] = "Asia/Shanghai"
        self.process = subprocess.Popen(
            self.command, preexec_fn=os.setsid, env=process_env
        )
        self._check_required_topics()
        rospy.Timer(rospy.Duration(self.check_period), self._monitor)

    def _monitor(self, _event):
        if self.process is None or self.stopping:
            return
        return_code = self.process.poll()
        if return_code is not None:
            self.stop_reason = 'recorder_exited'
            rospy.logerr("rosbag record 意外退出，返回码 %d", return_code)
            rospy.signal_shutdown("rosbag record exited")
            return
        free_gb = self._free_gb()
        if self.min_free_gb > 0.0 and free_gb <= self.min_free_gb:
            self.stop_reason = 'low_disk_space'
            rospy.logerr("磁盘仅剩 %.2f GB，正在安全停止录制", free_gb)
            rospy.signal_shutdown("low disk space")
            return
        self._check_required_topics()

    def _check_required_topics(self):
        if not self.required_topics:
            return
        try:
            published = {name for name, _type in rospy.get_published_topics()}
        except rospy.ROSException as exc:
            rospy.logwarn_throttle(5.0, "无法查询必录话题: %s", exc)
            return
        self.seen_required_topics.update(published.intersection(self.required_topics))
        missing_now = [topic for topic in self.required_topics if topic not in published]
        if missing_now:
            rospy.logwarn_throttle(
                5.0,
                "录包继续，但以下必录话题当前没有发布者: %s",
                ", ".join(missing_now),
            )
        else:
            rospy.loginfo_throttle(30.0, "必录话题均有发布者（不代表已录到消息；结束时核对bag索引）")

    def stop(self):
        if self.stopping:
            # A failed manifest write may be retried, but never resend signals.
            if self.stop_finished and not self.inventory_saved:
                self._save_final_inventory()
            return
        self.stopping = True
        self.stop_details = {'requested_reason': self.stop_reason,
                             'outcome': 'not_started', 'errors': []}
        try:
            if self.process is not None:
                self.stop_details['outcome'] = 'already_exited'
                if self.process.poll() is None:
                    rospy.loginfo("正在关闭 rosbag，请等待索引写入完成……")
                    for name, sig, timeout in (('sigint', signal.SIGINT, 20.0),
                                               ('sigterm', signal.SIGTERM, 5.0)):
                        stage = name + '_signal'
                        try:
                            os.killpg(os.getpgid(self.process.pid), sig)
                            stage = name + '_wait'
                            self.process.wait(timeout=timeout)
                            self.stop_details['outcome'] = name + '_exit'
                            break
                        except subprocess.TimeoutExpired as exc:
                            self.stop_details['outcome'] = 'stop_timeout'
                            self._record_stop_error(stage, exc)
                            rospy.logwarn('rosbag %s等待超时；仍将保存结束核验', name)
                        except Exception as exc:
                            self.stop_details['outcome'] = 'stop_error'
                            self._record_stop_error(stage, exc)
                            break
        except Exception as exc:
            self.stop_details['outcome'] = 'stop_error'
            self._record_stop_error('process_poll', exc)
        finally:
            self._save_final_inventory()
            self.stop_finished = True
        rospy.loginfo("录包关闭处理结束: %s", self.output_prefix)

    def _record_stop_error(self, stage, exc):
        self.stop_details['errors'].append({
            'stage': stage, 'error_type': type(exc).__name__, 'error': str(exc)})

    def _save_final_inventory(self):
        try:
            self._record_bag_inventory()
            self.inventory_saved = True
        except Exception as exc:
            if self.stop_details is not None:
                self._record_stop_error('inventory_save', exc)
            rospy.logerr('录包清单更新失败，未删除bag或原清单: %s', exc)


def default_config_path():
    package_dir = rospkg.RosPack().get_path("vehicle_bag_recorder")
    return os.path.join(package_dir, "config", "vehicle_data.yaml")


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
            recorder.stop_reason = 'signal_' + signal.Signals(_signum).name
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
