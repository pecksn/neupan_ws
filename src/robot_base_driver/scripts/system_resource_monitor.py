#!/usr/bin/python3
# Module contract: Read-only CPU, memory, and process metrics for the runtime logger.
# CPU accounting intervals are independent of navigation source timestamps.

"""Resource/config diagnostics independent of the NeuPAN planning process."""
import os
import time
import json
import hashlib
import yaml
import rospy
from std_msgs.msg import String


# Sample system/planner process resources and publish diagnostic
# configuration/measurements.
class SystemResourceMonitor:
    # Configure resource sampling, initialize CPU counter baselines, and establish
    # configuration/measurement publication.
    def __init__(self):
        self.planner_node = rospy.get_param("~planner_node", "/neupan_control")
        self.resource_pub = rospy.Publisher(
            rospy.get_param("~resources_topic", "/neupan/resources"),
            String,
            queue_size=2,
        )
        self.config_pub = rospy.Publisher(
            rospy.get_param("~snapshot_topic", "/neupan/config_snapshot"),
            String,
            queue_size=1,
            latch=True,
        )
        self.resource_previous_system_cpu = self.read_system_cpu_times()
        self.previous_process = None
        self.configured_pid = None
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_resources)

    # Find the planner process whose resource usage should be sampled.
    def planner_process(self):
        candidates = []
        for name in os.listdir("/proc"):
            if not name.isdigit():
                continue
            try:
                with open("/proc/" + name + "/cmdline", "rb") as stream:
                    args = stream.read().decode("utf-8", errors="replace").split("\0")
                if "__name:=" + self.planner_node.rsplit("/", 1)[-1] not in args:
                    continue
                if not any("neupan_node.py" in arg for arg in args):
                    continue
                with open("/proc/" + name + "/stat") as stream:
                    stat = stream.read()
                fields = stat[stat.rindex(")") + 2 :].split()
                candidates.append(
                    (
                        int(name),
                        int(fields[19]),
                        int(fields[11]) + int(fields[12]),
                        max(0, int(fields[21]))
                        * os.sysconf("SC_PAGE_SIZE")
                        / 1048576.0,
                    )
                )
            except (OSError, ValueError, IndexError):
                continue
        return candidates[0] if len(candidates) == 1 else None

    # Publish resource-monitor configuration so logged metrics have an interpretable
    # sampling context.
    # Inputs: pid.
    def publish_config(self, pid):
        if self.configured_pid == pid:
            return
        params = rospy.get_param(self.planner_node, {})
        path = params.get("config_file")
        if not path:
            return
        try:
            with open(path) as stream:
                config = yaml.safe_load(stream)
            checkpoint = params.get("dune_checkpoint")
            digest = None
            if checkpoint and os.path.isfile(checkpoint):
                with open(checkpoint, "rb") as stream:
                    digest = hashlib.sha256(stream.read()).hexdigest()
            self.config_pub.publish(
                json.dumps(
                    {
                        "stamp": rospy.Time.now().to_sec(),
                        "config_file": path,
                        "planner_config": config,
                        "dune_checkpoint": checkpoint,
                        "checkpoint_sha256": digest,
                        "node_parameters": params,
                    },
                    separators=(",", ":"),
                )
            )
            self.configured_pid = pid
        except (OSError, ValueError, yaml.YAMLError) as error:
            rospy.logwarn_throttle(5, "Cannot snapshot NeuPAN config: %s", error)

    # Collect and publish the current resource measurement payload.
    # Inputs: _event.
    def publish_resources(self, _event):
        now = time.monotonic()
        process = self.planner_process()
        cpu_core = None
        if process and self.previous_process:
            old, old_time = self.previous_process
            if process[:2] == old[:2] and now > old_time:
                cpu_core = (
                    100.0
                    * max(0, process[2] - old[2])
                    / os.sysconf("SC_CLK_TCK")
                    / (now - old_time)
                )
        self.previous_process = (process, now) if process else None
        payload = {
            "stamp": rospy.Time.now().to_sec(),
            "pid": process[0] if process else None,
            "planner_process_available": process is not None,
            "cpu_percent": (
                cpu_core / (os.cpu_count() or 1) if cpu_core is not None else None
            ),
            "cpu_core_percent": cpu_core,
            "cpu_percent_basis": "whole_machine",
            "rss_mb": process[3] if process else None,
            "cpu_count": os.cpu_count(),
            **self.system_resource_metrics(),
            **self.host_process_metrics(),
        }
        self.resource_pub.publish(json.dumps(payload, separators=(",", ":")))
        if process:
            self.publish_config(process[0])

    # Read cumulative CPU accounting counters used for interval utilization
    # calculations.
    @staticmethod
    def read_system_cpu_times():
        with open("/proc/stat") as stat:
            # guest times are already included in user/nice; do not double-count.
            return [int(value) for value in stat.readline().split()[1:9]]

    # Compute host resource metrics from the current and previous system samples.
    def system_resource_metrics(self):
        current = self.read_system_cpu_times()
        delta = [
            max(0, a - b) for a, b in zip(current, self.resource_previous_system_cpu)
        ]
        self.resource_previous_system_cpu = current
        total = sum(delta)
        idle = delta[3]
        iowait = delta[4]
        with open("/proc/meminfo") as meminfo:
            memory = {
                line.split()[0].rstrip(":"): int(line.split()[1]) for line in meminfo
            }
        memory_total = memory["MemTotal"]
        memory_used = max(0, memory_total - memory["MemAvailable"])
        return {
            "system_cpu_percent": (
                100.0 * (total - idle - iowait) / total if total else 0.0
            ),
            "system_cpu_iowait_percent": 100.0 * iowait / total if total else 0.0,
            "system_memory_percent": 100.0 * memory_used / memory_total,
            "system_memory_total_mb": memory_total / 1024.0,
            "system_memory_used_mb": memory_used / 1024.0,
            "system_memory_available_mb": memory["MemAvailable"] / 1024.0,
            "system_swap_used_mb": max(0, memory["SwapTotal"] - memory["SwapFree"])
            / 1024.0,
        }

    # Collect the supported process-level CPU/memory evidence for diagnostics.
    @staticmethod
    def host_process_metrics():
        path = "/home/nuc/lxw/neupan_ws/.runtime_diagnostics/host_processes.json"
        try:
            with open(path) as stream:
                snapshot = json.load(stream)
            age = time.time() - float(snapshot["stamp"])
            return {
                "host_process_snapshot_age_sec": age,
                "host_process_snapshot_valid": 0.0 <= age <= 5.0,
                "host_process_cpu_percent_basis": snapshot.get("cpu_percent_basis"),
                "host_top_processes": (
                    snapshot["top_processes"] if 0.0 <= age <= 5.0 else []
                ),
            }
        except (OSError, ValueError, TypeError, KeyError):
            return {"host_process_snapshot_valid": False, "host_top_processes": []}


if __name__ == "__main__":
    rospy.init_node("system_resource_monitor")
    SystemResourceMonitor()
    rospy.spin()
