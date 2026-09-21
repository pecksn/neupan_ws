#!/usr/bin/env python3
"""Run rosbag record inside a Beijing-time rosbagYYYYMMDD_HHMMSS directory."""

import argparse
import datetime
import os
import signal
import subprocess
import sys


def safe_name(value):
    value = os.path.basename(value.rstrip("/"))
    safe = "".join(char if char.isalnum() or char in "-_" else "_" for char in value)
    return safe.strip("_") or "recording"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-dir", default="/home/nuc/lxw/neupan_ws/rosbags")
    parser.add_argument("--prefix", default="open_loop")
    parser.add_argument("topics", nargs="+")
    args = parser.parse_args()

    timezone = datetime.timezone(datetime.timedelta(hours=8))
    now = datetime.datetime.now(timezone)
    output_dir = os.path.join(
        os.path.abspath(os.path.expanduser(args.base_dir)),
        "rosbag{}".format(now.strftime("%Y%m%d_%H%M%S")),
    )
    os.makedirs(output_dir, exist_ok=True)
    output_prefix = os.path.join(output_dir, safe_name(args.prefix))
    command = ["rosbag", "record", "-o", output_prefix] + args.topics
    print("rosbag output directory: {}".format(output_dir), flush=True)
    process = subprocess.Popen(command, preexec_fn=os.setsid)

    def stop(_signum, _frame):
        if process.poll() is None:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    return process.wait()


if __name__ == "__main__":
    sys.exit(main())
