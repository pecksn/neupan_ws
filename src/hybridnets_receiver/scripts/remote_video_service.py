#!/usr/bin/python3
"""Supervise ONLY a local ROS master or the raw-video launch inside Neupan.

Existing masters/video owners are never killed or taken over. Stop requests
target a locked, identity-checked service PID, not a name-based process kill.
"""
import fcntl
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
import xmlrpc.client

SCRIPT = str(Path(__file__).resolve())
RUNTIME = Path('/run/neupan-remote-video')
MASTER = 'http://127.0.0.1:11311'


class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = .5
        return connection


def master_pid():
    try:
        with xmlrpc.client.ServerProxy(MASTER, transport=TimeoutTransport()) as proxy:
            code, _, pid = proxy.getPid('/remote_video_service')
        return pid if code == 1 else None
    except (OSError, xmlrpc.client.Error):
        return None


def video_busy():
    # Do not let rospy.init_node replace a live, same-name node either.
    try:
        with xmlrpc.client.ServerProxy(MASTER, transport=TimeoutTransport()) as proxy:
            code, _, uri = proxy.lookupNode('/remote_video_service', '/rtp_camera_receiver')
        if code == 1:
            with xmlrpc.client.ServerProxy(uri, transport=TimeoutTransport()) as node:
                if node.getPid('/remote_video_service')[0] == 1:
                    return True
    except (OSError, xmlrpc.client.Error):
        pass
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        try:
            probe.bind(('0.0.0.0', 5000))
        except OSError:
            return True
    return False


def stop_child(child):
    if child is None or child.poll() is not None:
        return
    for sig, timeout in ((signal.SIGINT, 20), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
        try:
            os.killpg(child.pid, sig)  # Only our start_new_session child.
            child.wait(timeout=timeout)
            return
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            pass


def supervise(mode, stopped):
    child = None
    owner = None
    previous_notice = None
    def notice(message):
        nonlocal previous_notice
        if message != previous_notice:
            print(message, flush=True)
            previous_notice = message
    try:
        while not stopped.is_set():
            pid = master_pid()
            if child is not None and child.poll() is not None:
                notice('Owned %s exited; retrying' % mode)
                child = None
                if stopped.wait(2):
                    break
            if mode == 'master':
                if child is None and pid is None:
                    child = subprocess.Popen(['/usr/bin/python3', '/opt/ros/noetic/bin/roscore', '-p', '11311'],
                                             start_new_session=True)
                    notice('Started independent ROS master; no vehicle nodes')
                elif child is None:
                    notice('Existing ROS master detected; leaving it untouched')
            else:
                if child is not None and pid != owner:
                    stop_child(child)
                    child = None
                    notice('ROS master changed/lost; reconnecting raw video')
                if child is None:
                    if pid is None:
                        notice('Waiting for ROS master')
                    elif not Path('/dev/video0').exists():
                        notice('Waiting for /dev/video0')
                    elif video_busy():
                        notice('Existing video node/UDP 5000 owner; leaving it untouched')
                    else:
                        child = subprocess.Popen(['/usr/bin/python3', '/opt/ros/noetic/bin/roslaunch',
                            '--wait', '--sigint-timeout', '8', '--sigterm-timeout', '3',
                            'hybridnets_receiver', 'video_only.launch'], start_new_session=True)
                        owner = pid
                        notice('Started raw video only: UDP 5000 -> ROS image and /dev/video0')
            stopped.wait(1)
    finally:
        stop_child(child)


def wait_for_release(mode, timeout=30.):
    """Wait for the old container supervisor, not just the docker client."""
    deadline=time.monotonic()+timeout
    try:
        with (RUNTIME/(mode+'.pid')).open('r') as handle:
            while True:
                try:
                    fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
                    return
                except BlockingIOError:
                    if time.monotonic()>=deadline:
                        raise TimeoutError('old %s supervisor has not stopped' % mode)
                    time.sleep(.1)
    except FileNotFoundError:
        return


def stop_owned(mode):
    try:
        pid = int((RUNTIME / (mode + '.pid')).read_text().strip())
        command = Path('/proc/%d/cmdline' % pid).read_bytes().split(b'\0')
        if SCRIPT.encode() in command and mode.encode() in command:
            os.kill(pid, signal.SIGTERM)
            wait_for_release(mode)
    except (FileNotFoundError, ProcessLookupError):
        pass


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in ('master', 'video', 'stop-master', 'stop-video'):
        raise SystemExit('Expected master, video, stop-master or stop-video')
    mode = sys.argv[1]
    if mode.startswith('stop-'):
        stop_owned(mode[5:])
        return
    RUNTIME.mkdir(parents=True, exist_ok=True)
    with (RUNTIME / (mode + '.pid')).open('a+') as handle:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        handle.seek(0)
        handle.truncate()
        handle.write(str(os.getpid()))
        handle.flush()
        stopped = threading.Event()
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda *_: stopped.set())
        supervise(mode, stopped)


if __name__ == '__main__':
    main()
