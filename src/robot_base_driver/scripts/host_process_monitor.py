#!/usr/bin/python3
# Module contract: Read-only host process sampler for resource diagnosis. Process
# discovery and metric collection must not control or terminate vehicle processes.

"""Publish a host-wide process CPU snapshot for the ROS container to read."""
import json
import os
import time

OUTPUT = '/home/nuc/lxw/neupan_ws/.runtime_diagnostics/host_processes.json'


# Read the supported process information from the host process table.
def read_processes():
    processes = {}
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            with open('/proc/' + pid + '/stat') as stream:
                stat = stream.read()
            end = stat.rindex(')')
            fields = stat[end + 2 :].split()
            with open('/proc/' + pid + '/cmdline', 'rb') as stream:
                args = stream.read(4096).decode('utf-8', errors='replace').split('\0')
            # Record executable/script identity, not arbitrary arguments or credentials.
            command = args[0] or stat[stat.index('(') + 1 : end]
            if (
                'python' in os.path.basename(command)
                and len(args) > 1
                and args[1].endswith('.py')
            ):
                command += ' ' + args[1]
            processes[int(pid)] = {
                'start_ticks': int(fields[19]),
                'cpu_ticks': int(fields[11]) + int(fields[12]),
                'rss_mb': max(0, int(fields[21]))
                * os.sysconf('SC_PAGE_SIZE')
                / 1048576.0,
                'command': command,
                'state': fields[0],
                'threads': int(fields[17]),
            }
        except (OSError, ValueError, IndexError):
            continue
    return processes


# Collect and emit host process metrics through the tool's configured output path.
def main():
    os.makedirs(os.path.dirname(OUTPUT), exist_ok=True)
    previous = read_processes()
    previous_time = time.monotonic()
    ticks_per_second = os.sysconf('SC_CLK_TCK')
    cpu_count = os.cpu_count() or 1
    while True:
        time.sleep(1.0)
        current = read_processes()
        now = time.monotonic()
        elapsed = now - previous_time
        rows = []
        for pid, process in current.items():
            old = previous.get(pid)
            if old is None or old['start_ticks'] != process['start_ticks']:
                continue
            cpu = max(0, process['cpu_ticks'] - old['cpu_ticks']) / ticks_per_second
            rows.append(
                {
                    'pid': pid,
                    'command': process['command'],
                    'cpu_percent': round(100.0 * cpu / elapsed / cpu_count, 2),
                    'cpu_core_percent': round(100.0 * cpu / elapsed, 2),
                    'rss_mb': round(process['rss_mb'], 2),
                    'state': process['state'],
                    'threads': process['threads'],
                }
            )
        rows.sort(key=lambda row: (row['cpu_percent'], row['rss_mb']), reverse=True)
        snapshot = {
            'stamp': time.time(),
            'scope': 'host',
            'sample_interval_sec': elapsed,
            'cpu_count': cpu_count,
            'cpu_percent_basis': 'whole_machine',
            'top_processes': rows[:10],
        }
        temporary = OUTPUT + '.tmp'
        with open(temporary, 'w') as stream:
            json.dump(snapshot, stream, separators=(',', ':'))
        os.replace(temporary, OUTPUT)
        previous, previous_time = current, now


if __name__ == '__main__':
    main()
