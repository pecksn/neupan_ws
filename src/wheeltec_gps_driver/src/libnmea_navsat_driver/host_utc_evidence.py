"""Read host NTP evidence before entering Docker. Never adjust either clock."""
import argparse
import datetime
import ipaddress
import json
import math
import os
from pathlib import Path
import re
import subprocess
import time


def query(*args):
    return subprocess.check_output(args,text=True,timeout=3,env=dict(os.environ,LC_ALL='C'))


def seconds(text):
    units={'s':1.,'ms':.001,'us':1.e-6,'min':60.,'h':3600.}
    parts=re.findall(r'([+-]?\d+(?:\.\d+)?)\s*(min|ms|us|s|h)',text)
    if not parts:raise ValueError('unsupported time value: '+text)
    return sum(float(value)*units[unit] for value,unit in parts)


def validate(evidence, max_capture_age=120.):
    """Fail closed on stale snapshot, reboot, clock step or unqualified NTP."""
    try:
        age=(time.monotonic_ns()-evidence['monotonic_ns'])*1.e-9
        elapsed=(time.time_ns()-evidence['utc_ns'])*1.e-9
        return bool(evidence['qualified'] is True and 0<=age<=max_capture_age
            and abs(elapsed-age)<.1
            and 0<=evidence['sample_age_sec']+age<=evidence['max_sample_age_sec']
            and evidence['boot_id']==Path('/proc/sys/kernel/random/boot_id').read_text().strip())
    except (KeyError,TypeError,ValueError,OSError):return False


def collect():
    evidence=dict(utc_ns=time.time_ns(),monotonic_ns=time.monotonic_ns(),
        boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),qualified=False,
        scope='Independent host NTP snapshot; receiver FINE GPST/NMEA consistency remains separately required')
    try:
        sync=query('timedatectl','show','-p','NTPSynchronized','--value').strip()
        properties=query('timedatectl','show-timesync','--all')
        status=query('timedatectl','timesync-status')
        evidence.update(ntp_synchronized=sync,properties=properties,status=status)
        props=dict(line.split('=',1) for line in properties.splitlines() if '=' in line)
        message=props.get('NTPMessage','')
        server=ipaddress.ip_address(props['ServerAddress'])
        stamp=re.search(r'DestinationTimestamp=(\w+ \d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})',message).group(1)
        sampled=datetime.datetime.strptime(stamp,'%a %Y-%m-%d %H:%M:%S').astimezone().timestamp()
        offset=seconds(re.search(r'^\s*Offset:\s*(.+)$',status,re.M).group(1))
        delay=seconds(re.search(r'^\s*Delay:\s*(.+)$',status,re.M).group(1))
        root=seconds(re.search(r'^\s*Root distance:\s*(.+?)\s*\(',status,re.M).group(1))
        age=time.time()-sampled
        max_age=min(3600.,max(90.,2*seconds(props['PollIntervalUSec'])+60.))
        stratum=int(re.search(r'Stratum=(\d+)',message).group(1))
        qualified=(sync=='yes' and not server.is_loopback and not server.is_unspecified
            and 'Leap=0,' in message and 'Mode=4,' in message and 'Ignored=no' in message
            and 1<=stratum<=15 and abs(offset)<=.1 and 0<=delay<=1. and 0<=root<=.25
            and 0<=age<=max_age and all(math.isfinite(v) for v in (offset,delay,root,age)))
        evidence.update(qualified=qualified,server=str(server),offset_sec=offset,delay_sec=delay,
            root_distance_sec=root,sample_age_sec=age,max_sample_age_sec=max_age)
    except (OSError,ValueError,KeyError,AttributeError,subprocess.SubprocessError) as error:
        evidence['error']=str(error)[:300]
    return evidence


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True)
    parser.add_argument('--require-valid',action='store_true');args=parser.parse_args()
    data=collect();target=Path(args.output);target.parent.mkdir(parents=True,exist_ok=True)
    temp=target.with_suffix('.tmp');temp.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n');temp.replace(target)
    print(json.dumps(data,ensure_ascii=False))
    if args.require_valid and not validate(data):raise SystemExit('Host UTC evidence unavailable/unqualified: keep transport candidate disabled')
