#!/usr/bin/env python3
"""Private-master Gazebo outdoor regression, sensor bridge and result recorder.

No serial ports, real base driver, real GNSS, Livox or GPU receiver are started.
The real OSM executable, outdoor connection helper, model and solver are used.
"""
import argparse
import copy
from datetime import datetime, timedelta, timezone
import json
import math
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
import xmlrpc.client
import xml.etree.ElementTree as ET


SIMULATION = Path(__file__).resolve().parents[1]
DEFAULT_RESULTS = SIMULATION.parents[3] / 'videos' / 'outdoor_simulation'
PREFIX = '/outdoor_sim/'


def production_sensor_settings():
    """Read, never edit, the real-car mount, LIO blind/extrinsics and scan cut."""
    import yaml
    launch = ET.parse(SIMULATION.parent/'lxw_car/launch/full_system_run.launch').getroot()
    scan = next(node for node in launch.findall('node') if node.get('name') == 'pointcloud_to_laserscan')
    scan_params = {}
    for param in scan.findall('param'):
        value = param.get('value')
        scan_params[param.get('name')] = (value if param.get('name') == 'target_frame' else
            value == 'true' if value in ('true', 'false') else float(value))
    mounts = {name: next(node for node in launch.findall('node') if node.get('name') == name).get('args')
              for name in ('base_to_livox', 'base_to_body')}
    # A later real-car mount edit must not silently reuse a mismatched model.
    model = ET.parse(SIMULATION/'urdf/outdoor_lxwcar.urdf.xacro').getroot()
    for source, joints in (('base_to_livox', ('livox_joint', 'laser_joint')),
                           ('base_to_body', ('body_joint',))):
        configured = list(map(float, mounts[source].split()[:6]))
        for name in joints:
            origin = next(j for j in model.findall('joint') if j.get('name') == name).find('origin')
            xyz = list(map(float, origin.get('xyz').split()))
            rpy = list(map(float, origin.get('rpy', '0 0 0').split()))
            if any(abs(a-b) > 1.e-8 for a, b in zip(xyz+rpy[::-1], configured)):
                raise ValueError('Simulation '+name+' no longer matches production '+source)
    with (SIMULATION.parents[2]/'FAST_LIO/config/mid360.yaml').open() as stream:
        lio = yaml.safe_load(stream)
    return {'scan': scan_params, 'mounts': mounts,
            'lio_blind_m': float(lio['preprocess']['blind']),
            'lidar_to_imu_translation': lio['mapping']['extrinsic_T'],
            'lidar_to_imu_rotation': lio['mapping']['extrinsic_R']}


def production_planner_settings():
    """Resolve the current default outdoor OSM branch, without editing it."""
    import re
    import yaml
    source = SIMULATION.parent/'lxw_car/launch/full_system_run.launch'
    root = ET.parse(source).getroot()
    defaults = {a.get('name'): a.get('default', a.get('value')) for a in root.findall('arg')}
    candidates = [n for n in root.iter('node') if n.get('type') == 'neupan_node.py'
                  and any(p.get('name') == 'outdoor_reference_spacing_m' for p in n.findall('param'))]
    if len(candidates) != 1:
        raise ValueError('Cannot uniquely resolve default outdoor OSM NeuPAN branch')
    def resolve(value):
        for _ in range(8):
            previous = value
            value = value.replace('$(find neupan_ros)', str(SIMULATION.parents[1]))
            value = re.sub(r'\$\(arg ([^)]+)\)', lambda m: defaults[m[1]], value)
            if previous == value: break
        if '$(' in value: raise ValueError('Unsupported production substitution: '+value)
        return yaml.safe_load(value)
    params = {}
    for p in candidates[0].findall('param'):
        if p.get('name') == 'outdoor_start_connector_enabled':
            params[p.get('name')] = True  # This simulation uses the OSM input branch.
        elif p.get('name') not in ('osm_file', 'map_origin_lat', 'map_origin_lon',
                                    'camera_right_guard_enabled', 'camera_right_fallback_topic'):
            params[p.get('name')] = resolve(p.get('value'))
    params['camera_right_guard_enabled'] = False  # No synthetic camera steering input.
    return {'launch': str(source), 'parameters': params}


def save_json(path, data):
    temporary = path.with_suffix('.json.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2))
    os.replace(str(temporary), str(path))


def stop_process(process):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5)


def cleanup_simulation_processes(log_dir):
    """Reap ROS children that started their own process groups, scoped by env.

    roslaunch can exit before an internally setsid'ed rospy child. Never match
    node names or generic ros/python commands: only our unique ROS_LOG_DIR.
    """
    token = b'ROS_LOG_DIR='+str(log_dir).encode()
    victims = {}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            if token not in (entry/'environ').read_bytes().split(b'\0'):
                continue
            start = (entry/'stat').read_text().rsplit(')', 1)[1].split()[19]
            victims[int(entry.name)] = start
        except (OSError, IndexError):
            continue
    for signum in (signal.SIGTERM, signal.SIGKILL):
        for pid, start in victims.items():
            try:
                current = (Path('/proc')/str(pid)/'stat').read_text().rsplit(')', 1)[1].split()[19]
                if current == start:
                    os.kill(pid, signum)
            except (OSError, IndexError):
                pass
        if victims and signum == signal.SIGTERM:
            time.sleep(.3)


def run_cases(args):
    scenarios = ('wide', 'narrow', 'unknown', 'obstacle') if args.scenario == 'all' else (args.scenario,)
    if args.road_width is not None and 'narrow' in scenarios:
        raise ValueError('custom width requires --scenario wide/unknown/obstacle')
    stamp = datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
    root = Path(args.output_dir).resolve() if args.output_dir else DEFAULT_RESULTS/stamp
    root.mkdir(parents=True, exist_ok=True)
    results = []
    print('Results: '+str(root), flush=True)
    sensors = production_sensor_settings()
    planner = production_planner_settings()
    for scenario in scenarios:
        folder = root/scenario
        folder.mkdir(parents=True, exist_ok=True)
        with socket.socket() as ros_port, socket.socket() as gazebo_port:
            ros_port.bind(('127.0.0.1', 0))
            gazebo_port.bind(('127.0.0.1', 0))
            port, gzport = ros_port.getsockname()[1], gazebo_port.getsockname()[1]
        env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:'+str(port),
                   ROS_HOSTNAME='127.0.0.1', GAZEBO_MASTER_URI='http://127.0.0.1:'+str(gzport),
                   ROS_LOG_DIR=str(folder/'ros_logs'), ROS_NAMESPACE='',
                   OMP_NUM_THREADS='1', MKL_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1')
        env.pop('ROS_IP', None)
        width = args.road_width if args.road_width is not None else (3. if scenario == 'narrow' else 12.)
        world = getattr(args, 'world_file', str(SIMULATION/'worlds/outdoor_roads.world'))
        osm = getattr(args, 'osm_file', str(SIMULATION/'maps/outdoor_test.osm'))
        if args.road_width is not None and not hasattr(args, 'world_file'):
            world, osm = width_inputs(root/'inputs', width)
        save_json(folder/'run_manifest.json', {'scenario': scenario,
            'ros_master_uri': env['ROS_MASTER_URI'], 'gazebo_master_uri': env['GAZEBO_MASTER_URI'],
            'world': world, 'osm': osm, 'road_width_m': width,
            'turn_only': args.turn_only, 'initial_pose': [10. if args.turn_only else 0., 0. if scenario != 'narrow' else 25., 0.],
            'model': planner['parameters']['dune_checkpoint'], 'production_config': planner['parameters']['config_file'],
            'ground_truth_localization': True, 'record_bag': args.record_bag,
            'sensor_parameters': sensors, 'sensor_model': 'horizontal_ray_to_imu_cloud_to_production_scan',
            'real_hardware_started': False, 'connection_planner': args.connection_planner})
        # The real pointcloud_to_laserscan executable reads the exact production
        # parameters. Keep this generated test input beside results, not in src.
        save_json(folder/'scan_parameters.json', sensors['scan'])
        launch = ['roslaunch', '-p', str(port), 'neupan_ros', 'outdoor_simulation.launch',
                  'scenario:='+scenario, 'output_dir:='+str(folder),
                  'dune_checkpoint:='+planner['parameters']['dune_checkpoint'],
                  'connection_planner:='+args.connection_planner,
                  'road_width:='+str(width), 'turn_only:='+str(args.turn_only).lower(),
                  'world_file:='+world, 'osm_file:='+osm,
                  'scan_parameters:='+str(folder/'scan_parameters.json'),
                  'gui:='+str(args.gui).lower(), 'rviz:='+str(args.rviz).lower()]
        process = bag = None
        print('Starting '+scenario+' on private ROS master '+env['ROS_MASTER_URI'], flush=True)
        try:
            with (folder/'console.log').open('w') as console:
                process = subprocess.Popen(launch, env=env, stdout=console, stderr=subprocess.STDOUT,
                                           start_new_session=True)
                if args.record_bag:
                    ready_deadline = time.monotonic()+20.
                    while process.poll() is None and time.monotonic() < ready_deadline:
                        try:
                            if xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']).getPid('/outdoor_sim_recorder')[0] == 1:
                                break
                        except (OSError, xmlrpc.client.Error):
                            pass
                        time.sleep(.1)
                    topics = [PREFIX+t for t in ('odom', 'odometry/global', 'gps/fix', 'scan_raw', 'cloud_registered_body', 'scan',
                        'goal', 'cmd_vel', 'goal_arrived', 'osm_global_path', 'neupan_initial_path',
                        'neupan_ref_state', 'neupan_plan', 'neupan/runtime', 'neupan/route_connection',
                        'neupan/scan_health', 'neupan/failure', 'tf', 'tf_static', 'scan_mode')]+['/clock']
                    bag = subprocess.Popen(['rosbag', 'record', '--lz4', '-O', str(folder/'simulation.bag')]+topics,
                        env=env, stdout=console, stderr=subprocess.STDOUT, start_new_session=True)
                deadline = time.monotonic()+args.timeout
                last_report = 0.
                while process.poll() is None and not (folder/'result.json').exists() and time.monotonic() < deadline:
                    if time.monotonic()-last_report >= 20:
                        print('  '+scenario+': running; '+str(folder/'console.log'), flush=True)
                        last_report = time.monotonic()
                    time.sleep(.25)
        finally:
            # Finalise the bag while the private master is still alive.
            stop_process(bag)
            stop_process(process)
            cleanup_simulation_processes(folder/'ros_logs')
        if (folder/'result.json').exists():
            result = json.loads((folder/'result.json').read_text())
        else:
            result = {'scenario': scenario, 'passed': False, 'reason': 'launch_exited_or_wall_timeout',
                      'console': str(folder/'console.log')}
            save_json(folder/'result.json', result)
        results.append(result)
        print(scenario+': '+('PASS' if result['passed'] else 'FAIL')+' / '+result['reason'], flush=True)
    save_json(root/'summary.json', {'connection_planner': args.connection_planner,
        'results': results, 'all_passed': all(r['passed'] for r in results)})
    lines = ['# 室外仿真结果', '', '连接算法：`'+args.connection_planner+'`。', '',
             '| 场景 | 结果 | 说明 |', '|---|---|---|']
    for result in results:
        lines.append('| {} | {} | {} |'.format(result['scenario'], 'PASS' if result['passed'] else 'FAIL', result['reason']))
    lines += ['', '使用生产 OSM A*、室外配置、当前室外模型与 NeuPAN 求解器；定位为仿真真值，不代表实车 RTK/FAST-LIO 验收。',
              '雷达安装与过滤参数取自实车配置；0.5 m 近距预处理 → IMU 点云 → 生产扫描节点（base_link、range_min=0.4 m）。',
              '仅额外缓冲区内的近场未知采样点可保持未验证；车身进入未知区或碰到已知障碍仍拒绝，不宣称完整 12 cm 余量已观测。',
              '每个场景目录保存 overview.mp4（俯视轨迹，不是相机视频）、navigation.jsonl、console.log 和 result.json；可选 simulation.bag。']
    (root/'summary.md').write_text('\n'.join(lines)+'\n')
    print('Summary: '+str(root/'summary.md'), flush=True)
    return 0 if all(r['passed'] for r in results) else 1


def width_inputs(folder, width):
    """Generate only test copies; physical inside wall faces match OSM width."""
    if not math.isfinite(width) or not 3. <= width <= 12.:
        raise ValueError('comparison width must be between 3 and 12 m')
    folder.mkdir(parents=True, exist_ok=True)
    world = ET.parse(SIMULATION/'worlds/outdoor_roads.world')
    for model in world.getroot().find('world').findall('model'):
        name = model.get('name')
        if name in ('wide_north', 'wide_south'):
            pose = model.find('pose').text.split()
            pose[1] = str((1. if name == 'wide_north' else -1.)*(width/2+.1))
            model.find('pose').text = ' '.join(pose)
        elif name in ('wide_road', 'wide_west', 'wide_east'):
            for size in model.findall('.//box/size'):
                xyz = size.text.split()
                xyz[1] = str(width)
                size.text = ' '.join(xyz)
    osm = ET.parse(SIMULATION/'maps/outdoor_test.osm')
    for tag in osm.getroot().find("way[@id='101']").findall('tag'):
        if tag.get('k') == 'width':
            tag.set('v', str(width))
    world_path, osm_path = folder/'road.world', folder/'road.osm'
    world.write(str(world_path), encoding='utf-8', xml_declaration=True)
    osm.write(str(osm_path), encoding='utf-8', xml_declaration=True)
    return str(world_path), str(osm_path)


def compare_widths(args):
    """Paired closed-loop tests, threshold refinement and bounded cleanup."""
    stamp = datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
    root = Path(args.output_dir).resolve() if args.output_dir else DEFAULT_RESULTS/(stamp+'_width_comparison')
    if root.exists() and any(root.iterdir()):
        raise ValueError('comparison output must be a new empty directory')
    root.mkdir(parents=True, exist_ok=True)
    work = root/'_work'
    work.mkdir()
    records = []
    methods = ('dubins', 'hybrid_astar')
    print('Comparison: '+str(root), flush=True)

    def case(method, width, repeat=1):
        width = round(width, 3)
        for record in records:
            if (record['method'], record['road_width_m'], record['repeat']) == (method, width, repeat):
                return record
        name = '{}_{:.2f}m_r{}'.format(method, width, repeat)
        folder = work/name
        world, osm = width_inputs(work/('inputs_{:.2f}'.format(width)), width)
        config = copy.copy(args)
        config.output_dir, config.scenario = str(folder), 'wide'
        config.connection_planner, config.road_width, config.turn_only = method, width, True
        config.world_file, config.osm_file = world, osm
        config.timeout = min(args.timeout, 100.)
        run_cases(config)
        result = json.loads((folder/'wide/result.json').read_text())
        completed = (result.get('passed') is True and result.get('reason') == 'turn_completed' and
                     result.get('min_command_speed_mps', -1.) >= -.01 and
                     result.get('min_wall_clearance_m', -1.) >= 0.)
        record = dict(result, method=method, road_width_m=width, repeat=repeat,
                      turn_completed=completed, artifact_name=name)
        records.append(record)
        save_json(root/'comparison.json', {'complete': False, 'results': records})
        print('WIDTH {:.2f}m {} repeat {}: {}'.format(width, method, repeat,
              'COMPLETED' if completed else result.get('reason')), flush=True)
        return record

    widths = sorted(set(args.widths), reverse=True)
    for width in widths:
        for method in methods:
            case(method, width)
    for method in methods:
        # Refine only brackets with a completed and a narrower rejected test.
        for _ in range(8):
            good = [r['road_width_m'] for r in records if r['method'] == method and r['turn_completed']]
            if not good:
                break
            best = min(good)
            lower = [r['road_width_m'] for r in records if r['method'] == method and
                     not r['turn_completed'] and r['road_width_m'] < best]
            if not lower or best-max(lower) <= args.width_step+1.e-6:
                break
            middle = round((best+max(lower))/2/args.width_step)*args.width_step
            if not max(lower) < middle < best:
                break
            for paired_method in methods:
                case(paired_method, middle)
    boundaries = {}
    for method in methods:
        good = [r['road_width_m'] for r in records if r['method'] == method and r['turn_completed']]
        if not good:
            boundaries[method] = None
            continue
        best = min(good)
        lower = [r['road_width_m'] for r in records if r['method'] == method and
                 not r['turn_completed'] and r['road_width_m'] < best]
        boundary = [best]+([max(lower)] if lower else [])
        boundaries[method] = {'narrowest_completed_m': best, 'adjacent_unsuccessful_m': max(lower) if lower else None}
        for width in boundary:
            for repeat in range(2, args.repeats+1):
                case(method, width, repeat)
        repeats = [r for r in records if r['method'] == method and r['road_width_m'] == best]
        boundaries[method]['completed_repeats'] = sum(r['turn_completed'] for r in repeats)
        boundaries[method]['total_repeats'] = len(repeats)
        boundaries[method]['repeat_consistent'] = all(r['turn_completed'] for r in repeats)
        boundaries[method]['narrowest_repeat_verified_m'] = None
        # A one-off tight success is not a stable threshold. Verify the next
        # wider completed case rather than advertising the lucky run as best.
        wider = sorted({r['road_width_m'] for r in records if r['method'] == method and
                        r['turn_completed'] and r['road_width_m'] >= best})
        for width in wider:
            for repeat in range(2, args.repeats+1):
                case(method, width, repeat)
            repeated = [r for r in records if r['method'] == method and r['road_width_m'] == width]
            if all(r['turn_completed'] for r in repeated):
                boundaries[method]['narrowest_repeat_verified_m'] = width
                break
    # Keep numerical evidence for every run; only retain boundary videos/logs.
    selected = {r['artifact_name'] for r in records if r.get('reason') in
                ('vehicle_footprint_crossed_road_wall', 'neupan_exception', 'comparison_initial_pose_mismatch')}
    for method, boundary in boundaries.items():
        if boundary:
            for width in (boundary['narrowest_completed_m'], boundary['adjacent_unsuccessful_m'],
                          boundary['narrowest_repeat_verified_m']):
                matches = [r for r in records if r['method'] == method and r['road_width_m'] == width]
                for record in matches:
                    # Keep a second video only if the repeated outcome differs.
                    if record['repeat'] == 1 or record['turn_completed'] != matches[0]['turn_completed']:
                        selected.add(record['artifact_name'])
    for record in records:
        name = record['artifact_name']
        if name in selected:
            for source_name, suffix in (('overview.mp4', '.mp4'), ('navigation.jsonl', '.jsonl')):
                source = work/name/'wide'/source_name
                if source.exists():
                    shutil.copy2(str(source), str(root/(name+suffix)))
            if args.record_bag:
                source = work/name/'wide/simulation.bag'
                if source.exists():
                    shutil.copy2(str(source), str(root/(name+'.bag')))
        record['video'] = name+'.mp4' if (root/(name+'.mp4')).exists() else None
        if not record.get('passed') and record.get('reason') not in ('unexpected_block: no_safe_forward_connection',):
            source = work/name/'wide/console.log'
            if source.exists():
                shutil.copy2(str(source), str(root/(name+'_diagnostic.log')))
        record.pop('console', None)
    # This directory was created exclusively above, never a user rosbag/log root.
    shutil.rmtree(str(work))
    save_json(root/'comparison.json', {'complete': True, 'initial_pose': [10., 0., 0.],
        'return_goal': [0., 0., math.pi], 'search_budget_sec': 1., 'width_step_m': args.width_step,
        'sensor_parameters': production_sensor_settings(), 'boundaries': boundaries, 'results': records,
        'real_hardware_started': False, 'temporary_inputs_and_duplicate_outputs_removed': True})
    lines = ['# 曲线与 Hybrid A* 窄路掉头对比', '',
        '相同静止起点 `(10,0,0)`、返程目标 `(0,0,π)`，无倒车、无预先靠边；使用原室外模型/车辆/扫描配置。',
        'Hybrid A* 搜索预算 1 s；车体不越界且实际抵达才算完成。拒绝后验证持续零速度至少 2 s。', '',
        '| 总路宽 | 算法 | 次数 | 实际完成 | 原因 | 最小边界距离 | 视频 |', '|---|---|---|---|---|---|---|']
    for r in sorted(records, key=lambda r: (-r['road_width_m'], r['method'], r['repeat'])):
        video = '[视频]('+r['video']+')' if r['video'] else '—'
        lines.append('| {:.2f} m | {} | {} | {} | {} | {} | {} |'.format(r['road_width_m'], r['method'],
            r['repeat'], '是' if r['turn_completed'] else '否', r['reason'], r.get('min_wall_clearance_m'), video))
    lines += ['', '## 本次范围内的边界', '']
    for method, boundary in boundaries.items():
        if boundary:
            lines.append('- `{}`：最窄完成 {:.2f} m，复测 {}/{} 次完成；相邻未完成宽度 {} m。'.format(
                method, boundary['narrowest_completed_m'], boundary['completed_repeats'],
                boundary['total_repeats'], boundary['adjacent_unsuccessful_m']))
            lines.append('  重复全部完成的最窄已测宽度：{} m。'.format(boundary['narrowest_repeat_verified_m']))
        else:
            lines.append('- `{}`：本轮宽度均未完成，不能给出最小完成宽度。'.format(method))
    lines += ['', '临界宽度已复测；若重复结果不一致，不能称为稳定最小路宽。所有数值保留在 comparison.json。',
        '这是当前 1 s 搜索预算、候选接入点和中线起步条件下的测试结果，不是车辆物理极限。',
        '二维射线、真值定位和运动学底盘不代表真实 MID360/定位/轮胎动力学；不以此直接进行实车极限掉头。',
        '已删除本轮临时地图/场景、内部启动文件、重复视频和 ROS 内部日志；保留临界视频、轨迹日志及完整数值。历史实车数据未动。']
    (root/'comparison.md').write_text('\n'.join(lines)+'\n')
    print('Comparison report: '+str(root/'comparison.md'), flush=True)
    return 0


def sensor_bridge():
    import rospy
    import numpy as np
    import sensor_msgs.point_cloud2 as pc2
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan, PointCloud2, NavSatFix, NavSatStatus
    from std_msgs.msg import String
    rospy.init_node('outdoor_sim_sensors')
    mode = ['normal']
    odom_pub = rospy.Publisher(PREFIX+'odometry/global', Odometry, queue_size=5)
    gps_pub = rospy.Publisher(PREFIX+'gps/fix', NavSatFix, queue_size=5)
    cloud_pub = rospy.Publisher(PREFIX+'cloud_registered_body', PointCloud2, queue_size=1)
    sensors = production_sensor_settings()
    rotation = np.array(sensors['lidar_to_imu_rotation']).reshape(3, 3)
    translation = np.array(sensors['lidar_to_imu_translation'])

    def odometry(message):
        global_odom = copy.deepcopy(message)
        global_odom.header.frame_id = 'map'  # map->odom is identity in this test.
        global_odom.child_frame_id = 'base_link'
        odom_pub.publish(global_odom)
        fix = NavSatFix()
        fix.header = global_odom.header
        fix.header.frame_id = 'gps_link'
        fix.status.status, fix.status.service = NavSatStatus.STATUS_GBAS_FIX, NavSatStatus.SERVICE_GPS
        scale = 6378137.*math.pi/180.
        fix.latitude = 23.0398+message.pose.pose.position.y/scale
        fix.longitude = 113.39002+message.pose.pose.position.x/(scale*math.cos(math.radians(23.0398)))
        fix.position_covariance = [.01, 0, 0, 0, .01, 0, 0, 0, .02]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        gps_pub.publish(fix)

    def laser(message):
        if mode[0] == 'dropout':
            return
        ranges = np.asarray(message.ranges)
        angles = message.angle_min+np.arange(len(ranges))*message.angle_increment
        keep = (np.isfinite(ranges) & (ranges > max(message.range_min, sensors['lio_blind_m'])) &
                (ranges <= message.range_max))
        if mode[0] == 'unknown':
            keep[:] = False
        points = np.column_stack((ranges[keep]*np.cos(angles[keep]),
                                  ranges[keep]*np.sin(angles[keep]), np.zeros(np.count_nonzero(keep))))
        # Mirrors FAST-LIO's publish_frame_body only; no simulated LIO filter
        # or idealised "inf means free" scan is injected into the planner.
        points = points @ rotation.T+translation
        header = copy.deepcopy(message.header)
        header.frame_id = 'body'
        cloud_pub.publish(pc2.create_cloud_xyz32(header, points.tolist()))

    rospy.Subscriber(PREFIX+'odom', Odometry, odometry, queue_size=5)
    rospy.Subscriber(PREFIX+'scan_raw', LaserScan, laser, queue_size=1)
    rospy.Subscriber(PREFIX+'scan_mode', String, lambda msg: mode.__setitem__(0, msg.data), queue_size=1)
    rospy.spin()


class ScenarioTest:
    def __init__(self):
        import cv2
        import numpy as np
        import rospy
        from geometry_msgs.msg import PoseStamped, Twist
        from gazebo_msgs.msg import ModelStates
        from nav_msgs.msg import Odometry, Path as RosPath
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
        self.cv2, self.np, self.rospy = cv2, np, rospy
        self.PoseStamped, self.Twist, self.String = PoseStamped, Twist, String
        rospy.init_node('outdoor_sim_test')
        self.scenario = rospy.get_param('~scenario', 'wide')
        self.center = float(rospy.get_param('~start_y', 0.))
        self.width = float(rospy.get_param('~road_width', 3. if self.scenario == 'narrow' else 12.))
        self.turn_only = bool(rospy.get_param('~turn_only', False))
        self.connection_planner = rospy.get_param('~connection_planner', 'hybrid_astar')
        self.initial_turn_pose = None
        self.min_command_speed = 0.
        self.folder = Path(rospy.get_param('~output_dir'))
        self.folder.mkdir(parents=True, exist_ok=True)
        self.log = (self.folder/'navigation.jsonl').open('w', buffering=1)
        self.lock = threading.RLock()
        self.pose = self.command = self.connection = self.runtime = None
        self.gazebo_pose = None
        self.max_gazebo_error = 0.
        self.arrived = False
        self.scan_received = None
        self.stage, self.stage_started = 'waiting', None
        self.started_wall = time.monotonic()
        self.outbound_arrived = False
        self.seen_connecting = self.seen_completed = False
        self.blocked_since = self.blocked_pose = None
        self.zero_samples = 0
        self.blockers = []
        self.scan_metadata = None
        self.min_clearance = float('inf')
        self.trace, self.paths = [], {}
        self.last_path_signatures = {}
        self.frames, self.done = 0, False
        self.writer = cv2.VideoWriter(str(self.folder/'overview_raw.mp4'), cv2.VideoWriter_fourcc(*'mp4v'),
                                      10., (1100, 650))
        if not self.writer.isOpened():
            raise RuntimeError('cannot open simulation overview video')
        self.goal_pub = rospy.Publisher(PREFIX+'goal', PoseStamped, queue_size=1, latch=True)
        self.mode_pub = rospy.Publisher(PREFIX+'scan_mode', String, queue_size=1, latch=True)
        self.mode_pub.publish(String(data='normal'))
        rospy.Subscriber(PREFIX+'odometry/global', Odometry, self.odometry, queue_size=5)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_state, queue_size=1)
        rospy.Subscriber(PREFIX+'cmd_vel', Twist, self.commands, queue_size=10)
        rospy.Subscriber(PREFIX+'goal_arrived', String, self.arrival, queue_size=1)
        rospy.Subscriber(PREFIX+'scan', LaserScan, self.scan, queue_size=1)
        rospy.Subscriber(PREFIX+'neupan/route_connection', String, self.connection_status, queue_size=10)
        rospy.Subscriber(PREFIX+'neupan/runtime', String, self.runtime_status, queue_size=10)
        rospy.Subscriber(PREFIX+'neupan/failure', String, self.failure, queue_size=1)
        for name in ('osm_global_path', 'neupan_initial_path', 'neupan_plan'):
            rospy.Subscriber(PREFIX+name, RosPath, lambda msg, name=name: self.path(name, msg), queue_size=1)
        rospy.Timer(rospy.Duration(.1), self.tick)
        rospy.on_shutdown(self.close)

    def event(self, kind, data):
        self.log.write(json.dumps({'record_type': kind, 'sim_time': self.rospy.get_time(),
                                   'wall_time': time.time(), 'data': data})+'\n')

    def odometry(self, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        with self.lock:
            self.pose = (p.x, p.y, yaw, message.twist.twist.linear.x)

    def commands(self, message):
        with self.lock:
            self.command = (message.linear.x, message.angular.z)
            self.min_command_speed = min(self.min_command_speed, message.linear.x)
            if self.blocked_since is not None and self.rospy.get_time()-self.blocked_since > .2:
                if abs(message.linear.x) < 1.e-8:
                    self.zero_samples += 1
                else:
                    self.finish(False, 'nonzero_command_after_blocked_route')

    def gazebo_state(self, message):
        if 'outdoor_lxwcar' in message.name:
            p = message.pose[message.name.index('outdoor_lxwcar')].position
            with self.lock:
                self.gazebo_pose = (p.x, p.y)

    def arrival(self, message):
        with self.lock:
            self.arrived = message.data == '1'

    def scan(self, message):
        with self.lock:
            self.scan_received = self.rospy.get_time()
            metadata = {'frame_id': message.header.frame_id, 'range_min': message.range_min,
                        'range_max': message.range_max, 'angle_increment': message.angle_increment,
                        'bins': len(message.ranges)}
            if self.scan_metadata is None:
                self.event('scan_parameters', metadata)
            self.scan_metadata = metadata

    def connection_status(self, message):
        status = json.loads(message.data)
        with self.lock:
            if status != self.connection:
                self.event('route_connection', status)
            self.connection = status
            self.seen_connecting |= status.get('phase') == 'connecting'
            self.seen_completed |= status.get('reason') == 'connection_completed'

    def runtime_status(self, message):
        with self.lock:
            self.runtime = json.loads(message.data)

    def failure(self, message):
        with self.lock:
            self.event('neupan_failure', json.loads(message.data))
            self.finish(False, 'neupan_exception')

    def path(self, name, message):
        points = [(p.pose.position.x, p.pose.position.y) for p in message.poses]
        with self.lock:
            self.paths[name] = points
            signature = tuple((round(x, 3), round(y, 3)) for x, y in points)
            if name != 'neupan_plan' and signature != self.last_path_signatures.get(name):
                self.last_path_signatures[name] = signature
                self.event(name, {'points': points, 'stamp_ns': message.header.stamp.to_nsec()})

    def send_goal(self, x, yaw):
        goal = self.PoseStamped()
        goal.header.frame_id, goal.header.stamp = 'map', self.rospy.Time.now()
        goal.pose.position.x, goal.pose.position.y = x, self.center
        goal.pose.orientation.z, goal.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        self.arrived = False
        self.goal_pub.publish(goal)
        self.event('goal', {'x': x, 'y': self.center, 'yaw': yaw})

    def clearance(self):
        x, y, yaw, _ = self.pose
        corners = self.np.array([[-.14, -.425], [.81, -.425], [.81, .425], [-.14, .425]])
        c, s = math.cos(yaw), math.sin(yaw)
        world = corners @ self.np.array([[c, s], [-s, c]])+[x, y]
        return float(min(self.np.min(world[:, 0]+10), self.np.min(22-world[:, 0]),
                         self.np.min(world[:, 1]-self.center+self.width/2),
                         self.np.min(self.center+self.width/2-world[:, 1])))

    def install_blockers(self):
        from gazebo_msgs.srv import SpawnModel
        from geometry_msgs.msg import Pose
        self.rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=5.)
        spawn = self.rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        # Only installed after outbound arrival: both turning sides are blocked,
        # while the car's current footprint is clear and OSM stays 12 m wide.
        for side in (-1., 1.):
            x, y = self.pose[0]+2., self.pose[1]+side
            name = 'return_blocker_'+('left' if side > 0 else 'right')
            model = ('<sdf version="1.6"><model name="'+name+'"><static>true</static>'
                     '<link name="wall"><collision name="wall"><geometry><box>'
                     '<size>8 0.2 1.2</size></box></geometry></collision>'
                     '<visual name="wall"><geometry><box><size>8 0.2 1.2</size>'
                     '</box></geometry></visual></link></model></sdf>')
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z, pose.orientation.w = x, y, .6, 1.
            response = spawn(name, model, '', pose, 'world')
            if not response.success:
                raise RuntimeError('cannot spawn return obstacle: '+response.status_message)
            self.blockers.append((x-4., y-.1, x+4., y+.1))
        self.event('return_obstacles', {'boxes': self.blockers})

    def tick(self, _event):
        with self.lock:
            if self.done:
                return
            now = self.rospy.get_time()
            if time.monotonic()-self.started_wall > 180:
                self.finish(False, 'scenario_wall_timeout')
                return
            if self.pose is not None:
                self.trace.append(self.pose[:2])
                self.min_clearance = min(self.min_clearance, self.clearance())
                if self.min_clearance < (0. if self.turn_only else -.02):
                    self.finish(False, 'vehicle_footprint_crossed_road_wall')
                    return
                if self.gazebo_pose is not None:
                    self.max_gazebo_error = max(self.max_gazebo_error, math.dist(self.pose[:2], self.gazebo_pose))
            self.event('sample', {'stage': self.stage, 'pose': self.pose, 'command': self.command,
                                  'connection': self.connection, 'runtime': self.runtime})
            self.writer.write(self.render(now))
            self.frames += 1
            if self.stage == 'waiting':
                if (self.pose is not None and self.gazebo_pose is not None and self.scan_received is not None and
                        self.goal_pub.get_num_connections() >= 2):
                    if self.turn_only:
                        self.initial_turn_pose = self.pose
                        if math.dist(self.pose[:2], (10., self.center)) > 1.e-4 or abs(self.pose[2]) > 1.e-4:
                            self.finish(False, 'comparison_initial_pose_mismatch')
                            return
                        self.send_goal(0., math.pi)
                        self.stage, self.stage_started = 'return', now
                    else:
                        self.send_goal(10., 0.)
                        self.stage, self.stage_started = 'outbound', now
            elif self.stage == 'outbound' and self.arrived and math.dist(self.pose[:2], (10., self.center)) < .8:
                self.outbound_arrived = True
                self.stage, self.stage_started = 'settling', now
                self.event('outbound_arrived', {'pose': self.pose})
                if self.scenario == 'unknown':
                    self.mode_pub.publish(self.String(data='unknown'))
                elif self.scenario == 'obstacle':
                    self.install_blockers()
            elif self.stage == 'settling' and now-self.stage_started > 1.:
                self.send_goal(0., math.pi)
                self.stage, self.stage_started = 'return', now
            elif self.stage == 'return':
                phase = (self.connection or {}).get('phase')
                if self.scenario == 'wide' and not (self.turn_only and phase == 'blocked'):
                    if phase == 'blocked':
                        self.finish(False, 'unexpected_block: '+self.connection.get('reason', ''))
                    elif self.arrived and math.dist(self.pose[:2], (0., self.center)) < .8:
                        self.finish(self.seen_connecting and self.seen_completed,
                                    'turn_completed' if self.turn_only else 'round_trip_completed')
                elif phase == 'blocked':
                    if self.connection.get('reason') != 'no_safe_forward_connection':
                        self.finish(False, 'unexpected_stop_reason: '+self.connection.get('reason', ''))
                        return
                    checks = self.connection.get('admission') or {}
                    rejected = checks.get('scan_rejected', {})
                    if ((self.scenario == 'narrow' and not checks.get('road_rejected')) or
                        (self.scenario == 'unknown' and not (rejected.get('swept_body_unobserved') or
                                                           rejected.get('clearance_buffer_unobserved'))) or
                        (self.scenario == 'obstacle' and not rejected.get('observed_obstacle_or_occlusion'))):
                        self.finish(False, 'stop_not_explained_by_expected_geometry_or_observation')
                        return
                    if self.blocked_since is None:
                        self.blocked_since, self.blocked_pose = now, self.pose[:2]
                    elif now-self.blocked_since > 2.:
                        drift = math.dist(self.pose[:2], self.blocked_pose)
                        self.finish(self.zero_samples >= 5 and drift < .05, 'safe_stop_verified')

    def render(self, now):
        cv2, np = self.cv2, self.np
        frame = np.full((650, 1100, 3), (35, 43, 36), np.uint8)
        def pixel(x, y):
            return int(55+(x+12)*27), int(155+(9-(y-self.center))*25)
        a, b = pixel(-10, self.center+self.width/2), pixel(22, self.center-self.width/2)
        cv2.rectangle(frame, a, b, (65, 65, 65), -1)
        cv2.rectangle(frame, a, b, (210, 210, 210), 3)
        cv2.line(frame, pixel(-10, self.center), pixel(22, self.center), (100, 100, 100), 1)
        for x0, y0, x1, y1 in self.blockers:
            cv2.rectangle(frame, pixel(x0, y1), pixel(x1, y0), (60, 80, 235), -1)
        for name, color in (('osm_global_path', (240, 150, 40)), ('neupan_initial_path', (90, 230, 100)),
                            ('neupan_plan', (180, 140, 255))):
            points = self.paths.get(name, [])
            if len(points) >= 2:
                cv2.polylines(frame, [np.array([pixel(x, y) for x, y in points], np.int32)], False, color, 2)
        if len(self.trace) >= 2:
            cv2.polylines(frame, [np.array([pixel(x, y) for x, y in self.trace], np.int32)], False, (50, 220, 255), 2)
        if self.pose:
            x, y, yaw, _ = self.pose
            vertices = np.array([[-.14, -.425], [.81, -.425], [.81, .425], [-.14, .425]])
            c, s = math.cos(yaw), math.sin(yaw)
            corners = vertices @ np.array([[c, s], [-s, c]])+[x, y]
            cv2.fillPoly(frame, [np.array([pixel(*p) for p in corners], np.int32)], (230, 210, 90))
            cv2.arrowedLine(frame, pixel(x, y), pixel(x+1.2*c, y+1.2*s), (40, 50, 255), 2)
        for x, label in ((0., 'START / RETURN'), (10., 'OUTBOUND GOAL')):
            cv2.circle(frame, pixel(x, self.center), 7, (60, 90, 255), 2)
            px, py = pixel(x, self.center)
            cv2.putText(frame, label, (px-40, py+35), cv2.FONT_HERSHEY_SIMPLEX, .45, (240, 240, 240), 1)
        phase, reason = (self.connection or {}).get('phase', 'waiting'), (self.connection or {}).get('reason', '')
        speed = self.command[0] if self.command else 0.
        for y, text in ((30, 'Outdoor simulation: '+self.scenario+' | '+self.connection_planner+' | stage='+self.stage),
                        (62, 'sim={:.1f}s  v={:.2f}m/s  road_width={:.1f}m'.format(now, speed, self.width)),
                        (94, 'connection='+phase+' | '+reason),
                        (625, 'Blue: OSM | Green: accepted reference | Pink: local plan | Yellow: actual trajectory')):
            cv2.putText(frame, text, (22, y), cv2.FONT_HERSHEY_SIMPLEX, .65, (230, 235, 235), 1, cv2.LINE_AA)
        return frame

    def finish(self, passed, reason):
        if self.done:
            return
        if passed and self.max_gazebo_error > .25:
            passed, reason = False, 'gazebo_model_not_following_virtual_base'
        self.done = True
        finished_sim, finished_wall = self.rospy.get_time(), time.monotonic()
        self.event('result', {'passed': passed, 'reason': reason})
        self.writer.release()
        raw, final = self.folder/'overview_raw.mp4', self.folder/'overview.mp4'
        conversion = subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y', '-i', str(raw),
            '-c:v', 'libx264', '-threads', '2', '-preset', 'veryfast', '-pix_fmt', 'yuv420p',
            '-movflags', '+faststart', str(final)], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        video = final if conversion.returncode == 0 else raw
        if video == final:
            raw.unlink()
        result = {'scenario': self.scenario, 'passed': bool(passed), 'reason': reason,
            'outbound_arrived': self.outbound_arrived, 'seen_connection': self.seen_connecting,
            'initial_turn_pose': self.initial_turn_pose, 'min_command_speed_mps': self.min_command_speed,
            'seen_connection_completed': self.seen_completed, 'final_pose': self.pose,
            'min_wall_clearance_m': self.min_clearance if math.isfinite(self.min_clearance) else None,
            'zero_commands_after_block': self.zero_samples, 'connection': self.connection,
            'scan_parameters': self.scan_metadata, 'return_obstacles': self.blockers,
            'sim_time': finished_sim, 'wall_duration_sec': finished_wall-self.started_wall,
            'video_frames': self.frames, 'video': str(video), 'video_fps': 10,
            'max_gazebo_to_odometry_error_m': self.max_gazebo_error,
            'return_goal_error_m': math.dist(self.pose[:2], (0., self.center)) if self.pose else None,
            'ground_truth_localization': True, 'real_hardware_started': False}
        self.log.flush()
        os.fsync(self.log.fileno())
        save_json(self.folder/'result.json', result)
        self.rospy.loginfo('OUTDOOR TEST %s: %s', 'PASS' if passed else 'FAIL', reason)

    def close(self):
        with self.lock:
            if not self.done:
                self.writer.release()
            if not self.log.closed:
                self.log.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('run', 'compare', 'bridge', 'scenario'))
    parser.add_argument('--scenario', choices=('all', 'wide', 'narrow', 'unknown', 'obstacle'), default='all')
    parser.add_argument('--connection-planner', choices=('hybrid_astar', 'dubins'), default='hybrid_astar')
    parser.add_argument('--road-width', type=float)
    parser.add_argument('--turn-only', action='store_true')
    parser.add_argument('--widths', type=float, nargs='+', default=[10., 9., 8., 7., 6., 5.])
    parser.add_argument('--width-step', type=float, default=.25)
    parser.add_argument('--repeats', type=int, default=2)
    parser.add_argument('--output-dir')
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--rviz', action='store_true')
    parser.add_argument('--record-bag', action='store_true')
    parser.add_argument('--timeout', type=float, default=240.)
    args, unknown = parser.parse_known_args()
    if args.mode in ('run', 'compare'):
        if unknown:
            parser.error('unknown arguments: '+' '.join(unknown))
        if args.mode == 'compare':
            if not .1 <= args.width_step <= 1. or not 1 <= args.repeats <= 5:
                parser.error('invalid comparison step/repeats')
            return compare_widths(args)
        return run_cases(args)
    if args.mode == 'bridge':
        sensor_bridge()
    else:
        import rospy
        ScenarioTest()
        rospy.spin()
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
