#!/usr/bin/env python3
"""Export matched-trial audit, trajectory chart and time-aligned video pairs."""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import subprocess


def normalize_recording(path):
    """Trim an early Qt geometry restore, retaining the picture's true aspect."""
    import cv2
    import numpy as np
    capture = cv2.VideoCapture(str(path))
    frames = capture.get(cv2.CAP_PROP_FRAME_COUNT)
    width, height = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)), int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    bounds = []
    for fraction in (.25, .5, .75):
        capture.set(cv2.CAP_PROP_POS_FRAMES, int(frames*fraction))
        ok, frame = capture.read()
        if ok:
            ys, xs = np.where(frame.max(axis=2) > 20)
            if xs.size: bounds.append((int(xs.max())+1, int(ys.max())+1))
    capture.release()
    cropped = False
    if bounds and all(x < width-8 and y < height-8 for x, y in bounds):
        right = min(width, (max(x for x, _ in bounds)+1)//2*2)
        bottom = min(height, (max(y for _, y in bounds)+1)//2*2)
        temporary = path.with_name('gazebo_normalized.mp4')
        subprocess.run(['ffmpeg', '-v', 'error', '-y', '-i', str(path), '-vf',
            'crop={}:{}:0:0,scale=1280:720:force_original_aspect_ratio=decrease,'
            'pad=1280:720:(ow-iw)/2:(oh-ih)/2,setsar=1'.format(right,bottom),
            '-c:v', 'libx264', '-preset', 'fast', '-crf', '21', '-pix_fmt', 'yuv420p',
            '-movflags', '+faststart', str(temporary)], check=True)
        # Keep the original GUI evidence; trimming is a derived presentation.
        cropped = True
    return {'file': str(path), 'frame_size': [width,height], 'visible_bounds_samples': bounds,
            'unused_desktop_trimmed_without_stretching': cropped}


def report(root):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon
    spec = importlib.util.spec_from_file_location('comparison', Path(__file__).with_name('planner_comparison.py'))
    sim = importlib.util.module_from_spec(spec); spec.loader.exec_module(sim)
    results = json.loads((root/'comparison.json').read_text())['results']
    manifests = [json.loads((root/r['folder']/'manifest.json').read_text()) for r in results]
    profiles = {m.get('teb_profile','baseline') for m in manifests}
    neupan_profiles = {m.get('neupan_profile','fixed') for m in manifests}
    controller_settings = [m.get('controller_settings',{'neupan_profile': m.get('neupan_profile','fixed')}) for m in manifests]
    recordings = [normalize_recording(root/r['folder']/'gazebo.mp4') for r in results
                  if (root/r['folder']/'gazebo.mp4').exists()]
    (root/'recording_audit.json').write_text(json.dumps(recordings, indent=2)+'\n')
    refs = {scenario: {r.get('reference_sha256') for r in results if r['scenario']==scenario}
            for scenario in {r['scenario'] for r in results}}
    configs = [json.loads((root/r['folder']/'manifest.json').read_text())['config_sha256'] for r in results]
    reference_checks = {s: len(values)==1 and None not in values for s,values in refs.items()}
    audit = {'identical_reference_paths': all(reference_checks.values()),
             'reference_match_within_each_scenario': reference_checks,
             'identical_configs_across_trials': all(c == configs[0] for c in configs) and
                all(c == controller_settings[0] for c in controller_settings),
             'identical_controller_settings_across_trials': all(c == controller_settings[0] for c in controller_settings),
             'neupan_profiles': sorted(neupan_profiles), 'dynamic_trials': []}
    scenarios = tuple(s for s in sim.AVAILABLE_SCENARIOS if any(r['scenario'] == s for r in results))
    figure, axes = plt.subplots(len(scenarios), 2, figsize=(14, 3.5*len(scenarios)), sharex=True, sharey=True, squeeze=False)
    for row, scenario in enumerate(scenarios):
        for col, planner in enumerate(('neupan', 'teb')):
            ax = axes[row][col]
            trials = [r for r in results if r['scenario'] == scenario and r['planner'] == planner]
            reference_file = root/trials[0]['folder']/'reference.json' if trials else None
            reference = json.loads(reference_file.read_text()) if reference_file and reference_file.exists() else [(0,0),(8,0)]
            ax.plot([p[0] for p in reference], [p[1] for p in reference], '--', color='green', lw=1, label='Common reference')
            for trial in trials:
                source = root/trial['folder']/'trajectory.jsonl'
                if not source.exists(): continue
                data = [json.loads(line) for line in source.read_text().splitlines()]
                line, = ax.plot([r['pose'][0] for r in data], [r['pose'][1] for r in data], lw=1.5,
                    label='Trial {}: {}'.format(trial['repeat'], trial['reason']))
                if scenario in sim.DYNAMIC_SCENARIOS + ('crossing', 'crossing_stress'):
                    manifest = json.loads((root/trial['folder']/'manifest.json').read_text())
                    recorded_scenario = ('crossing_stress' if scenario == 'crossing' and
                                         manifest.get('scenario_version', 1) < 2 else scenario)
                    fixtures = manifest.get('actor_specs', sim.obstacle_specs(recorded_scenario))
                    if data:
                        for ox, oy, radius in data[-1]['obstacles']:
                            ax.add_patch(Circle((ox, oy), radius, color=line.get_color(), alpha=.25))
                        x, y, yaw = data[-1]['pose']; c, s = math.cos(yaw), math.sin(yaw)
                        corners = [(x+c*dx-s*dy, y+s*dx+c*dy) for dx,dy in
                                   [(-.14,-.375),(.81,-.375),(.81,.375),(-.14,.375)]]
                        ax.add_patch(Polygon(corners, fill=False, edgecolor=line.get_color(), linewidth=1))
                    errors = []
                    expected_positions = {s[0]: [] for s in fixtures if sim.moving_actor(recorded_scenario,s[0])}
                    for frame in data:
                        for i, fixture in enumerate(fixtures):
                            phase = frame.get('actor_states', {}).get(fixture[0], {}).get('phase_s', frame['t'])
                            expected = sim.obstacle_position(fixture, recorded_scenario, phase)
                            actual = frame['obstacles'][i]
                            errors.append(math.hypot(actual[0]-expected[0], actual[1]-expected[1]))
                            if fixture[0] in expected_positions:
                                expected_positions[fixture[0]].append(expected[:2])
                    travel = {name: length for name, length in trial.get('obstacle_path_lengths_m', {}).items()
                              if name != 'traffic_island'}
                    expected_travel = {name:sum(math.dist(a,b) for a,b in zip(points,points[1:]))
                                       for name,points in expected_positions.items()}
                    audit['dynamic_trials'].append({'folder': trial['folder'],
                        'policy': trial.get('actor_policy', 'prescribed trajectory'),
                        'actor_yield_events': trial.get('actor_yield_events', {}),
                        'maximum_actor_position_error_m': max(errors) if errors else None,
                        'path_lengths_m': travel,
                        'expected_path_lengths_m': expected_travel,
                        'verified': bool(errors) and max(errors) < .2 and bool(travel) and
                            all(travel.get(n,0) >= max(.05,.8*length) for n,length in expected_travel.items())})
            fixture_manifest = json.loads((root/trials[0]['folder']/'manifest.json').read_text()) if trials else {}
            fixture_scenario = ('crossing_stress' if scenario=='crossing' and
                                fixture_manifest.get('scenario_version',1)<2 else scenario)
            for _, x, y, radius in fixture_manifest.get('actor_specs',sim.obstacle_specs(fixture_scenario)):
                ax.add_patch(Circle((x, y), radius, color='gray', alpha=.7))
                if scenario in ('crossing', 'crossing_stress'):
                    ax.plot([x, x], [-abs(y), abs(y)], ':', color='red', lw=1)
                elif scenario in ('lead_slow','lead_stop'):
                    ax.plot([x,10], [y,y], ':', color='red', lw=1)
                elif scenario == 'crossing_once':
                    ax.plot([x,x],[-3,3],':',color='red',lw=1)
            ax.plot(8, 0, '*', ms=13, color='red', label='Goal')
            ax.set_title('{} / {} ({}/{} arrived)'.format(scenario, planner.upper(), sum(r['passed'] for r in trials), len(trials)))
            ax.set_xlim(-1, 12); ax.set_ylim(-5.3, 5.3); ax.set_aspect('equal', adjustable='box')
            ax.grid(alpha=.2); ax.legend(fontsize=7, loc='upper right')
            ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    figure.suptitle('Actual Gazebo rear-axle trajectories | Matched 10 Hz local-planner trials', fontsize=12, y=.995)
    figure.tight_layout(rect=(0,0,1,.965)); figure.savefig(root/'trajectories.png', dpi=150); plt.close(figure)
    speed_figure, speed_axes = plt.subplots(len(scenarios),1,figsize=(12,3*len(scenarios)),squeeze=False)
    for row,scenario in enumerate(scenarios):
        ax=speed_axes[row][0]
        for trial in [r for r in results if r['scenario']==scenario]:
            data=[json.loads(line) for line in (root/trial['folder']/'trajectory.jsonl').read_text().splitlines()]
            color='#16a6d3' if trial['planner']=='teb' else '#e8a118'
            ax.plot([r['t'] for r in data],[r['speed'] for r in data],color=color,alpha=.8,
                    label='{} / repeat {} / stops {}'.format(trial['planner'].upper(),trial['repeat'],trial.get('stop_count','?')))
            for stop in trial.get('stop_intervals',[]):
                ax.axvspan(stop['start_s'],stop['end_s'],color=color,alpha=.12)
        ax.set_title(scenario);ax.set_xlabel('Simulation time from goal (s)');ax.set_ylabel('Actual speed (m/s)')
        ax.set_ylim(-.55,.6);ax.axhline(0,color='gray',lw=.5);ax.grid(alpha=.2);ax.legend(fontsize=8)
    speed_figure.tight_layout();speed_figure.savefig(root/'speed_and_stops.png',dpi=150);plt.close(speed_figure)
    audit['all_dynamic_actors_verified'] = all(r['verified'] for r in audit['dynamic_trials']) if audit['dynamic_trials'] else None
    audit['fixed_trajectory_max_pair_difference_m'] = {}
    for scenario in sim.DYNAMIC_SCENARIOS:
        trials = [r for r in results if r['scenario']==scenario]
        if len(trials)<2: continue
        tracks = [[json.loads(line) for line in (root/r['folder']/'trajectory.jsonl').read_text().splitlines()] for r in trials]
        end=min(track[-1]['t'] for track in tracks)
        maximum=0.
        import numpy as np
        for t in np.arange(0,end,.1):
            points=[(np.interp(t,[r['t'] for r in track],[r['obstacles'][0][0] for r in track]),
                     np.interp(t,[r['t'] for r in track],[r['obstacles'][0][1] for r in track])) for track in tracks]
            maximum=max(maximum,max(math.dist(points[0],p) for p in points[1:]))
        audit['fixed_trajectory_max_pair_difference_m'][scenario]=maximum
    (root/'input_audit.json').write_text(json.dumps(audit, indent=2)+'\n')
    for scenario in scenarios:
        pair = [next((r for r in results if r['scenario'] == scenario and r['planner'] == planner and r['repeat'] == 1), None)
                for planner in ('neupan', 'teb')]
        if any(r is None for r in pair): continue
        if not all((root/r['folder']/'overview.mp4').exists() for r in pair): continue
        duration = max(r.get('elapsed_sim_s', 0) for r in pair)
        subprocess.run(['ffmpeg', '-v', 'error', '-y', '-i', str(root/pair[0]['folder']/'overview.mp4'),
            '-i', str(root/pair[1]['folder']/'overview.mp4'), '-filter_complex',
            '[0:v]setpts=PTS-STARTPTS,tpad=stop_mode=clone:stop_duration=120[a];'
            '[1:v]setpts=PTS-STARTPTS,tpad=stop_mode=clone:stop_duration=120[b];[a][b]hstack=inputs=2,setsar=1[v]',
            '-map', '[v]', '-t', str(duration), '-c:v', 'libx264', '-preset', 'fast', '-crf', '23',
            '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(root/(scenario+'_comparison.mp4'))], check=True)
    lines = ['# NeuPAN 与 TEB Gazebo 对比', '',
        '真实 Gazebo：同一后轴几何、同一参考线、扫描、0.5 m/s 限速及10 Hz控制，共 {} 轮；场景及重复次数见下表。'.format(len(results)),
        '每轮独立重启。lead_slow、lead_stop、crossing_once 按固定时间表运动，不根据车辆行为让路；轨迹误差见输入审计。',
        '历史 crossing 为主动让行场景，crossing_stress 为往返压测，分别解读；不向任一规划器提供障碍真值速度。', '',
        '[轨迹对比图](trajectories.png) · [速度与停顿曲线](speed_and_stops.png) · [逐轮数据](comparison.json) · [输入一致性与障碍运动审计](input_audit.json)', '',
        '| 场景 | 规划器 | 次数 | 结果 | 时间 s | 路长 m | 目标误差 m | 最小车体间距 m | 调用中位数 / P95 ms |',
        '|---|---|---|---|---|---|---|---|---|']
    lines[2:2]=['NeuPAN profile={}；fixed为历史固定参数对比，adaptive沿用原仿真的自适应调参。二者不能混称为常用配置复现。'.format(','.join(sorted(neupan_profiles))), '']
    if (root/'feasibility.json').exists():
        proof=json.loads((root/'feasibility.json').read_text())
        lines[4:4]=['[场景离线几何/运动学可行性证据](feasibility.json)：独立构造安全绕行路径，未向规划器提供该路径或障碍速度；逐个检查车体净距、速度和转角。',
                    'TEB profile={}；field来自用户确认的实车参数，坐标、车体、速度上限及终点容差作仿真适配，未复现实车move_base恢复状态机。'.format(','.join(sorted(profiles))), '']
    for r in results:
        lines.append('| {} | {} | {} | {} | {} | {} | {} | {} | {} / {} |'.format(r['scenario'],r['planner'],r['repeat'],r['reason'],
            r.get('elapsed_sim_s','-'),r.get('path_length_m','-'),r.get('goal_error_m','-'),r.get('minimum_body_clearance_m','-'),
            r.get('solver_median_ms','-'),r.get('solver_p95_ms','-')))
    pauses=[r for r in results if 'stop_count' in r]
    if pauses:
        lines += ['', '## 停顿与恢复（启动和终点停车排除）', '',
                  '速度低于0.03 m/s持续至少0.3 s，已起步且距离目标大于0.4 m；超过0.08 m/s记为恢复。未恢复的终端停顿也保留。', '',
                  '| 场景 | 规划器 | 次数 | 停顿次数 | 总停顿 s | 最长停顿 s | 初始等待 s | 完全超越时间 s | 无效控制采样 |',
                  '|---|---|---|---|---|---|---|---|---|']
        for r in pauses:
            lines.append('| {} | {} | {} | {} | {} | {} | {} | {} | {} |'.format(r['scenario'],r['planner'],r['repeat'],
                r['stop_count'],r['stop_total_s'],r['longest_stop_s'],r['initial_wait_s'],
                r.get('overtake_time_s'),r['invalid_control_samples']))
    interactive = [r for r in results if r.get('actor_yield_events')]
    if interactive:
        lines += ['', '## 障碍主动让行', '',
                  '这是双方交互测试：到达结果包含障碍主动让行的作用，不能作为机器人单方面动态避障能力的证明。', '',
                  '| 场景 | 规划器 | 次数 | 让行事件 | 让行时长 s |', '|---|---|---|---|---|']
        for r in interactive:
            lines.append('| {} | {} | {} | {} | {} |'.format(r['scenario'],r['planner'],r['repeat'],
                r['actor_yield_events'],r.get('actor_yield_duration_s',{})))
    lines += ['', '## 画面', '', '左右对照以仿真目标下发时刻对齐，是实际 Gazebo 真值轨迹回放；较早结束的一侧保留末帧：', '']
    for scenario in scenarios:
        if (root/(scenario+'_comparison.mp4')).exists():
            lines.append('- [{} 轨迹对照]({}_comparison.mp4)'.format(scenario,scenario))
    lines += ['', '每一轮的 `gazebo.mp4` 才是实际 Gazebo 窗口录屏；`overview.mp4` 是同轮轨迹回放。', '']
    for r in results:
        if (root/r['folder']/'gazebo.mp4').exists():
            lines.append('- [{} 真正 Gazebo 录屏]({}/gazebo.mp4)'.format(r['folder'], r['folder']))
    lines += ['', '## 判定与边界', '',
        '到达需规划器报告完成、位置误差≤{} m、航向误差≤0.35 rad、速度≤0.1 m/s并稳定0.5 s。'.format(
            '/'.join(str(v) for v in sorted({r.get('acceptance_position_tolerance_m',.5) for r in results}))),
        '车体侵入障碍超过2 cm、越界、扫描过期、Gazebo/里程计误差>0.25 m、超时均不算通过。最小间距用真实矩形车体包络计算。',
        '间距是本次动态时序中的几何量，不是实车安全距离。两者停止阈值/收敛精度仍会影响行驶时间，应同时看目标误差。',
        '调用耗时是同一CPU上各自优化器/插件调用的墙钟时间，不含完整感知流水线。TEB使用扫描代价地图；field保留实测TEB优化参数及插件默认值，baseline使用仿真轮廓聚类。未配置额外动态跟踪器。',
        '运动学模型不模拟轮胎、打滑或接触动力学；有限场景和两次重复不能证明算法普遍优劣。', '',
        '输入审计：参考线一致={}；参数版本一致={}；动态障碍真实运动验证={}。'.format(
            audit['identical_reference_paths'],audit['identical_configs_across_trials'],audit['all_dynamic_actors_verified'])]
    regression_paths = [('指令/扫描中断停车及扫描恢复', 'safety_check/result.json'),
        ('原局部入口6米目标', 'legacy_regression/result.json'),
        ('原室外宽路往返', 'outdoor_regression/wide/result.json')]
    available = [(name, path, json.loads((root/path).read_text())) for name,path in regression_paths if (root/path).exists()]
    if available:
        lines += ['', '## 原仿真与保护回归', '', '| 检查 | 结果 |', '|---|---|']
        for name,path,value in available:
            lines.append('| [{}]({}) | {} |'.format(name,path,'PASS' if value.get('passed') else 'FAIL'))
    (root/'comparison.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(audit, indent=2))
    return audit


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument('folder', type=Path)
    report(parser.parse_args().folder.resolve())
