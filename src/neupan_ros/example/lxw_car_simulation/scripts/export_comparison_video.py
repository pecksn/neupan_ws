#!/usr/bin/env python3
"""Export 1080p, clock-aligned TEB/NeuPAN videos without changing source evidence."""
import argparse
from bisect import bisect_right
import hashlib
import json
import math
from pathlib import Path
import subprocess

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

SIZE, FPS = (1920, 1080), 15
PLANNERS = ('teb', 'neupan')
NAMES = {'teb': 'TEB', 'neupan': 'NeuPAN'}
SCENES = {'open': '空场到达', 'static': '静态绕障', 'lead_slow':'超越前方慢行障碍',
          'lead_stop':'前方障碍停车再起步', 'crossing': '绕障与交互横穿（双方避让）',
          'crossing_once':'单次横穿与车辆让行',
          'crossing_stress': '强制横穿压测（障碍不避让）'}
REASONS = {'arrived': '到达目标', 'body_overlap': '车体与障碍重叠',
           'gazebo_odometry_mismatch': 'Gazebo / 里程计偏离',
           'stale_scan': '扫描过期', 'timeout': '超时'}
FONT = Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
BG, FG, MUTED = (24, 21, 18), (240, 237, 232), (175, 171, 164)


def text_layer(lines):
    """Render static Chinese labels once, outside the frame loop."""
    rgb = Image.new('RGB', SIZE, BG[::-1])
    draw = ImageDraw.Draw(rgb)
    fonts = {}
    for x, y, label, size, color in lines:
        fonts.setdefault(size, ImageFont.truetype(str(FONT), size))
        draw.text((x, y), label, font=fonts[size], fill=color[::-1])
    return cv2.cvtColor(np.array(rgb), cv2.COLOR_RGB2BGR)


def ascii_text(frame, label, point, scale=.75, color=FG):
    cv2.putText(frame, label, point, cv2.FONT_HERSHEY_SIMPLEX, scale,
                color, 1, cv2.LINE_AA)


class Trial:
    def __init__(self, root, result, gui):
        self.result = result
        self.folder = root/result['folder']
        self.manifest=json.loads((self.folder/'manifest.json').read_text())
        self.result = dict(result, neupan_profile=self.manifest.get('neupan_profile','fixed'))
        self.records = [json.loads(line) for line in (self.folder/'trajectory.jsonl').read_text().splitlines()]
        if not self.records:
            raise ValueError('Empty telemetry: '+str(self.folder))
        self.times = [r['t'] for r in self.records]
        self.speed_values = [r['speed'] for r in self.records]
        reference = self.folder/'reference.json'
        self.reference = json.loads(reference.read_text()) if reference.exists() else [(0.,0.), (8.,0.)]
        if self.times != sorted(self.times):
            raise ValueError('Simulation clock moved backwards: '+str(self.folder))
        self.cap = None
        self.next_frame = 0
        self.last_frame = None
        if gui:
            timing = json.loads((self.folder/'recording_timing.json').read_text())
            self.first_wall = timing['first_frame_wall_unix_s']
            self.cap = cv2.VideoCapture(str(self.folder/'gazebo.mp4'))
            if not self.cap.isOpened():
                raise ValueError('Cannot read Gazebo recording: '+str(self.folder))
            self.capture_fps = self.cap.get(cv2.CAP_PROP_FPS)
            self.capture_count = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
            offsets = [r['wall_time_unix_s']-self.first_wall for r in self.records]
            if min(offsets) < -.1 or max(offsets) > self.capture_count/self.capture_fps+.3:
                raise ValueError('Recording does not cover the episode: '+str(self.folder))

    def sample(self, t):
        index = max(0, bisect_right(self.times, t)-1)
        return self.records[index], index

    def gui_frame(self, t, record, index):
        wall = record['wall_time_unix_s']
        if index+1 < len(self.records):
            after = self.records[index+1]
            fraction = max(0., min(1., (t-record['t'])/(after['t']-record['t'])))
            wall += fraction*(after['wall_time_unix_s']-wall)
        wanted = max(0, min(self.capture_count-1, int((wall-self.first_wall)*self.capture_fps)))
        while self.next_frame <= wanted:
            ok, picture = self.cap.read()
            if not ok:
                raise ValueError('Incomplete video frame: '+str(self.folder))
            self.last_frame = picture
            self.next_frame += 1
        # Crop only the fixed Gazebo chrome. No stretching or content synthesis.
        h, w = self.last_frame.shape[:2]
        return self.last_frame[round(h*.081):round(h*.929), round(w*.169):round(w*.988)]

    def close(self):
        if self.cap is not None:
            self.cap.release()


def fit_picture(canvas, picture, box):
    x, y, width, height = box
    scale = min(width/picture.shape[1], height/picture.shape[0])
    fitted = cv2.resize(picture, (round(picture.shape[1]*scale), round(picture.shape[0]*scale)))
    left, top = x+(width-fitted.shape[1])//2, y+(height-fitted.shape[0])//2
    canvas[top:top+fitted.shape[0], left:left+fitted.shape[1]] = fitted


def trajectory(trial, record, index, width, height):
    picture = np.full((height, width, 3), (32, 30, 28), np.uint8)
    # Both axes use the same metres-to-pixels scale: circles stay circular.
    xmin,xmax,ymin,ymax=trial.view_bounds
    scale = min((width-50)/(xmax-xmin), (height-36)/(ymax-ymin))
    def pt(x, y):
        return (round(width/2+(x-(xmin+xmax)/2)*scale),
                round(height/2-(y-(ymin+ymax)/2)*scale))
    for x in range(math.ceil(xmin), math.floor(xmax)+1):
        cv2.line(picture, pt(x,ymin), pt(x,ymax), (54,51,48), 1)
    for y in range(math.ceil(ymin), math.floor(ymax)+1):
        cv2.line(picture, pt(xmin,y), pt(xmax,y), (54,51,48), 1)
    cv2.polylines(picture,[np.array([pt(x,y) for x,y in trial.reference],np.int32)],
                  False,(100,180,90),2,cv2.LINE_AA)
    cv2.circle(picture, pt(8,0), max(6,round(scale*.16)), (230,210,40), 2)
    for ox, oy, radius in record['obstacles']:
        cv2.circle(picture, pt(ox,oy), max(3,round(radius*scale)), (50,100,240), -1, cv2.LINE_AA)
    trace = np.array([pt(*r['pose'][:2]) for r in trial.records[:index+1]],np.int32)
    if len(trace)>1: cv2.polylines(picture,[trace],False,(20,220,255),2,cv2.LINE_AA)
    plan = np.array([pt(*point[:2]) for point in record['plan']],np.int32)
    if len(plan)>1: cv2.polylines(picture,[plan],False,(220,110,220),1,cv2.LINE_AA)
    x,y,yaw=record['pose']; c,s=math.cos(yaw),math.sin(yaw)
    body=np.array([pt(x+c*dx-s*dy,y+s*dx+c*dy) for dx,dy in
                   [(-.14,-.375),(.81,-.375),(.81,.375),(-.14,.375)]],np.int32)
    cv2.fillPoly(picture,[body],(195,195,195),cv2.LINE_AA)
    cv2.arrowedLine(picture,pt(x,y),pt(x+.8*c,y+.8*s),(255,255,255),1,cv2.LINE_AA)
    return picture


def speed_chart(trial, index, width, height):
    picture=np.full((height,width,3),(32,30,28),np.uint8)
    end=max(1.,trial.times[-1])
    def pt(t,v):
        return (round(32+(width-42)*t/end),round(height-16-(height-36)*(max(-.55,min(.6,v))+.55)/1.15))
    cv2.line(picture,pt(0,0),pt(end,0),(125,120,115),1)
    cv2.line(picture,pt(0,.5),pt(end,.5),(75,70,65),1)
    t=trial.records[index]['t']
    for stop in trial.result.get('stop_intervals',[]):
        if stop['start_s']>t:continue
        x1=pt(stop['start_s'],0)[0];x2=pt(min(t,stop['end_s']),0)[0]
        cv2.rectangle(picture,(x1,18),(max(x1+1,x2),height-16),(45,45,105),-1)
    points=np.array([pt(row['t'],row['speed']) for row in trial.records[:index+1]],np.int32)
    if len(points)>1:cv2.polylines(picture,[points],False,(20,220,255),2,cv2.LINE_AA)
    ascii_text(picture,'v m/s', (3,13),.34,MUTED)
    ascii_text(picture,'0', (5,pt(0,0)[1]+4),.34,MUTED)
    ascii_text(picture,'t(s)', (width-33,height-2),.34,MUTED)
    return picture


def scene_base(scenario, trials, gui):
    # A failed run can leave the normal lane. Keep its entire body visible,
    # with the same fixed bounds and metre scale on both sides of this pair.
    xs=[-1.,11. if scenario.startswith('lead_') else 9.]
    ys=[-3.3,3.3]
    for trial in trials:
        for record in trial.records:
            x,y=record['pose'][:2]
            xs.extend((x-.95,x+.95));ys.extend((y-.95,y+.95))
            for x,y,radius in record['obstacles']:
                xs.extend((x-radius-.2,x+radius+.2))
                ys.extend((y-radius-.2,y+radius+.2))
    for trial in trials:
        trial.view_bounds=(min(xs),max(xs),min(ys),max(ys))
    lines = [(32,18,'TEB × NeuPAN  |  '+SCENES[scenario],38,FG),
             (32,78,'同车、同参考线、同雷达模型、0.5 m/s 限速；第 {} 轮'.format(trials[0].result.get('repeat',1)),24,MUTED),
             (32,1028,'绿：共同参考线    黄：实际轨迹    紫：局部规划    橙：障碍    青：目标 (8, 0)',23,MUTED)]
    for col, trial in enumerate(trials):
        x=col*960+24
        lines += [(x,133,NAMES[trial.result['planner']],32,FG)]
        if gui:
            label='真实 Gazebo 窗口'
            if trial.result['planner']=='teb' and trial.manifest.get('teb_profile')=='field':
                label+=' · 实测参数适配'
            if trial.result['planner']=='neupan':
                label+=' · '+('原仿真自适应' if trial.result['neupan_profile']=='adaptive' else '固定参数')
            lines += [(x+155,141,label,23,MUTED),
                      (x+550,735,'真值轨迹回放',22,MUTED)]
        else:
            label='Gazebo 真值轨迹回放'
            if trial.result['planner']=='neupan':
                label+=' · '+('原仿真自适应' if trial.result['neupan_profile']=='adaptive' else '固定参数')
            lines += [(x+155,141,label,23,MUTED)]
    return text_layer(lines)


def draw_scene(base, trials, t, gui):
    frame=base.copy()
    cv2.line(frame,(959,130),(959,1010),(80,75,70),1)
    for col, trial in enumerate(trials):
        record,index=trial.sample(t)
        x=col*960+16
        ended=t >= trial.times[-1]
        if gui:
            fit_picture(frame,trial.gui_frame(t,record,index),(x,186,928,535))
            if 'stop_count' in trial.result:
                frame[770:910,x+548:x+928]=trajectory(trial,record,index,380,140)
                frame[918:1006,x+548:x+928]=speed_chart(trial,index,380,88)
            else:
                frame[770:1006,x+548:x+928]=trajectory(trial,record,index,380,236)
            pos=(x+8,776)
        else:
            frame[186:780,x:x+928]=trajectory(trial,record,index,928,594)
            pos=(x+8,826)
            if 'stop_count' in trial.result:
                frame[820:1006,x+548:x+928]=speed_chart(trial,index,380,186)
        error=math.hypot(record['pose'][0]-8,record['pose'][1])
        ascii_text(frame,'t={:05.2f}s  |  v={:.2f} m/s'.format(min(t,trial.times[-1]),record['speed']),pos)
        ascii_text(frame,'Body gap: {:+.3f} m'.format(record['clearance']),(pos[0],pos[1]+37),
                   color=(90,100,255) if record['clearance']<0 else FG)
        ascii_text(frame,'Goal error: {:.2f} m'.format(error),(pos[0],pos[1]+74))
        if ended:
            outcome='ARRIVED' if trial.result['passed'] else 'FAILED: '+trial.result['reason']
            if trial.result['passed'] and any(trial.result.get('actor_yield_events', {}).values()):
                outcome += ' (actors also yielded)'
            ascii_text(frame,outcome,(pos[0],pos[1]+116),.63,
                       (100,225,120) if trial.result['passed'] else (90,100,255))
        else:
            active=next((s for s in trial.result.get('stop_intervals',[])
                         if s['start_s']+.3<=record['t']<=s['end_s']),None)
            label='RUNNING'
            if active:
                label='STOP {:.2f}s'.format(record['t']-active['start_s'])
                runtime=record.get('controller_runtime',{})
                if runtime.get('valid') is False and record.get('controller_runtime_age_s',1e9)<=.5:
                    label+=' | plan invalid'
                elif record.get('gate_health',{}).get('command_ok') is False:
                    label+=' | command expired'
            ascii_text(frame,label,(pos[0],pos[1]+116),.65,(90,100,255) if active else MUTED)
        if 'stop_count' in trial.result:
            stops=[s for s in trial.result['stop_intervals'] if s['start_s']+.3<=record['t']]
            wait=sum(max(0,min(record['t'],s['end_s'])-s['start_s']) for s in stops)
            ascii_text(frame,'Stops: {} | Wait: {:.2f}s'.format(len(stops),wait),
                       (pos[0],pos[1]+153),.65,FG)
            passed=trial.result.get('overtake_time_s')
            label='Fully passed' if passed is not None and record['t']>=passed else 'Not yet passed'
            if trial.result['scenario'].startswith('lead_'):
                ascii_text(frame,label,(pos[0],pos[1]+180),.6,MUTED)
        elif record.get('actor_states'):
            yielding=sum(bool(s['yielding']) for s in record['actor_states'].values())
            events=sum(s['yield_events'] for s in record['actor_states'].values())
            ascii_text(frame,'Actors yielding: {}  |  Events: {}'.format(yielding,events),
                       (pos[0],pos[1]+153),.65,MUTED)
    return frame


def cards(results, gui):
    selected=[s for s in SCENES if any(r['scenario']==s for r in results)]
    interactive=any(r.get('actor_yield_events') for r in results)
    lead=any(s.startswith('lead_') for s in selected)
    profile=next(r.get('neupan_profile','fixed') for r in results if r['planner']=='neupan')
    profile_note=('NeuPAN：原仿真自适应调参；结果只对应本次接入和配置。' if profile=='adaptive' else
                  'NeuPAN：固定参数模式，未启用原仿真自适应；不能当作常用配置复现。')
    scene_line='  →  '.join(SCENES[s] for s in selected)
    intro=text_layer([(80,160,'TEB 与 NeuPAN · Gazebo 仿真对比',56,FG),
        (84,290,'左：TEB     右：NeuPAN',40,FG),
        (84,390,scene_line,30 if len(selected)>2 else 34,FG),
        (84,500,'目标：后轴 (8, 0)    车体：0.95 × 0.75 m    轴距：0.67 m',28,MUTED),
        (84,565,'共用参考线、雷达模型和控制限制；没有向规划器提供障碍真值速度。',28,MUTED),
        (84,650,'真实窗口 + 下方轨迹回放' if gui else '实际 Gazebo 位姿与规划数据的轨迹回放',32,FG),
        (84,720,'按仿真时间对齐，先结束的一侧冻结；窗口画面仍有渲染延迟。' if gui else
         '按仿真时间对齐、等比例俯视；先结束的一侧冻结。',27,MUTED),
        (84,825,'障碍按相同时间表行驶/停车，不给机器人让路；重点看停顿与恢复。' if lead else
         '本场景允许障碍主动让行；到达不等于机器人独立避障能力通过。' if interactive else
         '保留全部匹配场景的结果，包括失败过程。',28,(100,150,255)),
        (84,900,profile_note,26,(100,150,255))])
    lines=[(70,65,'本轮结果与仍需解决的问题',44,FG)]
    for row, scenario in enumerate(selected):
        entries=[r for r in results if r['scenario']==scenario]
        if not entries: continue
        lines.append((75,175+row*110,SCENES[scenario],24,FG))
        for col,planner in enumerate(PLANNERS):
            group=[r for r in entries if r['planner']==planner]
            r=group[0]
            reason=REASONS.get(r['reason'],r['reason']) if len(group)==1 else '{}/{}到达'.format(sum(t['passed'] for t in group),len(group))
            if len(group)>1 and all('stop_count' in t for t in group):
                reason+=' / 停顿{}次 {:.1f}s'.format(sum(t['stop_count'] for t in group),sum(t['stop_total_s'] for t in group))
            elif r.get('actor_yield_events'):
                reason+=' / 障碍让行{}次'.format(sum(r['actor_yield_events'].values()))
            elif 'stop_count' in r:
                reason+=' / 停顿{}次 {:.1f}s'.format(r['stop_count'],r['stop_total_s'])
            lines.append((590+col*630,175+row*110,NAMES[planner]+'：'+reason,27,
                          (100,225,120) if all(t['passed'] for t in group) else (100,150,255)))
    crossing=[r for r in results if r['scenario']=='crossing']
    dynamic_note=('同一障碍时序、同一参考线；红色速度区间表示中途停顿，启动和终点停车排除。' if lead else
                  '交互横穿仍有失败，需结合障碍让行记录检查接近过程。'
                  if any(not r['passed'] for r in crossing) else
                  '交互横穿到达包含障碍让行的作用；不能据此认定机器人独立动态避障通过。'
                  if interactive else '强制横穿压测与正常交互应分别解读。')
    medians={p:[r['solver_median_ms'] for r in results if r['planner']==p and r.get('solver_median_ms') is not None]
             for p in PLANNERS}
    timing='本轮优化调用中位数范围：'+'；'.join('{} {:.1f}～{:.1f} ms'.format(NAMES[p],min(v),max(v))
                                                        for p,v in medians.items() if v)
    lines += [(75,555,dynamic_note,29,FG),
              (75,620,timing,27,MUTED),
              (75,680,'本组未记录到 TEB 中途停顿；不能据此复现实车卡顿原因。'
               if lead and all(r.get('stop_count',0)==0 for r in results if r['planner']=='teb') else
               '有限场景的停顿差异不能直接推断实车表现或算法普遍优劣。',27,MUTED),
              (75,740,'运动学虚拟底盘不模拟轮胎和完整接触动力学。',27,MUTED),
              (75,795,profile_note,25,(100,150,255)),
              (75,850,'1920 × 1080  |  15 fps  |  H.264 / yuv420p  |  方形像素',27,FG)]
    return intro,text_layer(lines)


def encode(root, pairs, output, gui):
    intro,ending=cards([t.result for pair in pairs.values() for t in pair],gui)
    command=['ffmpeg','-v','error','-y','-f','rawvideo','-pix_fmt','bgr24','-s','1920x1080',
             '-r',str(FPS),'-i','pipe:0','-an','-c:v','libx264','-preset','fast','-crf','21',
             '-pix_fmt','yuv420p','-vf','setsar=1','-movflags','+faststart',str(output)]
    segments=[]; count=0
    with subprocess.Popen(command,stdin=subprocess.PIPE) as process:
        def write(frame):
            process.stdin.write(frame.tobytes())
        try:
            for _ in range(FPS*3): write(intro)
            count+=FPS*3
            for _,trials in pairs.items():
                scenario=trials[0].result['scenario']
                base=scene_base(scenario,trials,gui)
                duration=max(trial.times[-1] for trial in trials)
                frames=math.ceil(duration*FPS)+FPS*2
                segments.append({'scenario':scenario,'video_start_s':count/FPS,
                                 'repeat':trials[0].result.get('repeat',1),
                                 'simulation_duration_s':duration,'video_duration_s':frames/FPS})
                print('Exporting',output.name,scenario,flush=True)
                for i in range(frames): write(draw_scene(base,trials,min(i/FPS,duration),gui))
                count+=frames
            for _ in range(FPS*5): write(ending)
            count+=FPS*5
        finally:
            process.stdin.close()
        if process.wait()!=0: raise RuntimeError('ffmpeg export failed')
    return {'file':output.name,'frames':count,'duration_s':count/FPS,'segments':segments}


def export(root, output, view, all_repeats=False):
    output.mkdir(parents=True,exist_ok=True)
    results=json.loads((root/'comparison.json').read_text())['results']
    pairs={}
    repeats=sorted({r['repeat'] for r in results}) if all_repeats else [1]
    for scenario in SCENES:
        for repeat in repeats:
            pair=[]
            for planner in PLANNERS:
                r=next((r for r in results if r['planner']==planner and r['scenario']==scenario and r['repeat']==repeat),None)
                if r is not None: pair.append(r)
            if len(pair)==2: pairs[scenario+'_repeat_'+str(repeat)]=pair
    if not pairs: raise ValueError('No matched repeat-1 planner pairs')
    selected=[r for pair in pairs.values() for r in pair]
    references={s:{r.get('reference_sha256') for r in pair} for s,pair in pairs.items()}
    configs=[json.loads((root/r['folder']/'manifest.json').read_text())['config_sha256'] for r in selected]
    profiles=[json.loads((root/r['folder']/'manifest.json').read_text()).get('neupan_profile','fixed') for r in selected]
    if (any(len(values)!=1 or None in values for values in references.values()) or
            any(c!=configs[0] for c in configs) or len(set(profiles))!=1):
        raise ValueError('Comparison inputs differ; refusing to label these as matched trials')
    manifest={'source':str(root),'selection':'all matched scenarios; {} repeats; TEB left / NeuPAN right'.format('all' if all_repeats else 'first'),
              'comparison_json_sha256':hashlib.sha256((root/'comparison.json').read_bytes()).hexdigest(),
              'frame_size':list(SIZE),'fps':FPS,'codec':'H.264 / yuv420p','sample_aspect_ratio':'1:1',
              'reference_sha256_by_scenario':{s:next(iter(values)) for s,values in references.items()},
              'config_sha256':configs[0],
              'neupan_profile': profiles[0],
              'gui_alignment':'X11 input PTS mapped through measured telemetry simulation/wall clocks; render latency remains',
              'trajectory_view':'fixed common bounds per matched pair include full actual vehicle tracks; equal metre scale on both axes',
              'outputs':[]}
    for gui in ([True,False] if view=='both' else [view=='gazebo']):
        trials={s:[Trial(root,r,gui) for r in pair] for s,pair in pairs.items()}
        try:
            target=output/('teb_neupan_'+('gazebo' if gui else 'trajectory')+'_1080p.mp4')
            manifest['outputs'].append(encode(root,trials,target,gui))
            if gui:
                subprocess.run(['ffmpeg','-v','error','-y','-ss','8','-i',str(target),
                                '-frames:v','1',str(output/'preview.jpg')],check=True)
        finally:
            for pair in trials.values():
                for trial in pair: trial.close()
    (output/'video_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    print('Videos:',output,flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder',type=Path)
    parser.add_argument('--output-dir',type=Path)
    parser.add_argument('--view',choices=('gazebo','trajectory','both'),default='both')
    parser.add_argument('--all-repeats',action='store_true',help='include every matched repetition, not only repeat 1')
    args=parser.parse_args()
    root=args.folder.resolve()
    export(root,args.output_dir.resolve() if args.output_dir else root/'presentation',args.view,args.all_repeats)
