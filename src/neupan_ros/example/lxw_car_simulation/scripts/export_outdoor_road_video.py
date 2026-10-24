#!/usr/bin/env python3
"""Export actual Gazebo recordings with simulation-clock telemetry and Chinese HUD."""
import argparse
from bisect import bisect_right
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

FPS=15
FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
BG=(22,27,29)
REASONS={'arrived':'已到达目标','body_overlap_or_road_boundary':'未通过：包络重叠或越界',
         'neupan_exception':'未通过：求解器异常','simulation_timeout':'未通过：超时',
         'gazebo_odometry_mismatch':'未通过：车体与里程计偏离'}


def labels(frame,rows):
    rgb=Image.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB));draw=ImageDraw.Draw(rgb)
    fonts={}
    for x,y,label,size,color in rows:
        if size not in fonts:fonts[size]=ImageFont.truetype(FONT,size)
        draw.text((x,y),label,font=fonts[size],fill=color)
    return cv2.cvtColor(np.array(rgb),cv2.COLOR_RGB2BGR)


class Clip:
    def __init__(self,folder):
        self.folder=folder
        self.result=json.loads((folder/'result.json').read_text())
        self.manifest=json.loads((folder/'manifest.json').read_text())
        self.rows=[json.loads(line) for line in (folder/'trajectory.jsonl').read_text().splitlines()]
        if not self.rows:raise ValueError('Missing telemetry: '+str(folder))
        self.times=[r['t'] for r in self.rows]
        motion=folder/'motion_estimates.jsonl'
        self.motion=[json.loads(line) for line in motion.read_text().splitlines()] if motion.exists() else []
        self.motion_times=[r['stamp']-self.result.get('episode_start_sim_s',0) for r in self.motion]
        runtime=folder/'runtime.jsonl'
        self.runtime=[json.loads(line) for line in runtime.read_text().splitlines()] if runtime.exists() else []
        self.runtime_times=[r['stamp']-self.result.get('episode_start_sim_s',0) for r in self.runtime]
        repeat=next((int(match[1]) for part in folder.parts for match in [re.fullmatch(r'(?:[a-z]+_)?run(\d+)',part)] if match),1)
        self.repeat=repeat
        if self.times!=sorted(self.times):raise ValueError('Clock moved backwards')
        match=re.search(r'demuxer -> .*?pkt_pts:(\d+) ',(folder/'recorder.log').read_text())
        if not match:raise ValueError('Missing X11 input wall timestamp')
        self.first_wall=int(match[1])/1e6
        self.cap=cv2.VideoCapture(str(folder/'gazebo.mp4'))
        if not self.cap.isOpened():raise ValueError('Cannot open actual GUI video')
        self.fps=self.cap.get(cv2.CAP_PROP_FPS);self.count=int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        offsets=[r['wall_time_unix_s']-self.first_wall for r in self.rows]
        if min(offsets)<-.1 or max(offsets)>self.count/self.fps+.3:raise ValueError('GUI does not cover telemetry')
        self.cursor=0;self.last=None
        profile=self.manifest.get('profile','outdoor')
        self.profile={'outdoor':'实车室外原参数','sim_adaptive':'仅仿真：提前自适应避障','sim_motion':'仅仿真：扫描运动估计＋原参数','sim_motion_yield':'NeuPAN＋扫描预测让行保护（仅仿真）'}.get(profile,profile)
        if 'features' in self.manifest:
            flags=self.manifest['features']
            self.profile='速度估计：{}  |  预测让行：{}'.format('开' if flags['motion_estimation'] else '关','开' if flags['predictive_yield'] else '关')
        self.title={'static':'静态绕障','crossing':'固定时序动态横穿','campus':'园区五动态障碍物','campus_dense':'园区八动态障碍物','campus_long':'园区十动态障碍物 · 60米'}.get(self.manifest['scenario'],self.manifest['scenario'])+' · 第{}轮'.format(self.repeat)
        self.base=np.full((1080,1920,3),BG,np.uint8)
        restart=self.manifest.get('features',{}).get('solver_initialization_restarts',self.manifest.get('simulation_overrides',{}).get('simulation_solver_restarts',False))
        tuning=' / 仿真专用参数' if self.manifest.get('planner_overrides') else ''
        self.base=labels(self.base,[(36,20,'NeuPAN 室外道路仿真 · '+self.title,40,(240,240,240)),
            (38,82,self.profile+'  |  参考速度 {:g} m/s  |  目标 ({:g}, {:g})'.format(self.manifest.get('planner_overrides',{}).get('ref_speed',.8),*self.manifest['goal']),26,(190,210,220)),
            (38,124,'仿真双初值扩展：'+('开（道路边界保护）'+tuning if self.manifest.get('features',{}).get('tracking_guard_mode')=='road_bounds' else '开（室外权重与模型不变）') if restart else '仿真双初值扩展：关',21,(190,210,220)),
            (1450,490,'实际 Gazebo 轨迹',25,(210,220,230)),
            (35,1020,'真实 Gazebo 窗口；按仿真时间播放，渲染有延迟。定位为真值，校园场景为生成几何。',25,(180,190,195))])

    def frame(self,t):
        i=max(0,bisect_right(self.times,t)-1);row=self.rows[i]
        wall=row['wall_time_unix_s']
        if i+1<len(self.rows):
            after=self.rows[i+1];alpha=max(0,min(1,(t-row['t'])/(after['t']-row['t'])))
            wall+=alpha*(after['wall_time_unix_s']-wall)
        wanted=max(0,min(self.count-1,int((wall-self.first_wall)*self.fps)))
        while self.cursor<=wanted:
            ok,picture=self.cap.read()
            if not ok:raise ValueError('GUI video truncated')
            self.last=picture;self.cursor+=1
        # Fixed Gazebo chrome crop; preserve the actual content aspect ratio.
        h,w=self.last.shape[:2];picture=self.last[round(h*.07):round(h*.945),round(w*.14):round(w*.99)]
        scale=min(1395/picture.shape[1],850/picture.shape[0]);nw,nh=round(picture.shape[1]*scale),round(picture.shape[0]*scale)
        frame=self.base.copy();x,y=24+(1395-nw)//2,140+(850-nh)//2
        frame[y:y+nh,x:x+nw]=cv2.resize(picture,(nw,nh),interpolation=cv2.INTER_AREA)
        finished=t>=self.times[-1]-.001
        status=REASONS.get(self.result['reason'],self.result['reason']) if finished else '运行中'
        if finished and self.result['passed'] and self.result['minimum_clearance_m']<.1:status='到达，但净距不足 0.1 m'
        color=(110,235,145) if finished and self.result['passed'] and self.result['minimum_clearance_m']>=.1 else (255,130,110) if finished else (240,220,130)
        frame=labels(frame,[(1450,150,status,26,color),
            (1450,210,'仿真时间：{:.1f} s'.format(row['t']),26,(235,235,235)),
            (1450,263,'车速：{:.2f} m/s'.format(row['speed']),26,(235,235,235)),
            (1450,316,'目标误差：{:.2f} m'.format(row['goal_error']),26,(235,235,235)),
            (1450,369,'当前净距：{:.3f} m'.format(row['body_clearance']),26,(235,235,235)),
            (1450,423,'扫描年龄：{:.3f} s'.format(row['scan_age']),24,(190,205,215))])
        guard=None
        if self.motion:
            mi=max(0,bisect_right(self.motion_times,t)-1)
            guard=self.motion[mi].get('predictive_yield')
        if guard is not None:
            frame=labels(frame,[(1450,765,'预测让行：'+('等待' if guard['holding'] else '放行'),24,(240,210,110) if guard['holding'] else (120,230,160))])
        if self.runtime:
            ri=max(0,bisect_right(self.runtime_times,t)-1)
            state=self.runtime[ri];tracking=state.get('tracking_guard') or {}
            if state.get('command_source')=='outdoor_tracking_guard':
                text='道路边界保护：触发' if tracking.get('mode')=='road_bounds' else '原生路径保护：触发'
                detail=('路界净距：{:.2f} m'.format(tracking.get('road_clearance_m',0)) if tracking.get('mode')=='road_bounds' else '横向偏离：{:.2f} m'.format(tracking.get('cross_track_m',0)))
                frame=labels(frame,[(1450,752,text,24,(255,155,100)),(1450,787,detail,22,(255,185,130))])
            elif state.get('planner_stop'):
                frame=labels(frame,[(1450,765,'NeuPAN 原生停止',24,(255,155,100))])
        bounds=self.manifest['road_bounds'];goal=self.manifest['goal']
        map_scale=min(430/(bounds[2]-bounds[0]),150/(bounds[3]-bounds[1]))
        def px(x,y):return (int(1460+(x-bounds[0])*map_scale),int(675-y*map_scale))
        cv2.rectangle(frame,px(bounds[0],bounds[3]),px(bounds[2],bounds[1]),(90,100,105),1)
        cv2.line(frame,px(0,0),px(*goal),(210,150,65),2)
        pts=np.array([px(*r['pose'][:2]) for r in self.rows[:i+1]],np.int32)
        if len(pts)>1:cv2.polylines(frame,[pts],False,(60,225,245),2)
        for obstacle in row.get('obstacles',[row['obstacle']]):cv2.circle(frame,px(*obstacle[:2]),max(2,round(obstacle[2]*map_scale)),(40,100,250),-1)
        cv2.circle(frame,px(*goal),7,(255,230,0),2)
        x,y,yaw=row['pose'];cv2.arrowedLine(frame,px(x,y),px(x+math.cos(yaw),y+math.sin(yaw)),(230,230,230),2)
        # Signed speed over the episode shows reverse/stop intervals honestly.
        left,top,width,height=1460,870,430,85
        cv2.rectangle(frame,(left,top),(left+width,top+height),(80,90,95),1)
        values=[(left+int(r['t']/max(self.times[-1],1)*width),top+int((1.5-r['speed'])/3*height)) for r in self.rows[:i+1]]
        cv2.line(frame,(left,top+height//2),(left+width,top+height//2),(90,100,110),1)
        if len(values)>1:cv2.polylines(frame,[np.array(values,np.int32)],False,(60,220,245),2)
        cv2.putText(frame,'v(t): -1.5 ... +1.5 m/s',(left,top-12),cv2.FONT_HERSHEY_SIMPLEX,.6,(190,200,210),1,cv2.LINE_AA)
        return frame


def export(root,folders):
    output=root/'presentation';output.mkdir(parents=True,exist_ok=True)
    destination=output/'neupan_outdoor_gazebo_1080p.mp4'
    clips=[Clip(p) for p in folders]
    proc=subprocess.Popen(['ffmpeg','-v','error','-y','-f','rawvideo','-pix_fmt','bgr24','-s','1920x1080','-r',str(FPS),
        '-i','pipe:0','-an','-c:v','libx264','-threads','2','-preset','fast','-crf','21','-pix_fmt','yuv420p','-vf','setsar=1','-movflags','+faststart',str(destination)],stdin=subprocess.PIPE)
    segments=[];count=0
    try:
        for clip in clips:
            duration=clip.times[-1];frames=math.ceil(duration*FPS)+2*FPS
            segments.append({'source':str(clip.folder),'profile':clip.manifest.get('profile','outdoor'),'scenario':clip.manifest['scenario'],
                'start_s':count/FPS,'duration_s':frames/FPS,'result':clip.result['reason'],
                'result_sha256':hashlib.sha256((clip.folder/'result.json').read_bytes()).hexdigest()})
            print('Exporting',clip.folder,flush=True)
            for i in range(frames):proc.stdin.write(clip.frame(min(i/FPS,duration)).tobytes());count+=1
        proc.stdin.close()
        if proc.wait()!=0:raise RuntimeError('Video encoder failed')
    finally:
        for clip in clips:clip.cap.release()
        if proc.poll() is None:proc.kill();proc.wait()
    (output/'video_manifest.json').write_text(json.dumps({'file':destination.name,'resolution':[1920,1080],'fps':FPS,
        'selection':'all explicitly listed trials, including failures','alignment':'X11 input timestamp mapped through measured wall/simulation telemetry; render latency remains',
        'segments':segments,'duration_s':count/FPS},ensure_ascii=False,indent=2)+'\n')
    print(destination,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('root',type=Path);p.add_argument('--trials',nargs='+',required=True,type=Path)
    a=p.parse_args();export(a.root.resolve(),[t.resolve() for t in a.trials])
