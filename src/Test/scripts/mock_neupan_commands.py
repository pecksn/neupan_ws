#!/usr/bin/env python3
"""Temporary indoor cloud test. Twist.angular.z is FRONT STEER radians.

Requires explicit enabled:=true and a running neupan_control node, not TF/GPS.
Does not spoof localisation/runtime or disable mux protection. Can move a car!
Native zero commands can interleave; a native solver result retires this test.
"""
import json
import math
import random
import threading
import time


class RandomCommands:
    def __init__(self, speed_max=.25, steer_max=.20, change_sec=2., seed=None):
        if not all(math.isfinite(v) for v in (speed_max,steer_max,change_sec)):
            raise ValueError('limits must be finite')
        if not 0 < speed_max <= .5 or not 0 < steer_max <= .35 or not .5 <= change_sec <= 10.:
            raise ValueError('speed_max: (0,0.5] m/s; steer_max: (0,0.35] rad; change_sec: [0.5,10] s')
        self.speed_max=speed_max;self.steer_max=steer_max;self.change_sec=change_sec
        self.random=random.Random(seed);self.next_change=None
        self.speed=self.steer=0.;self.target_speed=self.target_steer=0.

    def step(self, now, dt):
        if not math.isfinite(now) or not math.isfinite(dt) or dt < 0:
            raise ValueError('invalid monotonic time')
        if self.next_change is None or now >= self.next_change:
            self.target_speed=self.random.uniform(.05,self.speed_max) if self.speed_max>=.05 else self.speed_max
            self.target_steer=self.random.uniform(-self.steer_max,self.steer_max)
            self.next_change=now+self.change_sec
        def move(value,target,limit):
            return value+max(-limit,min(limit,target-value))
        # Bound the first step and timing pauses, not just random targets.
        dt=min(dt,.2)
        self.speed=move(self.speed,self.target_speed,.2*dt)
        self.steer=move(self.steer,self.target_steer,.1*dt)
        return self.speed,self.steer


def main():
    import rospy
    import rosgraph
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    rospy.init_node('mock_neupan_commands')
    if rospy.get_param('~enabled',False) is not True:
        rospy.logwarn('Mock disabled. Raise drive wheels/disconnect motors, then explicitly set enabled:=true.')
        return
    duration=float(rospy.get_param('~duration_sec',120.))
    rate=float(rospy.get_param('~rate_hz',10.))
    if not math.isfinite(duration) or not 0 < duration <= 3600 or not math.isfinite(rate) or not 5 <= rate <= 50:
        raise ValueError('duration_sec must be (0,3600]; rate_hz must be [5,50]')
    generator=RandomCommands(float(rospy.get_param('~speed_max_mps',.25)),
        float(rospy.get_param('~steer_max_rad',.20)),float(rospy.get_param('~change_sec',2.)),
        rospy.get_param('~seed',None))
    required_node=rospy.get_param('~required_node','/neupan_control')
    if not isinstance(required_node,str) or not required_node.startswith('/'):
        raise ValueError('required_node must be an absolute ROS node name')
    master=rosgraph.Master(rospy.get_name())
    retired=[False]
    gate=threading.RLock()
    def native_runtime(msg):
        try:
            row=json.loads(msg.data)
            # A real solver result, even native stop/arrive, takes ownership.
            if isinstance(row,dict) and row.get('solver_ms') is not None:
                with gate:retired[0]=True
        except (TypeError,ValueError):pass
    rospy.Subscriber('/neupan/runtime',String,native_runtime,queue_size=1)
    def ready():
        pubs,subs,services=master.getSystemState()
        return any(required_node in nodes for _,nodes in pubs+subs+services)
    rospy.logwarn('CLOUD MOCK: /cmd_vel/auto may MOVE THE VEHICLE. Raise drive wheels/disconnect motors. '
                  'Native zero commands may interleave; localisation/mux protection remains unchanged.')
    while not rospy.is_shutdown() and not retired[0]:
        if ready():break
        rospy.loginfo_throttle(3.,'Waiting for %s; no GPS/TF required',required_node)
        time.sleep(.1)
    if rospy.is_shutdown() or retired[0]:return
    publisher=rospy.Publisher('/cmd_vel/auto',Twist,queue_size=1,latch=False)
    def stop():
        if not rospy.is_shutdown():
            for _ in range(3):
                publisher.publish(Twist());time.sleep(.03)
        else:
            publisher.publish(Twist())
    rospy.on_shutdown(stop)
    begin=previous=time.monotonic();next_health=begin
    rospy.logwarn('MOCK RUNNING %.0fs, speed <= %.2fm/s, FRONT STEER <= %.1fdeg. '
                  'Ctrl+C stops; real NeuPAN solve or node disappearance also stops.',
                  duration,generator.speed_max,math.degrees(generator.steer_max))
    try:
        while not rospy.is_shutdown() and not retired[0]:
            now=time.monotonic()
            if now-begin>=duration:break
            if now>=next_health:
                if not ready():break
                next_health=now+1.
            speed,steer=generator.step(now,now-previous);previous=now
            command=Twist();command.linear.x=speed;command.angular.z=steer
            with gate:
                if retired[0]:break
                publisher.publish(command)
            rospy.loginfo_throttle(1.,'MOCK /cmd_vel/auto: v=%.3fm/s front_steer=%.3frad (%.1fdeg), remaining=%.0fs',
                speed,steer,math.degrees(steer),max(0.,duration-(now-begin)))
            time.sleep(1./rate) # Wall time works without /clock in an indoor test.
    finally:
        stop()
        rospy.logwarn('MOCK STOPPED: zero sent; no localisation or control-mode setting was changed.')


if __name__=='__main__':
    main()
