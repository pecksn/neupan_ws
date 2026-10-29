"""ROS-side lifetime/dispatch of an accepted NeuPAN control sequence.

No optimiser, reference generator or independent prediction model lives here.
The existing numeric control-continuity helper rechecks the remaining native
controls from a fresh pose. A sequence has an immutable origin and finite end.
"""
import math
import sys
import threading
import time
import numpy as np
from neupan_navigation import directional_stopping_clearance
from neupan_safety import (MotionContinuity, candidate_snapshot, continuous_solution,
    validate_solution, checked_input_ages, trajectory_clearance, check_motion_feedback,
    InputUnavailable, ResultExpired, CandidateRejected, AuthorizationChanged)


def execution_clearance(info, points, robot, dt, calibration=None, dispatch_period=.1):
    """Full sweep first; optionally classify a remote conflict using measurements.

    The stopping reserve is bounded directional occupancy, NOT another predicted
    detour. It uses the calibrated operating-speed upper bound, never a
    command/1 Hz interval average as a measured instantaneous speed. Missing
    calibration does not penalise candidates that pass the original full sweep.
    It covers known static returns only, not unknown/dynamic objects.
    """
    states=np.hstack(info['opt_state_list'])
    try:
        result=trajectory_clearance(states,points,robot.vertices)
        return dict(result,native_stop=bool(info.get('stop')),full_scan_rejection=False,
                    result_expired=False,scope='full_native_horizon',classification='full_clear')
    except CandidateRejected as error:
        details=dict(error.details,native_stop=bool(info.get('stop')),full_scan_rejection=True,
                     result_expired=False,scope='full_native_horizon',
                     first_conflict_time_sec=(error.details.get('step',0)+error.details.get('fraction',0.))*dt)
        if not calibration or info.get('stop') or info.get('arrive'):
            details['continuation_unavailable']='no_measured_stopping_reserve' if not calibration else 'native_stop'
            details['classification']='conflict_without_measured_reserve' if not calibration else 'native_stop'
            if not calibration and details.get('step')==0 and details.get('fraction')==0.:
                details['classification']='near_unsafe'
            raise CandidateRejected(str(error),details) from error
        if not all(name in calibration for name in DIRECTIONAL_FIELDS):
            details.update(classification='conflict_without_directional_bounds',
                           continuation_unavailable='no_measured_directional_stopping_bounds')
            raise CandidateRejected(str(error),details) from error
        speed=float(calibration['speed_upper_bound_mps'])
        delay=float(calibration['total_delay_upper_bound_sec'])+dispatch_period
        deceleration=float(calibration['braking_deceleration_lower_bound_mps2'])
        duration=delay+speed/deceleration
        steps=int(math.ceil(duration/dt))
        if steps>=states.shape[1]:
            details['classification']='insufficient_stopping_horizon'
            raise CandidateRejected('native horizon cannot cover measured stopping reserve',details) from error
        # Identical full-scan body/margin/interpolation definitions, no fixed 1s cut.
        try:near=trajectory_clearance(states[:,:steps+1],points,robot.vertices)
        except CandidateRejected as near_error:
            raise CandidateRejected(str(near_error),dict(details,classification='near_unsafe',near_conflict=near_error.details)) from near_error
        try:
            reserve=directional_stopping_clearance(states,info['vel_tensor'],points,robot,dt,calibration,dispatch_period)
        except CandidateRejected as reserve_error:
            raise CandidateRejected(str(reserve_error),dict(details,classification='stopping_reserve_unsafe',
                stopping_conflict=reserve_error.details)) from reserve_error
        return dict(near,native_stop=False,full_scan_rejection=True,result_expired=False,
            scope='near_native_horizon_and_measured_stopping_reserve',classification='far_conflict_near_and_stopping_clear',
            first_conflict=details,first_conflict_time_sec=details['first_conflict_time_sec'],
            calibration_id=calibration['calibration_id'],stopping_reserve_sec=duration,
            directional_stopping_reserve=reserve,requires_replanning=True)


DIRECTIONAL_FIELDS=('steering_error_upper_bound_rad','steering_rate_upper_bound_radps',
                    'steering_abs_upper_bound_rad','post_stop_clearance_m')


def validate_calibration(value, robot):
    """Fail configuration, not moving control, for invented/incomplete profiles."""
    if not isinstance(value,dict) or value.get('verified') is not True or not value.get('calibration_id'):
        raise ValueError('execution calibration requires verified=true and a measurement calibration_id')
    result=dict(value)
    for name in ('speed_upper_bound_mps','total_delay_upper_bound_sec',
                 'braking_deceleration_lower_bound_mps2'):
        result[name]=float(result[name])
        if not math.isfinite(result[name]) or result[name]<=0:
            raise ValueError('invalid measured execution calibration: '+name)
    if result['speed_upper_bound_mps']<float(np.asarray(robot.max_speed).reshape(2)[0]):
        raise ValueError('measured speed bound must cover configured vehicle maximum')
    if any(name in result for name in DIRECTIONAL_FIELDS):
        if not all(name in result for name in DIRECTIONAL_FIELDS):
            raise ValueError('directional stopping profile requires ALL measured steering/rest bounds')
        for name in DIRECTIONAL_FIELDS:
            result[name]=float(result[name])
            if not math.isfinite(result[name]) or result[name]<0:raise ValueError('invalid directional calibration: '+name)
        if (result['steering_rate_upper_bound_radps']<=0 or
                not float(np.asarray(robot.max_speed).reshape(2)[1])<=result['steering_abs_upper_bound_rad']<math.pi/2):
            raise ValueError('invalid measured steering rate/absolute bound')
    return result


def check_measured_reserve_limits(safety,calibration,ages,feedback=None):
    """A calibrated remote-conflict exception cannot exceed its measured bounds."""
    if not safety.get('requires_replanning'):return
    delay_ms=1000*calibration['total_delay_upper_bound_sec']
    if max(float(ages.get(key) or 0.) for key in
            ('scan_measurement_age_ms','scan_receive_age_ms','state_age_ms'))>delay_ms:
        raise CandidateRejected('current input/compute delay exceeds measured stopping profile',
                                dict(safety,classification='measured_delay_bound_exceeded'))
    measured=(feedback or {}).get('speed_mps')
    if measured is not None and abs(measured)>calibration['speed_upper_bound_mps']:
        raise CandidateRejected('measured speed exceeds stopping calibration operating bound',
                                dict(safety,classification='measured_speed_bound_exceeded'))


class RevokingPublisher:
    """Every explicit planner publication supersedes the accepted sequence.

    Invalidation and publication share the same final gate as the dispatcher;
    no nonzero command can slip behind a stop from another planner branch.
    """
    def __init__(self,publisher,execution):self.publisher=publisher;self.execution=execution
    def publish(self,message):
        with self.execution.gate():
            self.execution.revoke('explicit_planner_command')
            self.publisher.publish(message)
    def __getattr__(self,name):return getattr(self.publisher,name)


class AcceptedExecution:
    """Finite native sequence; independent checking while the solver is blocked.

    inputs/authority/gate/emit are injected to keep ROS, TF and route ownership
    in the ROS wrapper. Only stage() installs a NEW solve; tick() never renews
    its origin, expiry or consumed progress. No last-speed heartbeat fallback.
    """
    def __init__(self,robot,dt,policy,inputs,authority,gate,emit,report,
                 timeout=.5,period=.1,calibration=None,clock=time.monotonic):
        self.robot=robot;self.dt=dt;self.policy=policy;self.inputs=inputs
        self.authority=authority;self.gate=gate;self.emit=emit;self.report=report
        self.timeout=timeout;self.period=period;self.calibration=calibration;self.clock=clock
        self.lock=threading.RLock();self.check_lock=threading.Lock();self.closed=threading.Event()
        self.active=None;self.version=0;self.last_send=None;self.last_result={};self.fault=None
        self.fault_info=None
        self.input_failure=None

    def revoke(self,reason):
        with self.lock:
            self.version+=1;self.active=None
            self.last_result=dict(continued=False,reason=reason,execution_version=self.version)

    def stage(self,info,context,origin,action=None,final_check=None):
        if info.get('stop') or info.get('arrive'):
            self.revoke('native_stop_or_arrival');return False
        frozen=candidate_snapshot(info)
        with self.gate(),self.lock:
            if self.closed.is_set() or self.fault or self.input_failure or context is None or context!=self.authority():
                raise AuthorizationChanged('execution authority changed during solve')
            if action is not None:
                history=getattr(self.policy,'history',None)
                if history is not None:
                    allowance=np.asarray(self.robot.max_acce).reshape(2)*max(0.,min(.5,self.clock()-history[1]))
                    if np.any(np.abs(np.asarray(action).reshape(2)-history[0])>allowance+1.e-4):
                        raise AuthorizationChanged('last sent action advanced during solve; recheck continuity')
                if final_check is not None:final_check()
            self.version+=1
            now=self.clock()
            self.active=dict(info=frozen,context=context,origin=dict(origin),version=self.version,
                installed=now,expires=now+frozen['vel_tensor'].shape[1]*self.dt)
            self.last_result=dict(continued=False,reason='new_native_sequence',execution_version=self.version)
            if action is not None:
                self.emit(action);self.last_send=self.clock();self.policy.record(action,self.last_send)
                self.last_result.update(continued=True,reason='new_native_first_action',origin=dict(origin))
        return True

    def _diagnostic(self,result):
        # A diagnostic transport failure must not bypass or undo a stop.
        try:self.report(result)
        except Exception:pass

    def tick(self):
        if not self.check_lock.acquire(False):return False
        active=None;checks=[];live_health=None;retry=0
        try:
            with self.lock:active=self.active
            if active is None:return False
            for retry in range(2):
                item=None
                try:
                    # Rebind the WHOLE pose+scan, never just the stamp, outside
                    # publication locks. At most one complete extra recheck.
                    live=self.inputs();now=self.clock()
                    with self.lock:
                        if self.active is not active:return False
                    if active['context']!=live['context']:
                        raise AuthorizationChanged('route/authority changed')
                    if not 0<=now-active['installed'] or active['expires']-now<=self.timeout:
                        raise InputUnavailable('native sequence finite coverage exhausted')
                    item=dict(attempt=retry,scan_stamp_ns=live['scan'][1],state_stamp_ns=live['state_stamp_ns'],
                        bind_scan_age_ms=(live['now_ns']()-live['scan'][1])/1.e6,
                        bind_receive_age_ms=(now-live['scan'][2])*1000,
                        bind_state_age_ms=(live['now_ns']()-live['state_stamp_ns'])/1.e6)
                    checks.append(item)
                    checked_input_ages(live['scan'],live['now_ns'](),now,self.timeout,live['state_stamp_ns'])
                    index=int((now-active['installed'])/self.dt)
                    info=candidate_snapshot(active['info'])
                    info['vel_tensor']=info['vel_tensor'][:,index:].copy()
                    info['opt_state_list']=[np.asarray(live['pose']).reshape(3,1).copy()]
                    policy=MotionContinuity();policy.cap=self.policy.cap
                    history=getattr(self.policy,'history',None)
                    if history is not None:policy.record(*history)
                    policy.feedback_provider=self.policy.feedback_provider
                    action,info=continuous_solution(info['vel_tensor'][:,:1],info,self.robot,self.dt,policy,now)
                    validate_solution(action,info,self.robot,self.dt)
                    safety=execution_clearance(info,live['scan'][0],self.robot,self.dt,self.calibration,self.period)
                    with self.gate(),self.lock:
                        if self.active is not active:return False
                        if active['context']!=self.authority():
                            raise AuthorizationChanged('authority changed during execution recheck')
                        check_motion_feedback(policy,action,self.robot,self.dt)
                        # Last exact-source gate; TF/geometry never under lock.
                        ages=checked_input_ages(live['scan'],live['now_ns'](),self.clock(),self.timeout,live['state_stamp_ns'])
                        check_measured_reserve_limits(safety,self.calibration,ages,policy.publication_feedback
                            if hasattr(policy,'publication_feedback') else None)
                        if active['expires']-self.clock()<=self.timeout:
                            raise InputUnavailable('native sequence expired during recheck')
                        self.emit(action)
                        self.last_send=self.clock();self.policy.record(action,self.last_send)
                        self.last_result=dict(continued=True,execution_version=active['version'],
                            origin=active['origin'],consumed_steps=index,remaining_sec=active['expires']-self.last_send,
                            checked_state_stamp_ns=live['state_stamp_ns'],command_sent_monotonic=self.last_send,
                            linear_x=float(action[0,0]),steering_rad=float(action[1,0]),
                            publish_input_ages=ages,trajectory_safety=safety,retry_count=retry,
                            rechecks=checks,result_expired=bool(retry),classification='accepted')
                        result=dict(self.last_result)
                    self._diagnostic(result)
                    return True
                except ResultExpired as error:
                    if item is None:
                        item=dict(attempt=retry,binding_failed=True);checks.append(item)
                    item.update(expired_reason=str(error),expired_at_mono=self.clock())
                    if retry==0:continue
                    # Two aged checks are not proof that CURRENT input is bad.
                    # This final bind is health-only, never used to authorise a
                    # command or renew the finite sequence.
                    try:
                        fresh=self.inputs()
                        live_health=checked_input_ages(fresh['scan'],fresh['now_ns'](),self.clock(),
                                                     self.timeout,fresh['state_stamp_ns'])
                    except InputUnavailable as current_error:
                        raise InputUnavailable('current mandatory inputs invalid after recheck expiry: '+str(current_error)) from current_error
                    if active['context']!=fresh['context']:
                        raise AuthorizationChanged('authority changed during expiry recovery')
                    raise ResultExpired('two rechecks expired; live inputs healthy; recompute without input recovery') from error
        except ResultExpired as error:
            with self.gate(),self.lock:
                if self.active is not active:return False
                classification='result_expired_live_healthy';kind='result_expired'
                if active['context']!=self.authority():
                    self.revoke('authority changed after expiry rebind')
                    classification=kind='authority'
                elif active['expires']-self.clock()<=self.timeout:
                    self.revoke('native sequence exhausted during expiry retries')
                    classification=kind='coverage'
                # No qualified action THIS tick. Preserve the finite segment
                # for another full check, not an old speed heartbeat.
                self.emit(np.zeros((2,1)));self.policy.record([0.,0.],self.clock())
                self.last_result=dict(continued=False,reason=str(error),fault_kind=kind,
                    classification=classification,result_expired=True,
                    execution_version=self.version,origin=active['origin'],retry_count=1,
                    rechecks=checks,rebound_input_ages=live_health,linear_x=0.,steering_rad=0.)
                result=dict(self.last_result)
            self._diagnostic(result)
            return False
        except (InputUnavailable,CandidateRejected,AuthorizationChanged) as error:
            with self.gate(),self.lock:
                if self.active is not active:return False
                self.revoke(str(error));self.emit(np.zeros((2,1)));self.policy.record([0.,0.],self.clock())
                kind=('geometry' if isinstance(error,CandidateRejected) else 'authority' if isinstance(error,AuthorizationChanged)
                      else 'coverage' if str(error).startswith('native sequence') else 'input')
                if kind=='input':self.input_failure=str(error)
                result=dict(self.last_result,fault_kind=kind,classification=kind,linear_x=0.,steering_rad=0.,
                    rechecks=checks,retry_count=retry,result_expired=any('expired_reason' in item for item in checks),
                    details=getattr(error,'details',{}))
                self.last_result=result
            self._diagnostic(result)
            return False
        except Exception as error:
            with self.gate(),self.lock:
                self.fault='%s: %s'%(type(error).__name__,error)
                self.fault_info=sys.exc_info()
                self.revoke('execution_internal_fault')
                self.emit(np.zeros((2,1)));self.policy.record([0.,0.],self.clock())
                result=dict(self.last_result,fault_kind='internal',linear_x=0.,steering_rad=0.,detail=self.fault)
            self._diagnostic(result)
            return False
        finally:self.check_lock.release()

    def resume(self):
        if self.tick():return True
        # A check already in progress may publish soon; this is NOT a new lease.
        with self.gate(),self.lock:
            active=self.active;now=self.clock()
            return bool(active is not None and active['context']==self.authority()
                and self.last_result.get('continued') is True
                and self.last_send is not None and 0<=now-self.last_send<2*self.period
                and active['expires']-now>self.timeout)

    def start(self):
        def dispatch():
            while not self.closed.wait(self.period):self.tick()
        self.thread=threading.Thread(target=dispatch,name='neupan-execution',daemon=True)
        self.thread.start()

    def close(self):
        self.closed.set()
        with self.gate():self.revoke('shutdown')
