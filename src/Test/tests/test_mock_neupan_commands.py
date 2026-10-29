"""Pure mock-command tests: no ROS master and no vehicle outputs."""
import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock,patch

spec=importlib.util.spec_from_file_location('mock_neupan_commands',Path(__file__).parents[1]/'scripts/mock_neupan_commands.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class MockCommandTests(unittest.TestCase):
    def run_fake_ros(self,enabled,takeover=False):
        clock=[100.];sent=[];callbacks={}
        class Twist:
            def __init__(self):self.linear=NS(x=0.);self.angular=NS(z=0.)
        def publish(msg):
            sent.append((msg.linear.x,msg.angular.z))
            if takeover and msg.linear.x>0:
                callbacks['runtime'](NS(data='{"solver_ms": 12, "command_source": "neupan_native"}'))
        publisher=Mock(return_value=NS(publish=publish))
        ros=NS(init_node=Mock(),get_param=lambda key,default=None:enabled if key=='~enabled' else
            .5 if key=='~duration_sec' else default,
            get_name=lambda:'/mock_neupan_commands',is_shutdown=lambda:False,
            loginfo=Mock(),logwarn=Mock(),loginfo_throttle=Mock(),on_shutdown=Mock(),Publisher=publisher,
            Subscriber=lambda topic,kind,callback,**kwargs:callbacks.update(runtime=callback))
        graph=NS(Master=lambda _:NS(getSystemState=lambda:([('/neupan_cmd_vel',['/neupan_control'])],[],[])))
        modules={'rospy':ros,'rosgraph':graph,'geometry_msgs':NS(),'geometry_msgs.msg':NS(Twist=Twist),
                 'std_msgs':NS(),'std_msgs.msg':NS(String=object)}
        with patch.dict(sys.modules,modules),patch.object(module,'time',NS(
                monotonic=lambda:clock[0],sleep=lambda dt:clock.__setitem__(0,clock[0]+dt))):
            module.main()
        return publisher,sent,ros

    def test_disabled_is_no_publisher_and_no_zero_interference(self):
        publisher,sent,_=self.run_fake_ros(False)
        publisher.assert_not_called();self.assertFalse(sent)

    def test_timeout_and_native_takeover_both_end_with_zero(self):
        for takeover in (False,True):
            publisher,sent,ros=self.run_fake_ros(True,takeover=takeover)
            self.assertEqual(publisher.call_args.args[0],'/cmd_vel/auto')
            self.assertTrue(any(speed>0 for speed,_ in sent))
            self.assertEqual(sent[-3:],[(0.,0.)]*3)
            self.assertEqual(ros.on_shutdown.call_count,1)
            if takeover:
                first=next(i for i,(speed,_) in enumerate(sent) if speed>0)
                self.assertTrue(all(speed==0 for speed,_ in sent[first+1:]))

    def test_bounds_and_slew(self):
        model=module.RandomCommands(seed=1);old=(0.,0.)
        for i in range(1000):
            speed,steer=model.step(i*.1,.1)
            self.assertTrue(0<=speed<=.25);self.assertLessEqual(abs(steer),.20)
            self.assertLessEqual(abs(speed-old[0]),.020000001)
            self.assertLessEqual(abs(steer-old[1]),.010000001)
            old=(speed,steer)

    def test_random_targets_are_held_not_changed_every_frame(self):
        model=module.RandomCommands(seed=1);model.step(0.,.1)
        old=(model.target_speed,model.target_steer)
        model.step(1.,.1);self.assertEqual(old,(model.target_speed,model.target_steer))
        model.step(2.,.1);self.assertNotEqual(old,(model.target_speed,model.target_steer))

    def test_invalid_limits_and_time(self):
        for kwargs in (dict(speed_max=math.nan),dict(speed_max=1.),dict(steer_max=.6),dict(change_sec=0.)):
            with self.assertRaises(ValueError):module.RandomCommands(**kwargs)
        for now,dt in ((math.inf,.1),(0.,-.1),(0.,math.nan)):
            with self.assertRaises(ValueError):module.RandomCommands().step(now,dt)

    def test_clock_pause_does_not_jump_to_target(self):
        model=module.RandomCommands(seed=2)
        speed,steer=model.step(100.,100.)
        self.assertLessEqual(speed,.04+.000001);self.assertLessEqual(abs(steer),.02+.000001)


if __name__=='__main__':unittest.main()
