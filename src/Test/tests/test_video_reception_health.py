"""Frame-driven reception state, no UDP, ROS master or vehicle startup."""
import ast
import math
from pathlib import Path
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

SOURCE = Path(__file__).resolve().parents[3]/'src/hybridnets_receiver/scripts/rtp_camera_receiver.py'
tree = ast.parse(SOURCE.read_text())
namespace = dict(math=math, threading=threading, time=NS(monotonic=lambda:1.))
classes = ast.Module(body=[v for v in tree.body if isinstance(v,ast.ClassDef)],type_ignores=[])
exec(compile(classes,str(SOURCE),'exec'),namespace)
Health = namespace['VideoReceptionHealth']


class VideoReceptionHealthTests(unittest.TestCase):
    def test_no_first_frame_is_waiting_not_a_disconnect(self):
        health=Health(0.)
        self.assertEqual(health.report(60.)['state'],'waiting')
        self.assertEqual(health.report(61.)['fps'],0.)

    def test_fps_comes_from_frames_not_nominal_thirty_fps(self):
        health=Health(0.)
        for i in range(20):health.accept((i+1)/20.)
        result=health.report(1.)
        self.assertEqual(result['state'],'receiving')
        self.assertEqual(result['fps'],20.)
        self.assertEqual(result['total_frames'],20)

    def test_exact_three_seconds_without_frames_is_disconnected(self):
        health=Health(0.);health.accept(1.)
        self.assertEqual(health.report(3.99)['state'],'receiving')
        result=health.report(4.)
        self.assertEqual(result['state'],'disconnected')
        self.assertEqual(result['fps'],0.)

    def test_recovery_is_driven_by_a_new_frame(self):
        health=Health(0.);health.accept(1.);health.report(4.)
        self.assertEqual(health.report(10.)['state'],'disconnected')
        health.accept(10.5)
        result=health.report(11.)
        self.assertEqual(result['state'],'receiving')
        self.assertEqual(result['fps'],1.)

    def test_zero_elapsed_time_is_finite(self):
        health=Health(1.);health.accept(1.)
        self.assertEqual(health.report(1.)['fps'],0.)

    def test_invalid_timeouts_rejected(self):
        for timeout in (0.,-1.,math.nan,math.inf):
            with self.assertRaises(ValueError):Health(0.,timeout)

    def sample(self,width=2,size=12):
        cls=namespace['RtpCameraReceiver'];node=cls.__new__(cls)
        node.expected_width=node.expected_height=2
        node.image_pub=NS(publish=Mock());node.info_pub=NS(publish=Mock())
        node.camera_info=NS(header=NS());node.frame_id='camera'
        node.queue_virtual_frame=Mock();node.reception_health=Health(0.)
        buffer=NS(map=lambda _: (True,NS(data=b'x'*size)),unmap=Mock())
        sample=NS(get_caps=lambda:NS(get_structure=lambda _:NS(get_value=lambda k:width if k=='width' else 2)),get_buffer=lambda:buffer)
        namespace.update(Gst=NS(FlowReturn=NS(ERROR='error',OK='ok'),MapFlags=NS(READ=1)),
            rospy=NS(Time=NS(now=lambda:1.),logerr_throttle=Mock()),Image=lambda:NS(header=NS()))
        node.on_sample(NS(emit=lambda _:sample))
        return node,buffer

    def test_only_valid_published_frames_count(self):
        node,buffer=self.sample()
        self.assertEqual(node.reception_health.report(1.)['total_frames'],1)
        node.image_pub.publish.assert_called_once()
        buffer.unmap.assert_called_once()

    def test_wrong_resolution_does_not_claim_reception_success(self):
        node,_=self.sample(width=4)
        self.assertEqual(node.reception_health.report(1.)['state'],'waiting')
        node.image_pub.publish.assert_not_called()

    def test_malformed_buffer_does_not_count_as_a_valid_frame(self):
        node,buffer=self.sample(size=1)
        self.assertEqual(node.reception_health.report(1.)['state'],'waiting')
        buffer.unmap.assert_called_once()


if __name__ == '__main__':
    unittest.main()
