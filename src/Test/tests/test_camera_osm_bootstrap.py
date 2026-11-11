"""Production policy: start on OSM, accept camera later, retain accepted reference."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'src/osm_map_handler/src'))
from osm_camera_path_guard import CameraPathGuard, path_digest
from test_camera_publication_protocol import path


def production_guard():
    source=Path(__file__).resolve().parents[3]/'src/neupan_ros/src/neupan_core.py'
    tree=ast.parse(source.read_text())
    assignment=next(node for node in ast.walk(tree) if isinstance(node,ast.Assign)
        and any(isinstance(t,ast.Attribute) and t.attr=='camera_guard' for t in node.targets))
    instance=NS()
    params={'~camera_right_guard_enabled':True,'~camera_right_strategy':'segment_reference'}
    namespace=dict(self=instance,CameraPathGuard=CameraPathGuard,
        rospy=NS(get_param=lambda key,default=None:params.get(key,default)))
    exec(compile(ast.Module(body=[assignment],type_ignores=[]),str(source),'exec'),namespace)
    return instance.camera_guard


class OsmBootstrapTests(unittest.TestCase):
    def test_production_starts_on_original_without_camera_or_confirmed_reference(self):
        guard=production_guard();original=path();candidate=path(middle=.2)
        guard.original_cb(original)
        selected,initial_signature,state=guard.select(10.)
        self.assertEqual(state['source'],'osm')
        self.assertIs(selected,original)
        guard.candidate_cb(candidate)
        status=dict(control_enabled=True,route_stamp_ns=123,route_frame_id='map',
            original_path_digest=path_digest(original),control_path_digest=path_digest(candidate),
            inference_diagnostics=dict(reference_role='advisory',reference_version=0,
                reference_accepted=False,reference_accepted_stamp=None))
        guard.status_cb(status,10.)
        selected,signature,state=guard.select(10.)
        self.assertEqual(state['source'],'osm')
        self.assertEqual(signature,initial_signature) # no repeated OSM reload for pending camera
        self.assertIs(selected,original)
        self.assertFalse(state['reference_accepted'])
        status['inference_diagnostics'].update(reference_accepted=True,reference_accepted_stamp=10.)
        guard.status_cb(status,10.)
        self.assertEqual(guard.select(14.)[2]['source'],'osm') # stale cannot first accept
        selected,_,state=guard.select(10.)
        self.assertEqual(state['source'],'camera')
        self.assertIs(selected,candidate)
        # Occlusion/heartbeat loss and partially received replacement cannot undo acceptance.
        self.assertEqual(guard.select(20.)[2]['source'],'camera')
        guard.candidate_cb(path(middle=.4))
        self.assertEqual(guard.select(20.)[2]['source'],'camera')
        guard.original_cb(path(stamp=124))
        self.assertEqual(guard.select(20.)[2]['source'],'osm')


if __name__=='__main__':unittest.main()
