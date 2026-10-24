#!/usr/bin/env python3
"""Boundary cases for the independent simulation road collision evaluator."""
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'neupan_ros/example/lxw_car_simulation/scripts'))
from outdoor_road_simulation import body_clearance,corners,resolve_features,scenario_actors,actor_position
from outdoor_osm_world import triangles

class RoadGeometry(unittest.TestCase):
    def test_front_tangent(self):
        self.assertAlmostEqual(body_clearance((0,0,0),(1.21,0,.4)),0)
    def test_rear_tangent(self):
        self.assertAlmostEqual(body_clearance((0,0,0),(-.54,0,.4)),0)
    def test_side_tangent(self):
        self.assertAlmostEqual(body_clearance((0,0,0),(.335,.825,.4)),0)
    def test_inside(self):
        self.assertLess(body_clearance((0,0,0),(.335,0,.4)),0)
    def test_corner_distance(self):
        self.assertAlmostEqual(body_clearance((0,0,0),(1.11,.825,.4)),.1)
    def test_rotation_translation_invariance(self):
        angle=.73;c,s=math.cos(angle),math.sin(angle)
        point=(1.2,.8,.4)
        transformed=(4+c*point[0]-s*point[1],7+s*point[0]+c*point[1],.4)
        self.assertAlmostEqual(body_clearance((4,7,angle),transformed),body_clearance((0,0,0),point))
    def test_rear_axle_not_body_center(self):
        self.assertEqual(sorted(set(round(p[0],3) for p in corners((0,0,0)))),[-.14,.81])
    def test_rotated_road_envelope(self):
        pts=corners((0,0,math.pi/2))
        self.assertAlmostEqual(max(y for _,y in pts),.81)
        self.assertAlmostEqual(min(x for x,_ in pts),-.425)

    def test_concave_building_mesh_preserves_footprint_area(self):
        _,faces=triangles([(0,0),(4,0),(4,1),(1,1),(1,4),(0,4),(0,0)])
        area=sum(abs((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))/2
                 for a,b,c in faces)
        self.assertAlmostEqual(area,7)

    def test_degenerate_building_is_rejected(self):
        with self.assertRaises(ValueError):
            triangles([(0,0),(1,1),(2,2)])

class FeatureSwitches(unittest.TestCase):
    def test_explicit_off_overrides_enabled_profile_and_selects_native(self):
        params,node=resolve_features({'controller_type':'outdoor_motion_controller.py','node_parameters':{'simulation_predictive_yield':True}},False,False)
        self.assertEqual(node,'neupan_node.py')
        self.assertFalse(params['simulation_motion_estimation'])
        self.assertFalse(params['simulation_predictive_yield'])
    def test_four_combinations_are_independent(self):
        for motion in (False,True):
            for yielding in (False,True):
                params,node=resolve_features({'node_parameters':{}},motion,yielding)
                self.assertEqual(params['simulation_motion_estimation'],motion)
                self.assertEqual(params['simulation_predictive_yield'],yielding)
                self.assertEqual(node,'outdoor_motion_controller.py' if motion or yielding else 'neupan_node.py')
    def test_scan_alignment_does_not_turn_on_either_motion_feature(self):
        params,node=resolve_features({'controller_type':'outdoor_motion_controller.py','node_parameters':{'simulation_scan_alignment':True,'simulation_motion_estimation':False,'simulation_predictive_yield':False}})
        self.assertEqual(node,'outdoor_motion_controller.py')
        self.assertFalse(params['simulation_motion_estimation'])
        self.assertFalse(params['simulation_predictive_yield'])

    def test_loaded_control_file_is_not_overwritten_by_inline_launch_defaults(self):
        import xml.etree.ElementTree as ET
        launch=Path(__file__).resolve().parents[2]/'neupan_ros/example/lxw_car_simulation/launch/outdoor_simulation.launch'
        node=next(n for n in ET.parse(launch).getroot().iter('node') if n.get('type')=='$(arg controller_type)')
        self.assertIsNotNone(node.find("rosparam[@file='$(arg control_parameters)']"))
        for param in node.findall('param'):
            if param.get('name') not in ('map_origin_lat','map_origin_lon'):
                self.assertEqual(param.get('unless'),"$(eval arg('control_parameters') != '')",param.get('name'))

    def test_campus_has_five_moving_actors_with_bounded_fixed_routes(self):
        actors=scenario_actors('campus')
        self.assertEqual(len(actors),5)
        self.assertEqual(len({a['name'] for a in actors}),5)
        for actor in actors:
            self.assertGreater(actor['speed'],0)
            self.assertEqual(actor_position(actor,0),tuple(actor['start']))
            self.assertEqual(actor_position(actor,100),tuple(actor['end']))

if __name__=='__main__':unittest.main()
