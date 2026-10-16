"""Regression tests for outdoor return-route admission; never drives hardware."""
import ast
from collections import deque
import importlib.util
import math
import os
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'src/osm_map_handler/src'))
from osm_outdoor_start_connection import (
    RoadCorridor, ScanEnvelope, StartRouteConnector,
    clean_route, dubins_candidates, footprint_points, wrap)
from osm_camera_path_guard import CameraPathGuard, path_digest

VERTICES = np.array([[-.14, .81, .81, -.14], [-.425, -.425, .425, .425]])


def route(yaw=0., origin=(0., 0.)):
    distance = np.linspace(0., 20., 41)
    return np.column_stack((origin[0]+distance*math.cos(yaw), origin[1]+distance*math.sin(yaw),
                            np.full(len(distance), yaw), np.ones(len(distance))))


def scan(now=1., distance=15., pose=(0., 0., 0.)):
    return ScanEnvelope(np.full(1441, distance), -math.pi, math.pi/720,
                        .05, 30., pose, now, now)


def production_scan(distance=15., pose=(0., 0., 0.)):
    return ScanEnvelope(np.full(723, distance), -3.14159, .0087,
                        .4, 30., pose, 1., 1.)


def connector(width=20.):
    return StartRouteConnector(VERTICES, 1.85, RoadCorridor([[-30, 0, 30, 0, width/2, 0]]))


class ConnectionTests(unittest.TestCase):
    def test_clean_duplicate_positions_without_changing_gear(self):
        result = clean_route([[0, 0, 0, 1], [0, 0, 0, 1], [1, 0, 0, 1]])
        self.assertEqual(len(result), 2)
        self.assertTrue(np.all(result[:, 3] == 1))

    def test_degenerate_or_nonfinite_reference_is_rejected(self):
        for path in ([[0, 0, 0, 1]]*2, [[0, 0, 0, 1], [np.nan, 0, 0, 1]],
                     [[0, 0, 0, 1], [1, 0, 0, -1]]):
            with self.assertRaises(ValueError):
                clean_route(path)

    def test_forward_route_remains_unchanged(self):
        state = connector()
        source = route()
        decision, result = state.prepare(source, ('map', 1), (0, 0, 0), 1, 1)
        self.assertEqual(decision, 'apply')
        np.testing.assert_array_equal(result, source)
        self.assertEqual(state.reason, 'normal_route_tracking')

    def test_nonfinite_vehicle_pose_is_held_before_solver(self):
        state = connector()
        self.assertEqual(state.prepare(route(), ('map', 1), (np.nan, 0, 0), 1, 1), ('reject', None))
        self.assertEqual(state.phase, 'blocked')

    def test_sparse_route_uses_segment_distance_not_distance_to_a_vertex(self):
        state = connector()
        source = np.array([[0., 0., 0., 1.], [2.5, 0., 0., 1.], [5., 0., 0., 1.]])
        decision, result = state.prepare(source, ('map', 1), (1.25, 0., 0.), 1., 1.)
        self.assertEqual(decision, 'apply')
        np.testing.assert_array_equal(result, source)
        self.assertEqual(state.reason, 'normal_route_tracking')
        self.assertAlmostEqual(state.start_gap, 0.)
        self.assertAlmostEqual(state.status()['nearest_vertex_gap_m'], 1.25)

    def test_logged_same_direction_offset_does_not_start_a_uturn_search(self):
        backend = SimpleNamespace(cancel=Mock(), submit=Mock())
        for yaw_error in (2.1764, 2.2978, 2.4218, 2.4044):
            state = StartRouteConnector(VERTICES, 1.85,
                RoadCorridor([[-30, 0, 30, 0, 3., 0]]), search=backend)
            state.scan = production_scan(distance=np.inf)  # unknown, never relabelled free
            source = route(origin=(0., 1.46))
            original_ranges = state.scan.ranges.copy()
            decision, result = state.prepare(source, ('map', 1), (0., 0., math.radians(yaw_error)), 1., 1.)
            self.assertEqual(decision, 'apply')
            self.assertEqual(state.phase, 'following_route')
            self.assertEqual(state.reason, 'normal_route_tracking')
            self.assertEqual(state.status()['connection_kind'], 'normal_route')
            self.assertFalse(state.status()['uturn_required'])
            self.assertFalse(state.admission['swept_path_prevalidated'])
            self.assertEqual(state.admission['scan_policy'], 'normal_neupan_scan_safety')
            np.testing.assert_array_equal(result, source)
            np.testing.assert_array_equal(state.scan.ranges, original_ranges)
        backend.submit.assert_not_called()

    def test_reverse_heading_with_same_offset_still_requires_strict_observation(self):
        state = connector(12.)
        source = route(math.pi, origin=(0., 1.46))
        state.scan = production_scan(distance=np.inf)
        self.assertEqual(state.prepare(source, ('map', 1), (0., 0., 0.), 1., 1.), ('reject', None))
        self.assertEqual(state.phase, 'blocked')
        self.assertEqual(state.connection_kind, 'uturn')
        self.assertTrue(state.uturn_required)

    def test_non_uturn_does_not_add_an_osm_body_admission_gate(self):
        backend = SimpleNamespace(cancel=Mock(), submit=Mock())
        state = StartRouteConnector(VERTICES, 1.85,
            RoadCorridor([[-30, 0, 30, 0, 1., 0]]), search=backend)
        state.scan = production_scan()
        source = route()
        self.assertFalse(state.corridor.contains([[0., 1.46, 0.]], VERTICES))
        decision, result = state.prepare(source, ('map', 1), (0., 1.46, 0.), 1., 1.)
        self.assertEqual(decision, 'apply')
        np.testing.assert_array_equal(result, source)
        backend.submit.assert_not_called()

    def test_large_offset_alone_keeps_original_tracking_without_map_or_scan(self):
        for gap in (.76, 1.46, 2.01, 5., 10.):
            with self.subTest(gap=gap):
                backend = SimpleNamespace(cancel=Mock(), submit=Mock())
                state = StartRouteConnector(VERTICES, 1.85, None, search=backend)
                source = route(origin=(0., gap))
                decision, result = state.prepare(source, ('map', 1), (0., 0., 0.), 1., 1.)
                self.assertEqual(decision, 'apply')
                np.testing.assert_array_equal(result, source)
                self.assertFalse(state.uturn_required)
                backend.submit.assert_not_called()

    def test_normal_corners_and_non_reversed_headings_never_start_search(self):
        for degrees in (0., 20., 21., 45., 50., 90., 120., 134.9, -50., -90., -120., -134.9):
            with self.subTest(degrees=degrees):
                backend = SimpleNamespace(cancel=Mock(), submit=Mock())
                state = StartRouteConnector(VERTICES, 1.85, None, search=backend)
                source = route(math.radians(degrees))
                decision, result = state.prepare(source, ('map', 1), (0., 0., 0.), 1., 1.)
                self.assertEqual(decision, 'apply')
                self.assertEqual(state.reason, 'normal_route_tracking')
                np.testing.assert_array_equal(result, source)
                backend.submit.assert_not_called()

    def test_goal_behind_alone_does_not_start_search_if_route_is_not_reversed(self):
        backend = SimpleNamespace(cancel=Mock(), submit=Mock())
        state = StartRouteConnector(VERTICES, 1.85, None, search=backend)
        source = route()
        decision, result = state.prepare(source, ('map', 1), (21.2, 0., 0.), 1., 1.)
        self.assertEqual(decision, 'apply')
        self.assertGreater(state.projection_overrun, .25)
        self.assertEqual(state.connection_kind, 'normal_route')
        np.testing.assert_array_equal(result, source)
        backend.submit.assert_not_called()

    def test_only_reversed_route_at_or_above_135_degrees_uses_search(self):
        for degrees in (135., 150., 175., 180., -135., -150., -175., -180.):
            with self.subTest(degrees=degrees):
                backend = SimpleNamespace(cancel=Mock(), submit=Mock(), budget_sec=1., max_iterations=12000)
                state = StartRouteConnector(VERTICES, 1.85,
                    RoadCorridor([[-30, 0, 30, 0, 10., 0]]), search=backend)
                state.scan = production_scan()
                self.assertEqual(state.prepare(route(math.radians(degrees)), ('map', 1), (0., 0., 0.), 1., 1.)[0], 'wait')
                self.assertEqual(state.phase, 'searching')
                self.assertEqual(state.connection_kind, 'uturn')
                self.assertTrue(state.status()['uturn_required'])
                self.assertEqual(state.status()['uturn_heading_threshold_deg'], 135.)
                backend.submit.assert_called_once()

    def test_dubins_comparator_is_also_uturn_only(self):
        candidates = Mock(side_effect=AssertionError('non-U-turn searched'))
        state = StartRouteConnector(VERTICES, 1.85, None, candidates=candidates)
        source = route(math.pi/2, origin=(0., 5.))
        self.assertEqual(state.prepare(source, ('map', 1), (0., 0., 0.), 1., 1.)[0], 'apply')
        candidates.assert_not_called()

    def test_non_uturn_does_not_read_added_corridor_or_scan_validators(self):
        corridor = SimpleNamespace(contains=Mock(side_effect=AssertionError('new road gate')))
        evidence = SimpleNamespace(fresh=Mock(side_effect=AssertionError('new scan gate')))
        state = StartRouteConnector(VERTICES, 1.85, corridor)
        state.scan = evidence
        self.assertEqual(state.prepare(route(math.pi/2), ('map', 1), (0., 0., 0.), 1., 1.)[0], 'apply')
        corridor.contains.assert_not_called()
        evidence.fresh.assert_not_called()

    def test_non_uturn_replacement_cancels_pending_uturn_search(self):
        backend = SimpleNamespace(cancel=Mock(), submit=Mock(), budget_sec=1., max_iterations=12000)
        state = StartRouteConnector(VERTICES, 1.85,
            RoadCorridor([[-30, 0, 30, 0, 10., 0]]), search=backend)
        state.scan = production_scan()
        state.prepare(route(math.pi), ('map', 1), (0., 0., 0.), 1., 1.)
        self.assertEqual(state.phase, 'searching')
        backend.cancel.reset_mock()
        source = route(math.pi/2)
        decision, result = state.prepare(source, ('map', 2), (0., 0., 0.), 1., 1.)
        self.assertEqual(decision, 'apply')
        np.testing.assert_array_equal(result, source)
        backend.cancel.assert_called_once()
        backend.submit.assert_called_once()  # only the previous U-turn

    def test_deprecated_normal_rejoin_parameters_are_not_exposed(self):
        root = ET.parse(ROOT/'src/neupan_ros/example/lxw_car/launch/full_system_run.launch').getroot()
        names = {arg.get('name') for arg in root.findall('arg')}
        self.assertNotIn('outdoor_normal_rejoin_heading_deg', names)
        self.assertNotIn('outdoor_normal_rejoin_max_gap_m', names)

    def test_reverse_heading_is_not_admitted_without_scan(self):
        state = connector()
        self.assertEqual(state.prepare(route(math.pi), ('map', 1), (0, 0, 0), 1, 1)[0], 'wait')
        self.assertEqual(state.prepare(route(math.pi), ('map', 1), (0, 0, 0), 7, 7)[0], 'reject')
        self.assertEqual(state.phase, 'blocked')

    def test_missing_corridor_never_falls_back_to_opposite_reference(self):
        state = connector()
        state.corridor = None
        state.scan = scan()
        self.assertEqual(state.prepare(route(math.pi), ('map', 1), (0, 0, 0), 1, 1), ('reject', None))
        self.assertEqual(state.reason, 'connection_road_corridor_unavailable')

    def test_stale_or_future_scan_is_not_accepted(self):
        self.assertTrue(scan().fresh(1.1, 1.1))
        self.assertFalse(scan().fresh(1.6, 1.6))
        self.assertFalse(scan().fresh(1., .8))

    def test_synthetic_inf_and_nan_are_unknown(self):
        for distance in (np.inf, np.nan):
            self.assertFalse(scan(distance=distance).contains([[1, 0, 0]], VERTICES, (0, 0, 0)))

    def test_occupied_space_blocks_the_inflated_body(self):
        self.assertFalse(scan(distance=.5).contains([[1, 0, 0]], VERTICES, (0, 0, 0)))

    def test_partial_fov_is_not_wrapped_into_rear_coverage(self):
        evidence = ScanEnvelope(np.full(721, 15.), -math.pi/2, math.pi/720,
                                .05, 30., (0, 0, 0), 1, 1)
        self.assertFalse(evidence.contains([[-1.5, 0, 0]], VERTICES, (0, 0, 0)))

    def test_scan_rotation_and_translation(self):
        evidence = scan(pose=(10, 20, math.pi/2))
        self.assertTrue(evidence.contains([[10, 21, math.pi/2]], VERTICES, (10, 20, math.pi/2)))

    def test_real_scan_exclusion_does_not_require_extra_buffer_to_be_observed(self):
        report = production_scan().check([[0, 0, 0], [1, 0, 0]], VERTICES, (0, 0, 0))
        self.assertTrue(report['ok'], report)
        self.assertEqual(report['body_unknown_points'], 0)
        self.assertGreater(report['unverified_near_margin_points'], 0)
        self.assertFalse(report['margin_fully_observed'])

    def test_body_cannot_enter_the_unobserved_near_circle(self):
        report = production_scan().check([[-.25, 0, 0]], VERTICES, (0, 0, 0))
        self.assertFalse(report['ok'])
        self.assertEqual(report['reason'], 'swept_body_enters_near_blind_zone')
        self.assertGreater(report['body_near_blind_points'], 0)

    def test_extra_margin_does_not_hide_unknown_space_outside_near_circle(self):
        report = production_scan(np.inf).check([[1, 0, 0]], VERTICES, (0, 0, 0))
        self.assertFalse(report['ok'])
        self.assertEqual(report['reason'], 'swept_body_unobserved')

    def test_known_obstacle_in_extra_margin_still_blocks(self):
        evidence = production_scan()
        point = footprint_points([[1, 0, 0]], VERTICES)[-5]
        self.assertGreater(point[1], VERTICES[1].max())
        index = int(round((math.atan2(point[1], point[0])-evidence.angle_min)/evidence.increment))
        evidence.ranges[index] = np.linalg.norm(point)
        report = evidence.check([[1, 0, 0]], VERTICES, (0, 0, 0))
        self.assertFalse(report['ok'])
        self.assertEqual(report['reason'], 'observed_obstacle_or_occlusion')

    def test_production_scan_turn_reports_the_limited_near_margin(self):
        state = connector(12.)
        state.scan = production_scan()
        decision, path = state.prepare(route(math.pi), ('map', 1), (0, 0, 0), 1, 1)
        self.assertEqual(decision, 'apply')
        self.assertTrue(np.all(path[:, 3] == 1))
        report = state.status()['admission']['accepted_scan_check']
        self.assertTrue(report['ok'])
        self.assertGreater(report['unverified_near_margin_points'], 0)
        self.assertEqual(report['scan_range_min_m'], .4)

    def test_no_room_and_unknown_explain_different_rejections(self):
        for width, distance, expected in ((3., 15., 'road'), (12., np.inf, 'swept_body_unobserved'),
                                          (12., 1., 'observed_obstacle_or_occlusion')):
            state = connector(width)
            state.scan = production_scan(distance)
            self.assertEqual(state.prepare(route(math.pi), ('map', 1), (0, 0, 0), 1, 1)[0], 'reject')
            checks = state.status()['admission']
            if expected == 'road':
                self.assertEqual(checks['road_rejected'], checks['candidate_count'])
            else:
                self.assertGreater(checks['scan_rejected'].get(expected, 0), 0)

    def test_invalid_scan_distance_metadata_is_not_fresh(self):
        for lower, upper in ((np.nan, 30.), (.4, np.inf), (1., .4), (-1., 30.)):
            evidence = production_scan()
            evidence.range_min, evidence.range_max = lower, upper
            self.assertFalse(evidence.fresh(1., 1.))

    def test_simulation_uses_production_mounts_and_actual_projection_node(self):
        simulation = ROOT/'src/neupan_ros/example/lxw_car_simulation'
        spec = importlib.util.spec_from_file_location('outdoor_simulation', simulation/'scripts/outdoor_simulation.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        settings = module.production_sensor_settings()
        urdf = ET.parse(simulation/'urdf/outdoor_lxwcar.urdf.xacro').getroot()
        for source, joint in (('base_to_livox', 'livox_joint'), ('base_to_livox', 'laser_joint'),
                              ('base_to_body', 'body_joint')):
            expected = list(map(float, settings['mounts'][source].split()[:3]))
            actual = next(j for j in urdf.findall('joint') if j.get('name') == joint).find('origin').get('xyz')
            self.assertEqual(list(map(float, actual.split())), expected)
        launch = ET.parse(simulation/'launch/outdoor_simulation.launch').getroot()
        projection = next(n for n in launch.iter('node') if n.get('name') == 'production_scan')
        self.assertEqual(projection.get('pkg'), 'pointcloud_to_laserscan')
        self.assertIsNotNone(projection.find('rosparam'))
        self.assertEqual(settings['scan']['range_min'], .4)
        self.assertEqual(settings['scan']['target_frame'], 'base_link')
        self.assertEqual(settings['lio_blind_m'], .5)

    def test_road_width_includes_body_not_just_center(self):
        corridor = RoadCorridor([[-20, 0, 20, 0, .6, 0]])
        self.assertTrue(corridor.contains([[0, 0, 0]], VERTICES))
        self.assertFalse(corridor.contains([[0, .4, 0]], VERTICES))

    def test_width_comparison_changes_only_generated_osm_and_wall_copies(self):
        import hashlib
        import tempfile
        simulation = ROOT/'src/neupan_ros/example/lxw_car_simulation'
        spec = importlib.util.spec_from_file_location('outdoor_width_inputs', simulation/'scripts/outdoor_simulation.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        originals = (simulation/'worlds/outdoor_roads.world', simulation/'maps/outdoor_test.osm')
        before = [hashlib.sha256(path.read_bytes()).hexdigest() for path in originals]
        with tempfile.TemporaryDirectory(prefix='outdoor_width_inputs_') as temporary:
            for width in (5., 7.25, 9.):
                world, osm = module.width_inputs(Path(temporary), width)
                models = {m.get('name'): m for m in ET.parse(world).getroot().find('world').findall('model')}
                north = float(models['wide_north'].find('pose').text.split()[1])-.1
                south = float(models['wide_south'].find('pose').text.split()[1])+.1
                self.assertAlmostEqual(north-south, width)
                self.assertAlmostEqual(north, width/2)
                for name in ('wide_road', 'wide_west', 'wide_east'):
                    for size in models[name].findall('.//box/size'):
                        self.assertEqual(float(size.text.split()[1]), width)
                tags = ET.parse(osm).getroot().find("way[@id='101']").findall('tag')
                self.assertEqual(float(next(t.get('v') for t in tags if t.get('k') == 'width')), width)
        self.assertEqual(before, [hashlib.sha256(path.read_bytes()).hexdigest() for path in originals])

    def test_width_comparison_is_opt_in_and_normal_simulation_defaults_stay(self):
        root = ET.parse(ROOT/'src/neupan_ros/example/lxw_car_simulation/launch/outdoor_simulation.launch').getroot()
        args = {arg.get('name'): arg.attrib for arg in root.findall('arg')}
        self.assertEqual(args['turn_only']['default'], 'false')
        self.assertEqual(args['connection_planner']['default'], 'hybrid_astar')
        self.assertIn('3.0', args['road_width']['default'])
        self.assertIn('12.0', args['road_width']['default'])
        self.assertIn('outdoor_roads.world', args['world_file']['default'])

    def test_width_comparison_does_not_advertise_a_one_off_success_as_stable(self):
        import json
        import tempfile
        simulation = ROOT/'src/neupan_ros/example/lxw_car_simulation'
        spec = importlib.util.spec_from_file_location('outdoor_width_compare', simulation/'scripts/outdoor_simulation.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        module.production_sensor_settings = lambda: {}

        def fake_run(config):
            folder = Path(config.output_dir)/'wide'
            folder.mkdir(parents=True)
            repeat = int(Path(config.output_dir).name.rsplit('_r', 1)[1])
            success = config.road_width == 9. or (config.connection_planner == 'hybrid_astar' and
                                                config.road_width == 8.5 and repeat == 1)
            module.save_json(folder/'result.json', {'passed': True,
                'reason': 'turn_completed' if success else 'safe_stop_verified',
                'min_command_speed_mps': 0., 'min_wall_clearance_m': .2, 'video': None})

        module.run_cases = fake_run
        with tempfile.TemporaryDirectory(prefix='outdoor_width_compare_') as temporary:
            root = Path(temporary)/'results'
            module.compare_widths(SimpleNamespace(output_dir=str(root), widths=[9., 8.],
                width_step=.5, repeats=2, timeout=100., record_bag=False))
            data = json.loads((root/'comparison.json').read_text())
            boundary = data['boundaries']['hybrid_astar']
            self.assertEqual(boundary['narrowest_completed_m'], 8.5)
            self.assertFalse(boundary['repeat_consistent'])
            self.assertEqual(boundary['narrowest_repeat_verified_m'], 9.)
            self.assertFalse((root/'_work').exists())

    def test_wrong_way_oneway_connection_is_rejected(self):
        corridor = RoadCorridor([[-20, 0, 20, 0, 10, 1]])
        self.assertTrue(corridor.contains([[0, 0, 0]], VERTICES))
        self.assertFalse(corridor.contains([[0, 0, math.pi]], VERTICES))

    def test_blocked_goal_remains_blocked_until_new_goal(self):
        state = connector()
        state.corridor = None
        state.scan = scan()
        state.prepare(route(math.pi), ('map', 1), (0, 0, 0), 1, 1)
        self.assertEqual(state.prepare(route(math.pi), ('map', 1), (0, 0, 0), 2, 2)[0], 'reject')
        self.assertEqual(state.prepare(route(), ('map', 2), (0, 0, 0), 2, 2)[0], 'apply')

    def test_camera_or_fallback_cannot_erase_active_connection(self):
        state = connector()
        state.key, state.phase, state.prefix_count = ('map', 1), 'connecting', 50
        for path in (route(math.pi), route(math.pi)+np.array([0, .2, 0, 0])):
            self.assertEqual(state.prepare(path, ('map', 1), (0, 0, 0), 2, 2)[0], 'keep')
        self.assertFalse(state.update_progress(48))
        self.assertTrue(state.update_progress(49))
        self.assertTrue(state.reseed)
        self.assertEqual(state.phase, 'following_route')

    def test_indoor_launch_and_hybrid_branch_do_not_enable_connector(self):
        tree = ET.parse(ROOT/'src/neupan_ros/example/lxw_car/launch/full_system_run.launch')
        for group in tree.getroot().iter('group'):
            if (group.get('unless') == '$(arg outdoor_mode)' or
                    group.get('if') == '$(arg outdoor_use_hybrid_astar)'):
                self.assertFalse(any(p.get('name') == 'outdoor_start_connector_enabled' for p in group.iter('param')))

    def test_real_forward_uturn_geometry(self):
        state = connector()
        state.scan = scan()
        source = route(math.pi)
        try:
            decision, result = state.prepare(source, ('map', 1), (0, 0, 0), 1, 1)
        except ImportError:
            self.skipTest('gctl is available in the ROS1 lxw environment')
        self.assertEqual(decision, 'apply')
        self.assertEqual(state.phase, 'connecting')
        np.testing.assert_allclose(result[0], [0, 0, 0, 1], atol=1.e-8)
        np.testing.assert_allclose(result[-1], source[-1])
        self.assertTrue(np.all(result[:, 3] == 1))
        prefix = result[:state.prefix_count]
        delta = np.diff(prefix[:, :2], axis=0)
        ds = np.linalg.norm(delta, axis=1)
        yaw_delta = np.array([wrap(b-a) for a, b in zip(prefix[:-1, 2], prefix[1:, 2])])
        self.assertLessEqual(np.max(np.abs(yaw_delta)/ds), 1/1.85+.002)
        tangent = np.column_stack((np.cos(prefix[:-1, 2]), np.sin(prefix[:-1, 2])))
        self.assertTrue(np.all(np.sum(delta*tangent, axis=1) > 0))

    def test_logged_175_degree_start_mismatch(self):
        pose = np.array([123.375, 332.447, math.radians(-85.257)])
        yaw = math.radians(89.726)
        source = route(yaw, pose[:2])
        direction = np.array([math.cos(yaw), math.sin(yaw)])
        a, b = pose[:2]-30*direction, pose[:2]+30*direction
        state = StartRouteConnector(VERTICES, 1.85, RoadCorridor([[*a, *b, 10., 0]]))
        state.scan = scan(pose=pose)
        try:
            decision, result = state.prepare(source, ('map', 2), pose, 1, 1)
        except ImportError:
            self.skipTest('gctl is available in the ROS1 lxw environment')
        self.assertAlmostEqual(state.heading_error_deg, 174.983, places=2)
        self.assertEqual(decision, 'apply')
        np.testing.assert_allclose(result[0, :3], pose, atol=1.e-8)
        self.assertLess(abs(wrap(result[0, 2]-pose[2])), 1.e-8)

    def test_no_room_and_unknown_scan_both_reject_without_crash(self):
        for width, distance in ((1., 15.), (20., np.inf), (20., 1.)):
            state = connector(width)
            state.scan = scan(distance=distance)
            try:
                decision, result = state.prepare(route(math.pi), ('map', 1), (0, 0, 0), 1, 1)
            except ImportError:
                self.skipTest('gctl is available in the ROS1 lxw environment')
            self.assertEqual((decision, result), ('reject', None))
            self.assertEqual(state.reason, 'no_safe_forward_connection')


class PackageLayoutTests(unittest.TestCase):
    def test_map_modules_are_flat_without_a_same_named_subdirectory(self):
        folder = ROOT/'src/osm_map_handler'
        self.assertFalse((folder/'src/osm_map_handler').exists())
        self.assertFalse((folder/'python').exists())
        expected = {'osm_camera_path_refinement', 'osm_camera_path_guard',
                    'osm_outdoor_start_connection', 'osm_hybrid_search'}
        setup = ast.parse((folder/'setup.py').read_text())
        declaration = next(n for n in ast.walk(setup) if isinstance(n, ast.Call) and
            isinstance(n.func, ast.Name) and n.func.id == 'generate_distutils_setup')
        modules = next(k.value for k in declaration.keywords if k.arg == 'py_modules')
        self.assertEqual(set(ast.literal_eval(modules)), expected)
        for name in expected:
            self.assertTrue((folder/'src'/(name+'.py')).is_file())

    def test_map_entry_and_implementation_filenames_do_not_overlap(self):
        folder = ROOT/'src/osm_map_handler'
        entries = {p.name for p in (folder/'scripts').glob('*.py')}
        modules = {p.name for p in (folder/'src').glob('*.py')}
        self.assertFalse(entries & modules)
        self.assertIn('osm_camera_path_refinement.py', modules)

    def test_neupan_entry_is_installed_and_core_stays_in_src(self):
        folder = ROOT/'src/neupan_ros'
        self.assertTrue((folder/'scripts/neupan_node.py').is_file())
        self.assertFalse((folder/'src/neupan_node.py').exists())
        self.assertTrue((folder/'src/neupan_core.py').is_file())
        self.assertTrue((folder/'src/neupan_diagnostics.py').is_file())
        self.assertIn('scripts/neupan_node.py', (folder/'CMakeLists.txt').read_text())

    def test_base_ros_nodes_are_installed_from_scripts(self):
        folder = ROOT/'src/robot_base_driver'
        cmake = (folder/'CMakeLists.txt').read_text()
        for filename in ('base_driver_node.py', 'control_mode_mux.py',
                'gps_fix_to_map_odom.py', 'global_odom_bridge.py', 'global_path_tracker.py',
                'lio_odom_guard.py', 'map_odom_align_bridge.py',
                'system_run_logger.py', 'system_resource_monitor.py'):
            self.assertTrue((folder/'scripts'/filename).is_file(), filename)
            self.assertFalse((folder/'src'/filename).exists(), filename)
            self.assertIn('scripts/'+filename, cmake)

    def test_neupan_imports_path_policy_from_map_package(self):
        tree = ast.parse((ROOT/'src/neupan_ros/src/neupan_core.py').read_text())
        modules = {node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)}
        self.assertIn('osm_camera_path_guard', modules)
        self.assertIn('osm_outdoor_start_connection', modules)
        self.assertNotIn('camera_path_guard', modules)
        self.assertNotIn('outdoor_start_connection', modules)

    def test_camera_launch_and_entrypoint_belong_to_map_package(self):
        folder = ROOT/'src/osm_map_handler'
        launch = ET.parse(folder/'launch/camera_right_path.launch').getroot()
        node = launch.find('node')
        self.assertEqual(node.attrib['pkg'], 'osm_map_handler')
        self.assertEqual(node.attrib['type'], 'camera_right_path.py')
        entry = ast.parse((folder/'scripts/camera_right_path.py').read_text())
        self.assertTrue(any(isinstance(n, ast.ImportFrom) and
                            n.module == 'osm_camera_path_refinement' for n in entry.body))
        system = ET.parse(ROOT/'src/neupan_ros/example/lxw_car/launch/full_system_run.launch')
        includes = [n.attrib.get('file', '') for n in system.iter('include')]
        self.assertIn('$(find osm_map_handler)/launch/camera_right_path.launch', includes)
        self.assertNotIn('$(find robot_base_driver)/launch/camera_right_path.launch', includes)


class CameraGuardTests(unittest.TestCase):
    @staticmethod
    def path(stamp=1, end=10.):
        def pose(x):
            return SimpleNamespace(pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=0., z=0.),
                orientation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
        return SimpleNamespace(header=SimpleNamespace(frame_id='map',
            stamp=SimpleNamespace(to_nsec=lambda: stamp)), poses=[pose(0.), pose(end)])

    def ready_guard(self):
        guard = CameraPathGuard()
        original, candidate = self.path(), self.path()
        candidate.poses[0].pose.position.y = .1
        guard.original_cb(original)
        guard.candidate_cb(candidate)
        guard.status_cb({'control_enabled': True, 'route_stamp_ns': 1,
            'route_frame_id': 'map', 'control_path_digest': path_digest(candidate)}, 10.)
        return guard, original, candidate

    def test_confirmed_camera_path_is_selected(self):
        guard, _, candidate = self.ready_guard()
        path, _, state = guard.select(10.1)
        self.assertIs(path, candidate)
        self.assertEqual(state['source'], 'camera')

    def test_three_second_timeout_falls_back_without_producer(self):
        guard, original, candidate = self.ready_guard()
        self.assertIs(guard.select(13.)[0], candidate)
        path, _, state = guard.select(13.001)
        self.assertIs(path, original)
        self.assertEqual(state['reason'], 'camera_heartbeat_stale')

    def test_new_route_cannot_reuse_old_camera_heartbeat(self):
        guard, _, _ = self.ready_guard()
        original = self.path(stamp=2)
        guard.original_cb(original)
        path, _, state = guard.select(10.1)
        self.assertIs(path, original)
        self.assertEqual(state['reason'], 'camera_route_mismatch')

    def test_unconfirmed_candidate_content_falls_back(self):
        guard, original, _ = self.ready_guard()
        changed = self.path()
        changed.poses[0].pose.position.y = .2
        guard.candidate_cb(changed)
        path, _, state = guard.select(10.1)
        self.assertIs(path, original)
        self.assertEqual(state['reason'], 'camera_path_not_confirmed_by_heartbeat')

    def test_missing_original_route_selects_stop(self):
        guard, _, _ = self.ready_guard()
        guard.original_cb(None)
        path, _, state = guard.select(10.1)
        self.assertIsNone(path)
        self.assertEqual(state['source'], 'stop')


class AdapterTests(unittest.TestCase):
    """Run the real ROS adapter method with fake publishers/planner, no master."""
    def node(self):
        filename = ROOT/'src/neupan_ros/src/neupan_core.py'
        tree = ast.parse(filename.read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'neupan_core')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'apply_pending_route')
        ros = SimpleNamespace(loginfo=Mock(), logwarn_throttle=Mock(), Time=SimpleNamespace(now=lambda: SimpleNamespace(to_sec=lambda: 1.)))
        namespace = {'np': np, 'time': SimpleNamespace(monotonic=lambda: 1.),
                     'rospy': ros, 'Twist': object, 'yellow_terminal_text': lambda x: x}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(filename), 'exec'), namespace)
        node = SimpleNamespace(route_update_lock=threading.Lock(), pending_route_update=None,
            minimum_path_stamp=0, diagnostic_route_update=None, start_connector=connector(),
            robot_state=np.zeros((3, 1)), publish_connection_status=Mock(), vel_pub=Mock(),
            waiting_for_new_goal=False, camera_guard=None, arrival_path_key=('map', 0),
            goal_arrived_timer_callback=Mock(), neupan_planner=Mock())
        return node, namespace['apply_pending_route']

    def test_rejected_return_route_does_not_reach_solver_or_resume_old_route(self):
        node, apply = self.node()
        node.start_connector.scan = scan()
        node.start_connector.corridor = None
        node.pending_route_update = ('path', [p.reshape(4, 1) for p in route(math.pi)], ('map', 1))
        apply(node)
        node.neupan_planner.set_initial_path.assert_not_called()
        node.neupan_planner.reset.assert_not_called()
        node.vel_pub.publish.assert_called_once()
        self.assertTrue(node.waiting_for_new_goal)

    def test_valid_forward_route_still_resets_arrival_for_second_goal(self):
        node, apply = self.node()
        node.pending_route_update = ('path', [p.reshape(4, 1) for p in route()], ('map', 1))
        apply(node)
        node.neupan_planner.set_initial_path.assert_called_once()
        self.assertFalse(node.waiting_for_new_goal)
        self.assertFalse(node.arrive)
        self.assertEqual(node.arrival_path_key, ('map', 1))

    def test_normal_offset_reaches_the_planner_without_disabling_scan_safety(self):
        node, apply = self.node()
        source = route(origin=(0., 1.46))
        node.pending_route_update = ('path', [p.reshape(4, 1) for p in source], ('map', 1))
        node.start_connector.scan = production_scan(distance=np.inf)
        apply(node)
        node.neupan_planner.set_initial_path.assert_called_once()
        self.assertFalse(node.waiting_for_new_goal)
        self.assertEqual(node.start_connector.reason, 'normal_route_tracking')

    def test_missing_scan_requeues_new_goal_and_holds_zero(self):
        node, apply = self.node()
        node.pending_route_update = ('path', [p.reshape(4, 1) for p in route(math.pi)], ('map', 1))
        apply(node)
        node.neupan_planner.set_initial_path.assert_not_called()
        self.assertIsNotNone(node.pending_route_update)
        self.assertTrue(node.waiting_for_new_goal)


class CaptureTransformTests(unittest.TestCase):
    def test_delayed_tf_is_retried_and_last_good_capture_is_not_erased(self):
        filename = ROOT/'src/neupan_ros/src/neupan_core.py'
        cls = next(n for n in ast.parse(filename.read_text()).body if isinstance(n, ast.ClassDef))
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'refresh_connection_scan')
        class TransformError(Exception):
            pass
        clock = [1.]
        namespace = {'time': SimpleNamespace(monotonic=lambda: clock[0]),
            'rospy': SimpleNamespace(Time=SimpleNamespace(now=lambda: SimpleNamespace(to_sec=lambda: clock[0]))),
            'tf': SimpleNamespace(Exception=TransformError), 'ScanEnvelope': ScanEnvelope}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(filename), 'exec'), namespace)
        message = SimpleNamespace(header=SimpleNamespace(frame_id='base_link', stamp=SimpleNamespace(to_sec=lambda: .95)),
            ranges=[15.]*723, angle_min=-3.14159, angle_increment=.0087, range_min=.4, range_max=30.)
        previous = scan(now=.8)
        helper = connector()
        helper.scan = previous
        listener = Mock()
        listener.lookupTransform.side_effect = TransformError('TF not arrived yet')
        node = SimpleNamespace(start_connector=helper, connection_scan_lock=threading.Lock(),
            connection_scan_queue=deque([(message, .97)], maxlen=8), listener=listener,
            map_frame='map', quat_to_yaw_list=lambda _: 0.)
        refresh = namespace['refresh_connection_scan']
        refresh(node)
        self.assertIs(helper.scan, previous)
        self.assertEqual(helper.scan_transform_state, 'using_previous_fresh_capture')
        self.assertEqual(len(node.connection_scan_queue), 1)
        listener.lookupTransform.side_effect = None
        listener.lookupTransform.return_value = ((1., 2., 0.), (0., 0., 0., 1.))
        refresh(node)
        self.assertEqual(helper.scan.stamp, .95)
        self.assertEqual(helper.scan.received, .97)
        self.assertEqual(helper.scan_transform_state, 'ready')
        self.assertEqual(len(node.connection_scan_queue), 0)
        clock[0] = 1.6
        refresh(node)
        self.assertFalse(helper.scan.fresh(1.6, 1.6))
        self.assertEqual(helper.scan_transform_state, 'no_fresh_scan')


class HybridAdmissionTests(unittest.TestCase):
    def pending(self):
        backend = SimpleNamespace(budget_sec=1., max_iterations=12000,
            cancel=Mock(), submit=Mock(), poll=Mock(return_value=None))
        helper = StartRouteConnector(VERTICES, 1.85,
            RoadCorridor([[-30, 0, 30, 0, 10., 0]]), search=backend)
        # Transport/admission unit tests use a simple complete scan. Actual
        # production near filtering is covered by SharedCppTests below.
        helper.scan = scan()
        source = route(math.pi, origin=(0., 3.7))
        self.assertEqual(helper.prepare(source, ('map', 1), (0, 0, 0), 1, 1), ('wait', None))
        return helper, backend, source

    def success(self, helper, backend, source):
        theta = np.linspace(0., math.pi, int(math.ceil(math.pi*1.85/.07))+1)
        turn = np.column_stack((1.85*np.sin(theta), 1.85*(1-np.cos(theta)), theta))
        distance = np.linspace(0., 1.5, int(math.ceil(1.5/.07))+1)[1:]
        tail = np.column_stack((-distance, np.full(len(distance), 3.7), np.full(len(distance), math.pi)))
        path = np.vstack((turn, tail))
        backend.poll.return_value = {'success': True, 'anchor': 3, 'path': path.tolist(),
            'reason': 'complete_goal_connection', 'stats': {'expanded': 1}}

    def test_searching_keeps_requeuing_without_restarting_the_worker(self):
        helper, backend, source = self.pending()
        self.assertEqual(helper.prepare(source, ('map', 1), (0, 0, 0), 1.01, 1.01), ('wait', None))
        backend.submit.assert_called_once()
        self.assertTrue(helper.status()['suspend_camera_control'])

    def test_fresh_scan_revalidation_accepts_a_complete_forward_result(self):
        helper, backend, source = self.pending()
        self.success(helper, backend, source)
        helper.scan = scan(now=1.8)
        decision, path = helper.prepare(source, ('map', 1), (0, 0, 0), 1.8, 1.8)
        self.assertEqual(decision, 'apply')
        self.assertTrue(np.all(path[:, 3] == 1))
        np.testing.assert_allclose(path[-1], source[-1])
        self.assertEqual(helper.admission['completion_scan_stamp_sec'], 1.8)

    def test_latest_unknown_scan_rejects_an_old_clear_search_result(self):
        helper, backend, source = self.pending()
        self.success(helper, backend, source)
        helper.scan = production_scan(distance=np.inf)
        self.assertEqual(helper.prepare(source, ('map', 1), (0, 0, 0), 1.1, 1.1), ('reject', None))
        self.assertIn('latest scan rejected', helper.admission['result_rejected'])

    def test_vehicle_movement_during_search_does_not_admit_the_old_prefix(self):
        helper, backend, source = self.pending()
        self.success(helper, backend, source)
        self.assertEqual(helper.prepare(source, ('map', 1), (.1, 0, 0), 1.1, 1.1), ('reject', None))
        self.assertIn('vehicle moved', helper.admission['result_rejected'])

    def test_incomplete_endpoint_is_never_accepted(self):
        helper, backend, source = self.pending()
        self.success(helper, backend, source)
        backend.poll.return_value['path'] = backend.poll.return_value['path'][:-4]
        self.assertEqual(helper.prepare(source, ('map', 1), (0, 0, 0), 1.1, 1.1), ('reject', None))

    def test_failed_worker_does_not_fall_back_to_the_original_return_line(self):
        helper, backend, source = self.pending()
        backend.poll.return_value = {'success': False, 'reason': 'search_time_budget', 'stats': {}}
        self.assertEqual(helper.prepare(source, ('map', 1), (0, 0, 0), 1.1, 1.1), ('reject', None))
        self.assertEqual(helper.phase, 'blocked')
        self.assertEqual(helper.admission['search']['reason'], 'search_time_budget')

    def test_new_goal_cancels_a_pending_search(self):
        helper, backend, source = self.pending()
        calls = backend.cancel.call_count
        helper.prepare(route(), ('map', 2), (0, 0, 0), 1.1, 1.1)
        self.assertGreater(backend.cancel.call_count, calls)
        self.assertEqual(helper.phase, 'following_route')


class HybridTransportTests(unittest.TestCase):
    def test_wrong_job_identity_is_rejected(self):
        import time
        from osm_hybrid_search import CppHybridSearch
        command = [sys.executable, '-c',
            'import json,sys; p=json.loads(sys.stdin.readline()); '
            'print(json.dumps({"id":p["id"]+1,"result":{"success":True}}))']
        backend = CppHybridSearch(command=command)
        try:
            backend.submit({})
            result, deadline = None, time.monotonic()+3
            while result is None and time.monotonic()<deadline:
                result = backend.poll()
                time.sleep(.005)
            self.assertFalse(result['success'])
            self.assertEqual(result['reason'], 'worker_protocol_error')
        finally:
            backend.close()

    def test_deadline_kills_only_the_pending_worker(self):
        import time
        from osm_hybrid_search import CppHybridSearch
        backend = CppHybridSearch(command=[sys.executable, '-c',
            'import sys,time; sys.stdin.readline(); time.sleep(10)'])
        try:
            backend.submit({})
            process = backend.process
            backend.deadline = time.monotonic()-1
            result = backend.poll()
            self.assertFalse(result['success'])
            self.assertEqual(result['reason'], 'worker_exited_or_deadline')
            self.assertIsNone(backend.pending)
            backend.close()
            self.assertIsNotNone(process.poll())
        finally:
            backend.close()


@unittest.skipUnless(os.environ.get('NEUPAN_CPP_SMOKE') == '1', 'optional compiled shared-core tests')
class SharedCppTests(unittest.TestCase):
    def run_worker(self, width=12., distance=15., start=(0., 0., 0.), **limits):
        import json
        import subprocess
        from osm_hybrid_search import search_request
        from roslib.packages import find_node
        evidence = production_scan(distance=distance, pose=start)
        corridor = RoadCorridor([[-30, 0, 30, 0, width/2, 0]])
        source = route(math.pi, origin=start[:2])
        request = search_request(start, VERTICES, 1.85, corridor, evidence,
            [(i, source[i, :3]) for i in (3, 6, 9)], **limits)
        completed = subprocess.run([find_node('osm_map_handler', 'outdoor_hybrid_worker')[0]],
            input=json.dumps({'id': 7, 'request': request}), text=True,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
        self.assertEqual(completed.returncode, 0, completed.stderr)
        reply = json.loads(completed.stdout)
        self.assertEqual(reply['id'], 7)
        return reply['result'], corridor, evidence, source

    def test_shared_core_completes_a_forward_turn_in_eight_metre_road(self):
        result, corridor, evidence, source = self.run_worker(width=8., budget_sec=2.)
        self.assertTrue(result['success'], result)
        path = np.asarray(result['path'])
        self.assertTrue(corridor.contains(path, VERTICES))
        self.assertTrue(evidence.contains(path, VERTICES, (0, 0, 0)))
        np.testing.assert_allclose(path[-1, :2], source[result['anchor'], :2], atol=1.e-5)
        self.assertLess(abs(wrap(path[-1, 2]-source[result['anchor'], 2])), 1.e-5)
        ds = np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1)
        dyaw = np.arctan2(np.sin(np.diff(path[:, 2])), np.cos(np.diff(path[:, 2])))
        self.assertTrue(np.all(abs(dyaw)/ds <= 1/1.85+.01))
        self.assertGreater(result['stats']['expanded'], 1)

    def test_iteration_limit_returns_failure_not_a_partial_route(self):
        result, _, _, _ = self.run_worker(width=3., max_iterations=1)
        self.assertFalse(result['success'])
        self.assertEqual(result['path'], [])
        self.assertEqual(result['reason'], 'search_iteration_limit')

    def test_nonzero_world_pose_is_not_rejected_by_float_body_rounding(self):
        result, _, _, _ = self.run_worker(start=(9.579886596686217, -.11005528975017269, -.08072781617666701))
        self.assertTrue(result['success'], result)
        self.assertGreater(result['stats']['expanded'], 0)

    def test_unknown_observations_never_become_free_cells(self):
        result, _, _, _ = self.run_worker(distance=np.inf)
        self.assertFalse(result['success'])
        self.assertEqual(result['stats']['observed_free_cells'], 0)
        self.assertTrue(result['stats']['scan_rejected'])

    def test_async_transport_round_trip_without_ros_master(self):
        import time
        from osm_hybrid_search import CppHybridSearch, search_request
        backend = CppHybridSearch()
        source = route(math.pi)
        try:
            backend.submit(search_request((0., 0., 0.), VERTICES, 1.85,
                RoadCorridor([[-30, 0, 30, 0, 6., 0]]), production_scan(), [(3, source[3, :3])]))
            deadline, result = time.monotonic()+4, None
            while result is None and time.monotonic()<deadline:
                result = backend.poll()
                time.sleep(.01)
            self.assertIsNotNone(result)
            # This anchor alone can be refused by the production near-blind
            # gate; a well-formed result, not guaranteed success, is required.
            self.assertIn('success', result)
        finally:
            backend.close()


@unittest.skipUnless(os.environ.get('NEUPAN_ROS_SMOKE') == '1', 'optional isolated ROS1 process test')
class RosProcessTest(unittest.TestCase):
    def test_blocked_second_goal_holds_zero_and_process_stays_alive(self):
        import json
        import signal
        import socket
        import subprocess
        import tempfile
        import time
        import rospy
        import tf
        from geometry_msgs.msg import PoseStamped, Twist
        from nav_msgs.msg import Path as RosPath
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String

        with socket.socket() as reservation:
            reservation.bind(('127.0.0.1', 0))
            port = reservation.getsockname()[1]
        env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:'+str(port), ROS_HOSTNAME='127.0.0.1')
        env['ROS_LOG_DIR'] = tempfile.mkdtemp(prefix='outdoor_start_connection_ros_')
        os.environ.update({k: env[k] for k in ('ROS_MASTER_URI', 'ROS_HOSTNAME', 'ROS_LOG_DIR')})
        master = subprocess.Popen(['roscore', '-p', str(port)], env=env,
                                  stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        process = camera_process = indoor_process = None
        indoor_log = open(os.path.join(env['ROS_LOG_DIR'], 'indoor_console.log'), 'w')
        running = threading.Event()
        running.set()
        worker = None
        try:
            rospy.init_node('outdoor_start_connection_test', anonymous=True, disable_signals=True)
            # Exercise the unchanged indoor entry/default profile in this
            # private master; all inputs/outputs are isolated from real nodes.
            from nav_msgs.msg import OccupancyGrid
            from geometry_msgs.msg import PoseWithCovarianceStamped
            indoor_process = subprocess.Popen(['rosrun', 'hybrid_astar', 'hybrid_astar',
                '__name:=indoor_shared_core_test', '/map:=/connection_test/indoor_map',
                '/initialpose:=/connection_test/indoor_start',
                '/move_base_simple/goal:=/connection_test/indoor_goal',
                '/path:=/connection_test/indoor_path', '/sPath:=/connection_test/indoor_smoothed'],
                env=env, cwd=env['ROS_LOG_DIR'], stdout=indoor_log, stderr=subprocess.STDOUT)
            indoor_paths = []
            indoor_subscriber = rospy.Subscriber('/connection_test/indoor_path', RosPath,
                lambda msg: indoor_paths.append(msg), queue_size=2)
            maps = rospy.Publisher('/connection_test/indoor_map', OccupancyGrid, queue_size=1, latch=True)
            starts = rospy.Publisher('/connection_test/indoor_start', PoseWithCovarianceStamped, queue_size=1, latch=True)
            ends = rospy.Publisher('/connection_test/indoor_goal', PoseStamped, queue_size=1, latch=True)
            limit = time.monotonic()+10
            while not (maps.get_num_connections() and starts.get_num_connections() and
                       ends.get_num_connections() and indoor_subscriber.get_num_connections()):
                self.assertIsNone(indoor_process.poll())
                self.assertLess(time.monotonic(), limit, 'indoor console: '+env['ROS_LOG_DIR'])
                time.sleep(.02)
            grid = OccupancyGrid()
            grid.header.frame_id = 'map'
            grid.info.width = grid.info.height = 100
            grid.info.resolution = .05
            grid.info.origin.position.x = grid.info.origin.position.y = -2.5
            grid.info.origin.orientation.w = 1.
            grid.data = [0]*10000
            maps.publish(grid)
            time.sleep(.2)
            start_pose = PoseWithCovarianceStamped()
            start_pose.header.frame_id = 'map'
            start_pose.pose.pose.orientation.w = 1.
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            # Use two of the unchanged legacy straight primitives (~0.707 m
            # each). This is an entry smoke test, not a retuning of indoor
            # lattice/goal tolerances to the outdoor vehicle profile.
            goal_pose.pose.position.x = 1.4
            goal_pose.pose.orientation.w = 1.
            starts.publish(start_pose)
            ends.publish(goal_pose)
            while not any(len(p.poses)>1 for p in indoor_paths):
                self.assertIsNone(indoor_process.poll())
                self.assertLess(time.monotonic(), limit, 'indoor console: '+env['ROS_LOG_DIR'])
                time.sleep(.02)
            self.assertLess(abs(indoor_paths[-1].poses[-1].pose.position.x-1.4), .1)
            base = ROOT/'src/neupan_ros/example/lxw_car'
            command = ['rosrun', 'neupan_ros', 'neupan_node.py', '__name:=connection_test_planner',
                '_config_file:='+str(base/'config/neupan_config_outdoor.yaml'),
                '_dune_checkpoint:='+str(base/'model/acker_robot_default_6/model_5000.pth'),
                '_refresh_initial_path:=true', '_include_initial_path_direction:=true',
                '_enable_adaptive_adjust:=false', '_outdoor_start_connector_enabled:=true',
                '_osm_file:='+str(ROOT/'src/neupan_ros/example/lxw_car_simulation/maps/outdoor_test.osm'), '_map_origin_lat:=23.0398',
                '_map_origin_lon:=113.39002', '/neupan_cmd_vel:=/connection_test/cmd_vel']
            # Only planner + this fake-sensor process are launched. The private
            # master has no base driver or control mux, and commands are remapped.
            process = subprocess.Popen(command, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            scans = rospy.Publisher('/scan', LaserScan, queue_size=1)
            paths = rospy.Publisher('/initial_path', RosPath, queue_size=1, latch=True)
            goals = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1, latch=True)
            commands, statuses = [], []
            subscribers = [rospy.Subscriber('/connection_test/cmd_vel', Twist,
                lambda msg: commands.append((time.monotonic(), msg.linear.x)), queue_size=100),
                rospy.Subscriber('/neupan/route_connection', String,
                lambda msg: statuses.append(json.loads(msg.data)), queue_size=100)]
            broadcaster = tf.TransformBroadcaster()

            def sensors():
                while running.is_set() and not rospy.is_shutdown():
                    stamp = rospy.Time.now()
                    broadcaster.sendTransform((0, 0, 0), (0, 0, 0, 1), stamp, 'base_link', 'map')
                    message = LaserScan()
                    message.header.stamp, message.header.frame_id = stamp, 'base_link'
                    message.angle_min, message.angle_max, message.angle_increment = -math.pi, math.pi, math.pi/720
                    message.range_min, message.range_max = .4, 30.
                    message.ranges = [float('inf')]*1441
                    scans.publish(message)
                    time.sleep(.04)

            worker = threading.Thread(target=sensors, daemon=True)
            worker.start()

            def wait_for(predicate, seconds=20):
                deadline = time.monotonic()+seconds
                while time.monotonic() < deadline:
                    self.assertIsNone(process.poll(), 'planner process exited unexpectedly')
                    if predicate():
                        return
                    time.sleep(.05)
                self.fail('ROS smoke timeout; statuses='+str(statuses[-3:]))

            wait_for(lambda: paths.get_num_connections() > 0 and goals.get_num_connections() > 0)

            def publish_goal_and_path(yaw, origin=(0., 0.)):
                goal = PoseStamped()
                goal.header.frame_id, goal.header.stamp = 'map', rospy.Time.now()
                goal.pose.position.x = origin[0]+20*math.cos(yaw)
                goal.pose.position.y = origin[1]+20*math.sin(yaw)
                goal.pose.orientation.w = 1.
                path = RosPath()
                path.header.frame_id, path.header.stamp = 'map', goal.header.stamp
                for point in route(yaw, origin=origin):
                    pose = PoseStamped()
                    pose.header = path.header
                    pose.pose.position.x, pose.pose.position.y = point[:2]
                    pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
                    path.poses.append(pose)
                goals.publish(goal)
                paths.publish(path)
                return path.header.stamp.to_nsec()

            publish_goal_and_path(0.)
            wait_for(lambda: any(v > .05 for _, v in commands))
            # Real planner/transport regression for the 2026-10-16 incident:
            # ~1.46 m lateral OSM offset, same direction and unknown scan rays.
            # It must solve under the normal scan-health chain, not be held by
            # the stricter full-turn observation admission.
            offset_started = time.monotonic()
            offset_key = publish_goal_and_path(0., origin=(0., 1.46))
            wait_for(lambda: statuses and statuses[-1].get('route_key') == ['map', offset_key] and
                statuses[-1]['reason'] == 'normal_route_tracking')
            wait_for(lambda: any(stamp > offset_started+.15 and v > .05 for stamp, v in commands))
            # Outside the discarded 20deg/2m extension, the real adapter must
            # still use the original tracking flow, not search or reject.
            for yaw, origin in ((0., (0., 3.)), (math.pi/2, (0., 0.))):
                normal_key = publish_goal_and_path(yaw, origin=origin)
                wait_for(lambda: statuses and statuses[-1].get('route_key') == ['map', normal_key] and
                    statuses[-1]['reason'] == 'normal_route_tracking')
                self.assertFalse(statuses[-1]['uturn_required'])
                self.assertEqual(statuses[-1]['phase'], 'following_route')
                self.assertNotIn('search', statuses[-1].get('admission') or {})
            # Exercise the relocated roslaunch entry, not just source imports.
            # Shadow mode only publishes reference paths; the private master
            # still has no hardware driver or selected-command consumer.
            camera_statuses = []
            subscribers.append(rospy.Subscriber('/camera_right/status', String,
                lambda msg: camera_statuses.append(json.loads(msg.data)), queue_size=10))
            camera_process = subprocess.Popen(['roslaunch', 'osm_map_handler',
                'camera_right_path.launch', 'input_path_topic:=/initial_path',
                'control_enabled:=false'], env=env,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            wait_for(lambda: bool(camera_statuses))
            self.assertIsNone(camera_process.poll(), 'relocated camera node exited')
            self.assertFalse(camera_statuses[-1]['control_enabled'])
            turn_key = publish_goal_and_path(math.pi)
            wait_for(lambda: statuses and statuses[-1].get('route_key') == ['map', turn_key] and
                statuses[-1]['phase'] == 'blocked')
            self.assertTrue(statuses[-1]['uturn_required'])
            self.assertEqual(statuses[-1]['connection_kind'], 'uturn')
            blocked_time = time.monotonic()
            time.sleep(.7)
            tail = [v for stamp, v in commands if stamp > blocked_time+.2]
            self.assertTrue(tail)
            self.assertTrue(all(v == 0. for v in tail), tail)
            self.assertIsNone(process.poll())
        finally:
            running.clear()
            if worker:
                worker.join(timeout=2)
            rospy.signal_shutdown('isolated regression completed')
            for child in (camera_process, indoor_process, process, master):
                if child is not None and child.poll() is None:
                    child.send_signal(signal.SIGINT)
                    try:
                        child.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        child.kill()
                        child.wait(timeout=5)
            indoor_log.close()


if __name__ == '__main__':
    unittest.main()
