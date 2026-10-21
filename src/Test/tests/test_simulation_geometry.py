"""Geometric and integration invariants, independent of planner implementation."""
import importlib.util
import math
from pathlib import Path
import unittest
from unittest.mock import patch
import threading

HERE = Path(__file__).resolve().parents[3] / 'src/neupan_ros/example/lxw_car_simulation'


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


sim = load('comparison', HERE/'scripts/planner_comparison.py')


class GeometryTest(unittest.TestCase):
    def test_diagnostic_point_clearance_uses_the_rotated_rear_axle_body(self):
        import numpy as np
        try:
            controller=load('sim_neupan_controller', HERE/'scripts/neupan_sim_controller.py')
        except ModuleNotFoundError as error:
            if error.name != 'neupan':
                raise
            self.skipTest('Run with the lxw planner Python environment to verify actual controller geometry')
        vertices=np.array([[-.14,.81,.81,-.14],[-.375,-.375,.375,.375]])
        state=np.array([2.,3.,math.pi/2]).reshape(3,1)
        # Point 0.2 m in front of the actual body, after a 90 degree turn.
        self.assertAlmostEqual(controller.SimulationController.point_clearance(
            np.array([[2.],[4.01]]),state,vertices),.2)
        self.assertLess(controller.SimulationController.point_clearance(
            np.array([[2.],[3.335]]),state,vertices),0)

    def test_comparison_adaptive_schedule_matches_the_original_simulation_entry(self):
        import xml.etree.ElementTree as ET
        import yaml
        cfg=yaml.safe_load((HERE/'config/comparison_neupan_adapter.yaml').read_text())
        launch=ET.parse(HERE/'launch/neupan_lxwcar_gazebo.launch')
        for key,value in cfg.items():
            original=launch.find('./arg[@name="'+key+'"]').attrib['default']
            expected=list(map(float,original.split())) if isinstance(value,list) else float(original)
            self.assertEqual(value,expected)

    def test_neupan_goal_capture_with_the_configured_distance_and_index_tolerances(self):
        try:
            from neupan.blocks.initial_path import InitialPath
        except ModuleNotFoundError:
            self.skipTest('Run with the lxw planner Python environment to verify actual NeuPAN arrival logic')
        import numpy as np
        import yaml
        cfg=yaml.safe_load((HERE/'config/comparison_neupan.yaml').read_text())['ipath']
        path=object.__new__(InitialPath)
        path.curve_list=[[np.array([x,0.,0.,1.]).reshape(4,1) for x in np.linspace(0.,8.,135)]]
        path.curve_index=0
        path.point_index=110
        captured=[]
        for x in np.arange(7.,8.21,.025):
            state=np.array([x,0.,0.]).reshape(3,1)
            path.closest_point(state,cfg['close_threshold'],cfg['ind_range'])
            if path.check_curve_arrive(state,cfg['arrive_threshold'],cfg['arrive_index_threshold']):
                captured.append(x)
        self.assertTrue(captured,'Goal window was missed by the configured index and distance tolerances')
        self.assertLess(abs(captured[0]-8.),sim.GOAL_POSITION_TOLERANCE)

    def test_lead_actor_motion_is_fixed_and_has_a_safe_kinematic_witness(self):
        for scenario in sim.DYNAMIC_SCENARIOS:
            proof=sim.dynamic_feasibility(scenario)
            self.assertTrue(proof['passed'])
            spec=sim.obstacle_specs(scenario)[0]
            xs=[sim.obstacle_position(spec,scenario,t*.1)[0] for t in range(401)]
            self.assertEqual(xs,sorted(xs))
        spec=sim.obstacle_specs('lead_stop')[0]
        self.assertEqual(sim.obstacle_position(spec,'lead_stop',6),sim.obstacle_position(spec,'lead_stop',10))
        self.assertGreater(sim.obstacle_position(spec,'lead_stop',11)[0],sim.obstacle_position(spec,'lead_stop',10)[0])

    def test_hesitation_excludes_start_and_goal_and_times_recovery(self):
        records=[]
        for i in range(61):
            speed=0. if i<10 or 20<=i<28 or i>=50 else .3
            records.append({'t':i*.1,'speed':speed,'pose':[7.9 if i>=50 else 1.,0.,0.]})
        metric=sim.hesitation_metrics(records)
        self.assertEqual(metric['stop_count'],1)
        self.assertAlmostEqual(metric['stop_total_s'],.8)
        self.assertEqual(metric['initial_wait_s'],1.)
        self.assertTrue(metric['stop_intervals'][0]['resumed'])

    def test_rear_axle_offset_and_rotation(self):
        self.assertAlmostEqual(sim.clearance((0, 0, 0), [(1.1, 0, .2)]), .09)
        self.assertAlmostEqual(sim.clearance((0, 0, math.pi/2), [(0, 1.1, .2)]), .09)
        self.assertAlmostEqual(sim.clearance((0, 0, 0), [(-.5, 0, .2)]), .16)

    def test_body_overlap_and_corner_distance(self):
        self.assertLess(sim.clearance((0, 0, 0), [(.335, 0, .1)]), 0)
        self.assertAlmostEqual(sim.clearance((0, 0, 0), [(1., .575, .1)]), math.hypot(.19, .2)-.1)

    def test_non_yielding_stress_trajectory_is_preserved(self):
        specs = sim.obstacle_specs('crossing_stress')
        self.assertAlmostEqual(sim.obstacle_position(specs[0], 'crossing_stress', 5)[1], 0)
        self.assertAlmostEqual(sim.obstacle_position(specs[1], 'crossing_stress', 6.5)[1], 0)
        self.assertAlmostEqual(sim.obstacle_position(specs[0], 'crossing_stress', 20)[1], 2.5)

    def test_actor_yields_to_occupied_lane_then_resumes_without_teleport(self):
        actors = sim.CrossingActors()
        previous = sim.obstacle_position(actors.specs[0], 'crossing', 0)
        for tick in range(1, 201):
            current = actors.update(tick*.05, (4.4, 0., 0., 0., 0.))[0]
            self.assertGreaterEqual(sim.clearance((4.4,0.,0.), [current]), sim.ACTOR_YIELD_GAP)
            self.assertLessEqual(math.dist(current[:2], previous[:2]), sim.ACTOR_SPEED*.05+1e-6)
            previous = current
        self.assertGreater(actors.states['crossing_a']['yield_events'], 0)
        for tick in range(201, 601):
            current = actors.update(tick*.05, (9., 0., 0., 0., 0.))[0]
            self.assertLessEqual(math.dist(current[:2], previous[:2]), sim.ACTOR_SPEED*.05+1e-6)
            previous = current
        self.assertAlmostEqual(current[1], -3.)

    def test_common_reference_clears_fixed_island_and_respects_turn_radius(self):
        path = sim.reference_points((0.,0.), (8.,0.), 'crossing')
        self.assertEqual(path[0], (0.,0.,0.))
        self.assertAlmostEqual(path[-1][0],8.); self.assertAlmostEqual(path[-1][1],0.)
        gaps = [sim.clearance(p, [(3.8,0.,.4)]) for p in path]
        self.assertGreater(min(gaps), .2)
        curvature = [abs(b[2]-a[2])/math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(path,path[1:])]
        self.assertLess(max(curvature), 1/1.85)

    def test_closed_bicycle_arc_and_partition_invariance(self):
        # This test is run in ROS1, but does not create a node or contact Gazebo.
        driver = load('driver', HERE/'scripts/ackermann_sim_driver.py')
        speed, steer, wheelbase = .5, .3, .67
        period = 2*math.pi*wheelbase/(speed*math.tan(steer))
        x, y, yaw = driver.integrate_ackermann(0, 0, 0, speed, steer, wheelbase, period)
        self.assertAlmostEqual(x, 0); self.assertAlmostEqual(y, 0); self.assertAlmostEqual(yaw, 2*math.pi)
        pose = (1., -2., .4)
        whole = driver.integrate_ackermann(*pose, speed, steer, wheelbase, 2.)
        for _ in range(20):
            pose = driver.integrate_ackermann(*pose, speed, steer, wheelbase, .1)
        for a, b in zip(pose, whole): self.assertAlmostEqual(a, b)

    def test_invalid_command_is_stopped_and_clock_reset_returns_to_spawn(self):
        driver = load('driver_checks', HERE/'scripts/ackermann_sim_driver.py')
        from geometry_msgs.msg import Twist
        import rospy
        obj = object.__new__(driver.AckermannSimDriver)
        obj.command_lock = threading.Lock()
        obj.max_speed, obj.max_steer = .5, .35
        command = Twist(); command.linear.x = float('nan'); command.angular.z = .2
        with patch.object(rospy.Time, 'now', return_value=rospy.Time(10)), patch.object(rospy, 'logerr_throttle'):
            obj.cmd_callback(command)
        self.assertEqual((obj.v, obj.steer), (0., 0.))
        obj.last_time = rospy.Time(10)
        obj.initial_pose = (0., 0., .1)
        obj.x, obj.y, obj.yaw, obj.wheel_pos = 8., 4., 2., 10.
        with patch.object(rospy.Time, 'now', return_value=rospy.Time(1)):
            obj.step()
        self.assertEqual((obj.x, obj.y, obj.yaw), obj.initial_pose)
        self.assertIsNone(obj.last_cmd_time)
        self.assertEqual(obj.wheel_pos, 0.)


if __name__ == '__main__': unittest.main()
