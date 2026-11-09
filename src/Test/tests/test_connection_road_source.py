"""Measured road alignment is source-bound, without removing scan checks."""
import unittest
import numpy as np
from osm_outdoor_start_connection import RoadCorridor, StartRouteConnector


class ConnectionRoadTests(unittest.TestCase):
    def test_accepted_edge_aligns_only_observed_corridor_with_existing_width(self):
        original=RoadCorridor([[-10,0,10,0,3,0]])
        connector=StartRouteConnector(np.array([[-.1,-.4],[.8,-.4],[.8,.4],[-.1,.4]]),2.,original)
        region=dict(frame_id='map',reference_version=2,right_edge_map=[[-5,-5],[5,-5]],original_route_map=[[-5,0],[5,0]])
        connector.use_advisory_road(region,2)
        self.assertEqual(connector.corridor_source,'accepted_right_edge_with_osm_width_v2')
        np.testing.assert_array_equal(connector.corridor.segments[0],[-5,-2,5,-2,3,0])
        np.testing.assert_array_equal(original.segments[0],[-10,0,10,0,3,0])
        connector.use_advisory_road(region,3)
        self.assertIsNone(connector.corridor)
        self.assertEqual(connector.corridor_source,'road_alignment_evidence_insufficient')
        connector.use_advisory_road(None,0)
        self.assertIs(connector.corridor,original)
