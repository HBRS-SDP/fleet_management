#! /usr/bin/env python3

from __future__ import print_function

import unittest

from fleet_management.config.loader import Configurator
from fleet_management.plugins import topology


class TestPathPlanner(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        config = Configurator(config="topology")
        cls.ccu_store = config.ccu_store
        config_params = config._config_params["plugins"]["topology"]
        plugins = topology.configure(**config_params)
        cls.path_planner = plugins["path_planner"]
        cls.docking_local_areas = cls.path_planner.map_bridge.get_all_nodes_of_behaviour_type(
            "docking"
        )
        cls.undocking_local_areas = cls.path_planner.map_bridge.get_all_nodes_of_behaviour_type(
            "undocking"
        )

    @classmethod
    def tearDownClass(cls):
        pass

    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_from_all_docking_to_all_undocking(self):
        for start_local_area in self.docking_local_areas:
            for destination_local_area in self.undocking_local_areas:
                plan = self.path_planner.get_path_plan(
                    start_local_area, destination_local_area
                )
                self.assertIsNotNone(plan)


if __name__ == "__main__":
    unittest.main()