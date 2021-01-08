#! /usr/bin/env python3

from __future__ import print_function

import unittest

from fleet_management.config.loader import Configurator
from fleet_management.plugins import topology

class TestPathPlanner(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        config = Configurator()
        cls.ccu_store = config.ccu_store
        config_params = config._config_params['plugins']['topology']
        plugins = topology.configure(**config_params)
        cls.path_planner = plugins['path_planner']


    @classmethod
    def tearDownClass(cls):
        pass

    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_path_planning_to_and_from_elevator(self):
        
        start_local_area = 'C018-1'
        destination_local_area = 'C022'
        plan = self.path_planner.get_path_plan(start_local_area, destination_local_area)
        self.assertIsNotNone(plan)


if __name__ == '__main__':
    unittest.main()