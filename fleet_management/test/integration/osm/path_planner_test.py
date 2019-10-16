#! /usr/bin/env python3

from __future__ import print_function

import unittest

from fleet_management.config.loader import Configurator
from fleet_management.plugins import osm

class TestPathPlanner(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        config = Configurator()
        cls.ccu_store = config.ccu_store
        config_params = config._config_params['plugins']['osm']
        plugins = osm.configure(**config_params)
        cls.path_planner = plugins['path_planner']
        cls.building = str(config_params['path_planner']['building'])
        cls.docking_local_areas = cls.path_planner.osm_bridge.get_all_local_area_of_behaviour_type(cls.building, 'docking')
        cls.undocking_local_areas = cls.path_planner.osm_bridge.get_all_local_area_of_behaviour_type(cls.building, 'undocking')

    @classmethod
    def tearDownClass(cls):
        pass

    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_path_planning_to_and_from_elevator_brsu(self):
        elevator_local_area = dict()
        if self.building == 'BRSU':
            elevator_local_area = {'0': 'BRSU_A_L0_A1_LA1', '2': 'BRSU_A_L2_A1_LA1'}
        elif self.building == 'AMK':
            elevator_local_area = {'-1': 'AMK_B_L-1_C2_LA1', '4': 'AMK_B_L4_C0_LA1'}
        else:
            self.skipTest('Unrecognised building')
        for start_local_area in self.docking_local_areas:
            floor = self._get_floor_of_local_area(start_local_area)
            destination_local_area = elevator_local_area[str(floor)]
            # from docking area to elevator
            plan = self.path_planner.get_path_plan_from_local_area(
                    start_local_area, destination_local_area)
            self.assertIsNotNone(plan)
            # from elevator to docking area
            plan = self.path_planner.get_path_plan_from_local_area(
                    destination_local_area, start_local_area)
            self.assertIsNotNone(plan)
        for start_local_area in self.undocking_local_areas:
            floor = self._get_floor_of_local_area(start_local_area)
            destination_local_area = elevator_local_area[str(floor)]
            # from undocking area to elevator
            plan = self.path_planner.get_path_plan_from_local_area(
                    start_local_area, destination_local_area)
            self.assertIsNotNone(plan)
            # from elevator to undocking area
            plan = self.path_planner.get_path_plan_from_local_area(
                    destination_local_area, start_local_area)
            self.assertIsNotNone(plan)

    def test_from_all_docking_to_all_undocking(self):
        for start_local_area in self.docking_local_areas:
            for destination_local_area in self.undocking_local_areas:
                plan = self.path_planner.get_path_plan_from_local_area(
                        start_local_area, destination_local_area)
                self.assertIsNotNone(plan)

    def _get_floor_of_local_area(self, local_area_name):
        local_area_obj = self.path_planner.osm_bridge.get_local_area(local_area_name)
        local_area_obj.geometry
        return local_area_obj.level

if __name__ == '__main__':
    unittest.main()
