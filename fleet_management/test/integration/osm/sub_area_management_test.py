#! /usr/bin/env python3

import uuid
import unittest
from datetime import timezone, datetime, timedelta

from fleet_management.db.models.environment import SubArea, SubareaReservation

from fleet_management.config.loader import Configurator
from fleet_management.plugins import osm


class TestSubAreaManagement(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        config = Configurator()
        cls.ccu_store = config.ccu_store
        config_params = config._config_params['plugins']['osm']
        plugins = osm.configure(**config_params)
        cls.subarea_monitor = plugins['subarea_monitor']

    @classmethod
    def tearDownClass(cls):
        pass

    def setUp(self):
        # adding a dummy subarea
        sub_area = SubArea(id=1000,
                           name='Dummy_sub_area',
                           behaviour='charging',
                           capacity=1)
        sub_area.save()

    def tearDown(self):
        self.ccu_store.clean()
        # pass

    # def test_add_and_get_sub_area(self):
    #     sub_area = SubArea(1000, 'Dummy_sub_area', 'charging', 1)
    #     sub_area.save()
    #     sub_area_from_db = SubArea.get_subarea(1000)
    #     self.assertTrue(sub_area.subarea_id == sub_area_from_db.subarea_id)

    def test_adding_sub_area_reservation(self):
        # adding a reservation
        sub_area_reservation = self._get_a_5_min_reservation_starting_n_min_from_now(n=0)
        sub_area_reservation.save()

        # test
        sub_area_reservation_db = SubareaReservation.get_subarea_reservation(
                sub_area_reservation.reservation_id)
        self.assertEqual(sub_area_reservation.subarea.id, 
                         sub_area_reservation_db.subarea.id)

    def test_updating_sub_area_reservation(self):
        # adding a reservation
        sub_area_reservation = self._get_a_5_min_reservation_starting_n_min_from_now(n=0)
        sub_area_reservation.save()

        sub_area_reservation.status = "scheduled"
        sub_area_reservation.save()
        sub_area_reservation_db = SubareaReservation.get_subarea_reservation(
                sub_area_reservation.reservation_id)
        self.assertEqual(sub_area_reservation_db.status, "scheduled")

    def test_get_future_reservations(self):
        sub_area_reservation1 = self._get_a_5_min_reservation_starting_n_min_from_now(n=5)
        sub_area_reservation1.save()

        sub_area_reservation2 = self._get_a_5_min_reservation_starting_n_min_from_now(n=15)
        sub_area_reservation2.save()

        sub_area_reservation3 = self._get_a_5_min_reservation_starting_n_min_from_now(n=-15)
        sub_area_reservation3.save()

        results = SubareaReservation.get_future_reservations_of_subarea(1000)
        self.assertEqual(len(results), 2)

    def test_if_reservation_possible(self):
        sub_area_reservation1 = self._get_a_5_min_reservation_starting_n_min_from_now(n=5)
        sub_area_reservation1.status = "scheduled"
        sub_area_reservation1.save()

        sub_area_reservation2 = self._get_a_5_min_reservation_starting_n_min_from_now(n=15)
        sub_area_reservation2.save()

        # this should fail as there is already scheduled reservation for this
        sub_area_reservation = self._get_a_5_min_reservation_starting_n_min_from_now(n=30)
        self.assertTrue(self.subarea_monitor.is_reservation_possible(sub_area_reservation))

    def test_if_reservation_possible_positive_conflicting_time(self):
        sub_area_reservation1 = self._get_a_5_min_reservation_starting_n_min_from_now(n=5)
        sub_area_reservation1.save()

        sub_area_reservation2 = self._get_a_5_min_reservation_starting_n_min_from_now(n=15)
        sub_area_reservation2.save()

        # by default reservation's status is unknown till its confirmed so
        # reservation should be possible
        sub_area_reservation = self._get_a_5_min_reservation_starting_n_min_from_now(n=7)
        self.assertTrue(self.subarea_monitor.is_reservation_possible(sub_area_reservation))

    def test_if_reservation_possible_negative_conflicting_time(self):
        sub_area_reservation1 = self._get_a_5_min_reservation_starting_n_min_from_now(n=5)
        sub_area_reservation1.status = "scheduled"
        sub_area_reservation1.save()

        sub_area_reservation2 = self._get_a_5_min_reservation_starting_n_min_from_now(n=15)
        sub_area_reservation2.save()

        # this should fail as there is already scheduled reservation for this
        sub_area_reservation = self._get_a_5_min_reservation_starting_n_min_from_now(n=7)
        self.assertFalse(self.subarea_monitor.is_reservation_possible(sub_area_reservation))

    def test_confirm_sub_area_reservation_positive(self):
        sub_area_reservation = self._get_a_5_min_reservation_starting_n_min_from_now(n=40)

        reservation_id = self.subarea_monitor.confirm_sub_area_reservation(
            sub_area_reservation)
        self.assertIsNotNone(reservation_id)

    def test_confirm_sub_area_reservation_negative(self):
        sub_area_reservation1 = self._get_a_5_min_reservation_starting_n_min_from_now(n=40)
        sub_area_reservation1.status = "scheduled"
        sub_area_reservation1.save()
        sub_area_reservation2 = self._get_a_5_min_reservation_starting_n_min_from_now(n=40)

        reservation_id = self.subarea_monitor.confirm_sub_area_reservation(
            sub_area_reservation2)
        self.assertIsNone(reservation_id)

    def test_cancel_sub_area_reservation(self):
        sub_area_reservation = self._get_a_5_min_reservation_starting_n_min_from_now(n=40)
        sub_area_reservation.status = "scheduled"
        sub_area_reservation.save()

        self.subarea_monitor.cancel_sub_area_reservation(sub_area_reservation)
        sub_area_reservation_db = SubareaReservation.get_subarea_reservation(
                sub_area_reservation.reservation_id)
        self.assertEqual(sub_area_reservation_db.status, "cancelled")

    def test_confirm_after_cancel_sub_area_reservation(self):
        sub_area_reservation = self._get_a_5_min_reservation_starting_n_min_from_now(n=40)
        sub_area_reservation.status = "scheduled"
        sub_area_reservation.save()

        self.subarea_monitor.cancel_sub_area_reservation(sub_area_reservation)
        sub_area_reservation_db = SubareaReservation.get_subarea_reservation(
                sub_area_reservation.reservation_id)
        self.assertEqual(sub_area_reservation_db.status, "cancelled")

        sub_area_reservation2 = self._get_a_5_min_reservation_starting_n_min_from_now(n=42)
        self.assertIsNotNone(self.subarea_monitor.confirm_sub_area_reservation(
            sub_area_reservation2))

    def test_earliest_available_reservation_slot(self):
        sub_area_reservation1 = self._get_a_5_min_reservation_starting_n_min_from_now(n=5)
        sub_area_reservation2 = self._get_a_5_min_reservation_starting_n_min_from_now(n=17)

        self.subarea_monitor.confirm_sub_area_reservation(sub_area_reservation1)
        self.subarea_monitor.confirm_sub_area_reservation(sub_area_reservation2)

        # now lets ask resource manager for 3 minutes slot
        earliest_time_slot1 = self.subarea_monitor.get_earliest_reservation_slot(
            1000, timedelta(minutes=3))
        self.assertLess(earliest_time_slot1, sub_area_reservation1.start_time)

        # now lets ask resource manager for 15 minutes slot
        earliest_time_slot2 = self.subarea_monitor.get_earliest_reservation_slot(
            1000, timedelta(minutes=15))
        self.assertGreaterEqual(earliest_time_slot2, sub_area_reservation2.end_time)

        # now lets ask resource manager for 6 minutes slot
        earliest_time_slot3 = self.subarea_monitor.get_earliest_reservation_slot(
            1000, timedelta(minutes=6))
        self.assertGreaterEqual(earliest_time_slot3, sub_area_reservation1.end_time)
        self.assertLess(earliest_time_slot3, sub_area_reservation2.start_time)

    # def test_dummy(self):
    #     sub_area_reservation1 = self._get_a_5_min_reservation_starting_n_min_from_now(n=5)
    #     sub_area_reservation2 = self._get_a_5_min_reservation_starting_n_min_from_now(n=17)

    #     self.subarea_monitor.confirm_sub_area_reservation(sub_area_reservation1)
    #     self.subarea_monitor.confirm_sub_area_reservation(sub_area_reservation2)

    #     print(sub_area_reservation1.to_son())
    #     print(sub_area_reservation2.to_son())
    #     print(type(sub_area_reservation1.subarea))

    #     # subareas = SubareaReservation.get_subarea_reservations_by_subarea_id(1000)
    #     # print(subareas)
    #     future_reservations = SubareaReservation.get_future_reservations_of_subarea(1000)
    #     print(future_reservations)


    # def test_load_and_get_sub_areas_from_osm(self):
    #     # Warning!!!: if we update task related areas in OSM this test will
    #     # fail!
    #     self.assertEqual(len(self.subarea_monitor.get_subarea_by_type('docking')), 2)
    #     self.assertEqual(len(self.subarea_monitor.get_subarea_by_type('undocking')), 2)
    #     self.assertEqual(len(self.subarea_monitor.get_subarea_by_type('charging')), 2)
    
    def _get_a_5_min_reservation_starting_n_min_from_now(self, n=0):
        sub_area_reservation = SubareaReservation()
        sub_area_reservation.reservation_id = uuid.uuid4()
        sub_area_reservation.subarea = SubArea.get_subarea(1000)
        sub_area_reservation.task_id = 1
        sub_area_reservation.robot_id = 1
        sub_area_reservation.status = "unknown"
        sub_area_reservation.start_time = datetime.now() + timedelta(minutes=n)
        sub_area_reservation.end_time = datetime.now() + timedelta(minutes=n+5)
        sub_area_reservation.required_capacity = 1
        return sub_area_reservation


if __name__ == '__main__':
    unittest.main()
