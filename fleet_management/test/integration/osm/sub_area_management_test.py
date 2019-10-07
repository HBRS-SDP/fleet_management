import unittest
from datetime import timezone, datetime, timedelta

from ropod.structs.area import SubArea, SubAreaReservation

from fleet_management.config.loader import Configurator
from fleet_management.plugins import osm


class TestSubAreaManagement(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        config = Configurator()
        cls.ccu_store = config.ccu_store
        # cls.api = config.api
        config_params = config._config_params['plugins']['osm']
        plugins = osm.configure(**config_params)

    @classmethod
    def tearDownClass(cls):
        cls.ccu_store.clean()
        # cls.api.shutdown()
        pass

    def setUp(self):
        pass

    def tearDown(self):
        # self.ccu_store.delete_sub_area_reservations()
        pass

    def test_dummy(self):
        self.assertTrue(True)

    # def test_add_and_get_sub_area(self):
    #     sub_area = SubArea()
    #     sub_area.id = 123
    #     sub_area.name = 'AMK_charging_area'
    #     sub_area.type = 'charging'
    #     sub_area.capacity = 1
    #     self.ccu_store.add_sub_area(sub_area)
    #     sub_area_from_db = self.ccu_store.get_sub_area(123)
    #     self.assertTrue(sub_area.id == sub_area_from_db.id)

    # def test_add_get_and_update_sub_area_reservation(self):
    #     # add test
    #     sub_area_reservation = SubAreaReservation()
    #     sub_area_reservation.sub_area_id = 123
    #     sub_area_reservation.task_id = 1
    #     sub_area_reservation.robot_id = 1
    #     sub_area_reservation.start_time = datetime.now(
    #         timezone.utc).isoformat()
    #     sub_area_reservation.end_time = datetime.now(timezone.utc).isoformat()
    #     sub_area_reservation.required_capacity = 1

    #     # get test
    #     obj = self.ccu_store.add_sub_area_reservation(sub_area_reservation)
    #     sub_area_reservation_db = self.ccu_store.get_sub_area_reservation(obj)
    #     self.assertTrue(sub_area_reservation.sub_area_id ==
    #                     sub_area_reservation_db.sub_area_id)

    #     # update test
    #     self.ccu_store.update_sub_area_reservation(obj, True)
    #     sub_area_reservation_db = self.ccu_store.get_sub_area_reservation(obj)
    #     self.assertTrue(sub_area_reservation_db.status)

    # def test_get_future_reservations(self):
    #     sub_area_reservation1 = SubAreaReservation()
    #     sub_area_reservation1.sub_area_id = 123
    #     sub_area_reservation1.task_id = 1
    #     sub_area_reservation1.robot_id = 1
    #     sub_area_reservation1.start_time = (datetime.now(
    #         timezone.utc) + timedelta(minutes=5)).isoformat()
    #     sub_area_reservation1.end_time = (datetime.now(
    #         timezone.utc) + timedelta(minutes=10)).isoformat()
    #     sub_area_reservation1.required_capacity = 1
    #     self.ccu_store.add_sub_area_reservation(sub_area_reservation1)

    #     sub_area_reservation2 = SubAreaReservation()
    #     sub_area_reservation2.sub_area_id = 123
    #     sub_area_reservation2.task_id = 1
    #     sub_area_reservation2.robot_id = 1
    #     sub_area_reservation2.start_time = (datetime.now(
    #         timezone.utc) + timedelta(minutes=15)).isoformat()
    #     sub_area_reservation2.end_time = (datetime.now(
    #         timezone.utc) + timedelta(minutes=20)).isoformat()
    #     sub_area_reservation2.required_capacity = 1
    #     self.ccu_store.add_sub_area_reservation(sub_area_reservation2)

    #     sub_area_reservation3 = SubAreaReservation()
    #     sub_area_reservation3.sub_area_id = 123
    #     sub_area_reservation3.task_id = 1
    #     sub_area_reservation3.robot_id = 1
    #     sub_area_reservation3.start_time = (datetime.now(
    #         timezone.utc) - timedelta(minutes=15)).isoformat()
    #     sub_area_reservation3.end_time = (datetime.now(
    #         timezone.utc) - timedelta(minutes=20)).isoformat()
    #     sub_area_reservation3.required_capacity = 1
    #     self.ccu_store.add_sub_area_reservation(sub_area_reservation3)

    #     results = self.ccu_store.get_all_future_reservations(123)
    #     self.assertTrue(len(results) == 2)

    # def test_if_reservation_possible(self):
    #     sub_area_reservation1 = SubAreaReservation()
    #     sub_area_reservation1.sub_area_id = 123
    #     sub_area_reservation1.task_id = 1
    #     sub_area_reservation1.robot_id = 1
    #     sub_area_reservation1.start_time = (datetime.now(
    #         timezone.utc) + timedelta(minutes=5)).isoformat()
    #     sub_area_reservation1.end_time = (datetime.now(
    #         timezone.utc) + timedelta(minutes=10)).isoformat()
    #     sub_area_reservation1.required_capacity = 1
    #     self.ccu_store.add_sub_area_reservation(sub_area_reservation1)

    #     sub_area_reservation2 = SubAreaReservation()
    #     sub_area_reservation2.sub_area_id = 123
    #     sub_area_reservation2.task_id = 1
    #     sub_area_reservation2.robot_id = 1
    #     sub_area_reservation2.status = 'scheduled'
    #     sub_area_reservation2.start_time = (datetime.now(
    #         timezone.utc) + timedelta(minutes=15)).isoformat()
    #     sub_area_reservation2.end_time = (datetime.now(
    #         timezone.utc) + timedelta(minutes=20)).isoformat()
    #     sub_area_reservation2.required_capacity = 1
    #     self.ccu_store.add_sub_area_reservation(sub_area_reservation2)

    #     # by default reservation status is unkonwn till its confirmed so
    #     # reservation should be possible
    #     sub_area_reservation = SubAreaReservation()
    #     sub_area_reservation.sub_area_id = 123
    #     sub_area_reservation.task_id = 1
    #     sub_area_reservation.robot_id = 1
    #     sub_area_reservation.start_time = (datetime.now(
    #         timezone.utc) + timedelta(minutes=7)).isoformat()
    #     sub_area_reservation.end_time = (datetime.now(
    #         timezone.utc) + timedelta(minutes=12)).isoformat()
    #     sub_area_reservation.required_capacity = 1
    #     self.assertTrue(
    #         self.osm_sub_area_monitor._is_reservation_possible(sub_area_reservation))

    #     # this should fail as there is already scheduled reservation for this
    #     sub_area_reservation.start_time = (datetime.now(
    #         timezone.utc) + timedelta(minutes=15)).isoformat()
    #     sub_area_reservation.end_time = (datetime.now(
    #         timezone.utc) + timedelta(minutes=10)).isoformat()
    #     self.assertFalse(
    #         self.osm_sub_area_monitor._is_reservation_possible(sub_area_reservation))

    #     # chage the reservation time beyond already scheduled reservations
    #     sub_area_reservation.start_time = (datetime.now(
    #         timezone.utc) + timedelta(minutes=40)).isoformat()
    #     sub_area_reservation.end_time = (datetime.now(
    #         timezone.utc) + timedelta(minutes=50)).isoformat()
    #     self.assertTrue(
    #         self.osm_sub_area_monitor._is_reservation_possible(sub_area_reservation))

    # def test_confirm_cancel_sub_area_reservation(self):
    #     sub_area_reservation = SubAreaReservation()
    #     sub_area_reservation.sub_area_id = 123
    #     sub_area_reservation.task_id = 1
    #     sub_area_reservation.robot_id = 1
    #     sub_area_reservation.start_time = (datetime.now(
    #         timezone.utc) + timedelta(minutes=40)).isoformat()
    #     sub_area_reservation.end_time = (datetime.now(
    #         timezone.utc) + timedelta(minutes=50)).isoformat()
    #     sub_area_reservation.required_capacity = 1

    #     # now lets confirm this reservation
    #     reservation_id = self.osm_sub_area_monitor.confirm_sub_area_reservation(
    #         sub_area_reservation)
    #     self.assertTrue(reservation_id)

    #     # now lets try to confirm another reservation on the same time. This
    #     # should fail
    #     self.assertFalse(
    #         self.osm_sub_area_monitor.confirm_sub_area_reservation(sub_area_reservation))

    #     # now lets cancel the reservation
    #     self.osm_sub_area_monitor.cancel_sub_area_reservation(reservation_id)

    #     # now lets try to confirm reservation on same time slot. Now it should
    #     # succeed
    #     self.assertTrue(
    #         self.osm_sub_area_monitor.confirm_sub_area_reservation(sub_area_reservation))

    # def test_earliest_available_reservation_slot(self):
    #     sub_area_reservation = SubAreaReservation()
    #     sub_area_reservation.sub_area_id = 123
    #     sub_area_reservation.task_id = 1
    #     sub_area_reservation.robot_id = 1
    #     sub_area_reservation.start_time = (datetime.now(
    #         timezone.utc) + timedelta(minutes=10)).isoformat()
    #     sub_area_reservation.end_time = (datetime.now(
    #         timezone.utc) + timedelta(minutes=20)).isoformat()
    #     sub_area_reservation.required_capacity = 1

    #     self.osm_sub_area_monitor.confirm_sub_area_reservation(
    #         sub_area_reservation)

    #     sub_area_reservation.start_time = (datetime.now(
    #         timezone.utc) + timedelta(minutes=30)).isoformat()
    #     sub_area_reservation.end_time = (datetime.now(
    #         timezone.utc) + timedelta(minutes=40)).isoformat()

    #     self.osm_sub_area_monitor.confirm_sub_area_reservation(
    #         sub_area_reservation)

    #     # now lets ask resource manager for 3 minutes slot
    #     earliest_time_slot1 = self.osm_sub_area_monitor.get_earliest_reservation_slot(
    #         123, 3)
    #     self.assertTrue(earliest_time_slot1 < (datetime.now(
    #         timezone.utc) + timedelta(minutes=10)).isoformat())

    #     # now lets ask resource manager for 15 minutes slot
    #     earliest_time_slot2 = self.osm_sub_area_monitor.get_earliest_reservation_slot(
    #         123, 15)
    #     self.assertTrue(earliest_time_slot2 > (datetime.now(
    #         timezone.utc) + timedelta(minutes=35)).isoformat())

    #     # now lets ask resource manager for 6 minutes slot
    #     earliest_time_slot3 = self.osm_sub_area_monitor.get_earliest_reservation_slot(
    #         123, 6)
    #     self.assertTrue(earliest_time_slot3 > (datetime.now(
    #         timezone.utc) + timedelta(minutes=15)).isoformat())

    # def test_load_and_get_sub_areas_from_osm(self):
    #     # Warning!!!: if we update task related areas in OSM this test will
    #     # fail!
    #     self.assertTrue(
    #         len(self.osm_sub_area_monitor.get_sub_areas_for_task('docking')) == 2)
    #     self.assertTrue(
    #         len(self.osm_sub_area_monitor.get_sub_areas_for_task('undocking')) == 2)
    #     self.assertTrue(
    #         len(self.osm_sub_area_monitor.get_sub_areas_for_task('charging')) == 2)


if __name__ == '__main__':
    unittest.main()
