import logging
from datetime import timezone, datetime, timedelta
from dateutil import parser

from ropod.structs.area import SubArea

class OSMSubAreaMonitor(object):

    """Monitor and manage OSM sub areas for dynamic information."""

    def __init__(self, config_params, ccu_store, osm_bridge):
        self.ccu_store = ccu_store
        self.osm_bridge = osm_bridge
        self.building = config_params.building

        self.logger = logging.getLogger('fms.resources.monitoring.osm_sub_area_monitor')

        # load task realated sub areas from OSM world model
        if self.osm_bridge is not None:
            self._load_sub_areas_from_osm()
        else:
            self.logger.error("Loading sub areas from OSM world model cancelled due to problem in intialising OSM bridge")
        
    def _load_sub_areas_from_osm(self):
        """loads sub areas from OSM
        """
        building = self.osm_bridge.get_building(self.building)
        for floor in building.floors:
            self._update_sub_area_database(floor.rooms)
            self._update_sub_area_database(floor.corridors)
            self._update_sub_area_database(floor.areas)

    def get_sub_areas_for_task(self, task):
        """returns sub areas available for the specified task
        :task: undocking/docking/charging etc.
        :returns: sub areas list
        """
        return self.ccu_store.get_sub_areas(task)

    def _update_sub_area_database(self, osm_areas):
        """returns sub areas available for spefcified task
        :task: undocking/docking/charging etc.
        :returns: sub areas list
        """
        if osm_areas is not None:
            for osm_area in osm_areas:
                self._convert_and_add_sub_areas_to_database(
                    osm_area.local_areas)

    def _convert_and_add_sub_areas_to_database(self, osm_sub_areas):
        """converts and adds list of sub areas to the database
        :osm_sub_area: list of OBL local areas.
        :returns: None
        """
        if osm_sub_areas is not None:
            for osm_sub_area in osm_sub_areas:
                # this is required since all tags are stored in geometrical
                # model
                osm_sub_area.geometry
                if osm_sub_area.behaviour:
                    sub_area = SubArea()
                    sub_area.id = osm_sub_area.id
                    sub_area.name = osm_sub_area.ref
                    sub_area.type = osm_sub_area.behaviour
                    sub_area.capacity = 1
                    self.ccu_store.add_sub_area(sub_area)

    def confirm_sub_area_reservation(self, sub_area_reservation):
        """checks if sub area can be reserved and confirms reservation if possible
        :sub_area_reservation: sub area reservation object
        :returns: reservation id if successful or false
        """
        if self._is_reservation_possible(sub_area_reservation):
            # TODO: get current status of sub area to reserve, from dynamic
            # world model
            sub_area_reservation.status = "scheduled"
            return self.ccu_store.add_sub_area_reservation(sub_area_reservation)
        else:
            return False

    def cancel_sub_area_reservation(self, sub_area_reservation_id):
        """cancells already confirmed sub area reservation
        :sub_area_reservation_id: sub area reservation id returned
        after confirmation
        """
        self.ccu_store.update_sub_area_reservation(
            sub_area_reservation_id, 'cancelled')

    def _is_reservation_possible(self, sub_area_reservation):
        """checks if sub area reservation is possible
        :sub_area_reservation: sub area reservation object
        :returns: true/ false
        """
        sub_area_capacity = self.ccu_store.get_sub_area(
            sub_area_reservation.sub_area_id).capacity
        available_capacity = int(sub_area_capacity) - \
            int(sub_area_reservation.required_capacity)
        future_reservations = self.ccu_store.get_all_future_reservations(
            sub_area_reservation.sub_area_id)
        for future_reservation in future_reservations:
            if future_reservation.status == 'scheduled':
                if (available_capacity > 0):
                    return True
                if(self._is_time_between(future_reservation.start_time,
                                         future_reservation.end_time,
                                         sub_area_reservation.start_time) or
                   self._is_time_between(future_reservation.start_time,
                                         future_reservation.end_time,
                                         sub_area_reservation.end_time)):
                    return False
        return True

    def _is_time_between(self, begin_time, end_time, check_time):
        """finds if check time lies between start and end time
        :begin_time: time or ISO format
        :end_time: time or ISO format
        :check_time: time or ISO format
        :returns: true/false
        """
        if begin_time < end_time:
            return check_time >= begin_time and check_time <= end_time
        else:  # crosses midnight
            return check_time >= begin_time or check_time <= end_time

    def get_earliest_reservation_slot(self, sub_area_id, slot_duration_in_mins):
        """finds earliest possible start time when given sub area can be
           reserved for specific amount of time
        :slot_duration_in_mins: duration for which sub area needs to be
                                reserved
        :sub_area_id: sub area id
        :returns: earliest start time is ISO format
        """
        future_reservations = self.ccu_store.get_all_future_reservations(
            sub_area_id)
        prev_time = (datetime.now(timezone.utc) +
                     timedelta(minutes=5)).isoformat()
        for future_reservation in future_reservations:
            diff = (parser.parse(future_reservation.start_time) -
                    parser.parse(prev_time)).total_seconds() / 60.0
            if diff > slot_duration_in_mins:
                return prev_time
            prev_time = future_reservation.end_time
        return prev_time

