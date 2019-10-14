import logging
from datetime import timezone, datetime, timedelta
from dateutil import parser

from fleet_management.models.subareas import Subarea, SubareaReservation


class _OSMSubAreaMonitor(object):

    """Monitor and manage OSM sub areas for dynamic information."""

    def __init__(self, osm_bridge=None, building='AMK'):
        self.osm_bridge = osm_bridge
        self.building = building

        self.logger = logging.getLogger('fms.resources.monitoring.osm_sub_area_monitor')

        # load task related sub areas from OSM world model
        if self.osm_bridge is not None:
            self._load_sub_areas_from_osm()
        else:
            self.logger.error("Loading sub areas from OSM world model cancelled "
                              "due to osm_bridge being None")
        
    def _load_sub_areas_from_osm(self):
        """loads sub areas from OSM
        """
        building = self.osm_bridge.get_building(self.building)
        for floor in building.floors:
            self._update_sub_area_database(floor.rooms)
            self._update_sub_area_database(floor.corridors)
            self._update_sub_area_database(floor.areas)

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
                osm_sub_area.geometry # required since all tags are in geometrical model
                if osm_sub_area.behaviour:
                    self.logger.debug("Initialising sub area: %s", osm_sub_area.ref)
                    sub_area = Subarea(osm_sub_area.id,
                                       osm_sub_area.ref,
                                       osm_sub_area.behaviour,
                                       1)
                    sub_area.save()

    def confirm_sub_area_reservation(self, subarea_reservation):
        """checks if sub area can be reserved and confirms reservation if possible
        :subarea_reservation: SubareaReservation object
        :returns: int or None (reservation id if successful else None)
        """
        if self.is_reservation_possible(subarea_reservation):
            # TODO: get current status of sub area to reserve, from dynamic
            # world model
            subarea_reservation.status = "scheduled"
            subarea_reservation.save()
            return subarea_reservation.reservation_id

    def cancel_sub_area_reservation(self, sub_area_reservation):
        """cancels already confirmed sub area reservation
        :sub_area_reservation: SubareaReservation obj
        :returns: None
        """
        sub_area_reservation.status = "cancelled"
        sub_area_reservation.save()

    def is_reservation_possible(self, subarea_reservation):
        """checks if sub area reservation is possible
        :subarea_reservation: SubareaReservation object
        :returns: bool
        """
        subarea_capacity = Subarea.get_subarea(subarea_reservation.subarea_id).capacity
        available_capacity = subarea_capacity - subarea_reservation.required_capacity
        future_reservations = SubareaReservation.get_future_reservations_of_subarea(
            subarea_reservation.subarea_id)
        for future_reservation in future_reservations:
            if future_reservation.status == 'scheduled':
                if (available_capacity > 0):
                    return True
                if future_reservation.start_time <= subarea_reservation.start_time <= future_reservation.end_time or\
                        future_reservation.start_time <= subarea_reservation.end_time <= future_reservation.end_time:
                    return False
        return True

    def get_earliest_reservation_slot(self, subarea_id, slot_duration):
        """finds earliest possible start time when given sub area can be
           reserved for specific amount of time

        :slot_duration: datetime.timedelta obj
        :subarea_id: int
        :returns: datetime.datetime obj

        """
        earliest_possible_time = datetime.now() + timedelta(minutes=1) # 1 min for calc and comm overhead
        future_reservations = SubareaReservation.get_future_reservations_of_subarea(
            subarea_id)
        future_reservations.sort(key=lambda x:x.start_time)
        for future_reservation in future_reservations:
            if future_reservation.status != "scheduled":
                continue
            diff = future_reservation.start_time - earliest_possible_time
            if diff > slot_duration:
                return earliest_possible_time
            earliest_possible_time = future_reservation.end_time + timedelta(seconds=1)
        return earliest_possible_time

    """
    def subarea_reservation_cb(self, msg):
        #TODO: This whole block is just a skeleton and should be reimplemented according to need.
        if 'payload' not in msg:
            self.logger.debug('SUB-AREA-RESERVATION msg did not contain payload')

        command = msg['payload'].get('command', None)
        valid_commands = ['RESERVATION-QUERY',
                          'CONFIRM-RESERVATION',
                          'EARLIEST-RESERVATION',
                          'CANCEL-RESERVATION']
        if command not in valid_commands:
            self.logger.debug('SUB-AREA-RESERVATION msg payload did not contain valid command')
        if command == 'RESERVATION-QUERY':
            task = msg['payload'].get('task', None)
            self.osm_sub_area_monitor.get_sub_areas_for_task(task)
        elif command == 'CONFIRM-RESERVATION':
            reservation_object = msg['payload'].get('reservation_object', None)
            self.osm_sub_area_monitor.confirm_sub_area_reservation(reservation_object)
        elif command == 'EARLIEST-RESERVATION':
            sub_area_id = msg['payload'].get('sub_area_id', None)
            duration = msg['payload'].get('duration', None)
            self.osm_sub_area_monitor.get_earliest_reservation_slot(sub_area_id, duration)
        elif command == 'CANCEL-RESERVATION':
            reservation_id = msg['payload'].get('reservation_id', None)
            self.osm_sub_area_monitor.cancel_sub_area_reservation(reservation_id)
    """

class OSMSubAreaMonitorBuilder:
    def __init__(self):
        self._instance = None

    def __call__(self, **kwargs):
        if not self._instance:
            self._instance = _OSMSubAreaMonitor(**kwargs)
        return self._instance

configure = OSMSubAreaMonitorBuilder()
