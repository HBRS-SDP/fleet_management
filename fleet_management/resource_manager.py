from ropod.pyre_communicator.base_class import RopodPyre
from ropod.structs.elevator import Elevator, RobotCallUpdate, RobotElevatorCallReply
from ropod.structs.elevator import ElevatorRequest
from ropod.structs.status import RobotStatus
from fleet_management.task_allocator import TaskAllocator
from datetime import timezone, datetime, timedelta
from dateutil import parser
from ropod.structs.area import SubArea
from ropod.utils.models import MessageFactory
import logging


class ResourceManager(RopodPyre):

    def __init__(self, config_params, ccu_store, osm_bridge):
        super().__init__(config_params.resource_manager_zyre_params.node_name,
                         config_params.resource_manager_zyre_params.groups,
                         config_params.resource_manager_zyre_params.message_types)
        self.robots = config_params.ropods
        self.elevators = config_params.elevators
        self.scheduled_robot_tasks = dict()
        self.elevator_requests = dict()
        self.robot_statuses = dict()
        self.ccu_store = ccu_store
        self.task_allocator = TaskAllocator(config_params, ccu_store)

        self.osm_bridge = osm_bridge

        self.building = config_params.building

        self.logger = logging.getLogger('fms.resources.manager')
        self.mf = MessageFactory()

        # parse out all our elevator information
        for elevator_param in self.elevators:
            elevator_dict = {}
            elevator_dict['elevatorId'] = elevator_param.id
            elevator_dict['floor'] = elevator_param.floor
            elevator_dict['calls'] = elevator_param.calls
            elevator_dict['isAvailable'] = elevator_param.isAvailable
            elevator_dict[
                'doorOpenAtGoalFloor'] = elevator_param.doorOpenAtGoalFloor
            elevator_dict[
                'doorOpenAtStartFloor'] = elevator_param.doorOpenAtStartFloor
            self.ccu_store.add_elevator(Elevator.from_dict(elevator_dict))

        # load task realated sub areas from OSM world model
        if self.osm_bridge is not None:
            self.load_sub_areas_from_osm()
        else:
            self.logger.error("Loading sub areas from OSM world model cancelled due to problem in intialising OSM bridge")

    def restore_data(self):
        self.elevators = self.ccu_store.get_elevators()
        self.robots = self.ccu_store.get_robots()

    '''Allocates a task or a list of tasks
    '''

    def get_robots_for_task(self, task):
        allocation = self.task_allocator.allocate(task)
        self.logger.info('Allocation: %s', allocation)
        return allocation

    ''' Returns a dictionary with the start and finish time of the task_id assigned to the robot_id
    '''

    def get_tasks_schedule_robot(self, task_id, robot_id):
        task_schedule = self.task_allocator.get_tasks_schedule_robot(
            task_id, robot_id)
        return task_schedule

    def receive_msg_cb(self, msg_content):
        dict_msg = self.convert_zyre_msg_to_dict(msg_content)
        if dict_msg is None:
            return

        msg_type = dict_msg['header']['type']
        if msg_type == 'ROBOT-ELEVATOR-CALL-REQUEST':
            command = dict_msg['payload']['command']
            query_id = dict_msg['payload']['queryId']

            if command == 'CALL_ELEVATOR':
                start_floor = dict_msg['payload']['startFloor']
                goal_floor = dict_msg['payload']['goalFloor']
                task_id = dict_msg['payload']['taskId']
                load = dict_msg['payload']['load']

                self.logger.info('Received elevator request from ropod')

                # TODO: Choose elevator, constructor uses by default
                # elevator_id=1
                robot_request = ElevatorRequest(start_floor, goal_floor,
                                                command,
                                                query_id=query_id,
                                                task_id=task_id, load=load,
                                                robot_id='ropod_001',
                                                status='pending')

                self.ccu_store.add_elevator_call(robot_request)
                self.request_elevator(robot_request)
            elif command == 'CANCEL_CALL':
                # TODO This is untested
                start_floor = dict_msg['payload']['startFloor']
                goal_floor = dict_msg['payload']['goalFloor']
                task_id = dict_msg['payload']['taskId']
                robot_request = ElevatorRequest(start_floor, goal_floor,
                                                command,
                                                query_id=query_id,
                                                task_id=task_id,
                                                robot_id='ropod_001',
                                                status='pending')
                self.cancel_elevator_call(robot_request)

        elif msg_type == 'ELEVATOR-CMD-REPLY':
            query_id = dict_msg['payload']['queryId']
            query_success = dict_msg['payload']['querySuccess']

            # TODO: Check for reply type: this depends on the query!
            command = dict_msg['payload']['command']
            self.logger.info('Received reply from elevator control for %s query', command)
            if command == 'CALL_ELEVATOR':
                self.confirm_elevator(query_id)

        elif msg_type == 'ROBOT-CALL-UPDATE':
            command = dict_msg['payload']['command']
            query_id = dict_msg['payload']['queryId']
            if command == 'ROBOT_FINISHED_ENTERING':
                # Close the doors
                self.logger.info('Received entering confirmation from ropod')

            elif command == 'ROBOT_FINISHED_EXITING':
                # Close the doors
                self.logger.info('Received exiting confirmation from ropod')
            self.confirm_robot_action(command, query_id)

        elif msg_type == 'ELEVATOR-STATUS':
            at_goal_floor = dict_msg['payload']['doorOpenAtGoalFloor']
            at_start_floor = dict_msg['payload']['doorOpenAtStartFloor']
            if at_start_floor:
                self.logger.info('Elevator reached start floor; waiting for confirmation...')
            elif at_goal_floor:
                self.logger.info('Elevator reached goal floor; waiting for confirmation...')
            elevator_update = Elevator.from_dict(dict_msg['payload'])
            self.ccu_store.update_elevator(elevator_update)

        elif msg_type == 'ROBOT-UPDATE':
            new_robot_status = RobotStatus.from_dict(dict_msg['payload'])
            self.ccu_store.update_robot(new_robot_status)

        else:
            self.logger.debug("Did not recognize message type %s", msg_type)

    def get_robot_status(self, robot_id):
        return self.robot_statuses[robot_id]

    def request_elevator(self, elevator_request):
        msg = self.mf.create_message(elevator_request)
        self.shout(msg, 'ELEVATOR-CONTROL')
        self.logger.info("Requested elevator...")

    def cancel_elevator_call(self, elevator_request):
        # TODO To cancel a call, the call ID should be sufficient:
        # read from ccu store, get info to cancel
        msg = self.mf.create_message(elevator_request)
        self.shout(msg, 'ELEVATOR-CONTROL')

    def confirm_robot_action(self, robot_action, query_id):

        if robot_action == 'ROBOT_FINISHED_ENTERING':
            # TODO Remove this hardcoded floor
            update = RobotCallUpdate(
                query_id, 'CLOSE_DOORS_AFTER_ENTERING', start_floor=1)
        elif robot_action == 'ROBOT_FINISHED_EXITING':
            # TODO Remove this hardcoded floor
            update = RobotCallUpdate(
                query_id, 'CLOSE_DOORS_AFTER_EXITING', goal_floor=1)

        msg = self.mf.create_message(update)

        # TODO This doesn't match the convention
        msg['header']['type'] = 'ELEVATOR-CMD'
        self.shout(msg, 'ELEVATOR-CONTROL')
        self.logger.debug('Sent robot confirmation to elevator')

    def confirm_elevator(self, query_id):
        # TODO This is using the default elevator
        # TODO How to obtain the elevator waypoint?
        reply = RobotElevatorCallReply(query_id)
        msg = self.mf.create_message(reply)
        self.shout(msg, 'ROPOD')
        self.logger.debug('Sent elevator confirmation to robot')

    def shutdown(self):
        super().shutdown()
        self.task_allocator.shutdown()

    def start(self):
        super().start()
        self.task_allocator.start()

    def load_sub_areas_from_osm(self):
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
