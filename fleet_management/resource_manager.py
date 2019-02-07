from __future__ import print_function
from ropod.pyre_communicator.base_class import RopodPyre
from ropod.structs.elevator import Elevator
from ropod.structs.elevator import ElevatorRequest
from ropod.structs.status import RobotStatus
from fleet_management.task_allocator import TaskAllocator
from datetime import timezone, datetime, timedelta
from dateutil import parser
from OBL import OSMBridge
from ropod.structs.area import SubArea


class ResourceManager(RopodPyre):
    def __init__(self, config_params, ccu_store):
        super().__init__(config_params.resource_manager_zyre_params.node_name,
                         config_params.resource_manager_zyre_params.groups,
                         config_params.resource_manager_zyre_params.message_types)
        self.robots = config_params.ropods
        self.elevators = config_params.elevators
        self.scheduled_robot_tasks = dict()
        self.elevator_requests = dict()
        self.robot_statuses = dict()
        self.ccu_store = ccu_store
        self.task_allocator = TaskAllocator(config_params)
        self.osm_bridge = OSMBridge(server_ip=config_params.overpass_server.ip, server_port=config_params.overpass_server.port)
        self.building = config_params.building

        # parse out all our elevator information
        for elevator_param in self.elevators:
            elevator_dict = {}
            elevator_dict['id'] = elevator_param.id
            elevator_dict['floor'] = elevator_param.floor
            elevator_dict['calls'] = elevator_param.calls
            elevator_dict['isAvailable'] = elevator_param.isAvailable
            elevator_dict['doorOpenAtGoalFloor'] = elevator_param.doorOpenAtGoalFloor
            elevator_dict['doorOpenAtStartFloor'] = elevator_param.doorOpenAtStartFloor
            self.ccu_store.add_elevator(Elevator.from_dict(elevator_dict))

        # parse out all our elevator information
        for elevator_param in self.elevators:
            elevator_dict = {}
            elevator_dict['id'] = elevator_param.id
            elevator_dict['floor'] = elevator_param.floor
            elevator_dict['calls'] = elevator_param.calls
            elevator_dict['isAvailable'] = elevator_param.isAvailable
            elevator_dict['doorOpenAtGoalFloor'] = elevator_param.doorOpenAtGoalFloor
            elevator_dict['doorOpenAtStartFloor'] = elevator_param.doorOpenAtStartFloor
            self.ccu_store.add_elevator(Elevator.from_dict(elevator_dict))

        # load task realated sub areas from OSM world model
        self.load_sub_areas_from_osm()

    def restore_data(self):
        self.elevators = self.ccu_store.get_elevators()
        self.robots = self.ccu_store.get_robots()

    '''Allocates a task or a list of tasks
    '''

    def get_robots_for_task(self, task):
        allocation = self.task_allocator.allocate(task)
        print(allocation)
        return allocation

    ''' Returns a dictionary with the start and finish time of the task_id assigned to the robot_id
    '''

    def get_tasks_schedule_robot(self, task_id, robot_id):
        task_schedule = self.task_allocator.get_tasks_schedule_robot(task_id, robot_id)
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

                print('[INFO] Received elevator request from ropod')

                robot_request = ElevatorRequest()

                # TODO: Choose elevator
                robot_request.elevator_id = 1
                robot_request.operational_mode = 'ROBOT'

                # TODO: Store this query somewhere
                robot_request.query_id = query_id
                robot_request.command = command
                robot_request.start_floor = start_floor
                robot_request.goal_floor = goal_floor
                robot_request.task_id = task_id
                robot_request.load = load
                robot_request.robot_id = 'ropod_001'
                robot_request.status = 'pending'

                self.ccu_store.add_elevator_call(robot_request)
                self.request_elevator(start_floor, goal_floor, robot_request.elevator_id,
                                      robot_request.query_id)
            elif command == 'CANCEL_CALL':
                start_floor = dict_msg['payload']['startFloor']
                goal_floor = dict_msg['payload']['goalFloor']

        elif msg_type == 'ELEVATOR-CMD-REPLY':
            query_id = dict_msg['payload']['queryId']
            query_success = dict_msg['payload']['querySuccess']

            # TODO: Check for reply type: this depends on the query!
            command = dict_msg['payload']['command']
            print('[INFO] Received reply from elevator control for %s query' % command)
            if command == 'CALL_ELEVATOR':
                self.confirm_elevator(query_id)

        elif msg_type == 'ROBOT-CALL-UPDATE':
            command = dict_msg['payload']['command']
            query_id = dict_msg['payload']['queryId']
            if command == 'ROBOT_FINISHED_ENTERING':
                # Close the doors
                print('[INFO] Received entering confirmation from ropod')

            elif command == 'ROBOT_FINISHED_EXITING':
                # Close the doors
                print('[INFO] Received exiting confirmation from ropod')
            self.confirm_robot_action(command, query_id)

        elif msg_type == 'ELEVATOR-STATUS':
            at_goal_floor = dict_msg['payload']['doorOpenAtGoalFloor']
            at_start_floor = dict_msg['payload']['doorOpenAtStartFloor']
            if at_start_floor:
                print('[INFO] Elevator reached start floor; waiting for confirmation...')
            elif at_goal_floor:
                print('[INFO] Elevator reached goal floor; waiting for confirmation...')
            elevator_update = Elevator.from_dict(dict_msg['payload'])
            self.ccu_store.update_elevator(elevator_update)

        elif msg_type == 'ROBOT-UPDATE':
            new_robot_status = RobotStatus.from_dict(dict_msg['payload'])
            self.ccu_store.update_robot(new_robot_status)

        else:
            if self.verbose:
                print("Did not recognize message type %s" % msg_type)

    def get_robot_status(self, robot_id):
        return self.robot_statuses[robot_id]

    def request_elevator(self, start_floor, goal_floor, elevator_id, query_id):
        msg_dict = dict()
        msg_dict['header'] = dict()
        msg_dict['payload'] = dict()

        msg_dict['header']['type'] = 'ELEVATOR-CMD'
        msg_dict['header']['metamodel'] = "ropod-msg-schema.json"
        msg_dict['header']['msgId'] = self.generate_uuid()
        msg_dict['header']['timestamp'] = ''
        msg_dict['header']['timestamp'] = self.get_time_stamp()

        msg_dict['payload']['metamodel'] = 'ropod-elevator-cmd-schema.json'
        msg_dict['payload']['startFloor'] = start_floor
        msg_dict['payload']['goalFloor'] = goal_floor
        msg_dict['payload']['elevatorId'] = elevator_id
        msg_dict['payload']['operationalMode'] = 'ROBOT'
        msg_dict['payload']['queryId'] = query_id
        msg_dict['payload']['command'] = 'CALL_ELEVATOR'
        self.shout(msg_dict, 'ELEVATOR-CONTROL')

    def confirm_robot_action(self, robot_action, query_id):
        msg_dict = dict()
        msg_dict['header'] = dict()
        msg_dict['payload'] = dict()

        msg_dict['header']['type'] = 'ELEVATOR-CMD'
        msg_dict['header']['metamodel'] = 'ropod-msg-schema.json'
        msg_dict['header']['msgId'] = self.generate_uuid()
        msg_dict['header']['timestamp'] = self.get_time_stamp()

        msg_dict['payload']['metamodel'] = 'ropod-robot-call-update-schema.json'
        msg_dict['payload']['queryId'] = query_id
        msg_dict['payload']['elevatorId'] = 1
        if robot_action == 'ROBOT_FINISHED_ENTERING':
            msg_dict['payload']['startFloor'] = 1
            msg_dict['payload']['command'] = 'CLOSE_DOORS_AFTER_ENTERING'
        elif robot_action == 'ROBOT_FINISHED_EXITING':
            msg_dict['payload']['command'] = 'CLOSE_DOORS_AFTER_EXITING'
            msg_dict['payload']['goalFloor'] = 1
        self.shout(msg_dict, 'ELEVATOR-CONTROL')

    # TODO: In ongoing calls, we need to check when we have arrived to the goal floor
    def confirm_elevator(self, query_id):
        msg_dict = dict()
        msg_dict['header'] = dict()
        msg_dict['payload'] = dict()

        msg_dict['header']['type'] = 'ROBOT-ELEVATOR-CALL-REPLY'
        msg_dict['header']['metamodel'] = 'ropod-msg-schema.json'
        msg_dict['header']['msgId'] = self.generate_uuid()
        msg_dict['header']['timestamp'] = self.get_time_stamp()

        msg_dict['payload']['metamodel'] = 'ropod-elevator-cmd-schema.json'
        msg_dict['payload']['queryId'] = query_id
        msg_dict['payload']['querySuccess'] = True
        msg_dict['payload']['elevatorId'] = 1
        msg_dict['payload']['elevatorWaypoint'] = 'door-1'
        self.shout(msg_dict, 'ROPOD')

    def cancel_elevator_call(self, elevator_id, start_floor, goal_floor):
        msg = dict()
        msg['header'] = dict()
        msg['payload'] = dict()

        msg['header']['type'] = "ELEVATOR-CMD"
        msg['header']['metamodel'] = 'ropod-msg-schema.json'
        msg['header']['msgId'] = self.generate_uuid()
        msg['header']['timestamp'] = self.get_time_stamp()

        msg['payload']['metamodel'] = 'ropod-elevator-cmd-schema.json'
        msg['payload']['queryId'] = self.generate_uuid()  # TODO this needs to be the id of the call
        msg['payload']['command'] = 'CANCEL_CALL'
        msg['payload']['elevatorId'] = elevator_id
        msg['payload']['startFloor'] = start_floor
        msg['payload']['goalFloor'] = goal_floor
        self.shout(msg, 'ELEVATOR-CONTROL')

    def shutdown(self):
        super().shutdown()
        self.task_allocator.shutdown()

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
                self._convert_and_add_sub_areas_to_database(osm_area.local_areas)

    def _convert_and_add_sub_areas_to_database(self, osm_sub_areas):
        """converts and adds list of sub areas to the database
        :osm_sub_area: list of OBL local areas.
        :returns: None
        """
        if osm_sub_areas is not None:
            for osm_sub_area in osm_sub_areas:
                osm_sub_area.geometry     # this is required since all tags are stored in geometrical model
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
            # TODO: get current status of sub area to reserve, from dynamic world model
            sub_area_reservation.status = "scheduled"
            return self.ccu_store.add_sub_area_reservation(sub_area_reservation)
        else:
            return False

    def cancel_sub_area_reservation(self, sub_area_reservation_id):
        """cancells already confirmed sub area reservation
        :sub_area_reservation_id: sub area reservation id returned after confirmation
        """
        self.ccu_store.update_sub_area_reservation(sub_area_reservation_id, 'cancelled')
        
    def get_earliest_reservation_slot(self, sub_area_id, slot_duration_in_mins):
        """finds earliest possible start time when given sub area can be reserved for specific amount of time
        :slot_duration_in_mins: duration for which sub area needs to be reserved
        :sub_area_id: sub area id
        :returns: earliest start time is ISO format
        """
        future_reservations = self.ccu_store.get_all_future_reservations(sub_area_id)
        prev_time = (datetime.now(timezone.utc) + timedelta(minutes=5)).isoformat()
        for future_reservation in future_reservations:
            diff = (parser.parse(future_reservation.start_time) - parser.parse(prev_time)).total_seconds()/60.0  
            if diff > slot_duration_in_mins:
                return prev_time
            prev_time = future_reservation.end_time
        return prev_time

    def _is_reservation_possible(self, sub_area_reservation):
        """checks if sub area reservation is possible
        :sub_area_reservation: sub area reservation object
        :returns: true/ false 
        """
        sub_area_capacity = self.ccu_store.get_sub_area(sub_area_reservation.sub_area_id).capacity
        available_capacity = int(sub_area_capacity) - int(sub_area_reservation.required_capacity)
        future_reservations = self.ccu_store.get_all_future_reservations(sub_area_reservation.sub_area_id)
        for future_reservation in future_reservations:
            if future_reservation.status == 'scheduled':
                if (available_capacity > 0):
                    return True
                if(self._is_time_between(future_reservation.start_time, future_reservation.end_time, sub_area_reservation.start_time)\
                or self._is_time_between(future_reservation.start_time, future_reservation.end_time, sub_area_reservation.end_time)):
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
        else: # crosses midnight
            return check_time >= begin_time or check_time <= end_time

