import logging

from ropod.pyre_communicator.base_class import RopodPyre
from ropod.structs.elevator import Elevator, RobotCallUpdate, RobotElevatorCallReply
from ropod.structs.elevator import ElevatorRequest
from ropod.structs.status import RobotStatus
from ropod.structs.area import SubArea
from ropod.utils.models import MessageFactory

from fleet_management.task_allocator import TaskAllocator
from fleet_management.resources.monitoring import osm_sub_area_monitor


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

        self.osm_sub_area_monitor = OSMSubAreaMonitor(config_params, ccu_store, osm_bridge)

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

    def restore_data(self):
        self.elevators = self.ccu_store.get_elevators()
        self.robots = self.ccu_store.get_robots()

    def get_robots_for_task(self, task):
        '''Allocates a task or a list of tasks
        '''
        allocation = self.task_allocator.allocate(task)
        self.logger.info('Allocation: %s', allocation)
        return allocation

    def get_tasks_schedule_robot(self, task_id, robot_id):
        ''' Returns a dictionary with the start and finish time of the task_id assigned to the robot_id
        '''
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

        elif msg_type == 'SUB-AREA-RESERVATION': #TODO:this msg type is not decided officially yet
            #TODO: This whole block is just a skeleton and should be reimplemented according to need.
            if 'payload' not in dict_msg:
                self.logger.debug('SUB-AREA-RESERVATION msg did not contain payload')

            command = dict_msg['payload'].get('command', None)
            valid_commands = ['RESERVATION-QUERY', 
                            'CONFIRM-RESERVATION', 
                            'EARLIEST-RESERVATION', 
                            'CANCEL-RESERVATION']
            if command not in valid_commands:
                self.logger.debug('SUB-AREA-RESERVATION msg payload did not contain valid command')
            if command == 'RESERVATION-QUERY':
                task = dict_msg['payload'].get('task', None)
                self.osm_sub_area_monitor.get_sub_areas_for_task(task)
            elif command == 'CONFIRM-RESERVATION':
                reservation_object = dict_msg['payload'].get('reservation_object', None)
                self.osm_sub_area_monitor.confirm_sub_area_reservation(reservation_object)
            elif command == 'EARLIEST-RESERVATION':
                sub_area_id = dict_msg['payload'].get('sub_area_id', None)
                duration = dict_msg['payload'].get('duration', None)
                self.osm_sub_area_monitor.get_earliest_reservation_slot(sub_area_id, duration)
            elif command == 'CANCEL-RESERVATION':
                reservation_id = dict_msg['payload'].get('reservation_id', None)
                self.osm_sub_area_monitor.cancel_sub_area_reservation(reservation_id)

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
