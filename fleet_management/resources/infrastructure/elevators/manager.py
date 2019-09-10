import logging

from fleet_management.db.models.ropod.elevator import RobotRequest
from fleet_management.resources.infrastructure.elevators.interface import ElevatorControlInterface
from fleet_management.resources.infrastructure.elevators.monitor import ElevatorMonitor
from ropod.structs.elevator import RobotElevatorCallReply, ElevatorRequestStatus


class ElevatorManager:

    def __init__(self, ccu_store, api, **kwargs):
        self.logger = logging.getLogger('fms.resources.elevator_manager')
        self.ccu_store = ccu_store
        self.api = api
        self.elevators = dict()
        self.pending_requests = list()
        self.ongoing_queries = dict()

        monitoring_config = kwargs.get('monitors')
        interface_config = kwargs.get('interfaces')
        self._elevator_builder = ElevatorBuilder(self.ccu_store, self.api,
                                                 monitoring_config=monitoring_config,
                                                 interface_config=interface_config)

    def add_elevator(self, elevator_id):
        self.elevators[elevator_id] = self._elevator_builder(elevator_id)

    def select_elevator(self, request):
        """Chooses the best elevator for a given call request.
        The current implementation simply returns the default elevator,
        this method will eventually be updated once additional
        elevators are automated.

        Returns:
            elevator: The elevator to be used
        """
        elevator = self.elevators.get(1)
        request.assign_elevator(elevator.elevator_id)

        return elevator

    def elevator_call_request_cb(self, msg):
        payload = msg.get('payload')
        command = payload.get('command')
        query_id = payload.get('queryId')

        robot_request = self.create_request(msg)
        elevator = self.select_elevator(robot_request)
        self.ongoing_queries[query_id] = {'request': robot_request, 'elevator': elevator}

        if command == 'CALL_ELEVATOR':
            self.logger.info('Received elevator request from ropod')

            # self.ccu_store.add_elevator_call(robot_request)
            elevator.request_elevator(robot_request)
        elif command == 'CANCEL_CALL':
            elevator.cancel_elevator_call(robot_request)

    def create_request(self, msg):
        payload = msg.get('payload')
        query_id = payload.get('queryId')

        robot_request = RobotRequest.from_msg(payload)
        return robot_request

    def robot_call_update_cb(self, msg):
        command = msg['payload']['command']
        query_id = msg['payload']['queryId']
        query = self.ongoing_queries.get(query_id)

        request = query.get('request')
        elevator = query.get('elevator')

        if command == 'ROBOT_FINISHED_ENTERING':
            # Close the doors
            self.logger.info('Received entering confirmation from ropod')
            request.update_status(ElevatorRequestStatus.GOING_TO_GOAL)
        elif command == 'ROBOT_FINISHED_EXITING':
            # Close the doors
            self.logger.info('Received exiting confirmation from ropod')
            # Remove the request from the ongoing queries
            self.ongoing_queries.pop(query_id)
            request.update_status(ElevatorRequestStatus.COMPLETED)

        elevator.confirm_robot_action(command, query_id)

    def confirm_elevator(self, query_id, elevator_id):
        # TODO This is using the default elevator
        reply = RobotElevatorCallReply(query_id, elevator_id=elevator_id)
        msg = self.api.create_message(reply)
        self.api.publish(msg, groups=['ROPOD'])
        self.logger.debug('Sent elevator confirmation to robot')

    def run(self):
        # Process queries
        for query_id, query in self.ongoing_queries.items():
            request = query.get('request')
            if request.status == ElevatorRequestStatus.ACCEPTED:
                elevator = query.get('elevator')
                self.confirm_elevator(query_id, elevator.elevator_id)
                request.update_status(ElevatorRequestStatus.GOING_TO_START)

    def configure_api(self, api_config):
        if self.api:
            self.api.register_callbacks(self, api_config)

    @classmethod
    def from_config(cls, ccu_store=None, api=None, **kwargs):
        return cls(ccu_store, api, **kwargs)


class ElevatorManagerBuilder:
    def __init__(self):
        self._elevator_mgr = None

    def __call__(self, **kwargs):
        if not self._elevator_mgr:
            self._elevator_mgr = ElevatorManager(**kwargs)
            api_config = kwargs.get('api_config')
            self._elevator_mgr.configure_api(api_config)

        return self._elevator_mgr


class Elevator:

    def __init__(self, elevator_id, monitor, interface):
        self.logger = logging.getLogger(__name__)
        self.elevator_id = elevator_id
        self.monitor = monitor
        self.interface = interface

    def request_elevator(self, robot_request):
        if self.interface:
            self.interface.request_elevator(robot_request)
        else:
            self.logger.warning("No interface defined. Requesting elevators not possible")

    def cancel_elevator(self, robot_request):
        if self.interface:
            self.interface.cancel_elevator_call(robot_request)
        else:
            self.logger.warning("No interface defined. Cannot cancel elevator request")

    def confirm_robot_action(self, command, query_id):
        if self.interface:
            self.interface.confirm_robot_action(command, query_id)
        else:
            self.logger.warning("No interface defined. Cannot confirm robot action")


class ElevatorBuilder:
    def __init__(self, ccu_store, api, monitoring_config=None, interface_config=None):
        self._monitoring_config = monitoring_config
        self._interface_config = interface_config
        self._params = dict({'ccu_store': ccu_store,
                             'api': api})

    def __call__(self, elevator_id, **kwargs):
        elevator_interface = ElevatorControlInterface(elevator_id, **self._params)
        elevator_interface.configure_api(**self._interface_config)
        elevator_monitor = ElevatorMonitor(elevator_id, **self._params)
        elevator_monitor.configure_api(**self._monitoring_config)
        return Elevator(elevator_id, elevator_monitor, elevator_interface)
