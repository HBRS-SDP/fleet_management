import logging

from fleet_management.resources.infrastructure.elevators.monitor import ElevatorMonitor
from ropod.structs.elevator import ElevatorRequest, RobotCallUpdate, RobotElevatorCallReply, ElevatorRequestStatus


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
        return self.elevators.get(1).get('interface')

    def elevator_call_request_cb(self, msg):
        payload = msg.get('payload')
        command = payload.get('command')
        query_id = payload.get('queryId')

        robot_request = ElevatorRequest.from_dict(payload)
        elevator = self.select_elevator(robot_request)
        robot_request.elevator_id = elevator.id
        self.ongoing_queries[query_id] = {'request': robot_request, 'elevator': elevator}

        if command == 'CALL_ELEVATOR':
            self.logger.info('Received elevator request from ropod')

            self.ccu_store.add_elevator_call(robot_request)
            elevator.request_elevator(robot_request)
        elif command == 'CANCEL_CALL':
            elevator.cancel_elevator_call(robot_request)

    def robot_call_update_cb(self, msg):
        command = msg['payload']['command']
        query_id = msg['payload']['queryId']
        query = self.ongoing_queries.get(query_id)

        request = query.get('request')
        elevator = query.get('elevator')

        if command == 'ROBOT_FINISHED_ENTERING':
            # Close the doors
            self.logger.info('Received entering confirmation from ropod')
            request.status = ElevatorRequestStatus.GOING_TO_GOAL
        elif command == 'ROBOT_FINISHED_EXITING':
            # Close the doors
            self.logger.info('Received exiting confirmation from ropod')
            # Remove the request from the ongoing queries
            # TODO Archive the request in ccu_store
            self.ongoing_queries.pop(query_id)
            request.status = ElevatorRequestStatus.COMPLETED

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
                self.confirm_elevator(query_id, elevator.id)
                request.status = ElevatorRequestStatus.GOING_TO_START

    def configure_api(self, api_config):
        if self.api:
            self.api.register_callbacks(self, api_config)

    @classmethod
    def from_config(cls, ccu_store=None, api=None, **kwargs):
        return cls(ccu_store, api, **kwargs)


class ElevatorControlInterface:

    def __init__(self, elevator_id, ccu_store, api, **kwargs):
        self.logger = logging.getLogger(self.__module__ + __name__)
        self.id = elevator_id
        self.ccu_store = ccu_store
        self.api = api
        self.pending_requests = dict()

    def elevator_cmd_reply_cb(self, msg):
        payload = msg.get('payload')

        query_id = payload.get('queryId')
        query_success = payload.get('querySuccess')

        if not query_success:
            self.logger.warning("Query %s failed", query_id)
            return

        command = payload.get('command')
        self.logger.info('Received reply from elevator control for %s query', query_id)

        if command == 'CALL_ELEVATOR':
            request = self.pending_requests.pop(query_id)
            request.status = ElevatorRequestStatus.ACCEPTED
            self.logger.debug("Set request status as going to start")
        elif command == 'CANCEL_ELEVATOR':
            request = self.pending_requests.pop(query_id)
            request.status = ElevatorRequestStatus.CANCELED
            self.logger.debug("Set request status as canceled")

    def request_elevator(self, elevator_request):
        self.pending_requests[elevator_request.query_id] = elevator_request
        msg = self.api.create_message(elevator_request)
        self.api.publish(msg, groups=['ELEVATOR-CONTROL'])
        self.logger.info("Requested elevator...")

    def cancel_elevator_call(self, elevator_request):
        # TODO To cancel a call, the call ID should be sufficient:
        # read from ccu store, get info to cancel
        self.pending_requests[elevator_request.query_id] = elevator_request
        msg = self.api.create_message(elevator_request)
        self.api.publish(msg, groups=['ELEVATOR-CONTROL'])

    def confirm_robot_action(self, robot_action, query_id):
        if robot_action == 'ROBOT_FINISHED_ENTERING':
            # TODO Remove this hardcoded floor
            update = RobotCallUpdate(query_id,
                                     'CLOSE_DOORS_AFTER_ENTERING', start_floor=1)
        elif robot_action == 'ROBOT_FINISHED_EXITING':
            # TODO Remove this hardcoded floor
            update = RobotCallUpdate(query_id,
                                     'CLOSE_DOORS_AFTER_EXITING', goal_floor=1)

        msg = self.api.create_message(update)

        # TODO This doesn't match the convention
        msg['header']['type'] = 'ELEVATOR-CMD'
        self.api.publish(msg, groups=['ELEVATOR-CONTROL'])
        self.logger.debug('Sent robot confirmation to elevator')

    def configure_api(self, api_config):
        self.api.register_callbacks(self, api_config)


class ElevatorManagerBuilder:
    def __init__(self):
        self._elevator_mgr = None

    def __call__(self, **kwargs):
        if not self._elevator_mgr:
            self._elevator_mgr = ElevatorManager(**kwargs)
            api_config = kwargs.get('api_config')
            self._elevator_mgr.configure_api(api_config)

        return self._elevator_mgr


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
        return {'interface': elevator_interface,
                'monitor': elevator_monitor}
