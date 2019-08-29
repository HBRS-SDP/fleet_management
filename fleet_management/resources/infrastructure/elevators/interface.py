import logging

from fleet_management.resources.infrastructure.elevators.monitor import ElevatorMonitor
from ropod.structs.elevator import ElevatorRequest, RobotCallUpdate


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
            request.status = ElevatorRequest.ACCEPTED
            self.logger.debug("Set request status as going to start")
        elif command == 'CANCEL_ELEVATOR':
            request = self.pending_requests.pop(query_id)
            request.status = ElevatorRequest.CANCELED
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
