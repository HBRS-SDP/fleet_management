import logging

from ropod.structs.elevator import Elevator


class ElevatorMonitor:

    def __init__(self, elevator_id, ccu_store, api, **kwargs):
        self.logger = logging.getLogger(__name__)
        self.id = elevator_id
        self.ccu_store = ccu_store
        self.api = api
        self.elevator = Elevator(self.id)
        self.ccu_store.add_elevator(self.elevator)
        self.query_progress = dict()

        api_config = kwargs.get('api_config', None)
        if api_config:
            self.__configure_api(api_config)

    def elevator_status_cb(self, msg):
        payload = msg.get('payload')
        query_id = payload.get('queryId')
        self.elevator.update(payload)
        if self.at_start_floor():
            self.logger.info('Elevator reached start floor; waiting for confirmation...')
        elif self.at_goal_floor():
            self.logger.info('Elevator reached goal floor; waiting for confirmation...')
        self.ccu_store.update_elevator(self.elevator)

    def __configure_api(self, api_config):
        self.api.register_callbacks(self, api_config)

    def at_start_floor(self):
        return self.elevator.at_start_floor()

    def at_goal_floor(self):
        return self.elevator.at_goal_floor()


if __name__ == '__main__':
    from fleet_management.config.loader import Configurator
    from fleet_management.api.zyre import FMSZyreAPI
    import time

    config = Configurator()
    ccu_store = config.ccu_store
    zyre_config = {'node_name': 'monitor_test',
                   'groups': ['ELEVATOR-CONTROL'],
                   'message_types': ['ELEVATOR-STATUS']
                   }
    api = FMSZyreAPI(zyre_config, 'fms.test.monitor')
    test = ElevatorMonitor(1, ccu_store, api)
    test.api.register_callback(test.elevator_status_cb, 'ELEVATOR-STATUS')

    try:
        test.api.start()
        while True:
            time.sleep(5)
    except (KeyboardInterrupt, SystemExit):
        test.api.shutdown()
