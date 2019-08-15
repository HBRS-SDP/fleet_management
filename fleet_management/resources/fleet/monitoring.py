import logging

from fleet_management.exceptions.config import InvalidConfig
from ropod.structs.status import RobotStatus


class FleetMonitor:
    def __init__(self, config, ccu_store, api, **kwargs):
        self.logger = logging.getLogger('fms.resources.fleet.monitoring')
        self.ccu_store = ccu_store
        self.api = api
        self.robots = dict()
        self.robot_config = dict()
        if config:
            robot_config = config.get('robots', None)
            if not robot_config:
                raise InvalidConfig
            else:
                self.api_monitor_config = config.get('api')
                for robot_type in robot_config:
                    print(robot_type)

        robots = kwargs.get('robots', list())
        for robot_id in robots:
            self.register_robot(robot_id)
        self.logger.debug("Initialized Fleet Monitor")

    def register_robot(self, robot_id):
        monitor = RopodMonitor(robot_id, self.ccu_store, self.api, api_config=self.api_monitor_config)
        self.robots[robot_id] = monitor


class RopodMonitor:
    def __init__(self, robot_id, ccu_store, api, **kwargs):
        self.logger = logging.getLogger('fms.resources.monitoring.robot')
        self.robot = robot_id
        self.ccu_store = ccu_store

        self.api = api
        api_config = kwargs.get('api_config', None)
        if api_config:
            self.__configure_api(api_config)

        self._status = None

    def robot_update_cb(self, msg):
        new_robot_status = RobotStatus.from_dict(msg['payload'])

        if new_robot_status.robot_id == self.robot:
            self.__update_status(new_robot_status)

    def __update_status(self, status):
        self.ccu_store.update_robot(status)
        self._status = status

    @property
    def status(self):
        return self._status

    @status.setter
    def status(self, new_status):
        self.__update_status(new_status)

    def __configure_api(self, api_config):
        self.api.register_callbacks(self, api_config)


if __name__ == '__main__':
    test = FleetMonitor({'robots': [{'type': 'ropod', 'class': 'ropod.structs.robot'}]}, None)
