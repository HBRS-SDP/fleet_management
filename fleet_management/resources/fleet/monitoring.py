import logging

from ropod.structs.status import RobotStatus


class FleetMonitor:
    def __init__(self, ccu_store, api, **kwargs):
        self.logger = logging.getLogger('fms.resources.fleet.monitoring')
        self.ccu_store = ccu_store
        self.api = api
        self.robots = list()

        robot_config = kwargs.get('robots', None)
        if robot_config:
            for robot_type in robot_config:
                print(robot_type)

        api_config = kwargs.get('api', None)
        if api_config:
            self.__configure_api(api_config)

        robots = kwargs.get('robots', list())
        for robot_id in robots:
            self.register_robot(robot_id)
        self.logger.debug("Initialized Fleet Monitor")

    def register_robot(self, robot_id):
        self.robots.append(robot_id)

    def robot_status_cb(self, msg):
        new_robot_status = RobotStatus.from_dict(msg['payload'])
        self.__update_status(new_robot_status)
        self.logger.debug('%s status change: %s', msg.get('payload'))

    def __update_status(self, status):
        self.ccu_store.update_robot(status)

    def __configure_api(self, api_config):
        self.api.register_callbacks(self, api_config)


if __name__ == '__main__':
    test = FleetMonitor({'robots': [{'type': 'ropod', 'class': 'ropod.structs.robot'}]}, None)
