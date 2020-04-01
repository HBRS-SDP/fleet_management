import logging

from fmlib.db.mongo import MongoStore, MongoStoreInterface
from fmlib.models.robot import SoftwareComponent
from fleet_management.db.models.robot import Ropod as Robot, RopodSoftwareStack


class FleetMonitor:

    def __init__(self, ccu_store, api, **kwargs):
        self.logger = logging.getLogger('fms.resources.fleet.monitoring')
        self.ccu_store = ccu_store
        self.api = api
        self.robots = dict()

        robot_config = kwargs.get('robots', None)
        if robot_config:
            for robot_type in robot_config:
                print(robot_type)

        api_config = kwargs.get('api', None)
        if api_config:
            self.__configure_api(api_config)

        self.logger.debug("Initialized Fleet Monitor")

    def register_robot(self, robot_id):
        """Adds the robot to the list of robots it will track.
        This method also initializes all the required documents in MongoDB

        Args:
            robot_id: The ID of the robot to register

        """
        robot = Robot.create_new(robot_id)
        self.robots[robot_id] = robot

    def robot_pose_cb(self, msg):
        payload = msg.get('payload')
        robot_id = payload.get('robotId')
        robot = self.robots.get(robot_id)
        robot.update_position(subarea=payload.get('subarea'), **payload.get('pose'))

    def __configure_api(self, api_config):
        self.api.register_callbacks(self, api_config)

    def robot_version_cb(self, msg):
        payload = msg.get('payload')
        robot_id = payload.get('robotId')
        robot = self.robots.get(robot_id)

        software_ = self._process_wstool_msg(payload.get('softwareVersion'))

        robot.update_version(software=software_,
                             hardware=payload.get('hardwareVersion'))

    def _process_wstool_msg(self, software):
        sw_version = RopodSoftwareStack()
        for component in software:
            name = component.get('localname')
            config_version = component.get('version')
            actual_version = component.get('actualversion')
            config_mismatch = component.get('specversion') != actual_version

            update = component.get('remote_revision') != actual_version
            modified = component.get('modified')
            if not modified:
                modified = False

            # Create the component document
            component_ = SoftwareComponent(name=name,
                                           version=config_version,
                                           version_uid=actual_version,
                                           update_available=update,
                                           config_mismatch=config_mismatch,
                                           uncommitted_changes=modified)
            # Categorize the entries
            properties = component.get('properties')
            if properties:
                category = properties[0].get('meta', dict()).get('category', 'uncategorized')
            else:
                category = 'uncategorized'

            if category == 'navigation':
                sw_version.navigation_stack.append(component_)
            elif category == 'diagnosis':
                sw_version.diagnosis.append(component_)
            elif category == 'communication':
                sw_version.communication.append(component_)
            elif category == 'execution':
                sw_version.execution.append(component_)
            elif category == 'world model':
                sw_version.world_model.append(component_)
            elif category == 'interfaces':
                sw_version.interfaces.append(component_)
            else:
                sw_version.uncategorized.append(component_)

        return sw_version


if __name__ == '__main__':
    store = MongoStore('fms_test', connectTimeoutMS=1)
    interface = MongoStoreInterface(store)
    monitor = FleetMonitor(interface, None)
    monitor.register_robot("ropod_001")
