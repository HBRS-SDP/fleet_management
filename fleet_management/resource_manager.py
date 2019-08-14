import logging

import inflection
from ropod.structs.area import Area, SubArea
from ropod.structs.robot import Robot
from ropod.structs.status import RobotStatus


class ResourceManager(object):

    def __init__(self, resources, ccu_store, api_config, **kwargs):
        self.logger = logging.getLogger('fms.resources.manager')
        self.ccu_store = ccu_store
        self.api = api_config

        self.robots = list()
        self.elevators = list()
        self.scheduled_robot_tasks = dict()
        self.elevator_requests = dict()
        self.robot_statuses = dict()

        self.add_resources(resources)
        self.allocations = list()

        self.elevator_manager = None

        plugins = kwargs.get('plugins', list())
        for plugin in plugins:
            self.add_plugin(plugin.__class__.__name__, plugin)

        self.logger.info("Resource Manager initialized...")

    def add_plugin(self, name, obj):
        key = inflection.underscore(name)
        self.__dict__[key] = obj
        self.logger.debug("Added %s plugin to %s", name, self.__class__.__name__)

    def add_resource(self, resource, category):
        pass

    def add_resources(self, resources):
        self.logger.info("Adding resources...")
        fleet = resources.get('fleet')
        # NOTE This is being used temporarily for testing purposes,
        # TODO needs to be done with empty values
        # and the information should be updated when we receive an update from the robot
        for robot_id in fleet:
            self.logger.info("Adding %s to the fleet", robot_id)
            area = Area()
            area.id = 'AMK_D_L-1_C39'
            area.name = 'AMK_D_L-1_C39'
            area.floor_number = -1
            area.type = ''
            area.sub_areas = list()

            subarea = SubArea()
            subarea.name = 'AMK_D_L-1_C39_LA1'
            area.sub_areas.append(subarea)

            ropod = Robot(robot_id)
            status = RobotStatus()
            status.robot_id = robot_id
            status.current_location = area
            status.current_operation = 'unknown'
            status.status = 'idle'
            status.available = 'unknown'
            status.battery_status = 'unknown'

            ropod.schedule = None
            ropod.status = status

            self.robots.append(ropod.to_dict())
            self.ccu_store.add_robot(ropod)

        infrastructure = resources.get('infrastructure')

        elevators = infrastructure.get('elevators')


        # TODO once resource manager is configured, this can be uncommented
        # load task realated sub areas from OSM world model
        # if self.osm_bridge is not None:
        #     self.load_sub_areas_from_osm()
        # else:
        #     self.logger.error("Loading sub areas from OSM world model cancelled due to problem in intialising OSM bridge")

    def restore_data(self):
        # TODO This needs to be updated to match the new config format
        self.elevators = self.ccu_store.get_elevators()
        self.robots = self.ccu_store.get_robots()

    def get_robots_for_task(self, tasks):
        ''' Adds a task or list of tasks to the list of tasks_to_allocate in the auctioneer
        '''
        self.auctioneer.allocate(tasks)

    def get_allocation(self):
        ''' Gets the allocation of a task when the auctioneer terminates an allocation round
        '''

        while self.auctioneer.allocations:
            allocation = self.auctioneer.allocations.pop()
            self.logger.debug("Allocation %s: ", allocation)
            self.allocations.append(allocation)

    ''' Returns a dictionary with the start and finish time of the task_id assigned to the robot_id
    '''
    def get_task_schedule(self, task_id, robot_id):
        task_schedule = self.auctioneer.get_task_schedule(
            task_id, robot_id)
        return task_schedule

    def robot_update_cb(self, msg):
        new_robot_status = RobotStatus.from_dict(msg['payload'])
        self.ccu_store.update_robot(new_robot_status)

    def subarea_reservation_cb(self, msg):
        #TODO: This whole block is just a skeleton and should be reimplemented according to need.
        if 'payload' not in msg:
            self.logger.debug('SUB-AREA-RESERVATION msg did not contain payload')

        command = msg['payload'].get('command', None)
        valid_commands = ['RESERVATION-QUERY',
                        'CONFIRM-RESERVATION',
                        'EARLIEST-RESERVATION',
                        'CANCEL-RESERVATION']
        if command not in valid_commands:
            self.logger.debug('SUB-AREA-RESERVATION msg payload did not contain valid command')
        if command == 'RESERVATION-QUERY':
            task = msg['payload'].get('task', None)
            self.osm_sub_area_monitor.get_sub_areas_for_task(task)
        elif command == 'CONFIRM-RESERVATION':
            reservation_object = msg['payload'].get('reservation_object', None)
            self.osm_sub_area_monitor.confirm_sub_area_reservation(reservation_object)
        elif command == 'EARLIEST-RESERVATION':
            sub_area_id = msg['payload'].get('sub_area_id', None)
            duration = msg['payload'].get('duration', None)
            self.osm_sub_area_monitor.get_earliest_reservation_slot(sub_area_id, duration)
        elif command == 'CANCEL-RESERVATION':
            reservation_id = msg['payload'].get('reservation_id', None)
            self.osm_sub_area_monitor.cancel_sub_area_reservation(reservation_id)

    def get_robot_status(self, robot_id):
        return self.robot_statuses[robot_id]

