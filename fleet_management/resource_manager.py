import logging

import inflection
from fleet_management.resources.fleet.monitoring import FleetMonitor
from fleet_management.resources.infrastructure import add_elevator_manager


class ResourceManager(object):

    def __init__(self, ccu_store, api, resources=None, **kwargs):
        self.logger = logging.getLogger('fms.resources.manager')
        self.ccu_store = ccu_store
        self.api = api

        self.robots = list()
        self.elevators = list()
        self.scheduled_robot_tasks = dict()
        self.elevator_requests = dict()
        self.robot_statuses = dict()

        fleet_monitor_config = kwargs.get('fleet_monitor_config', None)
        self.fleet_monitor = FleetMonitor(ccu_store, self.api, **fleet_monitor_config)

        if resources:
            self.add_resources(resources)

        self.allocations = list()

        self.elevator_manager = kwargs.get('elevator_manager')

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
        for robot_id in fleet:
            self.logger.info("Adding %s to the fleet", robot_id)
            self.fleet_monitor.register_robot(robot_id)

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

    def get_robot_status(self, robot_id):
        return self.robot_statuses[robot_id]

