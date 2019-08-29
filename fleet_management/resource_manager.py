import logging

import inflection


class ResourceManager(object):

    def __init__(self, ccu_store, api, **kwargs):
        self.logger = logging.getLogger('fms.resources.manager')
        self.ccu_store = ccu_store
        self.api = api

        self.scheduled_robot_tasks = dict()

        self.fleet_monitor = kwargs.get('fleet_monitor')

        self.allocations = list()

        self.elevator_manager = kwargs.get('elevator_manager')

        self.logger.info("Resource Manager initialized...")

    def add_plugin(self, obj, name=None):
        if name:
            key = inflection.underscore(name)
        else:
            key = inflection.underscore(obj.__class__.__name__)
        self.__dict__[key] = obj
        self.logger.debug("Added %s plugin to %s", key, self.__class__.__name__)

    def configure(self, **kwargs):
        if kwargs.get('resources'):
            self.add_resources(kwargs.get('resources'))

    def add_resource(self, resource, category):
        pass

    def add_resources(self, resources):
        self.logger.info("Adding resources...")
        fleet = resources.get('fleet')
        if self.fleet_monitor:
            self.register_fleet(fleet, self.fleet_monitor)

        if self.auctioneer:
            self.register_fleet(fleet, self.auctioneer)

        if self.elevator_manager:
            elevators = resources.get('infrastructure', list()).get('elevators', list())
            for elevator_id in elevators:
                self.logger.info("Adding %s to the elevator manager", elevator_id)
                self.elevator_manager.add_elevator(elevator_id)

    def register_fleet(self, fleet, component):
        for robot_id in fleet:
            self.logger.info("Adding %s to the fleet", robot_id)
            component.register_robot(robot_id)

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

