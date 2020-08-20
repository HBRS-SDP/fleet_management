import logging
from datetime import datetime

from fleet_management.api.api import API
from fleet_management.resources.infrastructure.brsu import DurationGraph
from fmlib.config.builders import Store
from mrs.allocation.auctioneer import Auctioneer
from fleet_management.plugins.mrta.bidder import Bidder
from fleet_management.plugins.mrta.timetable_monitor import TimetableMonitor, TimetableMonitorProxy
from fleet_management.plugins.mrta.schedule_execution_monitor import ScheduleExecutionMonitor
from mrs.config.builder import MRTABuilder, DelayRecovery, PerformanceTracker, Scheduler
from mrs.timetable.timetable import Timetable, TimetableManager
from ropod.utils.timestamp import TimeStamp

from fleet_management.plugins import osm
from fleet_management.plugins.planning import TaskPlannerInterface
from fleet_management.resources.fleet.monitoring import FleetMonitor
from fleet_management.resources.infrastructure import add_elevator_manager
from fleet_management.resources.manager import ResourceManager
from fleet_management.task.dispatcher import Dispatcher
from fleet_management.task.manager import TaskManager
from fleet_management.task.monitor import TaskMonitor

_component_modules = {'api': API,
                      'ccu_store': Store,
                      'elevator_manager': add_elevator_manager,
                      'duration_graph': DurationGraph.load_graph,
                      'fleet_monitor': FleetMonitor,
                      'resource_manager': ResourceManager,
                      'task_monitor': TaskMonitor,
                      'dispatcher': Dispatcher,
                      'task_manager': TaskManager
                      }

_config_order = ['ccu_store', 'api',
                 'elevator_manager',
                 'duration_graph',
                 'fleet_monitor', 'resource_manager',
                 'task_monitor', 'dispatcher', 'task_manager'
                 ]


class FMSBuilder:
    def __init__(self, **kwargs):
        self.logger = logging.getLogger('fms.config.components')
        self._components = dict()
        self.component_modules = kwargs.get('component_modules', _component_modules)
        self.config_order = kwargs.get('config_order', _config_order)

    def configure_component(self, key, **kwargs):
        self.logger.debug("Configuring %s", key)
        component = self.component_modules.get(key)
        if not component:
            raise ValueError(key)
        return component(**kwargs)

    def configure(self, config):
        for c in self.config_order:
            component_config = config.get(c, dict())
            self.logger.debug("Creating %s with components %s", c, self._components)
            component = self.configure_component(c, **component_config, **self._components)
            self._components[c] = component

        return self._components

    def get_component(self, name, create=True, **kwargs):
        if self._components.get(name):
            return self._components.get(name)
        elif create:
            print("Creating stuff!")
            component_ = self.configure_component(name, **kwargs, **self._components)
            self._components[name] = component_
            return component_
        else:
            return None


class RobotBuilder:

    def __init__(self, proxy=True):
        self.logger = logging.getLogger('fms.config.robot')
        self._component_modules = dict()
        self.proxy = proxy

    def register_component_module(self, component_name, component):
        self._component_modules[component_name] = component

    def api(self, robot_id, api_config):
        self.logger.debug("Creating api of %s", robot_id)
        if self.proxy:
            api_config['zyre']['zyre_node']['node_name'] = robot_id + '_proxy'
        else:
            api_config['zyre']['zyre_node']['node_name'] = robot_id + '_'
        api = API(**api_config)
        return api

    def robot_store(self, robot_id, robot_store_config):
        self.logger.debug("Creating robot_store %s", robot_id)
        robot_store_config['db_name'] = robot_store_config['db_name'] + '_' + robot_id.split('_')[1]
        robot_store = Store(**robot_store_config)
        return robot_store

    def __call__(self, robot_id, allocation_method, config):
        self._factory = MRTABuilder(allocation_method, component_modules=self._component_modules)
        self._factory.register_component('api', self.api(robot_id, config.pop('api')))
        self._factory.register_component('robot_store', self.robot_store(robot_id, config.pop('robot_store')))
        self._factory.register_component('robot_id', robot_id)

        components = self._factory(**config)
        return components


robot_proxy_builder = RobotBuilder()
robot_proxy_builder.register_component_module('timetable', Timetable)
robot_proxy_builder.register_component_module('bidder', Bidder)
robot_proxy_builder.register_component_module('timetable_monitor', TimetableMonitorProxy)

robot_builder = RobotBuilder(proxy=False)
robot_builder.register_component_module('timetable', Timetable)
robot_builder.register_component_module('scheduler', Scheduler)
robot_builder.register_component_module('delay_recovery', DelayRecovery)
robot_builder.register_component_module('schedule_execution_monitor', ScheduleExecutionMonitor)


class PluginBuilder:

    def __init__(self, **kwargs):
        self._builders = {}
        self.logger = logging.getLogger('fms.config.plugins')
        self.allocation_method = kwargs.get('allocation_method')

    def register_builder(self, plugin, builder):
        self.logger.debug("Adding builder for %s", plugin)
        self._builders[plugin] = builder

    def configure(self, key, **kwargs):
        self.logger.debug("Configuring %s", key)
        builder = self._builders.get(key)
        if key == 'mrta':
            component_modules = {'timetable_manager': TimetableManager,
                                 'delay_recovery': DelayRecovery,
                                 'auctioneer': Auctioneer,
                                 'dispatcher': kwargs.get('dispatcher'),
                                 'timetable_monitor': TimetableMonitor,
                                 }

            builder = builder(self.allocation_method, component_modules=component_modules)

        if not builder:
            raise ValueError(key)
        return builder(**kwargs)

    def get_builder(self, key):
        builder = self._builders.get(key)
        if not builder:
            raise ValueError(key)
        return builder


configure = FMSBuilder()

plugin_factory = PluginBuilder()
plugin_factory.register_builder('osm', osm.configure)
plugin_factory.register_builder('task_planner', TaskPlannerInterface)
plugin_factory.register_builder('mrta', MRTABuilder)
