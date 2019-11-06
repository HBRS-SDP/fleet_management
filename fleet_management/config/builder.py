import logging
from datetime import datetime

from fmlib.api import API
from fmlib.config.builders import Store
from mrs.bidding.bidder import Bidder
from mrs.config.mrta import MRTAFactory
from mrs.scheduling.monitor import ScheduleMonitor
from mrs.timetable.timetable import Timetable
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
                      'fleet_monitor': FleetMonitor,
                      'resource_manager': ResourceManager,
                      'task_monitor': TaskMonitor,
                      'dispatcher': Dispatcher,
                      'task_manager': TaskManager
                      }

_config_order = ['api', 'ccu_store',
                 'elevator_manager', 'fleet_monitor', 'resource_manager',
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


class RobotProxyBuilder:

    def __init__(self, robot_id, config_params):
        self.logger = logging.getLogger('fms.config.robot')

        self.robot_id = robot_id
        self.robot_config = config_params.get('robot_proxy')
        self.allocation_method = config_params.get('allocation_method')
        self.stp_solver = self.get_stp_solver()
        self._components = dict()

        self.register_component('bidder', Bidder)
        self.register_component('schedule_monitor', ScheduleMonitor)

    def register_component(self, component_name, component):
        self._components[component_name] = component

    def get_stp_solver(self):
        mrta_factory = MRTAFactory(self.allocation_method)
        stp_solver = mrta_factory.get_stp_solver()
        return stp_solver

    @property
    def api(self):
        if self._components.get('api') is None:
            self.logger.debug("Creating api")
            api_config = self.robot_config.pop('api')
            api_config['zyre']['zyre_node']['node_name'] = self.robot_id
            self.register_component('api', API(**api_config))
        return self._components.get('api')

    @property
    def robot_store(self):
        if self._components.get('robot_store') is None:
            self.logger.debug("Creating robot_store")
            robot_store_config = self.robot_config.pop('robot_store')
            robot_store_config['db_name'] = robot_store_config['db_name'] + '_' + self.robot_id.split('_')[1]
            self.register_component('robot_store', Store(**robot_store_config))
        return self._components.get('robot_store')

    @property
    def timetable(self):
        if self._components.get('timetable') is None:
            self.logger.debug("Creating timetable")
            timetable = Timetable(self.robot_id, self.stp_solver)
            timetable.fetch()
            today_midnight = datetime.today().replace(hour=0, minute=0, second=0, microsecond=0)
            timetable.zero_timepoint = TimeStamp()
            timetable.zero_timepoint.timestamp = today_midnight
            self.register_component('timetable', timetable)
        return self._components.get('timetable')

    def __call__(self):
        components = dict()
        components['api'] = self.api
        components['robot_store'] = self.robot_store
        components['timetable'] = self.timetable

        for component_name, configuration in self.robot_config.items():
            self.logger.debug("Creating %s", component_name)
            component = self._components.get(component_name)
            if component:
                _instance = component(allocation_method=self.allocation_method,
                                      robot_id=self.robot_id,
                                      stp_solver=self.stp_solver,
                                      api=self.api,
                                      robot_store=self.robot_store,
                                      timetable=self.timetable,
                                      **configuration)

                components[component_name] = _instance

        return components


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
            builder = builder(self.allocation_method)

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
plugin_factory.register_builder('mrta', MRTAFactory)
