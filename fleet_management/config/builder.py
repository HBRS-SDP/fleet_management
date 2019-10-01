import logging

from fmlib.api import API
from mrs.config.mrta import MRTAFactory
from fleet_management.plugins import osm
from fleet_management.plugins.planning import TaskPlannerInterface
from fleet_management.resources.fleet.monitoring import FleetMonitor
from fleet_management.resources.infrastructure import add_elevator_manager
from fleet_management.resources.manager import ResourceManager
from fleet_management.task.dispatcher import Dispatcher
from fleet_management.task.manager import TaskManager
from fleet_management.task.monitor import TaskMonitor
from fmlib.config.builders import Store

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

    _component_modules = {'api': API, 'robot_store': Store}
    _config_order = ['api', 'robot_store']

    @staticmethod
    def get_robot_config(robot_id, config_params):
        robot_config = config_params.get('robot_proxy')

        api_config = robot_config.get('api')
        api_config['zyre']['zyre_node']['node_name'] = robot_id
        robot_config.update({'api': api_config})

        db_config = robot_config.get('robot_store')
        db_config['db_name'] = db_config['db_name'] + '_' + robot_id.split('_')[1]
        robot_config.update({'robot_store': db_config})

        for component_name, config in robot_config.items():
            config.update({'robot_id': robot_id})

        return robot_config

    def __call__(self, robot_id, config_params):

        robot_config = self.get_robot_config(robot_id, config_params)

        fms_builder = FMSBuilder(component_modules=self._component_modules,
                                 config_order=self._config_order)
        fms_builder.configure(robot_config)

        api = fms_builder.get_component('api')
        robot_store = fms_builder.get_component('robot_store')

        robot_config.pop('api')
        robot_config.pop('robot_store')

        allocation_method = config_params.get('allocation_method')
        mrta_factory = MRTAFactory(allocation_method)
        components = mrta_factory(**robot_config)
        components.update({'api': api})

        for component_name, component in components.items():
            if hasattr(component, 'configure'):
                component.configure(api=api, robot_store=robot_store)

        return components


robot_builder = RobotProxyBuilder()


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
