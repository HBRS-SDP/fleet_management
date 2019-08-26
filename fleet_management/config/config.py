import logging

from fleet_management.db.ccu_store import CCUStore
from fleet_management.plugins import osm

from fleet_management.api import API
from fleet_management.plugins.task_planner import TaskPlannerInterface
from fleet_management.resources.infrastructure import add_elevator_manager

_component_modules = {'api': API,
                      'ccu_store': CCUStore}
                      'elevator_manager': add_elevator_manager,
                      'fleet_monitor': FleetMonitor,
                      'resource_manager': ResourceManager
                      }


class FMSBuilder:
    def __init__(self):
        self.logger = logging.getLogger('fms.config.components')
        self._components = {}

    def configure_component(self, key, **kwargs):
        self.logger.debug("Configuring %s", key)
        component = _component_modules.get(key)
        if not component:
            raise ValueError(key)
        return component(**kwargs)

    def configure(self, config):
        for k, v in _component_modules.items():
            component_config = config.get(k, dict())
            component = self.configure_component(k, **component_config, **self._components)
            self._components[k] = component

        return self._components

    def get_component(self, component):
        return self._components.get(component)


class RobotProxyBuilder:

    def configure(self, robot_id, **kwargs):
        pass


class PluginBuilder:

    def __init__(self):
        self._builders = {}
        self.logger = logging.getLogger('fms.config.plugins')

    def register_builder(self, plugin, builder):
        self.logger.debug("Adding builder for %s", plugin)
        self._builders[plugin] = builder

    def configure(self, key, **kwargs):
        self.logger.debug("Configuring %s", key)
        builder = self._builders.get(key)
        if not builder:
            raise ValueError(key)
        return builder(**kwargs)


configure = FMSBuilder()

plugin_factory = PluginBuilder()
plugin_factory.register_builder('osm', osm.configure)
plugin_factory.register_builder('task_planner', TaskPlannerInterface)
