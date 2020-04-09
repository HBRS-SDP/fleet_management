import logging

from fmlib.config.params import ConfigParams as ConfigParamsBase
from ropod.utils.logging.config import config_logger

from fleet_management.config.builder import FMSBuilder
from fleet_management.config.builder import plugin_factory
from fleet_management.config.builder import robot_builder


class ConfigParams(ConfigParamsBase):
    default_config_module = 'fleet_management.config.default'


default_config = ConfigParams.default()
default_logging_config = default_config.get('logger')


class Configurator(object):

    def __init__(self, config_file=None, logger=True, **kwargs):
        self.logger = logging.getLogger('fms.config.configurator')
        self._builder = FMSBuilder(**kwargs)
        self._plugin_factory = plugin_factory

        self._components = dict()
        self._plugins = dict()

        if config_file is None:
            self._config_params = default_config
        else:
            self._config_params = ConfigParams.from_file(config_file)

        if logger:
            log_file = kwargs.get('log_file', None)
            self.configure_logger(filename=log_file)

        self._plugin_factory.allocation_method = self._config_params.get('allocation_method')

    def configure(self):
        components = self._builder.configure(self._config_params)
        self._components.update(**components)
        plugins = self._configure_plugins(ccu_store=self._components.get('ccu_store'),
                                          api=self._components.get('api'))

        self._plugins.update(**plugins)
        for name, component in components.items():
            self.add_plugins(name)
            self.configure_components(name)

    def add_plugins(self, component_name):
        """Adds all the plugins specified in the config file to a component

        Args:
            component_name (str): The name of the component
        """
        component = self._components.get(component_name)
        component_config = self._config_params.get(component_name)
        self.logger.debug('Adding plugins to %s', component_name)
        if hasattr(component, 'add_plugin'):
            plugins = component_config.get('plugins', list())
            for plugin in plugins:
                self.add_plugin(component, plugin, attr_name=plugin)

    def add_plugin(self, component, plugin_name, attr_name=None):
        """ Adds one plugin to a component

        Args:
            component (object): The object instance to which the component should be added
            plugin_name (str): The plugin to add (should already be registered in
                                the plugins dictionary)
            attr_name (str): The name of the attribute of the plugin in the desired component
        """
        plugin = self._plugins.get(plugin_name)
        component.add_plugin(plugin, attr_name)

    def configure_components(self, component_name):
        """Use the configure interface of a component, if it has one

        Args:
            component_name (str): The name of the component
        """

        component = self._components.get(component_name)
        component_config = self._config_params.get(component_name)

        if hasattr(component, 'configure'):
            self.logger.debug('Configuring %s', component_name)
            component.configure(**component_config, api=self.api, ccu_store=self.ccu_store)

        for sub_component_name, sub_component in component.__dict__.items():
            if hasattr(sub_component, 'configure'):
                self.logger.debug('Configuring %s', sub_component_name)
                sub_component.configure(api=self.api, ccu_store=self.ccu_store)

    def __str__(self):
        return str(self._config_params)

    def configure_logger(self, logger_config=None, filename=None):
        if logger_config is not None:
            logging.info("Loading logger configuration from file: %s ", logger_config)
            config_logger(logger_config, filename=filename)
        elif 'logger' in self._config_params:
            logging.info("Using FMS logger configuration")
            fms_logger_config = self._config_params.get('logger', None)
            logging.config.dictConfig(fms_logger_config)
        else:
            logging.info("Using default ropod config...")
            config_logger(filename=filename)

    @property
    def api(self):
        return self.get_component('api')

    @property
    def ccu_store(self):
        return self.get_component('ccu_store')

    @property
    def task_manager(self):
        return self.get_component('task_manager')

    @property
    def resource_manager(self):
        return self.get_component('resource_manager')

    def get_component(self, component):
        if component in self._components.keys():
            return self._components.get(component)
        else:
            return self._create_component(component)

    def _create_component(self, name):
        component_config = self._config_params.get(name)
        component_ = self._builder.get_component(name, **component_config)
        self._components[name] = component_
        return component_

    def _configure_plugins(self, ccu_store, api):
        logging.info("Configuring FMS plugins...")
        plugin_config = self._config_params.get('plugins')
        if plugin_config is None:
            self.logger.debug("Found no plugins in the configuration file.")
            return None

        for plugin, config in plugin_config.items():
            try:
                component = self._plugin_factory.configure(plugin, ccu_store=ccu_store, api=api,
                                                           dispatcher=self.get_component('dispatcher'), **config)
            except ValueError:
                self.logger.error("No builder registered for %s", plugin)
                continue

            if isinstance(component, dict):
                self._plugins.update(**component)
            else:
                self._plugins[plugin] = component

        return self._plugins

    def configure_robot_proxy(self, robot_id):
        robot_components = robot_builder(robot_id, self._config_params)
        return robot_components


