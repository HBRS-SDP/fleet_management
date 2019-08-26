import logging
from fleet_management.plugins import osm


class PluginBuilder:

    def __init__(self):
        self._builders = {}
        self.logger = logging.getLogger('fms.config.config')

    def register_builder(self, plugin, builder):
        self.logger.debug("Adding builder for %s", plugin)
        self._builders[plugin] = builder

    def configure(self, key, **kwargs):
        self.logger.debug("Configuring %s", key)
        builder = self._builders.get(key)
        if not builder:
            raise ValueError(key)
        return builder(**kwargs)


plugin_factory = PluginBuilder()
plugin_factory.register_builder('osm', osm.configure)
