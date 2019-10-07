from fleet_management.exceptions.config import InvalidConfig
from fleet_management.plugins.osm import path_planner, bridge, subarea_monitor


class OSMBuilder:
    def __init__(self):
        self._osm_bridge = None
        self._path_planner = None
        self._subarea_monitor = None

    def __call__(self, **kwargs):
        plugins = dict()
        for plugin, config in kwargs.items():
            if plugin == 'osm_bridge':
                self.osm_bridge(**kwargs)
                plugins.update(osm_bridge=self._osm_bridge)
            if plugin == 'path_planner':
                self.path_planner(**kwargs)
                plugins.update(path_planner=self._path_planner)
            if plugin == 'subarea_monitor':
                self.subarea_monitor(**kwargs)
                plugins.update(subarea_monitor=self._subarea_monitor)

        return plugins

    def path_planner(self, **kwargs):
        osm_bridge = self.osm_bridge(**kwargs)
        if not self._path_planner:
            planner_config = kwargs.get('path_planner')
            self._path_planner = path_planner.configure(osm_bridge=osm_bridge, **planner_config)
        return self._path_planner

    def subarea_monitor(self, **kwargs):
        osm_bridge = self.osm_bridge(**kwargs)
        if not self._subarea_monitor:
            subarea_monitor_config = kwargs.get('subarea_monitor')
            self._subarea_monitor = subarea_monitor.configure(
                                        osm_bridge=osm_bridge,
                                        building=kwargs['path_planner']['building'])
        return self._subarea_monitor

    def osm_bridge(self, **kwargs):
        if not self._osm_bridge:
            bridge_config = kwargs.get('osm_bridge')
            try:
                self._osm_bridge = bridge.configure(**bridge_config)
            except InvalidConfig:
                raise InvalidConfig('OSM plugins require an osm_bridge configuration')
        return self._osm_bridge
