from fleet_management.exceptions.config import InvalidConfig
from fleet_management.plugins.osm import path_planner, bridge


def _get_osm_bridge(config):
    if not config:
        raise InvalidConfig
    else:
        return bridge.configure(**config)


class OSMBuilder:
    def __init__(self):
        self._osm_bridge = None
        self._path_planner = None
        self._subarea_monitor = None

    def __call__(self, **kwargs):
        if not self._osm_bridge:
            bridge_config = kwargs.get('osm_bridge')
            try:
                self._osm_bridge = _get_osm_bridge(bridge_config)
            except InvalidConfig:
                if 'path_planner' in kwargs.keys():
                    raise InvalidConfig('OSM plugins require an osm_bridge configuration')

        if not self._path_planner:
            planner_config = kwargs.get('path_planner')
            self._path_planner = self._get_path_planner(planner_config)

        if not self._subarea_monitor:
            subarea_monitor_config = kwargs.get('subarea_monitor')
            self._get_subarea_monitor(subarea_monitor_config)

        return self._osm_bridge, self._path_planner, self._subarea_monitor

    def _get_path_planner(self, config=None):
        if config:
            return path_planner.configure(osm_bridge=self._osm_bridge, **config)
        else:
            return None

    def _get_subarea_monitor(self, config=None):
        self._subarea_monitor = None
