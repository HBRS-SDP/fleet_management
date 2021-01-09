from fleet_management.exceptions.config import InvalidConfig
from fleet_management.plugins.topology import path_planner


class TopologyBuilder:
    def __init__(self):
        self._path_planner = None

    def __call__(self, **kwargs):
        plugins = dict()
        for plugin, config in kwargs.items():
            if plugin == "path_planner":
                self.path_planner(**kwargs)
                plugins.update(path_planner=self._path_planner)

        return plugins

    def path_planner(self, **kwargs):
        # osm_bridge = self.osm_bridge(**kwargs)
        if not self._path_planner:
            planner_config = kwargs.get("path_planner")
            self._path_planner = path_planner.configure(**planner_config)
        return self._path_planner
