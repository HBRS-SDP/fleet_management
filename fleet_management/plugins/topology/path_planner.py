import logging

from ropod.structs.area import Area, SubArea

# from fleet_management.plugins.topology.plot_map import plot
from fleet_management.plugins.topology.planner_area import PlannerArea
from fleet_management.plugins.topology.map_loader import TopologyPlannerMap


class _TopologyPathPlanner(object):
    def __init__(self, map_name="brsu-full"):
        """

        """
        self.occ_grid = None
        self.path_plan = None
        self.map_name = map_name
        self.map_bridge = TopologyPlannerMap(self.map_name)
        self.path_plan_fms = []

        self.logger = logging.getLogger("fms.plugins.path_planner")

        self.logger.info("Path planner service ready ...")

    def get_path_plan(self, start="", destination=""):

        path_plan = self.map_bridge.get_astar_path(start, destination)

        for node in path_plan:
            self.path_plan_fms.append(self.decode_planner_area(node))

        return self.path_plan_fms

    def topology_to_fms_area(self, topology_area):
        """Converts Topology area to FMS area

        Args:
            topology_area (Topology Area): eg. rooms, corridor, elevator etc.

        Returns:
            TYPE: FMS area

        """
        area = Area()
        area.id = topology_area.id
        area.name = topology_area.ref
        area.type = topology_area.type
        if topology_area.level:
            area.floor_number = int(topology_area.level)
        area.sub_areas = []
        if topology_area.navigation_areas is not None:
            for nav_area in topology_area.navigation_areas:
                area.sub_areas.append(self.topology_to_fms_subarea(nav_area))
        return area

    def topology_to_fms_subarea(self, topology_area):
        """Converts Topology to FMS subarea

        Args:
            topology_area (Topology LocalArea): eg. charging, docking,
                                                   undocking areas
        Returns:
            TYPE: FMS SubArea

        """
        sa = SubArea()
        sa.id = topology_area.id
        sa.name = topology_area.ref
        return sa

    def decode_planner_area(self, planner_area):
        """Topology Path planner path consist of PlannerAreas which has local areas and exit doors. In FMS we consider door at same level as area.
        This function is used to extract door from OBL PlannerArea and return it as separate area along with door

        Args:
            planner_area (Topology PlannerArea):

        Returns:
            TYPE: [FMS Area]

        """
        area = self.topology_to_fms_area(planner_area)

        return [area]

    def task_to_behaviour(self, task):
        """Convert FMS task to behaviours modelled in OSM world model

        Args:
            task (string):

        Returns:
            TYPE: Maybe string

        """
        if task == "DOCK":
            return "docking"
        elif task == "UNDOCK":
            return "undocking"
        elif task == "CHARGE":
            return "charging"
        elif task == "REQUEST_ELEVATOR" or task == "EXIT_ELEVATOR":
            return "waiting"
        return None


class TopologyPathPlannerBuilder:
    def __init__(self):
        self._instance = None

    def __call__(self, **kwargs):
        if not self._instance:
            self._instance = _TopologyPathPlanner(**kwargs)
        return self._instance


configure = TopologyPathPlannerBuilder()
