import logging

from ropod.structs.area import Area, SubArea
from fleet_management.exceptions.osm import OSMPlannerException

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

    def get_path_plan_from_local_area(self, start_local_area, destination_local_area):
        """ Plans path similar to `get_path_plan` but only needs start and 
        destination local area
        :start_local_area: int or str
        :destination_local_area: int or str
        :returns: list of FMS Areas, mean and variance of path
        """

        return self.get_path_plan(
            start_floor=0,
            destination_floor=0,
            start_area=start_local_area,
            destination_area=destination_local_area,
        )

    def get_path_plan(
        self,
        start_floor="",
        destination_floor="",
        start_area="",
        destination_area="",
        *args,
        **kwargs,
    ):
        """Plans path using A* in topological map
        Either start_local_area or robot_position is required
        destination_local_area required

        Args:
            start_floor (int): start floor
            destination_floor (int): destination floor
            start_area (str): start area ref
            destination_area (str): destination area ref
            
        Returns:
            TYPE: [FMS Area], mean and variance of path
        """
        path_plan = self.map_bridge.get_astar_path(start_area, destination_area)

        for node in path_plan[0]:
            self.path_plan_fms.append(self.decode_planner_area(node))

        return self.path_plan_fms, path_plan[1], path_plan[2]

    def get_sub_area(self, ref, *args, **kwargs):
        """Returns Topology local area in FMS SubArea format
        Args:
            ref (string/number): semantic or uuid
            behaviour: SubArea will be searched based on specified behaviour (inside specified Area scope)
            robot_position: SubArea will be searched based on robot position (inside specified Area scope)
        Returns:
            TYPE: FMS SubArea
        """
        if self.map_bridge:
            xpoint = kwargs.get("x")
            ypoint = kwargs.get("y")
            behaviour = kwargs.get("behaviour")
            sub_area = None
            if (xpoint and ypoint) or behaviour:
                sub_area = self.map_bridge.local_area_finder(
                    area_name=ref, *args, **kwargs
                )
                if not sub_area:
                    if behaviour:
                        self.logger.error(
                            "Local area finder did not return a sub area within area %s with behaviour %s"
                            % (ref, behaviour)
                        )
                        raise OSMPlannerException(
                            "Local area finder did not return a sub area within area %s with "
                            "behaviour %s" % (ref, behaviour)
                        )
                    else:
                        self.logger.error(
                            "Local area finder did not return a sub area within area %s for point ("
                            "%.2f, %.2f)" % (ref, xpoint, ypoint)
                        )
                        raise OSMPlannerException(
                            "Local area finder did not return a sub area within area %s for "
                            "point (%.2f, %.2f)" % (ref, xpoint, ypoint)
                        )
            else:
                sub_area = self.map_bridge.get_local_area(ref)

            return self.topology_to_fms_subarea(sub_area)

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
            # self.logger.info(topology_area.navigation_areas)
            # for nav_area in topology_area.navigation_areas:
            area.sub_areas.append(
                self.topology_to_fms_subarea(topology_area.navigation_areas)
            )
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
        sa.id = topology_area["topology_id"]
        sa.name = "BRSU_C_L0_" + topology_area["label"]
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

        return area

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
