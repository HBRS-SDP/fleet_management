from OBL import OSMBridge
from OBL import PathPlanner
from OBL.local_area_finder import LocalAreaFinder
from ropod.structs.area import Area, SubArea


class FMSPathPlanner(object):
    """Summary

    Attributes:
        building_ref (string): building name eg. 'AMK' or 'BRSU'
        local_area_finder (OBL LocalAreaFinder):
        osm_bridge (OBL OSMBridge): Description
        path_planner (OBL PathPlanner): Description
    """

    def __init__(self, *args, **kwargs):
        """Summary

        Args:
            server_ip(string ip address): overpass server ip address
            server_port(int): overpass server port
            building(string): building ref eg. 'AMK' or 'BRSU'
        """
        self.osm_bridge = OSMBridge(*args, **kwargs)
        self.path_planner = PathPlanner(self.osm_bridge)
        self.local_area_finder = LocalAreaFinder(self.osm_bridge)
        self.building_ref = kwargs.get("building")
        if self.building_ref is not None:
            self.set_building(self.building_ref)

    def set_building(self, ref):
        """Summary
        Setter function to switch to different building after initialisation
        Args:
            ref (string): building ref
        """
        self.path_planner.set_building(ref)
        self.building_ref = ref

    def set_coordinate_system(self, coordinate_system):
        """Summary
        Set coordinate system
        Args:
            coordinate_system (string): 'spherical' / 'coordinate'
        """
        self.path_planner.set_coordinate_system(coordinate_system)

    def get_path_plan(self, start_floor='', destination_floor='', start_area='', destination_area='', *args, **kwargs):
        """Summary
        Plans path using A* and semantic info in in OSM
        Either start_local_area or robot_position is required
        Either destination_local_area or destination_task id required
        (Destination_task currently works on assumption that only single docking,undocking,charging etc. exist in
        OSM world model for specified area)
        Args:
            start_floor (int): start floor
            destination_floor (int): destination floor
            start_area (str): start area ref
            destination_area (str): destination area ref
            start_local_area (str, optional): start sub area ref
            destination_local_area (str, optional): destination sub area ref
            robot_position([double,double], optional): either in x,y or lat,lng coordinate system
            destination_task(string,optional): task to be performed at destination eg. docking, undocking etc.

        Returns:
            TYPE: [FMS Area]
        """
        start_floor = self.get_floor_name(self.building_ref, start_floor)
        destination_floor = self.get_floor_name(self.building_ref, destination_floor)

        navigation_path = self.path_planner.get_path_plan(start_floor, destination_floor, start_area, destination_area,
                                                          *args, **kwargs)
        navigation_path_fms = []

        for pt in navigation_path:
            temp = self.decode_planner_area(pt)
            if len(temp) == 1:
                navigation_path_fms.append(temp[0])
            elif len(temp) == 2:
                navigation_path_fms.append(temp[0])
                navigation_path_fms.append(temp[1])

        return navigation_path_fms

    def get_estimated_path_distance(self, start_floor, destination_floor, start_area='', destination_area='', *args,
                                    **kwargs):
        """Summary
        Returns approximate path distance in meters
        Args:
            start_floor (int): start floor
            destination_floor (int): destination floor
            start_area (str): start area ref
            destination_area (str): destination area ref

        Returns:
            TYPE: double
        """
        start_floor = self.get_floor_name(self.building_ref, start_floor)
        destination_floor = self.get_floor_name(self.building_ref, destination_floor)
        return self.path_planner.get_estimated_path_distance(start_floor, destination_floor, start_area,
                                                             destination_area, *args, **kwargs)

    def get_area(self, ref):
        """Summary
        Returns OBL Area in FMS Area format
        Args:
            ref (string/number): semantic or uuid
        Returns:
            TYPE: FMS Area
        """
        area = self.osm_bridge.get_area(ref)
        return self.obl_to_fms_area(area)

    def get_sub_area(self, ref, *args, **kwargs):
        """Summary
        Returns OBL local area in FMS SubArea format
        Args:
            ref (string/number): semantic or uuid
            behaviour: SubArea will be searched based on specified behaviour (inside specifeid Area scope)
            robot_position: SubArea will be searched based on robot position (inside specifeid Area scope)
        Returns:
            TYPE: FMS SubArea
        """
        pointX = kwargs.get("x")
        pointY = kwargs.get("y")
        behaviour = kwargs.get("behaviour")
        sub_area = None
        if (pointX and pointY) or behaviour:
            sub_area = self.local_area_finder.get_local_area(area_name=ref, *args, **kwargs)
        else:
            sub_area = self.osm_bridge.get_local_area(ref)

        return self.obl_to_fms_subarea(sub_area)

    def obl_to_fms_area(self, osm_wm_area):
        """Summary
        Converts OBL area to FMS area
        Args:
            osm_wm_area (OBL Area): eg. rooms, corridor, elevator etc.
        Returns:
            TYPE: FMS area
        """
        area = Area()
        area.id = osm_wm_area.id
        area.name = osm_wm_area.ref
        area.type = osm_wm_area.type
        area.sub_areas = []
        if osm_wm_area.navigation_areas is not None:
            for nav_area in osm_wm_area.navigation_areas:
                area.sub_areas.append(self.obl_to_fms_subarea(nav_area))
        return area

    def obl_to_fms_subarea(self, osm_wm_local_area):
        """Summary
        Converts OBL to FMS subarea
        Args:
            osm_wm_local_area (OBL LocalArea): eg. charging, docking, undocking areas
        Returns:
            TYPE: FMS SubArea
        """
        sa = SubArea()
        sa.id = osm_wm_local_area.id
        sa.name = osm_wm_local_area.ref
        return sa

    def decode_planner_area(self, planner_area):
        """Summary
        OBL Path planner path consist of PlannerAreas which has local areas and exit doors. In FMS we consider door at
        same level as area. This function is used to extract door from OBL PlannerArea and return it as separate area
        along with door
        Args:
            planner_area (OBL PlannerArea):
        Returns:
            TYPE: [FMS Area]
        """
        area = self.obl_to_fms_area(planner_area)

        if planner_area.exit_door:
            return [area, self.obl_to_fms_area(planner_area.exit_door)]
        else:
            return [area]

    def task_to_behaviour(self, task):
        """Summary
        Convert FMS task to behaviours modelled in OSM world model
        Args:
            task (string):
        Returns:
            TYPE: Maybe string
        """
        if task == 'DOCK':
            return 'docking'
        elif task == 'UNDOCK':
            return 'undocking'
        elif task == 'CHARGE':
            return 'charging'
        elif task == 'REQUEST_ELEVATOR' or task == 'EXIT_ELEVATOR':
            return 'waiting'
        return None

    def get_floor_name(self, building_ref, floor_number):
        """Summary
        Constructs FMS compatible floor names given floor number and building ref
        Args:
            building_ref (string):
            floor_number (int):
        Returns:
            TYPE: string
        """
        return building_ref + '_L' + str(floor_number)
