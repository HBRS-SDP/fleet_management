from fleet_management.obl_to_fms_adapter import OBLToFMSAdapter
from OBL import OSMBridge
from OBL import PathPlanner
from OBL import LocalAreaFinder

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

    def get_path_plan(self,start_floor='', destination_floor='', start_area='', destination_area='', *args, **kwargs):
        """Summary
        Plans path using A* and semantic info in in OSM
        Either start_local_area or robot_position is required
        Either destination_local_area or destination_task id required
        (Destination_task currently works on assumption that only single docking,undocking,charging etc. exist in OSM world model for specified area)
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
        start_floor = OBLToFMSAdapter.get_floor_name(self.building_ref, start_floor)
        destination_floor = OBLToFMSAdapter.get_floor_name(self.building_ref, destination_floor)

        navigation_path = self.path_planner.get_path_plan(start_floor,destination_floor,start_area,destination_area,*args,**kwargs)
        navigation_path_fms = []

        for pt in navigation_path:
            temp = OBLToFMSAdapter.decode_planner_area(pt)
            if len(temp) == 1:
                navigation_path_fms.append(temp[0])
            elif len(temp) == 2:
                navigation_path_fms.append(temp[0])
                navigation_path_fms.append(temp[1])

        return navigation_path_fms

    def get_estimated_path_distance(self,start_floor, destination_floor, start_area='', destination_area='', *args, **kwargs):
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
        start_floor = OBLToFMSAdapter.get_floor_name(self.building_ref, start_floor)
        destination_floor = OBLToFMSAdapter.get_floor_name(self.building_ref, destination_floor)
        return path_planner.get_estimated_path_distance(start_floor, destination_floor, start_area, destination_area, *args, **kwargs)


    def get_area(self,ref):
        """Summary
        Returns OBL Area in FMS Area format
        Args:
            ref (string/number): semantic or uuid      
        Returns:
            TYPE: FMS Area
        """
        area = self.osm_bridge.get_area(ref)
        return OBLToFMSAdapter.obl_to_fms_area(area)

    def get_sub_area(self,ref,*args,**kwargs):
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
        behaviour =kwargs.get("behaviour")
        sub_area = None
        if (pointX and pointY) or behaviour:
            sub_area = self.local_area_finder.get_local_area(area_name=ref,*args, **kwargs)
        else:
            sub_area = self.osm_bridge.get_local_area(ref)

        return OBLToFMSAdapter.obl_to_fms_subarea(sub_area)
 