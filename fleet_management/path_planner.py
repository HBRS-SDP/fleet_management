from fleet_management.obl_to_fms_adapter import OBLToFMSAdapter
from OBL import OSMBridge
from OBL import PathPlanner
from OBL import LocalAreaFinder

class FMSPathPlanner(object):

    def __init__(self, *args, **kwargs):
        self.osm_bridge = OSMBridge(*args, **kwargs)
        self.path_planner = PathPlanner(self.osm_bridge)
        self.local_area_finder = LocalAreaFinder(self.osm_bridge)
        self.building_ref = kwargs.get("building")
        if self.building_ref is not None:
            self.set_building(self.building_ref)

    def set_building(self, ref):
        self.path_planner.set_building(ref)
        self.building_ref = ref

    def set_coordinate_system(self, coordinate_system):
        self.path_planner.set_coordinate_system(coordinate_system)

    def get_path_plan(self,start_floor=-1, destination_floor=-1, start_area='', destination_area='', *args, **kwargs):
        # try:
        start_floor_str = self.building_ref + '_L' + str(start_floor)
        destination_floor_str = self.building_ref + '_L' + str(destination_floor)
        navigation_path = self.path_planner.get_path_plan(start_floor_str,destination_floor_str,start_area,destination_area,*args,**kwargs)
        navigation_path_fms = []
        for pt in navigation_path:
            navigation_path_fms.append(OBLToFMSAdapter._decode_planner_area(pt))
        return navigation_path_fms

    def get_estimated_path_distance(self,start_floor=-1, destination_floor=-1, start_area='', destination_area='', *args, **kwargs):
        start_floor_str = building_ref + 'L' + str(start_floor)
        destination_floor_str = building_ref + 'L' + str(destination_floor)
        return path_planner.get_estimated_path_distance(start_floor_str, destination_floor_str, start_area, destination_area, *args, **kwargs)


    def get_area(self,ref):
        try:
            area = self.osm_bridge.get_area(ref)
            return OBLToFMSAdapter._obl_to_fms_area(area)
        except Exception as e:
            print(str(e))
            return None

    def get_sub_area(self,ref,*args,**kwargs):
        try:
            pointX = kwargs.get("x")
            pointY = kwargs.get("y")
            behaviour =kwargs.get("behaviour")

            sub_area = None

            if (pointX and pointY) or behaviour:
                sub_area = self.local_area_finder.get_local_area(area_name=ref,*args, **kwargs)
            else:
                sub_area = self.osm_bridge.get_local_area(ref)

            return OBLToFMSAdapter.obl_to_fms_subarea(sub_area)
        except Exception as e:
            print(str(e))
            return None