from fleet_management.obl_to_fms_adapter import OBLToFMSAdapter
from OBL import OSMBridge
from OBL import PathPlanner

class FMSPathPlanner(object):

    def __init__(self, *args, **kwargs):
        self.osm_bridge = OSMBridge(*args, **kwargs)
        self.path_planner = PathPlanner(self.osm_bridge)

    def set_building(self, ref):
        self.path_planner.set_building(ref)

    def set_coordinate_system(self, coordinate_system):
        self.path_planner.set_coordinate_system(coordinate_system)

    def get_path_plan(self,start_floor='', destination_floor='', start_area='', destination_area='', *args, **kwargs):
        try:
            navigation_path = self.path_planner.get_path_plan(start_floor,destination_floor,start_area,destination_area,*args,**kwargs)
            navigation_path_fms = []
            for pt in navigation_path:
                navigation_path_fms.append(OBLToFMSAdapter._decode_planner_area(pt))
            return navigation_path_fms
        except Exception as e:
            print(str(e))
            return None

    def get_estimated_path_distance(self,start_floor='', destination_floor='', start_area='', destination_area='', *args, **kwargs):
        return path_planner.get_estimated_path_distance(start_floor, destination_floor, start_area, destination_area, *args, **kwargs)


    def get_area(self,ref):
        try:
            area = self.osm_bridge.get_area(ref)
            return OBLToFMSAdapter._obl_to_fms_area(area)
        except Exception as e:
            print(str(e))
            return None

    def get_sub_area(self,ref):
        try:
            subarea = self.osm_bridge.get_local_area(ref)
            return OBLToFMSAdapter._obl_to_fms_local_area(ref)
        except Exception as e:
            print(str(e))
            return None