from OBL import OSMBridge
from OBL import PathPlanner
from fleet_management.structs.area import Area, SubArea

class OBLToFMSAdapter(object):
    
    @staticmethod
    def _obl_to_fms_area(osm_wm_area):
        area = Area()
        area.id = osm_wm_area.id
        area.name = osm_wm_area.name
        area.type = osm_wm_area.type
        area.subareas = []
        if osm_wm_area.navigation_areas is not None:
            for nav_area in osm_wm_area.navigation_areas:
                area.subareas.append(OBLToFMSAdapter.obl_to_fms_subarea(nav_area))
        return area

    @staticmethod
    def obl_to_fms_subarea(osm_wm_local_area):
        sa = SubArea()
        sa.id = osm_wm_local_area.id
        sa.name = osm_wm_local_area.ref
        return sa

    @staticmethod
    def _decode_planner_area(planner_area):
        area = OBLToFMSAdapter._obl_to_fms_area(planner_area)

        if planner_area.exit_door:
            return area, OBLToFMSAdapter._obl_to_fms_area(planner_area.exit_door)
        else:
            return area

    @staticmethod
    def task_to_behaviour(task):
        if task == 'DOCK':
            return 'docking'
        elif task == 'UNDOCK':
            return 'undocking'
