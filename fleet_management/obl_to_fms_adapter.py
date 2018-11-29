from OBL import OSMBridge
from OBL import PathPlanner
from fleet_management.structs.area import Area, SubArea

class OBLToFMSAdapter(object):

    """Summary
    Adapts OSM bridge to work with fms strucutures
    """
    
    @staticmethod
    def obl_to_fms_area(osm_wm_area):
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
        area.subareas = []
        if osm_wm_area.navigation_areas is not None:
            for nav_area in osm_wm_area.navigation_areas:
                area.subareas.append(OBLToFMSAdapter.obl_to_fms_subarea(nav_area))
        return area

    @staticmethod
    def obl_to_fms_subarea(osm_wm_local_area):
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

    @staticmethod
    def decode_planner_area(planner_area):
        """Summary
        OBL Path planner path consist of PlannerAreas which has local areas and exit doors. In FMS we consider door at same level as area. This function is used to extract door from OBL PlannerArea and return it as separate area along with door
        Args:
            planner_area (OBL PlannerArea):     
        Returns:
            TYPE: [FMS Area]
        """
        area = OBLToFMSAdapter.obl_to_fms_area(planner_area)

        if planner_area.exit_door:
            return [area, OBLToFMSAdapter.obl_to_fms_area(planner_area.exit_door)]
        else:
            return [area]

    @staticmethod
    def task_to_behaviour(task):
        """Summary
        Convert FMS task to behaviours modelled in OSM world model
        Args:
            task (string):
        Returns:
            TYPE: string
        """
        if task == 'DOCK':
            return 'docking'
        elif task == 'UNDOCK':
            return 'undocking'
        elif task == 'CHARGE':
            return 'charging'

    @staticmethod
    def get_floor_name(building_ref, floor_number):
        """Summary
        Constructs FMS compatible floor names given floor number and building ref
        Args:
            building_ref (string):
            floor_number (int): 
        Returns:
            TYPE: string
        """
        return building_ref + '_L' + str(floor_number)

