import logging

from OBL import OSMBridge

class OSMBridgeFMS(OSMBridge):

    """OSM bridge derived class for FMS functionality"""

    def __init__(self, **kwargs):
        super(OSMBridgeFMS, self).__init__(**kwargs)

    def get_all_local_area_of_behaviour_type(self, building, desired_behaviour=None):
        """ Get all the local areas in a building of certain behaviour and return
        their reference.

        :building: str
        :desired_behaviour: str (docking, undocking, charging or waiting)
        :returns: list of str

        """
        building = self.get_building(building)
        areas = []
        for floor in building.floors:
            temp = floor.areas
            if temp is not None:
                areas.extend(temp)
            temp = floor.corridors
            if temp is not None:
                areas.extend(temp)
            temp = floor.rooms
            if temp is not None:
                areas.extend(temp)

        local_areas = []
        for area in areas:
            local_areas.extend(area.local_areas)

        local_areas_with_desired_behaviour = []
        for local_area in local_areas:
            local_area.geometry # required since all tags are in geometrical model
            if local_area.behaviour:
                if desired_behaviour is None or local_area.behaviour == desired_behaviour:
                    local_areas_with_desired_behaviour.append(local_area.ref)
        return local_areas_with_desired_behaviour

        


class OSMBridgeBuilder:
    def __init__(self):
        self._instance = None
        self.logger = logging.getLogger(__name__)

    def __call__(self, **kwargs):
        if not self._instance:
            self.logger.debug("Configuring osm_bridge")
            try:
                self._instance = OSMBridgeFMS(**kwargs)
            except Exception as e:
                self.logger.error("There is a problem in connecting to Overpass server. Error: %s", e)
                self._instance = None
            self.logger.info("Connected to osm_bridge (%s:%s)", kwargs.get('server_ip'), kwargs.get('server_port'))
        return self._instance


configure = OSMBridgeBuilder()
