from __future__ import print_function
import fleet_management.extern.overpass as overpass
from fleet_management.structs.area import Area
from fleet_management.path_planning import GlobalPathPlanner, LocalPathPlanner

class PathPlanner(object):
    '''Returns a list of fleet_management.structs.area.Area objects representing
    the path through which a robot should from "start_location" to "destination"

    @param start_location a fleet_management.structs.area.Area object
    @param destination a fleet_management.structs.area.Area object

    '''
    @staticmethod
    def get_path_plan(start_location, destination):
        start_area = start_location.name
        destination_area = destination.name
        start_floor = start_location.floor_number
        destination_floor = destination.floor_number

        api_url = "http://192.168.92.10:8000/api/interpreter"
        api = overpass.API(endpoint=api_url)

        start = start_area.split("_")
        destination = destination_area.split("_")

        organisation = start[0]

        source_building = start[1]
        source_floor = start[2]
        source_element = start[3]
        source_local_area = start[4]

        destination_building = destination[1]
        destination_floor = destination[2]
        destination_element = destination[3]
        destination_local_area = destination[4]

        print('Planning global path...')
        gpp = GlobalPathPlanner(organisation, api)
        gpp.set_start_position(source_building, source_floor, source_element)
        gpp.set_destination_position(destination_building, destination_floor, destination_element)

        gpp.plan_path()
        gpp.fetch_path_info()

        print('Generating waypoints...')
        lpp = LocalPathPlanner(organisation, gpp.path_elements_ids, api)
        lpp.set_start_position(source_building, source_floor, source_element, source_local_area)
        lpp.set_destination_position(destination_building, destination_floor,
                                     destination_element, destination_local_area)

        lpp.plan_path()
        path_list = lpp.fetch_path_info()

        print('Waypoints generated; processing path...')
        dict_plan = PathPlanner.__generate_osm_plan(path_list)
        path_plan = PathPlanner.__parse_plan(dict_plan)
        print('Successfully processed path')
        return path_plan

    @staticmethod
    def __generate_osm_plan(path_list):
        plan = dict()
        plan['payload'] = dict()
        plan['payload']['topologicalNodes'] = list()

        for way_pt in path_list:
            wp = dict()
            wp['tags'] = dict()
            wp['tags']['id'] = way_pt[0]
            wp['tags']['name'] = way_pt[1]
            wp['tags']['floor_number'] = way_pt[3]
            plan['payload']['topologicalNodes'].append(wp)
        return plan

    @staticmethod
    def __parse_plan(json_plan):
        plan_areas = list()
        for json_area in json_plan['payload']['topologicalNodes']:
            area = Area()
            area.id = json_area['tags']['id']
            area.name = json_area['tags']['name']
            area.floor_number = json_area['tags']["floor_number"]
            plan_areas.append(area)
        return plan_areas

    @staticmethod
    def __generate_test_osm_plan(start_location, destination):
        plan = dict()
        plan['payload'] = dict()
        plan['payload']['topologicalNodes'] = list()

        if start_location.name == 'pickup_location' and destination.name == 'delivery_location':
            wp1 = {'tags': {}}
            wp1['tags']['id'] = '1'
            wp1['tags']['name'] = 'hallway1'
            wp1['tags']['floor_number'] = 0
            plan['payload']['topologicalNodes'].append(wp1)

            wp2 = {'tags': {}}
            wp2['tags']['id'] = '2'
            wp2['tags']['name'] = 'hallway2'
            wp2['tags']['floor_number'] = 0
            plan['payload']['topologicalNodes'].append(wp2)

            wp3 = {'tags': {}}
            wp3['tags']['id'] = '3'
            wp3['tags']['name'] = 'hallway3'
            wp3['tags']['floor_number'] = 0
            plan['payload']['topologicalNodes'].append(wp3)
        elif start_location.name == 'delivery_location' and destination.name == 'charging_station':
            wp1 = {'tags': {}}
            wp1['tags']['id'] = '1'
            wp1['tags']['name'] = 'hallway3'
            wp1['tags']['floor_number'] = 0
            plan['payload']['topologicalNodes'].append(wp1)

            wp2 = {'tags': {}}
            wp2['tags']['id'] = '2'
            wp2['tags']['name'] = 'hallway2'
            wp2['tags']['floor_number'] = 0
            plan['payload']['topologicalNodes'].append(wp2)

            wp3 = {'tags': {}}
            wp3['tags']['id'] = '3'
            wp3['tags']['name'] = 'hallway1'
            wp3['tags']['floor_number'] = 0
            plan['payload']['topologicalNodes'].append(wp3)

        return plan
