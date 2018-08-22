from fleet_management.structs.area import Area

class PathPlanner(object):
    '''Returns a list of fleet_management.structs.area.Area objects representing
    the path through which a robot should from "start_location" to "destination"

    @param start_location a fleet_management.structs.area.Area object
    @param destination a fleet_management.structs.area.Area object

    '''
    @staticmethod
    def get_path_plan(start_location, destination):
        json_plan = PathPlanner.__generate_test_osm_plan(start_location, destination)
        path_plan = PathPlanner.__parse_plan(json_plan)
        path_plan.append(destination)
        return path_plan

    @staticmethod
    def __parse_plan(json_plan):
        plan_areas = list()
        for json_area in json_plan['payload']['topologicalNodes']:
            area = Area()
            area.id = json_area['tags']['id']
            area.name = json_area['tags']['name']
            area.floor_number = json_area['tags']["floor"]
            plan_areas.append(area)
        return plan_areas

    @staticmethod
    def __generate_test_osm_plan(start_location, destination):
        plan = dict()
        plan['payload'] = dict()
        plan['payload']['topologicalNodes'] = list()

        if start_location.name == 'pickup_location' and destination.name == 'delivery_location':
            wp1 = dict()
            wp1['tags']['id'] = '1'
            wp1['tags']['name'] = 'hallway1'
            wp1['tags']['floor_number'] = 0
            plan['payload']['topologicalNodes'].append(wp1)

            wp2 = dict()
            wp2['tags']['id'] = '2'
            wp2['tags']['name'] = 'hallway2'
            wp2['tags']['floor_number'] = 0
            plan['payload']['topologicalNodes'].append(wp2)

            wp3 = dict()
            wp3['tags']['id'] = '3'
            wp3['tags']['name'] = 'hallway3'
            wp3['tags']['floor_number'] = 0
            plan['payload']['topologicalNodes'].append(wp3)
        elif start_location.name == 'delivery_location' and destination.name == 'charging_station':
            wp1 = dict()
            wp1['tags']['id'] = '1'
            wp1['tags']['name'] = 'hallway3'
            wp1['tags']['floor_number'] = 0
            plan['payload']['topologicalNodes'].append(wp1)

            wp2 = dict()
            wp2['tags']['id'] = '2'
            wp2['tags']['name'] = 'hallway3'
            wp2['tags']['floor_number'] = 0
            plan['payload']['topologicalNodes'].append(wp2)

            wp3 = dict()
            wp3['tags']['id'] = '3'
            wp3['tags']['name'] = 'hallway1'
            wp3['tags']['floor_number'] = 0
            plan['payload']['topologicalNodes'].append(wp3)

        return plan
