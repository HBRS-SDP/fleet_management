class Waypoint(object):
    def __init__(self):
        self.semantic_id = ''
        self.area_id = ''
        self.x = -1.
        self.y = -1.

    def to_dict(self):
        waypoint_dict = dict()
        waypoint_dict['semantic_id'] = self.semantic_id
        waypoint_dict['area_id'] = self.area_id
        waypoint_dict['x'] = self.x
        waypoint_dict['y'] = self.y
        return waypoint_dict

    @staticmethod
    def from_dict(waypoint_dict):
        waypoint = Waypoint()
        waypoint.semantic_id = waypoint_dict['semantic_id']
        waypoint.area_id = waypoint_dict['area_id']
        waypoint.x = waypoint_dict['x']
        waypoint.y = waypoint_dict['y']
        return waypoint


class Area(object):
    def __init__(self):
        self.id = ''
        self.name = ''
        self.waypoints = list()
        self.floor_number = -1

    def to_dict(self):
        area_dict = dict()
        area_dict['id'] = self.id
        area_dict['name'] = self.name
        area_dict['floor_number'] = self.floor_number
        area_dict['waypoints'] = list()
        for waypoint in self.waypoints:
            area_dict['waypoints'].append(waypoint.to_dict())
        return area_dict

    @staticmethod
    def from_dict(area_dict):
        area = Area()
        area.id = area_dict['id']
        area.name = area_dict['name']
        area.floor_number = area_dict['floor_number']
        for _, waypoint_dict in area_dict['waypoints']:
            waypoint = Waypoint.from_dict(waypoint_dict)
            area.waypoints.append(waypoint)
        return area
