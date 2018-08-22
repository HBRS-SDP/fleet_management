class Waypoint(object):
    def __init__(self):
        self.semantic_id = ''
        self.area_id = ''
        self.x = -1.
        self.y = -1.

    def to_json(self):
        waypoint_json = dict()
        waypoint_json['semantic_id'] = self.semantic_id
        waypoint_json['area_id'] = self.area_id
        waypoint_json['x'] = self.x
        waypoint_json['y'] = self.y
        return waypoint_json

    @staticmethod
    def from_json(waypoint_json):
        waypoint = Waypoint()
        waypoint.semantic_id = waypoint_json['semantic_id']
        waypoint.area_id = waypoint_json['area_id']
        waypoint.x = waypoint_json['x']
        waypoint.y = waypoint_json['y']
        return waypoint


class Area(object):
    def __init__(self):
        self.id = ''
        self.name = ''
        self.waypoints = list()
        self.floor_number = -1

    def to_json(self):
        area_json = dict()
        area_json['id'] = self.id
        area_json['name'] = self.name
        area_json['floor_number'] = self.floor_number
        area_json['waypoints'] = list()
        for waypoint in self.waypoints:
            area_json['waypoints'].append(waypoint.to_json())
        return area_json

    @staticmethod
    def from_json(area_json):
        area = Area()
        area.id = area_json['id']
        area.name = area_json['name']
        area.floor_number = area_json['floor_number']
        for _, waypoint_json in area_json['waypoints']:
            waypoint = Waypoint.from_json(waypoint_json)
            area.waypoints.append(waypoint)
        return area
