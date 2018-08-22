from fleet_management.structs.area import Area, Waypoint

class Action(object):
    def __init__(self):
        self.id = ''
        self.type = ''

        # fields for goto actions
        self.areas = list()
        self.waypoints = list()

        # fields for elevator request actions
        self.start_floor = -1
        self.goal_floor = -1

        # fields for entering/exiting elevators
        self.level = -1
        self.elevator_id = -1

        # pending, in progress, etc.
        self.execution_status = ''
        self.eta = -1.

    def to_json(self):
        action_json = dict()

        action_json['id'] = self.id
        action_json["type"] = self.type
        action_json["start_floor"] = self.start_floor
        action_json["goal_floor"] = self.goal_floor
        action_json["level"] = self.level
        action_json["elevator_id"] = self.elevator_id
        action_json["execution_status"] = self.execution_status
        action_json["eta"] = self.eta

        action_json['areas'] = list()
        for area in self.areas:
            area_json = area.to_json()
            action_json['areas'].append(area_json)

        action_json['waypoints'] = list()
        for waypoint in self.waypoints:
            waypoint_json = waypoint.to_json()
            action_json['waypoints'].append(waypoint_json)

        return action_json

    @staticmethod
    def from_json(action_json):
        action = Action()

        action.id = action_json['id']
        action.type = action_json['type']

        action.start_floor = action_json['start_floor']
        action.goal_floor = action_json['goal_floor']

        action.level = action_json['level']
        action.elevator_id = action_json['elevator_id']

        action.execution_status = action_json['execution_status']
        action.eta = action_json['eta']

        for _, area_json in action_json['areas'].items():
            area = Area.from_json(area_json)
            action.areas.append(area)

        for _, waypoint_json in action_json['waypoints'].items():
            waypoint = Waypoint.from_json(waypoint_json)
            action.waypoints.append(waypoint)

        return action
