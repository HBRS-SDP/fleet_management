from fleet_management.structs.status import RobotStatus

class Robot(object):
    def __init__(self):
        self.robot_id = ''
        self.schedule = ''
        self.status = RobotStatus()

    def to_json(self):
        robot_json = dict()
        robot_json['robot_id'] = self.robot_id
        robot_json['schedule'] = self.schedule
        robot_json['status'] = self.status.to_json()
        return robot_json
