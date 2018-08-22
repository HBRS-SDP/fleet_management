class ElevatorRequests(object):
    def __init__(self):
        self.current_floor = -1
        self.number_of_active_requests = -1


class ElevatorRequest(object):
    def __init__(self):
        self.elevator_id = -1
        self.operational_mode = ''
        self.start_floor = -1
        self.goal_floor = -1
        self.query_id = ''
        self.command = ''
        self.task_id = ''
        self.load = ''
        self.robot_id = ''
        self.status = ''

    def to_json(self):
        request_json = dict()
        request_json['elevator_id'] = self.elevator_id
        request_json['operational_mode'] = self.operational_mode
        request_json['start_floor'] = self.start_floor
        request_json['goal_floor'] = self.goal_floor
        request_json['query_id'] = self.query_id
        request_json['command'] = self.task_id
        request_json['load'] = self.load
        request_json['robot_id'] = self.robot_id
        request_json['status'] = self.status
        return request_json


class Elevator(object):
    def __init__(self):
        self.elevator_id = -1
        self.floor = -1 # TODO: Need to match floors from toma messages to world model ones
        self.calls = -1
        self.is_available = False

    def to_json(self):
        elevator_json = dict()
        elevator_json['elevator_id'] = self.elevator_id
        elevator_json['floor'] = self.floor
        elevator_json['calls'] = self.calls
        elevator_json['is_available'] = self.is_available
        return elevator_json
