class ResourceManager(object):
    def __init__(self, config_params, ccu_store):
        self.robots = config_params.ropods
        self.elevators = config_params.elevators
        self.scheduled_robot_tasks = dict()
        self.elevator_requests = dict()
        self.robot_statuses = dict()
        self.ccu_store = ccu_store

    def restore_data(self):
        self.robot_statuses = self.ccu_store.get_robot_statuses()
        self.elevators = self.ccu_store.get_elevators()
        self.robots = self.ccu_store.get_robots()

    def get_robots_for_task(self, request, task_plan):
        return list()

    def receive_msg_cb(self, msg_content):
        pass

    def get_robot_status(self, robot_id):
        pass

    def request_elevator(self, start_floor, goal_floor, elevator_id, query_id):
        pass

    def confirm_robot_action(self, robot_action, robot_id):
        pass

    def confirm_elevator(self, query_id):
        pass
