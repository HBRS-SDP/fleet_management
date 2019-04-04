class UnsucessfulAllocationError(Exception):
    """ Raised when a task could not be allocated in the desired time slot.
    A suggested start time for the task is passed as an argument"""

    def __init__(self, task_id, robot_id, suggested_start_time):
        Exception.__init__(self, task_id, robot_id, suggested_start_time)
        self.task_id = task_id
        self.robot_id = robot_id
        self.suggested_start_time = suggested_start_time
