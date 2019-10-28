class TaskPlanError(Exception):
    """Task planning base exception"""

    def __init__(self, task_id, msg=None):
        if msg is None:
            msg = "An error occurred in the task planner for task %s" % task_id
        super(TaskPlanError, self).__init__(msg)
        self.task_id = task_id


class NoPlanFound(TaskPlanError):
    """Task planner could not find a plan"""

    def __init__(self, task_id, msg=None, cause=None):
        if msg is None:
            msg = "Task plan could not be found for task %s" % task_id
            super(NoPlanFound, self).__init__(task_id, msg)

        self.cause = cause
