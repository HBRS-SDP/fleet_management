import logging

from ropod.structs.task import TaskStatus as TaskStatusConst
from fleet_management.db.models.task import TransportationTask as Task


class Dispatcher:
    def __init__(self, ccu_store, api, **kwargs):
        self.logger = logging.getLogger('fms.task.dispatcher')
        self.ccu_store = ccu_store
        self.api = api
        self.scheduled_tasks = dict()

    def dispatch_tasks(self):
        """
        Dispatches all scheduled tasks that are ready for dispatching
        """
        planned_tasks = Task.get_tasks_by_status(TaskStatusConst.PLANNED)
        for task in planned_tasks:
            if task.is_executable():
                self.logger.info('Dispatching task %s', task.task_id)
                for robot in task.assigned_robots:
                    self.dispatch_task(task, robot.robot_id)
                task.update_status(TaskStatusConst.DISPATCHED)

    def dispatch_task(self, task, robot_id):
        """
        Sends a task to the appropriate robot fleet

        Args:
            task: a ropod.structs.task.Task object
            robot_id: a robot UUID
        """
        self.logger.info("Dispatching task to robot %s", robot_id)
        task_msg = self.api.create_message(task)
        task_msg["payload"].pop("constraints")
        task_msg["payload"]["assignedRobots"] = [robot_id for robot_id, robot in
                                                 task_msg["payload"]["assignedRobots"].items()]
        self.api.publish(task_msg, groups=['ROPOD'])

