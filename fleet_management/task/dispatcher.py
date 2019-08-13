import logging

from ropod.structs.status import TaskStatus


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
        self.logger.error(self.scheduled_tasks)
        for task_id, task in self.scheduled_tasks.items():
            # if task_id not in self.ongoing_task_ids:
            if task.is_executable():
                self.logger.info('Dispatching task %s', task_id)
                for robot_id, actions in task.robot_actions.items():
                    self.dispatch_task(task, robot_id)
                # self.ongoing_task_ids.append(task_id)
                # TODO remove from scheduled tasks collection
                self.ccu_store.add_ongoing_task(task_id)
                # self.__initialise_task_status(task_id)
                task.set_status(TaskStatus.ONGOING, task=task)
                self.ccu_store.add_task_status(task.status)

    def dispatch_task(self, task, robot_id):
        """
        Sends a task to the appropriate robot fleet

        @param task a ropod.structs.task.Task object
        @param robot_id
        """
        self.logger.info("Dispatching task to robot %s", robot_id)
        task_msg = self.api.create_message(task, recipients=[robot_id])
        self.api.publish(task_msg)

    def add_scheduled_task(self, task):
        self.scheduled_tasks[task.id] = task
