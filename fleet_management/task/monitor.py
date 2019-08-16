import logging

from mrs.task_execution.task_monitor import TaskMonitor as MRTATaskMonitor
from ropod.structs.status import TaskStatus


class TaskMonitor(MRTATaskMonitor):
    def __init__(self, ccu_store, task_cls, api):
        self.logger = logging.getLogger('fms.task.monitor')
        super().__init__(ccu_store, task_cls, api)

        self.ongoing_task_ids = list()
        self.task_statuses = dict()
        self.scheduled_tasks = dict()
        self.ccu_store = ccu_store

    def task_progress_cb(self, msg):
        action_type = msg['payload']['actionType']
        self.logger.debug("Received task progress message... Action %s %s " % (msg["payload"]["actionType"],
                                                                               msg["payload"]['status']["areaName"]))
        task_id = msg["payload"]["taskId"]
        robot_id = msg["payload"]["robotId"]
        current_action = msg["payload"]["actionId"]
        action_type = msg["payload"]["actionType"]
        area_name = msg["payload"]["status"]["areaName"]
        action_status = msg["payload"]["status"]["actionStatus"]
        if action_type == "GOTO":
            current_action = msg["payload"]["status"]["sequenceNumber"]
            total_actions = msg["payload"]["status"]["totalNumber"]

        task_status = msg["payload"]["status"]["taskStatus"]

        self.__update_task_status(task_id, robot_id, current_action, task_status)

    def __update_task_status(self, task_id, robot_id, current_action, task_status):
        '''Updates the status of the robot with ID 'robot_id' that is performing
        the task with ID 'task_id'

        If 'task_status' is "terminated", removes the task from the list of scheduled
        and ongoing tasks and saves a historical database entry for the task.
        On the other hand, if 'task_status' is "ongoing", the task's entry
        is updated for the appropriate robot.

        @param task_id UUID representing a previously scheduled task
        @param robot_id name of a robot
        @param current_action UUID representing an action
        @param task_status a string representing the status of a task;
               takes the values "unallocated", "allocated", "ongoing", "terminated", and "completed"
        '''
        self.logger.debug("New task status: %s ", task_status)
        status = self.task_statuses[task_id]
        self.logger.debug("Previous task status: %s ", status.status)
        status.status = task_status

        if task_status == TaskStatus.CANCELED or task_status == TaskStatus.COMPLETED:
            if task_status == TaskStatus.CANCELED:
                self.logger.debug("Task terminated")
            elif task_status == TaskStatus.COMPLETED:
                self.logger.debug("Task completed!")
            task = self.scheduled_tasks[task_id]
            self.ccu_store.archive_task(task, task.status)
            self.scheduled_tasks.pop(task_id)
            self.task_statuses.pop(task_id)
            if task_id in self.ongoing_task_ids:
                self.ongoing_task_ids.remove(task_id)
        elif task_status == TaskStatus.ONGOING:
            previous_action = status.current_robot_action[robot_id]
            status.completed_robot_actions[robot_id].append(previous_action)
            status.current_robot_action[robot_id] = current_action
            self.ccu_store.update_task_status(status)

            # TODO: update the estimated time duration based on the current timestamp
            # and the estimated duration of the rest of the tasks
