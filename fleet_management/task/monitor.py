import logging

from ropod.structs.status import TaskStatus


class TaskMonitor(object):
    def __init__(self, ccu_store, **_):
        self.logger = logging.getLogger('fms.task.monitor')

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

        self._update_task_status(task_id, task_status)

    def _update_task_status(self, task_id, task_status):
        """Updates the status of the robot with ID 'robot_id' that is performing
        the task with ID 'task_id'

        If 'task_status' is "terminated", removes the task from the list of scheduled
        and ongoing tasks and saves a historical database entry for the task.
        On the other hand, if 'task_status' is "ongoing", the task's entry
        is updated for the appropriate robot.

        Args:
            task_id UUID representing a previously scheduled task
            robot_id name of a robot
            current_action UUID representing an action
            task_status a string representing the status of a task;
               takes the values "unallocated", "allocated", "ongoing", "terminated", and "completed"
        """

        # Get the task from the database and update its status
        task = get_task(task_id)
        task.update_status(task_status)
        self.logger.debug("New task status: %s ", task_status)

        if task_status == TaskStatus.ONGOING:
            previous_action = status.current_robot_action[robot_id]
            status.completed_robot_actions[robot_id].append(previous_action)
            status.current_robot_action[robot_id] = current_action
            self.ccu_store.update_task_status(status)

            # TODO: update the estimated time duration based on the current timestamp
            # and the estimated duration of the rest of the tasks
