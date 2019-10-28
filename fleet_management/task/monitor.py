"""Monitoring of tasks
"""
import logging

from fmlib.models.tasks import Task
from fmlib.utils.messages import Document, Message


class TaskMonitor:
    """A component to monitor tasks

    Args:
        ccu_store (MongoInterface): An interface to the MongoDB database
        api (API): A component to communicate through the network

    """

    def __init__(self, ccu_store, api, **_):
        self.logger = logging.getLogger('fms.task.monitor')

        self.ccu_store = ccu_store
        self.api = api

    def task_status_cb(self, msg):
        """Callback for a task status message

        Args:
            msg (dict): A message in ROPOD format

        """
        message = Message(**msg)
        payload = Document.from_payload(message.payload)

        task_id = payload.get("task_id")
        status = payload.get("task_status")
        robot_id = payload.get("robot_id")
        self.logger.debug("Received task status message for task %s by %s", task_id, robot_id)
        self._update_task_status(task_id, status, robot_id)

        task_progress = payload.get("task_progress")
        if task_progress:
            self._update_task_progress(**payload)

    def _update_task_status(self, task_id, status, robot_id):
        """Updates the status of a task with id=task_id

        Args:
            task_id: The id of the task to update
            status (const): The corresponding status code for a task
            robot_id: The id of the robot that update

        """
        self.logger.debug("Task %s status by %s: %s", task_id, robot_id, status)
        task = Task.get_task(task_id)
        task.update_status(status)

    def _update_task_progress(self, task_id, task_progress, **_):
        """Updates the progress field of the task status with the current action

        Args:
            task_id: The id of the task to update
            task_progress: A task progress dictionary, as specified by ropod schema
            task_status: The corresponding status code for a task
            robot_id: The id of the robot that update
        """
        self.logger.critical("Received progress for task %s", task_id)
        task = Task.get_task(task_id)
        action_id = task_progress.get('action_id')
        action_status = task_progress.get('action_status').get('status')
        task.update_progress(action_id, action_status)
