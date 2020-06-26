"""Monitoring of tasks
"""
import logging

import inflection
from fleet_management.db.models.task import TransportationTask as Task
from fmlib.utils.messages import Document, Message
from ropod.structs.status import TaskStatus, ActionStatus
from ropod.utils.timestamp import TimeStamp


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

    def add_plugin(self, obj, name=None):
        if name:
            key = inflection.underscore(name)
        else:
            key = inflection.underscore(obj.__class__.__name__)
        self.__dict__[key] = obj
        self.logger.debug("Added %s plugin to %s", key, self.__class__.__name__)

    def _update_timetable(self, timestamp, task_id, robot_id, task_progress, **_):
        task = Task.get_task(task_id)
        self.timetable_monitor.update_timetable(task, robot_id, task_progress, timestamp.to_datetime())

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
        timestamp = TimeStamp.from_str(message.timestamp)

        self.logger.debug("Received task status message for task %s by %s", task_id, robot_id)
        self._update_task_status(task_id, status, robot_id)

        failure_warning = ''
        if status == TaskStatus.FAILED:
            failure_warning = "Task %s has failed. " % task_id

        task_progress = payload.get("task_progress")
        if task_progress:
            action_status = self._update_task_progress(**payload)
            action_type = task_progress.get('action_type')
            action_id = task_progress.get('action_id')

            action_failure = ''

            if status == TaskStatus.ONGOING:
                self._update_timetable(timestamp, **payload)

            if action_status == ActionStatus.FAILED:
                action_failure = "Action %s (%s) returned status code %i (FAILED)." % (action_type,
                                                                                       action_id,
                                                                                       action_status)

            failure_warning = failure_warning + action_failure

        # Notify the user if there is a need for recovery actions from them
        if failure_warning:
            self.request_human_assistance(failure_warning, robot_id, task_progress.get("area"))

    def request_human_assistance(self, reason, robot_id, location):
        from fmlib.utils.messages import Header
        self.logger.warning(reason + " Notifying user...")
        assistance_msg = {"header": Header("HUMAN-REQUIRED-NOTIFICATION"),
                          'payload': {"reason": reason,
                                      "robot": robot_id,
                                      "location": location
                                      }
                          }
        self.api.publish(assistance_msg)

    def _update_task_status(self, task_id, status, robot_id):
        """Updates the status of a task with id=task_id

        Args:
            task_id: The id of the task to update
            status (const): The corresponding status code for a task
            robot_id: The id of the robot that update

        """
        self.logger.debug("Task %s status by %s: %s", task_id, robot_id, status)
        task = Task.get_task(task_id)

        if status == TaskStatus.UNALLOCATED:
            self.timetable_monitor.re_allocate(task)

        elif status == TaskStatus.PREEMPTED:
            self.timetable_monitor.preempt(task)

        elif status in [TaskStatus.ABORTED, TaskStatus.COMPLETED]:
            self.timetable_monitor.remove_task(task, status)

        else:
            task.update_status(status)

    def _update_task_progress(self, task_id, task_progress, **_):
        """Updates the progress field of the task status with the current action

        Args:
            task_id: The id of the task to update
            task_progress: A task progress dictionary, as specified by ropod schema
            task_status: The corresponding status code for a task
            robot_id: The id of the robot that update
        """
        self.logger.debug("Received progress for task %s", task_id)
        task = Task.get_task(task_id)
        action_id = task_progress.get('action_id')
        action_status = task_progress.get('action_status').get('status')
        task.update_progress(action_id, action_status)

        return action_status
