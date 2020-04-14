from fmlib.models.tasks import TransportationTask as Task
from fmlib.utils.messages import Document, Message
from mrs.execution.schedule_execution_monitor import ScheduleExecutionMonitor as ScheduleExecutionMonitorBase
from mrs.messages.task_status import TaskStatus, TaskProgress
from ropod.structs.status import TaskStatus as TaskStatusConst
from ropod.utils.timestamp import TimeStamp


class ScheduleExecutionMonitor(ScheduleExecutionMonitorBase):
    def __init__(self, robot_id, timetable, scheduler, delay_recovery, **kwargs):
        super().__init__(robot_id, timetable, scheduler, delay_recovery, **kwargs)

    def task_status_cb(self, msg):
        message = Message(**msg)
        payload = Document.from_payload(message.payload)
        task_progress = payload.get("task_progress")
        if task_progress:
            task_progress = TaskProgress.from_dict(task_progress)
            payload.update(task_progress=task_progress)

        task_status = TaskStatus(**payload)
        timestamp = TimeStamp.from_str(message.timestamp).to_datetime()

        if self.robot_id == task_status.robot_id:

            task = Task.get_task(task_status.task_id)
            self.logger.debug("Received task status %s for task %s", task_status.task_status, task.task_id)

            if task_status.task_status == TaskStatusConst.ONGOING:
                self.update_timetable(task, task_status.task_progress, timestamp)

            elif task_status.task_status == TaskStatusConst.COMPLETED:
                self.logger.debug("Completing execution of task %s", task.task_id)
                self.task = None

            task.update_status(task_status.task_status)

    def task_cb(self, msg):
        payload = msg['payload']
        task = Task.from_payload(payload)
        self.logger.debug("Received task %s", task.task_id)
        task.update_status(TaskStatusConst.DISPATCHED)

    def send_task(self, task):
        self.logger.debug("Sending task %s to executor", task.task_id)
        task_msg = self.api.create_message(task)
        task_msg["payload"].pop("constraints")
        task_msg["payload"]["plan"][0]["_id"] = task_msg["payload"]["plan"][0].pop("robot")
        self.api.publish(task_msg, groups=['ROPOD'])
