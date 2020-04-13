from fmlib.models.tasks import TransportationTask as Task
from mrs.execution.schedule_execution_monitor import ScheduleExecutionMonitor as ScheduleExecutionMonitorBase
from ropod.structs.status import TaskStatus as TaskStatusConst


class ScheduleExecutionMonitor(ScheduleExecutionMonitorBase):
    def __init__(self, robot_id, timetable, scheduler, delay_recovery, **kwargs):
        super().__init__(robot_id, timetable, scheduler, delay_recovery, **kwargs)

    def task_cb(self, msg):
        payload = msg['payload']
        task = Task.from_payload(payload)
        if self.robot_id in task.assigned_robots:
            self.logger.debug("Received task %s", task.task_id)
            task.update_status(TaskStatusConst.DISPATCHED)

    def send_task(self, task):
        self.logger.debug("Sending task %s to executor", task.task_id)
        task_msg = self.api.create_message(task)

        task_msg["payload"].pop("constraints")
        task_msg["payload"]["assignedRobots"] = [robot.robot_id for robot in task.assigned_robots]
        task_msg["payload"]["plan"][0]["_id"] = task_msg["payload"]["plan"][0].pop("robot")

        self.api.publish(task_msg, groups=['ROPOD'])
