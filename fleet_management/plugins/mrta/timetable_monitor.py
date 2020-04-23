from mrs.exceptions.allocation import TaskNotFound
from mrs.timetable.monitor import TimetableMonitor as TimetableMonitorBase
from ropod.structs.status import TaskStatus as TaskStatusConst


class TimetableMonitor(TimetableMonitorBase):
    def __init__(self, auctioneer, dispatcher, delay_recovery, **kwargs):
        super().__init__(auctioneer, dispatcher, delay_recovery, **kwargs)

    def remove_task_from_timetable(self, task, status):
        self.logger.debug("Deleting task %s from timetable", task.task_id)
        for robot in task.assigned_robots:
            timetable = self.timetable_manager.get_timetable(robot.robot_id)

            if not timetable.has_task(task.task_id):
                self.logger.warning("Robot %s does not have task %s in its timetable: ", robot.robot_id, task.task_id)
                raise TaskNotFound

            next_task = timetable.get_next_task(task)
            prev_task = timetable.get_previous_task(task)

            if status == TaskStatusConst.COMPLETED and next_task:
                finish_current_task = timetable.stn.get_time(task.task_id, 'delivery', False)
                timetable.stn.assign_earliest_time(finish_current_task, next_task.task_id, 'start', force=True)

            timetable.remove_task(task.task_id)

            if prev_task and next_task:
                self.update_pre_task_constraint(prev_task, next_task, timetable)

            self.logger.debug("STN robot %s: %s", robot.robot_id, timetable.stn)
            self.logger.debug("Dispatchable graph robot %s: %s", robot.robot_id, timetable.dispatchable_graph)
            self.timetable_manager.update({timetable.robot_id: timetable})
            timetable.store()

            self.send_remove_task(task.task_id, status, robot.robot_id)
            self._re_compute_dispatchable_graph(timetable, next_task)
