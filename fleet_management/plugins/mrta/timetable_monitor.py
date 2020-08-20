from mrs.exceptions.allocation import TaskNotFound
from mrs.exceptions.execution import EmptyTimetable
from mrs.timetable.monitor import TimetableMonitor as TimetableMonitorBase
from mrs.timetable.monitor import TimetableMonitorProxy as TimetableMonitorProxyBase


class TimetableMonitor(TimetableMonitorBase):
    def __init__(self, auctioneer, delay_recovery, **kwargs):
        super().__init__(auctioneer, delay_recovery, **kwargs)

    def remove_task(self, task, status):
        robot_ids = [robot.robot_id for robot in task.assigned_robots]
        for robot_id in robot_ids:
            try:
                timetable = self.get_timetable(robot_id)
                next_task = timetable.get_next_task(task)
                self.remove_task_from_timetable(timetable, task, status, next_task)
                task.update_status(status)
                self.auctioneer.changed_timetable.append(timetable.robot_id)
                self.send_remove_task(task.task_id, status, robot_id)
                self._re_compute_dispatchable_graph(timetable, next_task)
            except (TaskNotFound, EmptyTimetable):
                return


class TimetableMonitorProxy(TimetableMonitorProxyBase):
    def __init__(self, robot_id, bidder, **kwargs):
        super().__init__(robot_id, bidder, **kwargs)

    def remove_task(self, task, status):
        try:
            timetable = self.get_timetable(self.robot_id)
            next_task = timetable.get_next_task(task)
            self.remove_task_from_timetable(timetable, task, status, next_task)
            task.update_status(status)
            self.bidder.changed_timetable = True
            self._re_compute_dispatchable_graph(timetable, next_task)
        except (TaskNotFound, EmptyTimetable):
            return
