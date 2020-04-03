from fleet_management.db.models.robot import Ropod
from fmlib.models.tasks import TaskPlan as TaskPlanBase
from fmlib.models.tasks import TransportationTask as Task, TaskManager
from pymodm import fields


class TaskPlan(TaskPlanBase):
    robot = fields.ReferenceField(Ropod)


class TransportationTask(Task):
    assigned_robots = fields.EmbeddedDocumentListField(Ropod)
    plan = fields.EmbeddedDocumentListField(TaskPlan, blank=True)

    objects = TaskManager()

    def to_dict(self):
        dict_repr = super().to_dict()
        robots_dict = dict()
        for robot in self.assigned_robots:
            robots_dict[robot.robot_id] = robot.to_dict()
        dict_repr["assigned_robots"] = robots_dict
        return dict_repr

    @classmethod
    def get_tasks_by_robot(cls, robot_id):
        return [task for task in cls.objects.all() if robot_id in [robot.robot_id for robot in task.assigned_robots]]

    @classmethod
    def get_tasks(cls, robot_id=None, status=None):
        if status:
            tasks = cls.get_tasks_by_status(status)
        else:
            tasks = cls.objects.all()

        tasks_by_robot = [task for task in tasks if robot_id in [robot.robot_id for robot in task.assigned_robots]]

        return tasks_by_robot
