from pymodm.manager import Manager
from pymodm.queryset import QuerySet


class RobotQuerySet(QuerySet):
    def get_robot(self, robot_id):
        """Return all published Posts."""
        return self.get({'_id': robot_id})



RobotManager = Manager.from_queryset(RobotQuerySet)
