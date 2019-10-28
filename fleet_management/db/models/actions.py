from fleet_management.db.models.elevator import Elevator
from fleet_management.db.models.environment import Area
from fmlib.models.actions import Action
from pymodm import fields


class GoTo(Action):
    areas = fields.EmbeddedDocumentListField(Area)


class EnterElevator(Action):
    elevator_id = fields.ReferenceField(Elevator)


class ExitElevator(Action):
    areas = fields.ListField()


class WaitForElevator(Action):
    pass


class RequestElevator(Action):
    start_floor = fields.IntegerField()
    goal_floor = fields.IntegerField()


class RideElevator(Action):
    level = fields.IntegerField()


class Dock(Action):
    areas = fields.EmbeddedDocumentListField(Area)


class Undock(Action):
    pass
