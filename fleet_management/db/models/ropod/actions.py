from fleet_management.db.models.actions import Action
from fleet_management.db.models.ropod.elevator import Elevator
from pymodm import fields


class EnterElevator(Action):
    elevator_id = fields.ReferenceField(Elevator)


class ExitElevator(Action):
    locations = fields.ListField()
    pass


class WaitForElevator(Action):
    pass


class RequestElevator(Action):
    start_floor = fields.IntegerField()
    goal_floor = fields.IntegerField()


class RideElevator(Action):
    level = fields.IntegerField()


class Dock(Action):
    locations = fields.ListField()


class Undock(Action):
    pass