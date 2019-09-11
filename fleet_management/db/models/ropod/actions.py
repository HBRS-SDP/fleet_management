from fleet_management.db.models.actions import Action
from fleet_management.db.models.ropod.elevator import Elevator
from pymodm import fields, MongoModel, EmbeddedMongoModel


class OSMArea(EmbeddedMongoModel):
    name = fields.CharField()
    id = fields.CharField()
    type = fields.CharField()

    class Meta:
        ignore_unknown_fields = True


class SubArea(OSMArea):
    capacity = fields.CharField()

    class Meta:
        ignore_unknown_fields = True
        final = True


class Area(OSMArea):
    floor_number = fields.IntegerField()
    subareas = fields.EmbeddedDocumentListField(SubArea)

    class Meta:
        ignore_unknown_fields = True
        final = True


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
    area = fields.ListField()


class Undock(Action):
    pass
