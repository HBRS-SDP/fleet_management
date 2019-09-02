from pymodm import EmbeddedMongoModel, fields
from ropod.utils.uuid import generate_uuid


class Action(EmbeddedMongoModel):
    action_id = fields.UUIDField(primary_key=True, default=generate_uuid())
    type = fields.CharField()


class GoToAction(Action):
    locations = fields.ListField()


class ElevatorAction(Action):
    start_floor = fields.IntegerField()
    goal_floor = fields.IntegerField()
