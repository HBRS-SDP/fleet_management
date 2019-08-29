from pymodm import fields, MongoModel, EmbeddedMongoModel


class ElevatorStatus(EmbeddedMongoModel):
    floor = fields.IntegerField()
    calls = fields.IntegerField()
    is_available = fields.BooleanField()
    door_open_at_goal_floor = fields.BooleanField()
    door_open_at_start_floor = fields.BooleanField()

    class Meta:
        ignore_unknown_fields = True


class Elevator(MongoModel):
    elevator_id = fields.CharField(primary_key=True)
    id = fields.IntegerField()
    status = fields.EmbeddedDocumentField(ElevatorStatus)
