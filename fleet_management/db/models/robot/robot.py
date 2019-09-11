import uuid

# from fleet_management.db.models.robot.status import RobotStatus
from fleet_management.db.models.robot.version import Version
from fleet_management.db.queries.sets.robot import RobotManager
from pymodm import MongoModel, fields, EmbeddedMongoModel


class Position(EmbeddedMongoModel):
    area = fields.CharField()
    subarea = fields.CharField()
    x = fields.FloatField()
    y = fields.FloatField()
    theta = fields.FloatField()

    class Meta:
        ignore_unknown_fields = True


class Robot(MongoModel):
    robot_id = fields.CharField(primary_key=True)
    uuid = fields.UUIDField(default=str(uuid.uuid4()))
    version = fields.EmbeddedDocumentField(Version)
    # status = fields.EmbeddedDocumentField(RobotStatus)
    position = fields.EmbeddedDocumentField(Position)

    objects = RobotManager()

    class Meta:
        archive_collection = 'robot_archive'
        ignore_unknown_fields = True



