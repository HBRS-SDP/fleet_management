from fleet_management.db.models.environment import Area
from pymodm import EmbeddedMongoModel, fields, MongoModel


class PathPlan(MongoModel, EmbeddedMongoModel):
    areas = fields.EmbeddedDocumentListField(Area)
    mean = fields.FloatField()
    variance = fields.FloatField()
