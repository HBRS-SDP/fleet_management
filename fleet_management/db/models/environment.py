from pymodm import EmbeddedMongoModel, fields
from fmlib.models.environment import Position as PositionBaseModel


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


class Position(PositionBaseModel):

    area = fields.EmbeddedDocumentField(Area)
