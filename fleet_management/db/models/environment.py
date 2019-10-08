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

    def update_area(self, area_name):
        self.area = area_name

    def update_position(self, **kwargs):
        self.x = kwargs.get('x')
        self.y = kwargs.get('y')
        self.theta = kwargs.get('theta')

        self.area = Area(kwargs.get('area'))
