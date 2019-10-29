import uuid
from datetime import datetime
from bson.son import SON

from pymodm import fields, MongoModel, EmbeddedMongoModel
from pymodm.errors import OperationError
from pymodm.queryset import QuerySet
from pymodm.manager import Manager

from fmlib.models.environment import Position as PositionBaseModel


class OSMArea(EmbeddedMongoModel):

    name = fields.CharField()
    id = fields.IntegerField(blank=True)
    type = fields.CharField()

    class Meta:
        ignore_unknown_fields = True


class SubareaQuerySet(QuerySet):

    def get_subarea(self, subarea_id):
        """Return a subarea object matching to a subarea_id.

        :subarea_id: int
        """
        if isinstance(subarea_id, str):
            try:
                subarea_id = int(subarea_id)
            except Exception as e:
                raise OperationError('Expecting subarea_id of type int or string \
                        which is convertible to an int. But found ' + str(subarea_id))

        return self.get({'id': subarea_id})

    def get_subarea_from_name(self, subarea_name):
        """Return a subarea object matching to a subarea_name.

        :subarea_name: str
        """
        return self.get({'_id': subarea_name})

    def get_subarea_by_type(self, subarea_type):
        return self.raw({"type": subarea_type})


SubareaManager = Manager.from_queryset(SubareaQuerySet)


class SubareaReservationQuerySet(QuerySet):

    def get_subarea_reservation(self, reservation_id):
        """Return a subarea_reservation object matching to a subarea_id.
        """
        if isinstance(reservation_id, str):
            reservation_id = uuid.UUID(reservation_id)
        return self.get({'_id': reservation_id})

    def get_subarea_reservations_by_subarea_id(self, subarea_id):
        if isinstance(subarea_id, str):
            try:
                subarea_id = int(subarea_id)
            except Exception as e:
                raise OperationError('Expecting subarea_id of type int or string \
                        which is convertible to an int. But found ' + str(subarea_id))

        subarea = SubArea.get_subarea(subarea_id)
        return self.raw({'subarea': subarea.name})

    def get_subarea_reservations_by_subarea_name(self, subarea_name):
        return self.raw({"subarea": subarea_name})

    def get_future_reservations_of_subarea(self, subarea_id):
        if isinstance(subarea_id, str):
            try:
                subarea_id = int(subarea_id)
            except Exception as e:
                raise OperationError('Expecting subarea_id of type int or string \
                        which is convertible to an int. But found ' + str(subarea_id))

        subarea = SubArea.get_subarea(subarea_id)
        return self.raw({"subarea": subarea.name,
                         "start_time": {"$gte": datetime.now()}})


SubareaReservationManager = Manager.from_queryset(SubareaReservationQuerySet)


class SubArea(OSMArea):
    behaviour = fields.CharField()
    capacity = fields.IntegerField()
    objects = SubareaManager()

    class Meta:
        archive_collection = 'subarea_archive'
        ignore_unknown_fields = True

    @staticmethod
    def get_subarea(subarea_id):
        return SubArea.objects.get_subarea(subarea_id)

    def get_subarea_from_subarea_name(subarea_name):
        return SubArea.objects.get_subarea_from_name(subarea_name)

    @staticmethod
    def get_subarea_by_type(subarea_type):
        return [subarea for subarea in SubArea.objects.get_subarea_by_type(subarea_type)]


class SubareaReservation(MongoModel):
    reservation_id = fields.UUIDField(primary_key=True)
    start_time = fields.DateTimeField()
    end_time = fields.DateTimeField()
    subarea = fields.ReferenceField(SubArea)
    status = fields.CharField()
    robot_id = fields.IntegerField()
    task_id = fields.IntegerField()
    required_capacity = fields.IntegerField()
    objects = SubareaReservationManager()

    class Meta:
        archive_collection = 'subarea_reservation_archive'
        ignore_unknown_fields = True

    @staticmethod
    def get_subarea_reservation(reservation_id):
        return SubareaReservation.objects.get_subarea_reservation(reservation_id)

    @staticmethod
    def get_subarea_reservations_by_subarea_id(subarea_id):
        return [subarea_reservation for subarea_reservation in 
                SubareaReservation.objects.get_subarea_reservations_by_subarea_id(subarea_id)]

    @staticmethod
    def get_subarea_reservations_by_subarea_name(subarea_name):
        return [subarea_reservation for subarea_reservation in 
                SubareaReservation.objects.get_subarea_reservations_by_subarea_name(subarea_name)]

    @staticmethod
    def get_future_reservations_of_subarea(subarea_id):
        return [subarea_reservation for subarea_reservation in 
                SubareaReservation.objects.get_future_reservations_of_subarea(subarea_id)]


class Area(OSMArea):

    floor_number = fields.IntegerField()
    subareas = fields.EmbeddedDocumentListField(SubArea)

    class Meta:
        ignore_unknown_fields = True
        final = True


class Position(PositionBaseModel):

    subarea = fields.EmbeddedDocumentField(SubArea)

    def update_subarea(self, area_name):
        self.subarea = area_name

    def update_position(self, **kwargs):
        self.x = kwargs.get('x')
        self.y = kwargs.get('y')
        self.theta = kwargs.get('theta')

        self.subarea = SubArea(kwargs.get('subarea'))
