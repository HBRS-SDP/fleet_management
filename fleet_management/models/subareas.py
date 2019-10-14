import logging
import uuid
from datetime import timezone, datetime, timedelta

from pymodm import fields, MongoModel
from pymodm.queryset import QuerySet
from pymodm.manager import Manager

class SubareaQuerySet(QuerySet):

    def get_subarea(self, subarea_id):
        """Return a subarea object matching to a subarea_id.
        """
        if isinstance(subarea_id, str):
            try:
                subarea_id = int(subarea_id)
            except Exception as e:
                return None

        return self.get({'_id': subarea_id})

    def get_subarea_by_type(self, subarea_type):
        return self.raw({"subarea_type": subarea_type})

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
                return None

        return self.raw({"subarea_id": subarea_id})

    def get_future_reservations_of_subarea(self, subarea_id):
        if isinstance(subarea_id, str):
            try:
                subarea_id = int(subarea_id)
            except Exception as e:
                return None

        return self.raw({"subarea_id": subarea_id,
                         "start_time": {"$gte": datetime.now()}})

SubareaReservationManager = Manager.from_queryset(SubareaReservationQuerySet)

class Subarea(MongoModel):
    subarea_id = fields.IntegerField(primary_key=True)
    name = fields.CharField()
    subarea_type = fields.CharField()
    capacity = fields.IntegerField()
    objects = SubareaManager()

    class Meta:
        archive_collection = 'subarea_archive'
        ignore_unknown_fields = True

    @staticmethod
    def get_subarea(subarea_id):
        return Subarea.objects.get_subarea(subarea_id)

    @staticmethod
    def get_subarea_by_type(subarea_type):
        return [subarea for subarea in Subarea.objects.get_subarea_by_type(subarea_type)]

class SubareaReservation(MongoModel):
    reservation_id = fields.UUIDField(primary_key=True)
    start_time = fields.DateTimeField()
    end_time = fields.DateTimeField()
    subarea_id = fields.IntegerField()
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
    def get_future_reservations_of_subarea(subarea_id):
        return [subarea_reservation for subarea_reservation in 
                SubareaReservation.objects.get_future_reservations_of_subarea(subarea_id)]

