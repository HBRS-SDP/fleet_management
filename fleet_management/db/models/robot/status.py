from fleet_management.db.models.task import Task
from fleet_management.db.models.actions import Action
from pymodm import EmbeddedMongoModel, fields
from ropod.structs.status import AvailabilityStatus


class ComponentStatus(EmbeddedMongoModel):
    pass


class CurrentTask(EmbeddedMongoModel):
    status = fields.CharField()
    task_id = fields.ReferenceField(Task)
    action_id = fields.ReferenceField(Action)


class Availability(EmbeddedMongoModel):
    status = fields.IntegerField(default=AvailabilityStatus.NO_COMMUNICATION, blank=True)
    current_task = fields.EmbeddedDocumentField(CurrentTask, default=None, blank=True)


class RobotStatus(EmbeddedMongoModel):
    availability = fields.EmbeddedDocumentField(Availability)
    component_status = fields.EmbeddedDocumentField(ComponentStatus)
