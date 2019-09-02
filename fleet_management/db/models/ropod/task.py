from fleet_management.db.models.ropod.ropod import Ropod
from fleet_management.db.models.task import Task
from pymodm import fields


class TransportationTask(Task):
    assigned_robots = fields.EmbeddedDocumentListField(Ropod)
