from fleet_management.db.models.robot import Ropod
from fmlib.models.tasks import Task
from pymodm import fields


class TransportationTask(Task):
    assigned_robots = fields.EmbeddedDocumentListField(Ropod)
