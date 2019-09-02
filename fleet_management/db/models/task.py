from fleet_management.db.models.actions import Action
from fleet_management.db.models.robot.robot import Robot
from fleet_management.db.models.users import User
from fleet_management.utils.messages import Document
from pymodm import EmbeddedMongoModel, fields, MongoModel
from ropod.structs.status import TaskStatus as RequestStatus
from ropod.structs.task import TaskPriority
from ropod.utils.uuid import generate_uuid


class TaskRequest(MongoModel):
    request_id = fields.UUIDField(primary_key=True)
    pickup_location = fields.CharField()
    delivery_location = fields.CharField()
    earliest_pickup_time = fields.DateTimeField()
    latest_pickup_time = fields.DateTimeField()
    user_id = fields.ReferenceField(User)
    load_type = fields.CharField()
    load_id = fields.CharField()
    priority = fields.IntegerField(default=TaskPriority.NORMAL)

    class Meta:
        archive_collection = 'task_request_archive'
        ignore_unknown_fields = True

    @classmethod
    def from_payload(cls, payload):
        document = Document.from_msg(payload)
        document['_id'] = document.pop('request_id')
        request = TaskRequest.from_document(document)
        request.save()
        return request

    def to_dict(self):
        dict_repr = self.to_son().to_dict()
        dict_repr.pop('_cls')
        dict_repr["request_id"] = str(dict_repr.pop('_id'))
        dict_repr["earliest_pickup_time"] = self.earliest_pickup_time.isoformat()
        dict_repr["latest_pickup_time"] = self.latest_pickup_time.isoformat()
        return dict_repr


class TaskConstraints(EmbeddedMongoModel):
    est = fields.DateTimeField()
    lst = fields.DateTimeField()
    eft = fields.DateTimeField()
    lft = fields.DateTimeField()
    hard = fields.BooleanField(default=True)


class TaskPlan(EmbeddedMongoModel):
    actions = fields.EmbeddedDocumentListField(Action)


class Task(MongoModel):
    task_id = fields.UUIDField(primary_key=True, default=generate_uuid())
    request_id = fields.ReferenceField(TaskRequest)
    assigned_robots = fields.EmbeddedDocumentListField(Robot)
    actions = fields.DictField()
    constraints = fields.EmbeddedDocumentField(TaskConstraints)
    duration = fields.FloatField()
    start_time = fields.DateTimeField()
    finish_time = fields.DateTimeField()

    class Meta:
        archive_collection = 'task_archive'
        ignore_unknown_fields = True

    @classmethod
    def from_payload(cls, payload):
        document = Document.from_msg(payload)
        document['_id'] = document.pop('task_id')
        task = Task.from_document(document)
        task.save()
        return task

    def to_dict(self):
        dict_repr = self.to_son().to_dict()
        dict_repr.pop('_cls')
        dict_repr["task_id"] = str(dict_repr.pop('_id'))
        return dict_repr

    @classmethod
    def from_request(cls, request):
        task = cls(request_id=request.request_id)
        task.save()
        status = TaskStatus(task.task_id)
        status.save()
        return task

    def update_duration(self, duration):
        self.duration = duration
        self.save()

    def update_status(self, status):
        TaskStatus(self.task_id, status).save()


class TaskStatus(MongoModel):
    task_id = fields.ReferenceField(Task, primary_key=True)
    status = fields.IntegerField(default=RequestStatus.UNALLOCATED)
