import logging

from fleet_management.db.models.actions import Action
from fleet_management.db.models.users import User
from fleet_management.db.queries.sets.tasks import TaskManager, TaskStatusManager
from fleet_management.utils.messages import Document
from pymodm import EmbeddedMongoModel, fields, MongoModel
from pymodm.context_managers import switch_collection
from pymongo.errors import ServerSelectionTimeoutError
from ropod.structs.status import TaskStatus as RequestStatus
from ropod.structs.task import TaskPriority, TaskStatus as TaskStatusConst
from ropod.utils.uuid import generate_uuid
from fleet_management.utils.messages import Message


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
    hard_constraints = fields.BooleanField(default=True)
    _task_template = None

    class Meta:
        archive_collection = 'task_request_archive'
        ignore_unknown_fields = True

    def save(self):
        try:
            super().save(cascade=True)
        except ServerSelectionTimeoutError:
            logging.warning('Could not save models to MongoDB')

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
    request = fields.ReferenceField(TaskRequest)
    assigned_robots = fields.ListField()
    actions = fields.DictField()
    constraints = fields.EmbeddedDocumentField(TaskConstraints)
    duration = fields.FloatField()
    start_time = fields.DateTimeField()
    finish_time = fields.DateTimeField()

    objects = TaskManager()

    class Meta:
        archive_collection = 'task_archive'
        ignore_unknown_fields = True

    def save(self):
        try:
            super().save(cascade=True)
        except ServerSelectionTimeoutError:
            logging.warning('Could not save models to MongoDB')

    @classmethod
    def create_new(cls, **kwargs):
        task = cls(**kwargs)
        task.save()
        task.update_status(RequestStatus.UNALLOCATED)
        return task

    @classmethod
    def from_payload(cls, payload):
        document = Document.from_msg(payload)
        document['_id'] = document.pop('task_id')
        task = Task.from_document(document)
        task.save()
        task.update_status(RequestStatus.UNALLOCATED)
        return task

    def to_dict(self):
        dict_repr = self.to_son().to_dict()
        dict_repr.pop('_cls')
        dict_repr["task_id"] = str(dict_repr.pop('_id'))
        return dict_repr

    def to_msg(self):
        msg = Message.from_dict(self.to_dict(), '')
        return msg

    @classmethod
    def from_request(cls, request):
        constraints = TaskConstraints(hard=request.hard_constraints)
        task = cls(request=request.request_id, constraints=constraints)
        task.save()
        task.update_status(RequestStatus.UNALLOCATED)
        return task

    def update_duration(self, duration):
        self.duration = duration
        self.save()

    def archive(self):
        with switch_collection(Task, Task.Meta.archive_collection):
            super().save()
        self.delete()

    def update_status(self, status):
        status = TaskStatus(self.task_id, status)
        status.save()
        if status in [TaskStatusConst.COMPLETED, TaskStatusConst.CANCELED]:
            self.archive()
            status.archive()
            self.save()

    def assign_robots(self, robot_ids):
        self.assigned_robots = robot_ids
        self.save()

    def update_plan(self, robot_id, task_plan):
        self.actions[robot_id] = task_plan
        self.save()

    def update_schedule(self, schedule):
        self.start_time = schedule['start_time']
        self.finish_time = schedule['finish_time']
        self.save()

    def update_constraints(self, constraints):
        pass


class TaskStatus(MongoModel):
    task = fields.ReferenceField(Task, primary_key=True, required=True)
    status = fields.IntegerField(default=RequestStatus.UNALLOCATED)
    delayed = fields.BooleanField(default=False)

    objects = TaskStatusManager()

    class Meta:
        archive_collection = 'task_status_archive'
        ignore_unknown_fields = True

    def archive(self):
        with switch_collection(Task, Task.Meta.archive_collection):
            super().save()
        self.delete()
