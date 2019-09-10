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
from ropod.utils.timestamp import TimeStamp


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


class TimepointConstraints(EmbeddedMongoModel):
    timepoint_id = fields.UUIDField(primary_key=True, default=generate_uuid())
    earliest_time = fields.DateTimeField()
    latest_time = fields.DateTimeField()

    @staticmethod
    def relative_to_ztp(timepoint, ztp, resolution="minutes"):
        """ Returns the timepoint constraints relative to a ZTP (zero timepoint)

        Args:
            timepoint (TimepointConstraints): timepoint
            ztp (TimeStamp): Zero Time Point. Origin time to which the timepoint will be referenced to
            resolution (str): Resolution of the difference between the timepoint constraints and the ztp

        Return: r_earliest_time (float): earliest time relative to the ztp
                r_latest_time (float): latest time relative to the ztp
        """

        r_earliest_time = TimeStamp.from_datetime(timepoint.earliest_time).get_difference(ztp, resolution)
        r_latest_time = TimeStamp.from_datetime(timepoint.latest_time).get_difference(ztp, resolution)

        return r_earliest_time, r_latest_time

    @classmethod
    def from_payload(cls, payload):
        document = Document.from_msg(payload)
        document['_id'] = document.pop('timepoint_id')
        timepoint_constraints = TimepointConstraints.from_document(document)
        return timepoint_constraints

    def to_dict(self):
        dict_repr = self.to_son().to_dict()
        dict_repr.pop('_cls')
        dict_repr["timepoint_id"] = str(dict_repr.pop('_id'))
        dict_repr["earliest_time"] = self.earliest_time.isoformat()
        dict_repr["latest_time"] = self.latest_time.isoformat()
        return dict_repr


class TaskConstraints(EmbeddedMongoModel):
    hard = fields.BooleanField(default=True)
    timepoint_constraints = fields.EmbeddedDocumentListField(TimepointConstraints)

    @classmethod
    def from_payload(cls, payload):
        document = Document.from_msg(payload)
        timepoint_constraints = [TimepointConstraints.from_payload(timepoint_constraint)
                                 for timepoint_constraint in document.get("timepoint_constraints")]
        document["timepoint_constraints"] = timepoint_constraints
        task_constraints = TaskConstraints.from_document(document)
        return task_constraints

    def to_dict(self):
        dict_repr = self.to_son().to_dict()
        dict_repr.pop('_cls')
        timepoint_constraints = [timepoint_constraint.to_dict() for timepoint_constraint in self.timepoint_constraints]
        dict_repr["timepoint_constraints"] = timepoint_constraints
        return dict_repr


class TaskPlan(EmbeddedMongoModel):
    robot = fields.CharField(primary_key=True)
    actions = fields.EmbeddedDocumentListField(Action)


class Task(MongoModel):
    task_id = fields.UUIDField(primary_key=True, default=generate_uuid())
    request = fields.ReferenceField(TaskRequest)
    assigned_robots = fields.ListField()
    plan = fields.EmbeddedDocumentListField(TaskPlan)
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
        task_status = TaskStatus(task=self.task_id, status=status)
        task_status.save()
        if status in [TaskStatusConst.COMPLETED, TaskStatusConst.CANCELED]:
            self.archive()
            task_status.archive()

    def assign_robots(self, robot_ids):
        self.assigned_robots = robot_ids
        self.save()

    def update_plan(self, robot_id, task_plan):
        # This might not work for tasks with multiple robots
        for robot in robot_id:
            task_plan.robot = robot
            self.plan.append(task_plan)
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
