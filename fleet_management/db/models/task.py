from pymodm import MongoModel, fields, EmbeddedMongoModel


class Action(EmbeddedMongoModel):
    action_id = fields.UUIDField(primary_key=True)


class Task(MongoModel):
    task_id = fields.UUIDField(primary_key=True)
    actions = fields.EmbeddedDocumentListField(Action)


class TaskStatus(MongoModel):
    task_id = fields.ReferenceField(Task)
