import datetime

from pymodm import fields, MongoModel


class Message(MongoModel):
    msg_id = fields.UUIDField(primary_key=True)
    timestamp = fields.DateTimeField(default=datetime.datetime.now())
    metamodel = fields.CharField()
    type = fields.CharField()
    payload = fields.DictField()
    receiver_ids = fields.ListField(blank=True)

    class Meta:
        ignore_unknown_fields = True
        collection_name = 'messages'
