import datetime

from pymodm import fields, MongoModel


class Message(MongoModel):
    msg_id = fields.UUIDField(primary_key=True)
    timestamp = fields.DateTimeField(blank=True)
    metamodel = fields.CharField(blank=True)
    type = fields.CharField(blank=True)
    payload = fields.DictField(blank=True)
    receiver_ids = fields.ListField(blank=True)

    class Meta:
        ignore_unknown_fields = True
        collection_name = 'messages'
