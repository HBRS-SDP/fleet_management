# Inspired by https://realpython.com/inheritance-composition-python/
import inflection
from ropod.utils.timestamp import TimeStamp
from ropod.utils.uuid import generate_uuid

meta_model_template = 'ropod-%s-schema.json'


class Message:

    def __init__(self, contents, message_type):
        self.__dict__.update(header=Message.create_header(message_type))
        self.__dict__.update(payload=contents)

    @classmethod
    def create_payload(cls, contents, model):
        payload = cls._from_dict(contents.to_dict())
        metamodel = meta_model_template % model
        payload.update(metamodel=metamodel)
        return {"payload": payload}

    @staticmethod
    def create_header(message_type, meta_model='msg', **kwargs):
        recipients = kwargs.get('recipients', list())
        if recipients is not None and not isinstance(recipients, list):
            raise Exception("Recipients must be a list of strings")

        return {"header": {'type': message_type,
                           'metamodel': meta_model_template % meta_model,
                           'msgId': generate_uuid(),
                           'timestamp': TimeStamp().to_str(),
                           'receiverIds': recipients}}

    @classmethod
    def from_dict(cls, payload, message_type, meta_model="msg"):
        msg = cls.create_header(message_type)
        contents = cls._from_dict(payload)
        contents.update(metamodel=meta_model_template % meta_model)
        msg.update(payload=contents)
        return msg

    @classmethod
    def _from_dict(cls, value):
        return {inflection.camelize(prop, False): cls._format_dict(value)
                for prop, value in value.items()}

    @classmethod
    def _format_dict(cls, value):
        if isinstance(value, dict):
            return cls._from_dict(value)
        else:
            return value


class Document(dict):

    @classmethod
    def from_msg(cls, payload):
        return {inflection.underscore(prop): cls._format_dict(value)
                for prop, value in payload.items()}

    @classmethod
    def _format_dict(cls, value):
        if isinstance(value, dict):
            return cls.from_msg(value)
        else:
            return value

