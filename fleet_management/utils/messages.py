# Inspired by https://realpython.com/inheritance-composition-python/
import json

import inflection
from ropod.utils.timestamp import TimeStamp
from ropod.utils.uuid import generate_uuid

meta_model_template = 'ropod-%s-schema.json'


class Message(dict):

    def __init__(self, payload, header=None, **kwargs):
        super().__init__()

        if header:
            self.update(header=header)
        else:
            msg_type = kwargs.get('message_type', '')
            self.update(header=Message.create_header(msg_type))

        self.update(payload=payload)

    def __str__(self):
        return json.dumps(self, indent=2)

    @property
    def type(self):
        if self.get('header'):
            return self.get('header').get('type')
        else:
            return None

    @property
    def payload(self):
        return self.get('payload')

    @property
    def header(self):
        return self.get('header')

    @classmethod
    def create_payload(cls, contents, model):
        payload = cls._from_dict(contents.to_dict())
        metamodel = meta_model_template % model
        payload.update(metamodel=metamodel)
        return payload

    @staticmethod
    def create_header(message_type, meta_model='msg', **kwargs):
        recipients = kwargs.get('recipients', list())
        if recipients is not None and not isinstance(recipients, list):
            raise Exception("Recipients must be a list of strings")

        return {'type': message_type,
                'metamodel': meta_model_template % meta_model,
                'msgId': str(generate_uuid()),
                'timestamp': TimeStamp().to_str(),
                'receiverIds': recipients}

    @classmethod
    def from_dict(cls, payload, message_type, meta_model="msg"):
        header = cls.create_header(message_type)
        payload = cls._from_dict(payload)
        payload.update(metamodel=meta_model_template % meta_model)
        return cls(payload, header)

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


class AsDictionaryMixin:
    def to_dict(self):
        return {
            prop: self._represent(value)
            for prop, value in self.__dict__.items()
            if not self._is_internal(prop)
        }

    def _represent(self, value):
        if isinstance(value, object):
            if hasattr(value, 'to_dict'):
                return value.to_dict()
            else:
                return str(value)
        else:
            return value

    def _is_internal(self, prop):
        return prop.startswith('_')
