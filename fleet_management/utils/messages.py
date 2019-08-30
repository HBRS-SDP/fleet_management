import inflection


# Inspired by https://realpython.com/inheritance-composition-python/

class Message:

    @classmethod
    def from_dict(cls, msg):
        return {inflection.camelize(prop, False): cls._format_dict(value)
                for prop, value in msg.items()}

    @classmethod
    def _format_dict(cls, value):
        if isinstance(value, dict):
            return cls.from_dict(value)
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

