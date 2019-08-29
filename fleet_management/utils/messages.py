import inflection


# Inspired by https://realpython.com/inheritance-composition-python/

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

