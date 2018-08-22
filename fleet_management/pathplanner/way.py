class Way(object):
    def __init__(self, elm):
        self.element = elm
        self.tags = self.element.get('tags')
        self.nodes = []

    @property
    def id(self):
        return self.element.get('id')

    @property
    def highway(self):
        return self.tags.get('highway')

    @property
    def oneway(self):
        return True if self.tags.get('oneway') == "yes" else False

    def __eq__(self, other):
        return other is not None and self.name == other.name

    def __repr__(self):
        return "<Way id=%(id)s, highway=%(highway)s, oneway=%(oneway)s>" % {
            'id': self.id,
            'highway': self.highway,
            'oneway': self.oneway,
        }