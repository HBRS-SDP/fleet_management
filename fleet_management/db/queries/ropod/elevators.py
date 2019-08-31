from pymodm.manager import Manager
from pymodm.queryset import QuerySet


class ElevatorRequestQuerySet(QuerySet):
    def get_query(self, query_id):
        """Return all published Posts."""
        return self.get({'_id': query_id})


ElevatorRequestManager = Manager.from_queryset(ElevatorRequestQuerySet)
