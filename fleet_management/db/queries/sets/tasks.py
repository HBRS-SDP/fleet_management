import uuid
from pymodm.manager import Manager
from pymodm.queryset import QuerySet
from ropod.structs.status import TaskStatus


class TaskQuerySet(QuerySet):

    def get_task(self, task_id):
        """Return a task object matching to a task_id.
        """
        return self.get({'_id': uuid.UUID(task_id)})


class TaskStatusQuerySet(QuerySet):

    def by_status(self, status):
        return self.raw({"status": status})

    def unallocated(self):
        return self.raw({"status": TaskStatus.UNALLOCATED})

    def allocated(self):
        return self.raw({"status": TaskStatus.ALLOCATED})

    def planned(self):
        return self.raw({"status": TaskStatus.PLANNED})

    def scheduled(self):
        return self.raw({"status": TaskStatus.SCHEDULED})

    def shipped(self):
        return self.raw({"status": TaskStatus.SHIPPED})

    def ongoing(self):
        return self.raw({"status": TaskStatus.ONGOING})

    def completed(self):
        return self.raw({"status": TaskStatus.COMPLETED})

    def aborted(self):
        return self.raw({"status": TaskStatus.ABORTED})

    def failed(self):
        return self.raw({"status": TaskStatus.FAILED})

    def canceled(self):
        return self.raw({"status": TaskStatus.CANCELED})

    def preempted(self):
        return self.raw({"status": TaskStatus.PREEMPTED})


TaskManager = Manager.from_queryset(TaskQuerySet)
TaskStatusManager = Manager.from_queryset(TaskStatusQuerySet)
