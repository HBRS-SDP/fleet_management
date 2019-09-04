from fleet_management.db.models.task import Task, TaskStatus


def get_task(task_id):
    return Task.objects.get_task(task_id)

