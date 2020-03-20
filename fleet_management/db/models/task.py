from fleet_management.db.models.robot import Ropod
from fmlib.models.tasks import Task, TimepointConstraint, TaskConstraints, TemporalConstraints, InterTimepointConstraint
from pymodm import fields


class TransportationTask(Task):
    assigned_robots = fields.EmbeddedDocumentListField(Ropod)

    @classmethod
    def from_request(cls, request):
        pickup_constraint = TimepointConstraint(name="pickup",
                                                earliest_time=request.earliest_pickup_time,
                                                latest_time=request.latest_pickup_time)
        travel_time = InterTimepointConstraint(name="travel_time")
        work_time = InterTimepointConstraint(name="work_time")
        temporal = TemporalConstraints(timepoint_constraints=[pickup_constraint],
                                       inter_timepoint_constraints=[travel_time, work_time])
        constraints = TaskConstraints(hard=request.hard_constraints, temporal=temporal)
        task = cls.create_new(request=request.request_id, constraints=constraints)
        return task
