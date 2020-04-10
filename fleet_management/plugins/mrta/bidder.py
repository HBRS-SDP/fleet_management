from fleet_management.db.models.actions import GoTo
from fleet_management.db.models.environment import Area
from fleet_management.db.models.robot import Ropod
from fleet_management.db.models.task import TaskPlan
from fleet_management.exceptions.osm import OSMPlannerException
from fmlib.models.tasks import InterTimepointConstraint
from mrs.allocation.bidder import Bidder as BidderBase


class Bidder(BidderBase):
    def __init__(self, robot_id, timetable, bidding_rule, auctioneer_name, **kwargs):
        super().__init__(robot_id, timetable, bidding_rule, auctioneer_name, **kwargs)

    def get_previous_location(self, insertion_point):
        if insertion_point == 1:
            previous_location = Ropod.get_robot(self.robot_id).position.subarea.name
        else:
            previous_task = self.timetable.get_task(insertion_point - 1)
            previous_location = self.get_task_delivery_location(previous_task)
        self.logger.debug("Previous location: %s ", previous_location)
        return previous_location

    def get_task_delivery_location(self, task):
        return self.path_planner.get_sub_area(task.request.delivery_location, behaviour="undocking").name

    def get_travel_duration(self, task, previous_location):
        """ Returns time (mean, variance) to go from previous_location to task.pickup_location
        """
        pickup_subarea = self.path_planner.get_sub_area(task.request.pickup_location, behaviour="docking")

        try:
            self.logger.debug('Planning path between %s and %s', previous_location, pickup_subarea.name)

            areas = self.path_planner.get_path_plan_from_local_area(previous_location, pickup_subarea.name)
            path_plan = list()

            for area in areas:
                model_area = Area(**area.to_dict())
                path_plan.append(model_area)

            action = GoTo.create_new(type="GOTO", areas=path_plan)
            task_plan = TaskPlan(actions=[action])
            mean, variance = self.duration_graph.get_duration(task_plan)

            travel_duration = InterTimepointConstraint(mean=mean, variance=variance)
            self.logger.debug("Travel duration: %s", travel_duration)
            return travel_duration

        except OSMPlannerException:
            self.logger.warning("OSPlanner failed at computing a path between %s and %s", previous_location,
                                pickup_subarea.name)
            return
