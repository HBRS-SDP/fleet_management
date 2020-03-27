import logging
import uuid

from fleet_management.db.models import actions
from fleet_management.exceptions.osm import OSMPlannerException
from fleet_management.exceptions.planning import NoPlanFound
from fmlib.models.tasks import TaskPlan
from fmlib.utils.messages import Message
from ropod.structs.area import Area, SubArea
from ropod.structs.task import TaskRequest
from task_planner.knowledge_base_interface import KnowledgeBaseInterface
from task_planner.lama_interface import LAMAInterface


class TaskPlannerInterface(object):
    """An interface for generating ROPOD task plans.

    .. codeauthor:: Alex Mitrevski <aleksandar.mitrevski@h-brs.de>
    """

    def __init__(self, kb_database_name, domain_file, planner_cmd, plan_file_path, **_):
        self.logger = logging.getLogger('fms.task.planner.interface')

        self.kb_interface = KnowledgeBaseInterface(kb_database_name)

        # we initialize the knowledge base with some common knowledge,
        # such as the locations of the elevators in the environment
        initialize_knowledge_base(kb_database_name)

        self.logger.info("Configured knowledge base...")
        self.planner_interface = LAMAInterface(kb_database_name=kb_database_name,
                                               domain_file=domain_file,
                                               planner_cmd=planner_cmd,
                                               plan_file_path=plan_file_path)

        self.logger.info("Configured task planner")

    def plan(self, request, path_planner):
        """Temporary solution to translate between the TaskRequest model and
        the existing TaskRequest struct
        """
        formatted_dict = Message.from_model(request).get('payload')
        formatted_dict["pickupLocationLevel"] = self._get_location_floor(formatted_dict.get('pickupLocation'))
        formatted_dict["deliveryLocationLevel"] = self._get_location_floor(formatted_dict.get('deliveryLocation'))
        task_request = TaskRequest.from_dict(formatted_dict)

        plan = self._get_task_plan_without_robot(task_request, path_planner)

        task_plan = TaskPlan()
        for action in plan:
            if action.type == "DOCK":
                model = actions.Dock
            elif action.type == "UNDOCK":
                model = actions.Undock
            elif action.type == "GOTO":
                model = actions.GoTo
            elif action.type == "REQUEST_ELEVATOR":
                model = actions.RequestElevator
            elif action.type == "ENTER_ELEVATOR":
                model = actions.EnterElevator
            elif action.type == "WAIT_FOR_ELEVATOR":
                model = actions.WaitForElevator
            elif action.type == "RIDE_ELEVATOR":
                model = actions.RideElevator
            elif action.type == "EXIT_ELEVATOR":
                model = actions.ExitElevator
            else:
                self.logger.warning("Invalid action of type %s", action.type)
                continue

            a = model.from_document(action.to_dict())
            task_plan.actions.append(a)
        return task_plan

    def _get_task_plan_without_robot(self, task_request: TaskRequest, path_planner):
        """Generates a task plan based on the given task request and
        returns a list of ropod.structs.action.Action objects
        representing the plan's actions

        Args:
            task_request: task request parameters
            path_planner: an interface to a path planner used for planning paths
                          once a task plan is obtained

        """
        # at this point, we don't know which robot will be
        # used for the task, so we plan for a dummy robot
        robot_name = 'dummy_robot_{0}'.format(str(uuid.uuid4()))

        # load IDs come as numbers, so we append "load_" in front
        # in order to make the name a valid ground value
        load_id = 'load_' + task_request.load_id

        # we want to plan from the pickup location to the delivery location,
        # so we assume that the robot is already there; note that the locations of the
        # robot and the cart are inserted in the knowledge base as temporal fluents,
        # while the gripper state of the robot is inserted as a fact
        robot_location_fluent = ('robot_at', [('bot', robot_name)],
                                 task_request.pickup_pose.name)

        cart_location_fluent = ('load_at', [('load', load_id)],
                                task_request.pickup_pose.name)

        gripper_state_fact = ('empty_gripper', [('bot', robot_name)])

        self.kb_interface.insert_facts([gripper_state_fact])
        self.kb_interface.insert_fluents([robot_location_fluent,
                                          cart_location_fluent])

        # the floors of the locations and the elevators are
        # inserted in the knowledge base as fluents
        pickup_pose_floor_fluent = ('location_floor',
                                    [('loc', task_request.pickup_pose.name)],
                                    'floor{0}'.format(task_request.pickup_pose.floor_number))

        delivery_pose_floor_fluent = ('location_floor',
                                      [('loc', task_request.delivery_pose.name)],
                                      'floor{0}'.format(task_request.delivery_pose.floor_number))

        robot_floor_fluent = ('robot_floor', [('bot', robot_name)],
                              'floor{0}'.format(task_request.pickup_pose.floor_number))

        load_floor_fluent = ('load_floor', [('load', load_id)],
                             'floor{0}'.format(task_request.pickup_pose.floor_number))

        self.kb_interface.insert_fluents([pickup_pose_floor_fluent,
                                          delivery_pose_floor_fluent,
                                          robot_floor_fluent,
                                          load_floor_fluent])

        actions_ = []
        try:
            # we set the task goals based on the task request
            task_goals = []
            if task_request.load_type == 'mobidik':
                task_goals = [('load_at', [('load', load_id),
                                           ('loc', task_request.delivery_pose.name)]),
                              ('empty_gripper', [('bot', robot_name)])]
            elif task_request.load_type == 'sickbed':
                # TBD
                pass

            # we get the action plan
            plan_found, actions_ = self.planner_interface.plan(task_request,
                                                               robot_name,
                                                               task_goals)
            if plan_found:
                for action in actions_:
                    self.logger.debug("Action %s added: %s", action.id, action.type)
            else:
                self.logger.warning('Task plan could not be found')
                raise NoPlanFound(task_request.id)
        except Exception as exc:
            self.logger.error('A plan could not be created: %s', str(exc))
            raise NoPlanFound(task_request.id, cause=exc) from exc

        # we remove the location of the dummy robot and
        # the gripper state from the knowledge base
        self.kb_interface.remove_facts([gripper_state_fact])
        self.kb_interface.remove_fluents([robot_location_fluent, robot_floor_fluent])

        try:
            task_plan_with_paths = self._plan_paths(actions_, path_planner)
        except OSMPlannerException:
            raise
        except Exception as e:
            self.logger.error(str(e))
            raise
        return task_plan_with_paths

    def _get_location_floor(self, location):
        """Return the floor number of a given location.
        For ROPOD, this can either be done through the OSM path planner or
        by parsing an Area string

        Args:
            location: An Area string

        Returns:
            floor (int): The floor number of an area
        """
        return int(location.split('_')[2].replace('L', ''))

    def _plan_paths(self, task_plan: list, path_planner):
        """Plans paths between the areas involved in the task plan. Returns
        the list of task actions in ``task_plan`` with added paths between
        the areas involved in the plan.

        Args:
            task_plan (list): a list of ropod.structs.action.Action objects
            path_planner: an interface to a path planner

        """
        task_plan_with_paths = list()

        previous_area = Area()
        if task_plan[0].areas:
            previous_area = task_plan[0].areas[-1]
        previous_sub_area = SubArea()

        # we assume that the last action of a plan is never a GOTO action
        for i in range(len(task_plan)):
            self.logger.debug('++++++++++++++++++++++++++++++++++++++++++++++++++++++++')
            self.logger.debug('Action %s: %s', i, task_plan[i].type)
            action = task_plan[i]

            if action.type.lower() == 'exit_elevator':
                previous_area = action.areas[0]
                previous_sub_area = path_planner.get_sub_area(action.areas[0].name,
                                                              behaviour=path_planner.task_to_behaviour(action.type))

                # we set the destination level of the previous action,
                # which is assumed to be RIDE_ELEVATOR, to the level
                # of the area in the current EXIT_ELEVATOR action
                task_plan_with_paths[-1].level = previous_area.floor_number
                task_plan_with_paths.append(action)
            # we don't have areas for other elevator actions, so we simply
            # add such actions to the list without any modifications
            elif action.type.lower().find('elevator') != -1:
                task_plan_with_paths.append(action)
            # actions such as docking and undocking have areas included,
            # so we want to keep those
            elif action.type.lower().find('goto') == -1:
                if action.areas:
                    previous_area = action.areas[0]
                    previous_sub_area = path_planner.get_sub_area(action.areas[0].name,
                                                                  behaviour=path_planner.task_to_behaviour(
                                                                      action.type))
                task_plan_with_paths.append(action)
            # we plan a path for GOTO actions
            else:
                next_sub_area = path_planner.get_sub_area(task_plan[i].areas[0].name,
                                                          behaviour=path_planner.task_to_behaviour(
                                                              task_plan[i + 1].type))

                destination = action.areas[0]
                self.logger.debug('Planning path between %s and %s ', previous_sub_area.name, next_sub_area.name)
                try:
                    path_plan = path_planner.get_path_plan(start_floor=previous_area.floor_number,
                                                           destination_floor=destination.floor_number,
                                                           start_area=previous_area.name,
                                                           destination_area=destination.name,
                                                           start_local_area=previous_sub_area.name,
                                                           destination_local_area=next_sub_area.name)
                except Exception as e:
                    self.logger.error("Path planner error", exc_info=True)
                    raise OSMPlannerException("Task planning failed") from e

                action.areas = path_plan
                task_plan_with_paths.append(action)

                self.logger.debug('Path plan length: %i', len(path_plan))
                self.logger.debug('Sub areas: ')
                for area in path_plan:
                    for sub_area in area.sub_areas:
                        self.logger.debug(sub_area.name)

        return task_plan_with_paths


def initialize_knowledge_base(kb_database_name):
    kb_interface = KnowledgeBaseInterface(kb_database_name)

    print('[initialize_knowledge_base] Initializing elevators')
    # TODO: Use the actual areas where the elevators are
    elevator_facts = [('elevator_at', [('elevator', 'elevator0'),
                                       ('loc', 'BRSU_A_L0_A8')]),
                      ('elevator_at', [('elevator', 'elevator0'),
                                       ('loc', 'BRSU_A_L2_A1')])]
    kb_interface.insert_facts(elevator_facts)

    elevator_fluents = [('elevator_floor', [('elevator', 'elevator0')], 'floor100'),
                        ('destination_floor', [('elevator', 'elevator0')], 'floor100')]
    kb_interface.insert_fluents(elevator_fluents)

    elevator_location_fluents = [('location_floor', [('loc', 'BRSU_A_L0_A8')], 'floor0'),
                                 ('location_floor', [('loc', 'BRSU_A_L2_A1')], 'floor2')]
    kb_interface.insert_fluents(elevator_location_fluents)
