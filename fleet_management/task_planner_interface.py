import logging
import uuid

from ropod.structs.area import Area, SubArea
from ropod.structs.task import TaskRequest
from task_planner.knowledge_base_interface import KnowledgeBaseInterface
from task_planner.metric_ff_interface import MetricFFInterface

from fleet_management.db.init_db import initialize_knowledge_base
from fleet_management.exceptions.osm_planner_exception import OSMPlannerException
from fleet_management.path_planner import FMSPathPlanner


class TaskPlannerInterface(object):
    '''An interface for generating ROPOD task plans.

    @author Alex Mitrevski
    @maintainer Alex Mitrevski, Argentina Ortega Sainz
    @contact aleksandar.mitrevski@h-brs.de, argentina.ortega@h-brs.de
    '''
    def __init__(self, planner_params):
        self.logger = logging.getLogger('fms.task.planner.interface')

        self.kb_interface = KnowledgeBaseInterface(planner_params.get('kb_database_name'))

        # we initialize the knowledge base with some common knowledge,
        # such as the locations of the elevators in the environment
        initialize_knowledge_base(planner_params.get('kb_database_name'))

        self.logger.info("Configured knowledge base...")
        self.planner_interface = MetricFFInterface(kb_database_name=planner_params.get('kb_database_name'),
                                                   domain_file=planner_params.get('domain_file'),
                                                   planner_cmd=planner_params.get('planner_cmd'),
                                                   plan_file_path=planner_params.get('plan_file_path'))

        self.logger.info("Configured task planner")

    def get_task_plan_without_robot(self, task_request: TaskRequest,
                                    path_planner: FMSPathPlanner):
        '''Generates a task plan based on the given task request and
        returns a list of ropod.structs.action.Action objects
        representing the plan's actions

        @param task_request -- task request parameters
        @param path_planner -- an interface to a path planner used for planning paths once a task plan is obtained

        '''
        # at this point, we don't know which robot will be
        # used for the task, so we plan for a dummy robot
        robot_name = 'dummy_robot_{0}'.format(str(uuid.uuid4()))

        # load IDs come as numbers, so we append "load_" in front
        # in order to make the name a valid ground value
        load_id = 'load_' + task_request.load_id

        # we want to plan from the pickup location to the delivery location,
        # so we assume that the robot is already there
        self.kb_interface.insert_facts([('robot_at', [('bot', robot_name),
                                                      ('loc', task_request.pickup_pose.name)]),
                                        ('load_at', [('load', load_id),
                                                     ('loc', task_request.pickup_pose.name)]),
                                        ('empty_gripper', [('bot', robot_name)])])

        self.kb_interface.insert_fluents([('location_floor',
                                           [('loc', task_request.pickup_pose.name)],
                                           task_request.pickup_pose.floor_number),
                                          ('location_floor',
                                           [('loc', task_request.delivery_pose.name)],
                                           task_request.delivery_pose.floor_number),
                                          ('robot_floor', [('bot', robot_name)],
                                           task_request.pickup_pose.floor_number),
                                          ('load_floor', [('load', load_id)],
                                           task_request.pickup_pose.floor_number)])

        actions = []
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
            plan_found, actions = self.planner_interface.plan(task_request,
                                                              robot_name,
                                                              task_goals)
            if plan_found:
                for action in actions:
                    self.logger.debug("Action added: %s", action.type)
            else:
                self.logger.warning('Task plan could not be found' )
                return []
        except Exception as exc:
            self.logger.error('A plan could not be created: %s', str(exc))
            return actions

        # we remove the location of the dummy robot from the knowledge base
        self.kb_interface.remove_facts([('robot_at', [('bot', robot_name),
                                                      ('loc', task_request.pickup_pose.name)]),
                                        ('empty_gripper', [('bot', robot_name)])])

        try:
            task_plan_with_paths = self.__plan_paths(actions, path_planner) 
        except Exception as e:
            self.logger.error(str(e))
            raise OSMPlannerException(str(e))
        return task_plan_with_paths

    def __plan_paths(self, task_plan: list, path_planner: FMSPathPlanner):
        '''Plans paths between the areas involved in the task plan. Returns
        the list of task actions in "task_plan" with added paths between
        the areas involved in the plan.

        @param task_plan -- a list of ropod.structs.action.Action objects
        @param path_planner -- an interface to a path planner

        '''
        task_plan_with_paths = []
        task_plan_with_paths.append(task_plan[0])
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
                                                                  behaviour=path_planner.task_to_behaviour(action.type))
                task_plan_with_paths.append(action)
            # we plan a path for GOTO actions
            else:
                next_sub_area = path_planner.get_sub_area(task_plan[i].areas[0].name,
                                                          behaviour=path_planner.task_to_behaviour(
                                                              task_plan[i+1].type))

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
                    self.logger.error("Task planning failed | Error: %s", e)
                    return None

                action.areas = path_plan
                task_plan_with_paths.append(action)

                self.logger.debug('Path plan length: %i', len(path_plan))
                self.logger.debug('Sub areas: ')
                for area in path_plan:
                    for sub_area in area.sub_areas:
                        self.logger.debug(sub_area.name)

        return task_plan_with_paths
