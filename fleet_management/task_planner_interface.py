import uuid
from termcolor import colored
from ropod.structs.task import TaskRequest
from task_planner.knowledge_base_interface import KnowledgeBaseInterface
from task_planner.metric_ff_interface import MetricFFInterface
from fleet_management.path_planner import FMSPathPlanner

class TaskPlannerInterface(object):
    '''An interface for generating ROPOD task plans.

    @author Alex Mitrevski
    @maintainer Alex Mitrevski, Argentina Ortega Sainz
    @contact aleksandar.mitrevski@h-brs.de, argentina.ortega@h-brs.de
    '''
    def __init__(self, planner_params):
        self.kb_interface = KnowledgeBaseInterface(planner_params.kb_database_name)
        self.planner_interface = MetricFFInterface(planner_params.kb_database_name,
                                                   planner_params.domain_file,
                                                   planner_params.planner_cmd,
                                                   planner_params.plan_file_path)

    def get_task_plan_without_robot(self, task_request: TaskRequest,
                                    path_planner: FMSPathPlanner):
        '''Generates a task plan based on the given task request and
        returns a list of ropod.structs.action.Action objects
        representing the plan's actions

        @param task_request -- task request parameters
        @param path_planner -- an interface to a path planner used for planning
                               paths once a task plan is obtained

        '''
        # at this point, we don't know which robot will be
        # used for the task, so we plan for a dummy robot
        robot_name = 'dummy_robot_{0}'.format(str(uuid.uuid4()))

        # cart IDs come as numbers, so we append "cart_" in front
        # in order to make the name a valid ground value
        cart_id = 'cart_' + task_request.cart_id

        # we want to plan from the pickup location to the delivery location,
        # so we assume that the robot is already there
        self.kb_interface.insert_facts([('robot_at', [('bot', robot_name),
                                                      ('loc', task_request.pickup_pose.name)]),
                                        ('cart_at', [('cart', cart_id),
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
                                          ('cart_floor', [('cart', cart_id)],
                                           task_request.pickup_pose.floor_number)])

        actions = []
        try:
            # we set the task goals based on the task request
            task_goals = []
            if task_request.cart_type == 'mobidik':
                task_goals = [('cart_at', [('cart', cart_id),
                                           ('loc', task_request.delivery_pose.name)]),
                              ('empty_gripper', [('bot', robot_name)])]
            elif task_request.cart_type == 'sickbed':
                # TBD
                pass

            # we get the action plan
            plan_found, actions = self.planner_interface.plan(task_request,
                                                              robot_name,
                                                              task_goals)
            if plan_found:
                for action in actions:
                    print("Action added: ", action.type)
            else:
                print(colored('[task_planner_interface] Task plan could not be found', 'warning'))
                return []
        except Exception as exc:
            print(colored('[task_planner_inteface] A plan could not be created: {0}'.format(str(exc)), 'red'))
            return actions

        # we remove the location of the dummy robot from the knowledge base
        self.kb_interface.remove_facts([('robot_at', [('bot', robot_name),
                                                      ('loc', task_request.pickup_pose.name)]),
                                        ('empty_gripper', [('bot', robot_name)])])

        task_plan_with_paths = self.__plan_paths(actions, path_planner)
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
        previous_area = ''
        if task_plan[0].areas:
            previous_area = task_plan[0].areas[-1]
        previous_sub_area = ''

        for i in range(len(task_plan)):
            print('++++++++++++++++++++++++++++++++++++++++++++++++++++++++')
            print(colored('[task_planner_interface] Action {0}: {1}'.format(i, task_plan[i].type), 'green'))
            action = task_plan[i]

            # we don't have areas for elevator actions, so we simply
            # add such actions to the list without any modifications
            if action.type.lower().find('elevator') != -1:
                task_plan_with_paths.append(action)
            # actions such as docking and undocking have areas included,
            # so we want to keep those
            elif action.type.lower().find('goto') == -1:
                previous_area = action.areas[0]
                previous_sub_area = path_planner.get_sub_area(action.areas[0].name,
                                                              behaviour=path_planner.task_to_behaviour(action.type))
                task_plan_with_paths.append(action)
            # we plan a path for GOTO actions
            else:
                next_sub_area = path_planner.get_sub_area(task_plan[i].areas[0].name,
                                                          behaviour=path_planner.task_to_behaviour(
                                                              task_plan[i].type))

                destination = action.areas[0]
                print(colored('[task_planner_interface] Planning path between ' +
                              previous_sub_area.name + ' and ' + next_sub_area.name, 'green'))
                try:
                    path_plan = path_planner.get_path_plan(start_floor=previous_area.floor_number,
                                                           destination_floor=destination.floor_number,
                                                           start_area=previous_area.name,
                                                           destination_area=destination.name,
                                                           start_local_area=previous_sub_area.name,
                                                           destination_local_area=next_sub_area.name)
                except Exception as e:
                    print(colored("Task planning failed | Error: {0}".format(str(e)), 'red'))
                    return None

                action.areas = path_plan
                task_plan_with_paths.append(action)

                print('[task_planner_interface] Path plan length: ', len(path_plan))
                print('[task_planner_interface] Sub areas: ')
                for area in path_plan:
                    for sub_area in area.sub_areas:
                        print(sub_area.name)

        return task_plan_with_paths
