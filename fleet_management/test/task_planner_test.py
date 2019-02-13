from __future__ import print_function
import time
from ropod.structs.task import TaskRequest, Task
from fleet_management.task_planner import TaskPlanner
from fleet_management.path_planner import FMSPathPlanner

if __name__ == '__main__':
    task_request = TaskRequest()
    task_request.user_id = 1
    task_request.load_type = 'mobidik'
    task_request.load_id = '4800001663'
    task_request.pickup_pose.name = 'AMK_D_L-1_C41'  # 'pickup_location'
    task_request.pickup_pose.floor_number = -1
    task_request.delivery_pose.name = 'AMK_B_L4_C2'  # 'delivery_location'
    task_request.delivery_pose.floor_number = 4
    task_request.start_time = int(round(time.time()) * 1000)

    path_planner = FMSPathPlanner(server_ip='127.0.0.1', server_port=8000, building='AMK')

    print('Generating a task plan...')
    task_planner = TaskPlanner()
    plan = task_planner.get_task_plan(task_request, path_planner=path_planner)
    print('Task plan generated')

    print('Creating task...')

    if plan is not None:
        task = Task()
        task.robot_actions['ropod_1'] = plan
        task.load_type = 'mobidik'
        task.load_id = '4800001663'
        task.team_robot_ids = ['ropod_1']
        task.start_time = int(round(time.time()) * 1000)
        task.estimated_duration = -1.
        print('Task created')
    else:
        print("Task planning failed")
