from __future__ import print_function
import time
from fleet_management.structs.task import TaskRequest, Task
from fleet_management.task_planner import TaskPlanner
from fleet_management.db.ccu_store import CCUStore

if __name__ == '__main__':
    task_request = TaskRequest()
    task_request.user_id = 1
    task_request.cart_type = 'mobidik'
    task_request.cart_id = '4800001663'
    task_request.pickup_pose.name = 'AMK_D_L-1_C41_LA1'   #'pickup_location'
    task_request.pickup_pose.floor_number = -1
    task_request.delivery_pose.name = 'AMK_A_L-1_RoomBU21_LA1' #'delivery_location'
    task_request.delivery_pose.floor_number = -1
    task_request.start_time = int(round(time.time()) * 1000)

    print('Generating a task plan...')
    task_planner = TaskPlanner()
    plan = task_planner.get_task_plan(task_request)
    print('Task plan generated')

    print('Creating task...')
    task = Task()
    task.robot_actions['ropod_1'] = plan
    task.cart_type = 'mobidik'
    task.cart_id = '4800001663'
    task.team_robot_ids = ['ropod_1']
    task.start_time = int(round(time.time()) * 1000)
    task.estimated_duration = -1.
    print('Task created')

    print('Saving task to database "ropod_task_data_test"...')
    ccu_store = CCUStore('ropod_task_data_test')
    ccu_store.add_task(task)
    print('Successfully saved task')
