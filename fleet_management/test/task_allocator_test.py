from __future__ import print_function
import uuid
import time
import datetime
SLEEP_TIME = 0.350

from fleet_management.task_allocator import TaskAllocator
from fleet_management.structs.task import Task
from fleet_management.config.config_file_reader import ConfigFileReader
from fleet_management.allocation import Robot

if __name__ == '__main__':
    earliest_start_time = datetime.datetime.now() + datetime.timedelta(seconds=300)

    task = Task()
    task.id = str(uuid.uuid4())
    task.cart_type = 'mobidik'
    task.cart_id = '4800001663'
    task.earliest_start_time = earliest_start_time.timestamp()
    task.latest_start_time = task.earliest_start_time + 5
    task.estimated_duration = 4
    task.pickup_pose.name = 'AMK_B_L-1_C24_LA2'
    task.pickup_pose.floor_number = 0
    task.delivery_pose.name = 'AMK_B_L-1_C4_LA1'
    task.delivery_pose.floor_number = 0
    task.priority = 1

    print("Task: {} \n earliest_start_time: {} \n latest_start_time {}\n".format(task.id, task.earliest_start_time, task.latest_start_time))

    config_params = ConfigFileReader.load("../../config/ccu_config.yaml")

    # Start the robots
    # TODO Who should launch the zyre nodes of the robots?
    robots = list()
    robots.append(Robot('ropod_001', config_params, verbose_mrta = True))
    robots.append(Robot('ropod_002', config_params, verbose_mrta = True))
    robots.append(Robot('ropod_003', config_params, verbose_mrta = True))
    robots.append(Robot('ropod_004', config_params, verbose_mrta = True))
    robots.append(Robot('ropod_005', config_params, verbose_mrta = True))

    task_allocator = TaskAllocator(config_params)

    # Wait for the robot to joing the zyre group
    time.sleep(SLEEP_TIME)

    # Showing robot information
    for robot in robots:
        print(robot)
    task_allocator.get_information()

    print ("Allocating task ...")
    allocation = task_allocator.get_assignment(task)
    print ("Allocation:", allocation)

    schedule = task_allocator.get_schedule()
    print("Schedule:", schedule)

    unsuccessful_allocations = task_allocator.get_unsuccessful_allocations()
    print("unsuccessful_allocations: ", [task.id for task in unsuccessful_allocations])

    allocation_robot = task_allocator.get_allocations_robot('ropod_001')
    print("Task allocated to ropod_001: ", allocation_robot)

    schedule_robot = task_allocator.get_schedule_robot('ropod_001')
    print("Shedule of ropod 001: ", schedule_robot)

    time_schedule = task_allocator.get_time_schedule()
    print("Time schedule: ", time_schedule)

    task_allocator.shutdown()
    for robot in robots:
        robot.shutdown()
