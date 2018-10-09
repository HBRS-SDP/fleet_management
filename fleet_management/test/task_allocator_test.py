from __future__ import print_function
import uuid
import time

from task_allocation.task_allocator import TaskAllocator
from fleet_management.structs.task import Task
from fleet_management.config.config_file_reader import ConfigFileReader

if __name__ == '__main__':
    task = Task()
    task.id = str(uuid.uuid4())
    task.cart_type = 'mobidik'
    task.cart_id = '4800001663'
    task.earliest_start_time = int(round(time.time()) * 1000)
    task.latest_start_time = task.earliest_start_time + int(round(time.time()) * 10)
    task.pickup_pose.name = 'pickup_location'
    task.pickup_pose.floor_number = 0
    task.delivery_pose.name = 'delivery_location'
    task.delivery_pose.floor_number = 0
    task.priority = 1

    config_params = ConfigFileReader.load("../../config/ccu_config.yaml")
    print ("Task allocator information: ")
    task_allocator = TaskAllocator(config_params)
    task_allocator.get_information()
    print ("Allocating task ...")
    allocation = task_allocator.get_assignment(task)
    print (allocation)
    task_allocator.shutdown()
