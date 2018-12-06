from __future__ import print_function
from fleet_management.task_allocator import TaskAllocator
from fleet_management.structs.task import Task
from fleet_management.config.config_file_reader import ConfigFileReader
from fleet_management.task_allocation import Robot
from fleet_management.structs.robot import Robot as RobotStruct
from fleet_management.structs.status import RobotStatus
from fleet_management.db.ccu_store import CCUStore
from fleet_management.structs.area import Area
from fleet_management.structs.area import Waypoint
import uuid
import time
import datetime

SLEEP_TIME = 0.350


def initialize_ccu_store_for_testing():
    print("Initializing CCU")

    ccu_store = CCUStore('ropod_ccu_store')

    # create two Waypoints (one for each area)
    waypoint_A = Waypoint()
    waypoint_A.semantic_id = '0'
    waypoint_A.area_id = 1
    waypoint_A.x = '1'
    waypoint_A.y = '1'

    waypoint_B = Waypoint()
    waypoint_B.semantic_id = '1'
    waypoint_B.area_id = 2
    waypoint_B.x = '2'
    waypoint_B.y = '2'

    # create an area for each one of our Waypoints
    area_A = Area()
    area_A.id = list('area_A_id')
    area_A.name = 'area_A'
    area_A.floor_number = 1
    area_A.type = ''
    area_A.waypoints = list()
    area_A.waypoints.append(waypoint_A)

    area_B = Area()
    area_B.id = list('area_B_id')
    area_B.name = 'area_B'
    area_B.floor_number = 1
    area_B.type = ''
    area_B.waypoints = list()
    area_B.waypoints.append(waypoint_B)

    status_A = RobotStatus()
    status_A.robot_id = 'ropod_001'
    status_A.current_location = area_A
    status_A.current_operation = 'hangout'
    status_A.status = 'idle'
    status_A.available = 'na'
    status_A.battery_status = 'voll Saft'

    ccu_store.add_robot_status(status_A)

    print("Added")

    robot_A = RobotStruct()

    robot_A.robot_id = 'ropod_001'
    robot_A.schedule = 'N/A'
    robot_A.status = status_A

    ccu_store.add_robot(robot_A)
    print("Added robot A")

    robot_B = robot_A
    robot_B.robot_id = 'ropod_002'
    robot_B.status.robot_id = 'ropod_B'
    ccu_store.add_robot(robot_B)
    print("Added robot B")

    robot_C = robot_A
    robot_C.robot_id = 'ropod_003'
    robot_C.status.robot_id = 'ropod_C'
    ccu_store.add_robot(robot_C)
    print("Added robot C")

    robot_D = robot_A
    robot_D.robot_id = 'ropod_004'
    robot_D.status.robot_id = 'ropod_D'
    ccu_store.add_robot(robot_D)
    print("Added robot D")

    robot_E = robot_A
    robot_E.robot_id = 'ropod_005'
    robot_E.status.robot_id = 'ropod_E'
    ccu_store.add_robot(robot_E)
    print("Added robot E")

    return ccu_store


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

    print("Task: {} \n earliest_start_time: {} \n latest_start_time {}\n".format(task.id, task.earliest_start_time,
                                                                                 task.latest_start_time))

    config_params = ConfigFileReader.load("../../config/ccu_config.yaml")

    print("Initializing CCU")
    ccu_store = initialize_ccu_store_for_testing()

    # Start the robot zyre nodes
    robots = list()
    robots.append(Robot('ropod_001', config_params, ccu_store, verbose_mrta=True))
    robots.append(Robot('ropod_002', config_params, ccu_store, verbose_mrta=True))
    robots.append(Robot('ropod_003', config_params, ccu_store, verbose_mrta=True))
    robots.append(Robot('ropod_004', config_params, ccu_store, verbose_mrta=True))
    robots.append(Robot('ropod_005', config_params, ccu_store, verbose_mrta=True))

    task_allocator = TaskAllocator(config_params)

    # Wait for the robot to joing the zyre group
    time.sleep(SLEEP_TIME)

    # Showing robot information
    for robot in robots:
        print(robot)
    task_allocator.get_information()

    print("Allocating task ...")
    allocation = task_allocator.allocate(task)
    print("Allocation:", allocation)

    schedule = task_allocator.get_scheduled_tasks()
    print("Schedule:", schedule)

    unsuccessful_allocations = task_allocator.get_unsuccessful_allocations()
    print("unsuccessful_allocations: ", [task.id for task in unsuccessful_allocations])

    allocation_robot = task_allocator.get_allocations_robot('ropod_001')
    print("Task allocated to ropod_001: ", allocation_robot)

    schedule_robot = task_allocator.get_scheduled_tasks_robot('ropod_001')
    print("Shedule of ropod 001: ", schedule_robot)

    time_schedule = task_allocator.get_tasks_schedule()
    print("Time schedule: ", time_schedule)

    task_allocator.shutdown()
    for robot in robots:
        robot.shutdown()
