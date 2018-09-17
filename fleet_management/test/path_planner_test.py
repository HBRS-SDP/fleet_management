from __future__ import print_function
from fleet_management.path_planner import PathPlanner
from fleet_management.structs.area import Area

if __name__ == '__main__':
    planner = PathPlanner("127.0.0.1:8000")
    start = planner.get_area('AMK_D_L-1_C41_LA1')
    start.floor_number = -1

    destination = planner.get_area('AMK_A_L-1_RoomBU21_LA1')
    destination.floor_number = -1

    print('Generating plan from', start.name, " to ", destination.name)
    
    plan = planner.get_path_plan(start, destination)
    print(plan)
