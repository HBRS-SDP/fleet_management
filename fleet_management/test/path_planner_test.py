from __future__ import print_function
from fleet_management.path_planner import PathPlanner
from fleet_management.structs.area import Area

if __name__ == '__main__':
    start = Area()
    start.name = 'AMK_D_L-1_C41_LA1'
    start.floor_number = -1

    destination = Area()
    destination.name = 'AMK_B_L4_C1_LA2'
    destination.floor_number = 4

    print('Generating plan from "AMK_D_L-1_C41_LA1" to "AMK_B_L4_C1_LA2"')
    plan = PathPlanner.get_path_plan(start, destination)
    print(plan)
