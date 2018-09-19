from __future__ import print_function
from fleet_management.path_planner import PathPlanner
from fleet_management.structs.area import Area
import time

if __name__ == '__main__':
    time.sleep(5)

    planner = PathPlanner("127.0.0.1:8000")

    print("Planning path from start location {AMK_D_L-1_C41_LA1} to docking location {AMK_B_L4_C1_LA2}")
    start = planner.get_area('AMK_D_L-1_C41_LA1')
    start.floor_number = -1

    destination = planner.get_area('AMK_B_L4_C1_LA2')
    destination.floor_number = 4

    print('Generating plan from', start.name, " to ", destination.name)

    plan = planner.get_path_plan(start, destination)

    print("*********************************************************************************************")

    print("Planning path from start location {AMK_B_L-1_C24_LA2} to docking location {AMK_B_L-1_C4_LA1}")
    start1 = planner.get_area('AMK_B_L-1_C24_LA2')
    start1.floor_number = -1

    destination1 = planner.get_area('AMK_B_L-1_C4_LA1')
    destination1.floor_number = -1

    print('Generating plan from', start1.name, " to ", destination1.name)

    plan = planner.get_path_plan(start1, destination1)
    print("*********************************************************************************************")

    print("Planning path from docking location {AMK_B_L-1_C4_LA1} to undocking location (right){AMK_A_L-1_C42_LA1}")
    start2 = planner.get_area('AMK_B_L-1_C4_LA1')
    start2.floor_number = -1

    destination2 = planner.get_area('AMK_A_L-1_C42_LA2')
    destination2.floor_number = -1

    print('Generating plan from', start2.name, " to ", destination2.name)

    plan = planner.get_path_plan(start2, destination2)

    print("*********************************************************************************************")

    print("Planning path from undocking location (right) {AMK_A_L-1_C42_LA2} to elevator entry area (right){AMK_B_L-1_C2_LA1}")
    start3 = planner.get_area('AMK_A_L-1_C42_LA2')
    start3.floor_number = -1

    destination3 = planner.get_area('AMK_B_L-1_C2_LA1')
    destination3.floor_number = -1

    print('Generating plan from', start3.name, " to ", destination3.name)

    plan = planner.get_path_plan(start3, destination3)

    print("*********************************************************************************************")

    print("Planning path from docking location {AMK_B_L-1_C4_LA1} to undocking location (left){AMK_A_L-1_C11_LA1}")
    start4 = planner.get_area('AMK_B_L-1_C4_LA1')
    start4.floor_number = -1

    destination4 = planner.get_area('AMK_A_L-1_C11_LA1')
    destination4.floor_number = -1

    print('Generating plan from', start4.name, " to ", destination4.name)

    plan = planner.get_path_plan(start4, destination4)

    print("*********************************************************************************************")

    print("Planning path from undocking location (left) {AMK_A_L-1_C11_LA1} to elevator entry area (right){AMK_B_L-1_C2_LA1}")
    start5 = planner.get_area('AMK_A_L-1_C11_LA1')
    start5.floor_number = -1

    destination5 = planner.get_area('AMK_B_L-1_C2_LA1')
    destination5.floor_number = -1

    print('Generating plan from', start5.name, " to ", destination5.name)

    plan = planner.get_path_plan(start5, destination5)

    print("*********************************************************************************************")
