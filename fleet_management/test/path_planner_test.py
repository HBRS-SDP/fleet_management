from __future__ import print_function

from fleet_management.path_planner import FMSPathPlanner

if __name__ == '__main__':
    planner = FMSPathPlanner(server_ip='127.0.0.1',
                             server_port=8000, building='AMK')

    print("Planning path from basement pickup area {AMK_D_L-1_C41_LA1}\
    to ward {AMK_B_L4_C1_LA2}")

    plan = planner.get_path_plan(start_floor=-1, destination_floor=4,
                                 start_area='AMK_D_L-1_C41',
                                 destination_area='AMK_B_L4_C1',
                                 start_local_area='AMK_D_L-1_C41_LA1',
                                 destination_local_area='AMK_B_L4_C1_LA2')
    if plan is None:
        raise Exception('No plan returned')

    print("******************************************************************")

    print("Planning path from docking location {AMK_B_L-1_C4_LA1}\
    to undocking location (right){AMK_A_L-1_C42_LA1}")

    plan = planner.get_path_plan(start_floor=-1, destination_floor=-1,
                                 start_area='AMK_B_L-1_C4',
                                 destination_area='AMK_A_L-1_C42',
                                 start_local_area='AMK_B_L-1_C4_LA1',
                                 destination_local_area='AMK_A_L-1_C42_LA1')
    if plan is None:
        raise Exception('No plan returned')

    print("******************************************************************")

    print("Planning path from undocking location (right) {AMK_A_L-1_C42_LA2}\
    to elevator entry area (right){AMK_B_L-1_C2_LA1}")

    plan = planner.get_path_plan(start_floor=-1, destination_floor=-1,
                                 start_area='AMK_A_L-1_C42',
                                 destination_area='AMK_B_L-1_C2',
                                 start_local_area='AMK_A_L-1_C42_LA2',
                                 destination_local_area='AMK_B_L-1_C2_LA1')
    if plan is None:
        raise Exception('No plan returned')

    print("******************************************************************")

    print("Planning path from docking location {AMK_B_L-1_C4_LA1} to undocking\
    location (left){AMK_A_L-1_C11_LA1}")

    plan = planner.get_path_plan(start_floor=-1, destination_floor=-1,
                                 start_area='AMK_B_L-1_C4',
                                 destination_area='AMK_A_L-1_C11',
                                 start_local_area='AMK_B_L-1_C4_LA1',
                                 destination_local_area='AMK_A_L-1_C11_LA1')

    if plan is None:
        raise Exception('No plan returned')

    print("******************************************************************")

    print("Planning path from undocking location (left) {AMK_A_L-1_C11_LA1}\
    to elevator entry area (right){AMK_B_L-1_C2_LA1}")

    plan = planner.get_path_plan(start_floor=-1, destination_floor=-1,
                                 start_area='AMK_A_L-1_C11',
                                 destination_area='AMK_B_L-1_C2',
                                 start_local_area='AMK_A_L-1_C11_LA1',
                                 destination_local_area='AMK_B_L-1_C2_LA1')

    if plan is None:
        raise Exception('No plan returned')

    print("******************************************************************")
