from __future__ import print_function
from fleet_management.path_planner import FMSPathPlanner

if __name__ == '__main__':
    planner = FMSPathPlanner(building='AMK')
    plan = planner.get_path_plan(start_floor=-1, destination_floor=4, start_area='AMK_D_L-1_C41',
                                 destination_area='AMK_B_L4_C1', start_local_area='AMK_D_L-1_C41_LA1',
                                 destination_local_area='AMK_B_L4_C1_LA2')
    assert len(plan) == 32

