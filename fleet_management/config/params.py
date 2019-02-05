from ropod.pyre_communicator.zyre_params import ZyreParams

class RopodParams(object):
    def __init__(self):
        self.id = ''

class ElevatorParams(object):
    def __init__(self):
        self.id = ''

class OverpassParams(object):
    def __init__(self):
        self.ip = ''
        self.port = ''

class PlannerParams(object):
    def __init__(self):
        self.kb_database_name = ''
        self.planner_name = ''
        self.domain_file = ''
        self.planner_cmd = ''
        self.plan_file_path = ''

class ConfigParams(object):
    def __init__(self):
        self.ccu_store_db_name = ''
        self.ropods = list()
        self.elevators = list()
        self.allocation_method = ''

        self.message_version = ''
        self.zyre_group_name = ''

        self.task_manager_zyre_params = ZyreParams()
        self.resource_manager_zyre_params = ZyreParams()
        self.overpass_server = OverpassParams()
        self.building = ''
        self.task_allocator_zyre_params = ZyreParams()

        self.planner_params = PlannerParams()
