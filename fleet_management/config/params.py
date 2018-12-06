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
