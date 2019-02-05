from ropod.structs.robot import Robot
from ropod.structs.area import Area, SubArea
from ropod.structs.status import RobotStatus
from task_planner.knowledge_base_interface import KnowledgeBaseInterface
from fleet_management.db.ccu_store import CCUStore
from fleet_management.config.config_file_reader import ConfigFileReader

def initialize_knowledge_base(kb_database_name):
    kb_interface = KnowledgeBaseInterface(kb_database_name)

    print('[initialize_knowledge_base] Initializing elevators')
    # TODO: Use the actual areas where the elevators are
    elevator_facts = [('elevator_at', [('elevator', 'elevator0'),
                                       ('loc', 'AMK_D_L-1_C40')]),
                      ('elevator_at', [('elevator', 'elevator0'),
                                       ('loc', 'AMK_D_L4_C40')])]
    kb_interface.insert_facts(elevator_facts)

    elevator_fluents = [('elevator_floor', [('elevator', 'elevator0')], 100)]
    kb_interface.insert_fluents(elevator_fluents)

    elevator_location_fluents = [('location_floor', [('loc', 'AMK_D_L-1_C40')], -1),
                                 ('location_floor', [('loc', 'AMK_D_L4_C40')], 4)]
    kb_interface.insert_fluents(elevator_location_fluents)

def initialize_robot_db(config_params):
    ccu_store = CCUStore('ropod_ccu_store')

    for ropod in config_params.ropods:
        print(ropod.id)

        area = Area()
        area.id = 'AMK_D_L-1_C41'
        area.name = 'AMK_D_L-1_C41'
        area.floor_number = -1
        area.type = ''
        area.sub_areas = list()

        subarea = SubArea()
        subarea.name = 'AMK_D_L-1_C41_LA1'
        area.sub_areas.append(subarea)

        ropod_001 = Robot()
        status_001 = RobotStatus()
        status_001.robot_id = ropod.id
        status_001.current_location = area
        status_001.current_operation = 'unknown'
        status_001.status = 'idle'
        status_001.available = 'unknown'
        status_001.battery_status = 'unknown'

        ropod_001.robot_id = 'ropod_001'
        ropod_001.schedule = None
        ropod_001.status = status_001
        ccu_store.add_robot(ropod_001)


if __name__ == '__main__':
    config_params = ConfigFileReader.load("../../config/ccu_config.yaml")
    initialize_robot_db(config_params)
