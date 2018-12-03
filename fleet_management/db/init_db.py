from fleet_management.db.ccu_store import CCUStore
from fleet_management.config.config_file_reader import ConfigFileReader
from fleet_management.structs.robot import Robot
from fleet_management.structs.area import Area
from fleet_management.structs.status import RobotStatus


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