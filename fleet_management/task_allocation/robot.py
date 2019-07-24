from allocation.robot import Robot
from fleet_management.config.loader import Config
from fleet_management.api.zyre import FMSZyreAPI
import argparse
import time
import logging

if __name__ == '__main__':

    config = Config(initialize=False)
    config.configure_logger()
    ccu_store = config.configure_ccu_store()

    parser = argparse.ArgumentParser()
    parser.add_argument('robot_id', type=str, help='example: ropod_001')
    args = parser.parse_args()
    robot_id = args.robot_id

    api_config = config.config_params.get('api')

    zyre_config = api_config.get('zyre').get('zyre_node')  # Arguments for the zyre_base class
    zyre_config['node_name'] = robot_id + '_proxy'
    zyre_config['groups'] = ['TASK-ALLOCATION']

    api = FMSZyreAPI(zyre_config)

    robot_config = config.configure_robot_proxy(robot_id, ccu_store)

    time.sleep(5)

    robot = Robot(api=api, **robot_config)
    robot.api.start()

    try:
        while True:
            robot.api.run()
            time.sleep(0.5)
    except (KeyboardInterrupt, SystemExit):
        logging.info("Terminating %s proxy ...", robot_id)
        robot.api.shutdown()
        logging.info("Exiting...")
