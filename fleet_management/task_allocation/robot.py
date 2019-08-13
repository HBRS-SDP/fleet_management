import argparse
import time

from fleet_management.config.loader import Config, register_api_callbacks

if __name__ == '__main__':

    config = Config(initialize=False)
    config.configure_logger()
    ccu_store = config.configure_ccu_store()

    parser = argparse.ArgumentParser()
    parser.add_argument('robot_id', type=str, help='example: ropod_001')
    args = parser.parse_args()
    robot_id = args.robot_id

    robot_proxy = config.configure_robot_proxy(robot_id, ccu_store, dispatcher=True)

    register_api_callbacks(robot_proxy, robot_proxy.api)

    robot_proxy.run()