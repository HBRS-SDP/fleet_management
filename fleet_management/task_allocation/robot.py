from fleet_management.config.loader import Config
import argparse
import time

if __name__ == '__main__':

    config = Config(initialize=False)
    config.configure_logger()
    ccu_store = config.configure_ccu_store()

    parser = argparse.ArgumentParser()
    parser.add_argument('robot_id', type=str, help='example: ropod_001')
    args = parser.parse_args()
    robot_id = args.robot_id

    robot_proxy = config.configure_robot_proxy(robot_id, ccu_store)

    time.sleep(5)

    robot_proxy.run()
