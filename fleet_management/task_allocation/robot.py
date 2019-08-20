import argparse

from fleet_management.config.loader import Config

if __name__ == '__main__':

    config = Config(initialize=False)
    config.configure_logger()

    parser = argparse.ArgumentParser()
    parser.add_argument('robot_id', type=str, help='example: ropod_001')
    args = parser.parse_args()
    robot_id = args.robot_id

    robot_proxy = config.configure_robot_proxy(robot_id)
    robot_proxy.common.api.register_callbacks(robot_proxy)

    robot_proxy.run()
