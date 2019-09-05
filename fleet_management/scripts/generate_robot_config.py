import yaml

from ropod.structs.robot import Hardware, Software, BlackBox
from ropod.utils.uuid import generate_uuid


if __name__ == '__main__':
    robot_id = 'ropod_001'
    nick_name = 'Eddie'

    version = {'version': 1}
    header = {'id': robot_id,
              'uuid': generate_uuid(),
              'nick_name': nick_name}

    config = {'hardware': Hardware.full_version(),
              'software': Software.full_version(),
              'black_box': BlackBox.full_version()}

    with open(robot_id + '.yaml', 'w') as yaml_file:
        yaml.dump(version, yaml_file, default_flow_style=False)

    with open(robot_id+ '.yaml', 'a') as yaml_file:
        yaml.dump(header, yaml_file, default_flow_style=False)
        yaml.dump(config, yaml_file, default_flow_style=False)
