from __future__ import print_function
import yaml
from fleet_management.config.params import ConfigParams, RopodParams, ElevatorParams

'''An interface for reading CCU configuration files

@author Alex Mitrevski
@contact aleksandar.mitrevski@h-brs.de
'''
class ConfigFileReader(object):
    '''Loads CCU configuration parameters from the given YAML file

    Keyword arguments:
    @param config_file absolute path of a config file

    '''
    @staticmethod
    def load(config_file):
        config_params = ConfigParams()
        config_data = ConfigFileReader.__read_yaml_file(config_file)

        if 'ccu_store_db_name' in config_data.keys():
            config_params.ccu_store_db_name = config_data['ccu_store_db_name']
        else:
            print('Config error: "ccu_store_db_name" not specified')
            return ConfigParams()

        if 'ropods' in config_data.keys():
            for ropod_id, params in config_data['ropods'].items():
                ropod_params = RopodParams()
                ropod_params.id = ropod_id
                config_params.ropods.append(ropod_params)
                # other parameters can be processed here
        else:
            print('Config error: "ropods" not specified')
            return ConfigParams()

        if 'elevators' in config_data.keys():
            for elevator_id, params in config_data['elevators'].items():
                elevator_params = ElevatorParams()
                elevator_params.id = elevator_id
                config_params.elevators.append(elevator_params)
                # other parameters can be processed here
        else:
            print('Config error: "elevators" not specified')
            return ConfigParams()

        if 'message_version' in config_data.keys():
            config_params.message_version = config_data['message_version']
        else:
            print('Config warning: "message_version" not specified')

        if 'zyre_group_name' in config_data.keys():
            config_params.zyre_group_name = config_data['zyre_group_name']
        else:
            print('Config error: "zyre_group_name" not specified')
            return ConfigParams()

        if 'task_manager_zyre_params' in config_data.keys():
            config_params.task_manager_zyre_params.node_name = config_data['task_manager_zyre_params']['node_name']
            config_params.task_manager_zyre_params.groups = config_data['task_manager_zyre_params']['groups']
            config_params.task_manager_zyre_params.message_types = config_data['task_manager_zyre_params']['message_types']
        else:
            print('Config error: "task_manager_zyre_params" not specified')
            return ConfigParams()

        if 'resource_manager_zyre_params' in config_data.keys():
            config_params.resource_manager_zyre_params.node_name = config_data['resource_manager_zyre_params']['node_name']
            config_params.resource_manager_zyre_params.groups = config_data['resource_manager_zyre_params']['groups']
            config_params.resource_manager_zyre_params.message_types = config_data['resource_manager_zyre_params']['message_types']
        else:
            print('Config error: "resource_manager_zyre_params" not specified')
            return ConfigParams()

        return config_params

    @staticmethod
    def __read_yaml_file(config_file_name):
        file_handle = open(config_file_name, 'r')
        data = yaml.load(file_handle)
        file_handle.close()
        return data
