from __future__ import print_function
import yaml
import logging
from fleet_management.config.params import ConfigParams, RopodParams, ElevatorParams


class ConfigFileReader(object):
    '''An interface for reading CCU configuration files.

    @author Alex Mitrevski
    @contact aleksandar.mitrevski@h-brs.de
    '''
    @staticmethod
    def load(config_file):
        '''Loads CCU configuration parameters from the given YAML file

        Keyword arguments:
        @param config_file absolute path of a config file

        '''
        logger = logging.getLogger('fms.config.reader')
        config_params = ConfigParams()
        config_data = ConfigFileReader.__read_yaml_file(config_file)

        if 'ccu_store_db_name' in config_data.keys():
            config_params.ccu_store_db_name = config_data['ccu_store_db_name']
        else:
            logger.error('Config error: "ccu_store_db_name" not specified')
            return ConfigParams()

        if 'ropods' in config_data.keys():
            for ropod_id, params in config_data['ropods'].items():
                ropod_params = RopodParams()
                ropod_params.id = ropod_id
                config_params.ropods.append(ropod_params)
        else:
            logger.error('Config error: "ropods" not specified')
            return ConfigParams()

        if 'elevators' in config_data.keys():
            for elevator_id, params in config_data['elevators'].items():
                elevator_params = ElevatorParams()
                elevator_params.id = params['id']
                elevator_params.floor = params['floor']
                elevator_params.calls = params['calls']
                elevator_params.isAvailable = params['isAvailable']
                elevator_params.doorOpenAtGoalFloor = params['doorOpenAtGoalFloor']
                elevator_params.doorOpenAtStartFloor = params['doorOpenAtStartFloor']
                config_params.elevators.append(elevator_params)
        else:
            logger.error('Config error: "elevators" not specified')
            return ConfigParams()

        if 'allocation_method' in config_data.keys():
            config_params.allocation_method = config_data['allocation_method']
        else:
            logger.error('Config error: "allocation_method" not specified')
            return ConfigParams()

        if 'auction_time' in config_data.keys():
            config_params.auction_time = config_data['auction_time']
        else:
            logger.error('Config error: "auction_time" not specified')
            return ConfigParams()

        if 'message_version' in config_data.keys():
            config_params.message_version = config_data['message_version']
        else:
            logger.error('Config warning: "message_version" not specified')

        if 'zyre_group_name' in config_data.keys():
            config_params.zyre_group_name = config_data['zyre_group_name']
        else:
            logger.error('Config error: "zyre_group_name" not specified')
            return ConfigParams()

        if 'task_manager_zyre_params' in config_data.keys():
            config_params.task_manager_zyre_params.node_name = config_data['task_manager_zyre_params']['node_name']
            config_params.task_manager_zyre_params.groups = config_data['task_manager_zyre_params']['groups']
            config_params.task_manager_zyre_params.message_types = config_data['task_manager_zyre_params']['message_types']
        else:
            logger.error('Config error: "task_manager_zyre_params" not specified')
            return ConfigParams()

        if 'resource_manager_zyre_params' in config_data.keys():
            config_params.resource_manager_zyre_params.node_name = config_data['resource_manager_zyre_params']['node_name']
            config_params.resource_manager_zyre_params.groups = config_data['resource_manager_zyre_params']['groups']
            config_params.resource_manager_zyre_params.message_types = config_data['resource_manager_zyre_params']['message_types']
        else:
            logger.error('Config error: "resource_manager_zyre_params" not specified')
            return ConfigParams()

        if 'task_allocator_zyre_params' in config_data.keys():
            config_params.task_allocator_zyre_params.groups = config_data['task_allocator_zyre_params']['groups']
            config_params.task_allocator_zyre_params.message_types = config_data['task_allocator_zyre_params']['message_types']
        else:
            logger.error('Config error: "task_allocator_zyre_params" not specified')

        if 'overpass_server' in config_data.keys():
            config_params.overpass_server.ip = config_data['overpass_server']['ip']
            config_params.overpass_server.port = config_data['overpass_server']['port']
        else:
            logger.error('Config error: "overpass_server" details not specified')
            return ConfigParams()

        if 'building' in config_data.keys():
            config_params.building = config_data['building']
        else:
            logger.error('Config error: "building" not specified')
            return ConfigParams()

        if 'planner_params' in config_data:
            config_params.planner_params.kb_database_name = config_data['planner_params']['kb_database_name']
            config_params.planner_params.planner_name = config_data['planner_params']['planner_name']
            config_params.planner_params.domain_file = config_data['planner_params']['domain_file']
            config_params.planner_params.planner_cmd = config_data['planner_params']['planner_cmd']
            config_params.planner_params.plan_file_path = config_data['planner_params']['plan_file_path']
        else:
            logger.error('Config error: "planner_params" not specified')
            return ConfigParams()

        return config_params

    @staticmethod
    def __read_yaml_file(config_file_name):
        file_handle = open(config_file_name, 'r')
        data = yaml.load(file_handle)
        file_handle.close()
        return data
