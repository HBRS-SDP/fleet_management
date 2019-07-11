import logging

from fleet_management.api.zyre import FMSZyreAPI
from fleet_management.db.ccu_store import CCUStore, initialize_robot_db
from fleet_management.resource_manager import ResourceManager
from fleet_management.task_manager import TaskManager
from fleet_management.path_planner import FMSPathPlanner
from fleet_management.task_planner_interface import TaskPlannerInterface
from fleet_management.task_allocator import TaskAllocator
from fleet_management.task_allocation.auctioneer import Auctioneer

from ropod.utils.logging.config import config_logger

from OBL import OSMBridge

from ropod.utils.config import read_yaml_file, get_config

from fleet_management.exceptions.config import InvalidConfig

from importlib_resources import open_text

logging.getLogger(__name__)


def load_version(config):
    version = config.get('version', None)
    if version not in (1, 2):
        raise InvalidConfig


def load_resources(config):
    resources = config.get('resources', None)
    if resources is None:
        raise InvalidConfig

    fleet = resources.get('fleet', None)
    if fleet is None:
        raise InvalidConfig

    infrastructure = resources.get('infrastructure', None)
    if infrastructure is None:
        logging.debug("No infrastructure resources added.")
    else:
        logging.debug(infrastructure)


def load_plugins(config):
    plugins = config.get('plugins', None)

    if plugins is None:
        logging.info("No plugins added.")

def load_api(config):
    api = config.get('api', None)

    if api is None:
        logging.error("Missing API configuration. At least one API option must be configured")
        raise InvalidConfig

    # Check if we have any valid API config
    if not any(elem in ['zyre', 'ros', 'rest'] for elem in api.keys()):
        raise InvalidConfig

    zyre_config = api.get('zyre', None)
    if zyre_config is None:
        logging.debug('FMS missing Zyre API')
        raise InvalidConfig

    rest_config = api.get('rest', None)
    if rest_config is None:
        logging.debug('FMS missing REST API')

    ros_config = api.get('ros', None)
    if ros_config is None:
        logging.debug('FMS missing ROS API')


class Config(object):
    def __init__(self, config_file=None, initialize=False):

        if config_file is None:
            config = Config.load_default_config()
        else:
            config = Config.load_file(config_file)

        self.config_params = dict()
        self.config_params.update(**config)
        if initialize:
            self.api = self.configure_api()
            self.ccu_store = self.configure_ccu_store()

    def __str__(self):
        return str(self.config_params)

    @staticmethod
    def load_file(config_file):
        config = read_yaml_file(config_file)
        return config

    @staticmethod
    def load_default_config():
        config_file = open_text('fleet_management.config.default', 'fms_config-v2.yaml')
        config = get_config(config_file)
        return config

    def configure_logger(self, logger_config=None):
        logging.info("Configuring logger...")
        if logger_config is not None:
            logging.info("Loading logger configuration from file: %s ", logger_config)
            config_logger(logger_config)
        elif 'logger' in self.config_params:
            logging.info("Using FMS logger configuration")
            fms_logger_config = self.config_params.get('logger', None)
            logging.config.dictConfig(fms_logger_config)
        else:
            logging.info("Using default ropod config...")
            config_logger()

    def configure_ccu_store(self):
        store_config = self.config_params.get('ccu_store', dict())
        if not store_config:
            logging.info('Using default ccu_store config')
            store_config.update(dict(db_name='ropod_ccu_store', port=27017))
        else:
            store_config.update(db_name=store_config.get('db_name', 'ropod_ccu_store'))
            store_config.update(port=store_config.get('port', 27017))

        ccu_store = CCUStore(**store_config)

        robots = self.config_params.get('resources').get('fleet')
        print(robots)

        initialize_robot_db(robots)

        return ccu_store

    def configure_task_manager(self, db):
        task_manager_config = self.config_params.get('task_manager', None)
        if task_manager_config is None:
            logging.info('Using default task manager config')
        else:
            api = self.config_params.get('api')

        return TaskManager(db, api_config=self.api, plugins=[])

    def configure_resource_manager(self, db):
        rm_config = self.config_params.get('resource_manager', None)
        resources = self.config_params.get('resources', None)
        if rm_config is None:
            logging.info('Using default resource manager config')
        else:
            api = self.config_params.get('api')

        return ResourceManager(resources, ccu_store=db, api_config=self.api)

    def configure_plugins(self, ccu_store):
        logging.info("Configuring FMS plugins...")
        plugin_config = self.config_params.get('plugins')
        # TODO add conditions to only configure plugins listed in the config file
        osm_bridge = self.configure_osm_bridge()
        path_planner = self.configure_path_planner(osm_bridge)
        task_planner = self.configure_task_planner()
        auctioneer = self.configure_task_allocator(ccu_store)
        return {'osm_bridge': osm_bridge, 'path_planner': path_planner, 'task_planner': task_planner,
                'auctioneer': auctioneer}

    def configure_osm_bridge(self):
        logging.info("Configuring osm_bridge")
        osm_bridge_config = self.config_params.get('plugins').get('osm_bridge')

        ip = osm_bridge_config.get('server_ip')
        port = osm_bridge_config.get('server_port', '8000')

        try:
            osm_bridge = OSMBridge(server_ip=ip,
                                   server_port=port)
        except Exception as e:
            logging.error("There is a problem in connecting to Overpass server. Error: %s", e)
            osm_bridge = None

        logging.info("Connected to osm_bridge (%s:%s)", ip, port)

        return osm_bridge

    def configure_path_planner(self, osm_bridge=None):
        logging.info("Configuring path_planner...")
        path_planner_config = self.config_params.get('plugins').get('path_planner')
        building = path_planner_config.get('building')
        if osm_bridge is None:
            osm_bridge = self.configure_osm_bridge()
        path_planner = FMSPathPlanner(osm_bridge=osm_bridge, building=building)

        return path_planner

    def configure_task_planner(self):
        logging.info("Configuring task planner...")
        planner_config = self.config_params.get('plugins').get('task_planner')
        task_planner = TaskPlannerInterface(planner_config)
        return task_planner

    def configure_task_allocator(self, ccu_store):
        logging.info("Configuring task allocator...")
        allocator_config = self.config_params.get("plugins").get("task_allocation")
        api_config = self.config_params.get('api')
        fleet = self.config_params.get('resources').get('fleet')
        auctioneer = Auctioneer(**allocator_config, robot_ids=fleet, ccu_store=ccu_store,
                                api_config=self.api)

        return auctioneer

    def configure_robot_proxy(self, robot_id, ccu_store, path_planner):
        allocation_config = self.config_params.get('plugins').get('task_allocation')
        api_config = self.config_params.get('api')
        api_config['zyre']['node_name'] = robot_id

        proxy = {'robot_id': robot_id,
                'allocation_method': allocation_config.get('allocation_method'),
                'api_config': api_config,
                'ccu_store': ccu_store,
                'path_planner': path_planner,
                'auctioneer': allocation_config.get('auctioneer')
                }
        return proxy

    def configure_api(self):
        api_config = self.config_params.get('api')
        zyre_config = api_config.get('zyre')
        zyre_api = FMSZyreAPI(zyre_config)
        return zyre_api


class ZyreConfig(object):
    def __init__(self, node_name, groups, msg_types, interface):
        self.node_name = node_name
        self.groups = groups
        self.message_types = msg_types
        self.interface = interface
