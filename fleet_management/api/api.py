import logging

from fleet_management.api.rest import RESTInterface
from fleet_management.api.ros import ROSInterface
from fleet_management.api.zyre import FMSZyreAPI


class API(object):
    def __init__(self, config):
        self.logger = logging.getLogger('fms.api')
        self.publish_dict = dict()
        self.zyre = API.get_zyre_api(config.get('zyre').get('zyre_node', None))
        self.ros = API.get_ros_api('test')
        rest_config = {
            'bind': '%s:%s' % ('127.0.0.1', '8080'),
            'workers': 1,
        }
        self.rest = API.get_rest_api(rest_config)
        self.__configure(config)

    def publish(self, msg, **kwargs):
        msg_type = msg.get('header').get('type')

        if self.zyre:
            method = self.publish_dict.get(msg_type.lower()).get('method').get('zyre')
            getattr(self.zyre, method)(msg, **kwargs)

    def __configure(self, config_params):
        self.publish_dict.update(config_params.get('publish'))

    @staticmethod
    def get_zyre_api(zyre_config):
        logging.info("Configuring Zyre interface")
        zyre_api = FMSZyreAPI(zyre_config)
        return zyre_api

    @staticmethod
    def get_ros_api(ros_config):
        logging.info("Configuring ROS interface")
        return ROSInterface(ros_config)

    @staticmethod
    def get_rest_api(rest_config):
        logging.info("Configuring REST interface")
        return RESTInterface(rest_config)

