import logging

from ropod.utils.models import MessageFactory

from fleet_management.api.rest import RESTInterface
from fleet_management.api.ros import ROSInterface
from fleet_management.api.zyre import FMSZyreAPI


class API(object):
    def __init__(self, config):
        self.logger = logging.getLogger('fms.api')

        self.publish_dict = dict()
        self.middleware_collection = config.get('middleware', list())
        self.config_params = config
        self.__configure(config)

        self.logger.debug("Configuring Zyre interface")
        self.zyre = API.get_zyre_api(config.get('zyre'))

        rest_config = {
            'bind': '%s:%s' % ('127.0.0.1', '8080'),
            'workers': 1,
        }
        self.logger.debug("Configuring REST interface")
        self.rest = API.get_rest_api(rest_config)

        self.logger.debug("Configuring ROS interface")
        self.ros = API.get_ros_api('test')

        self.mf = MessageFactory()

        self.logger.info("Initialized API")

    def publish(self, msg, **kwargs):
        try:
            msg_type = msg.get('header').get('type')
        except AttributeError:
            self.logger.error("Could not get message type from message: %s", msg, exc_info=True)
            return

        self.logger.debug("Publishing message of type %s", msg_type)

        try:
            method = self.publish_dict.get(msg_type.lower()).get('method')
        except ValueError:
            self.logger.error("No method defined for message %", msg_type)
            return

        for option in self.middleware_collection:
            self.logger.debug('Using method %s to publish message using %s', method, option)
            getattr(self.__dict__[option], method)(msg, **kwargs)

    def __configure(self, config_params):
        for option in self.middleware_collection:
            config = config_params.get(option)

        self.publish_dict.update(config_params.get('zyre').get('publish'))
        self.logger.debug("Publish dictionary: %s", self.publish_dict)

    @staticmethod
    def get_zyre_api(zyre_config):
        zyre_api = FMSZyreAPI(**zyre_config)
        return zyre_api

    @staticmethod
    def get_ros_api(ros_config):
        return ROSInterface(ros_config)

    @staticmethod
    def get_rest_api(rest_config):
        return RESTInterface(rest_config)

    def create_message(self, contents, **kwargs):
        return self.mf.create_message(contents, **kwargs)

    def register_callback(self, middleware, function, **kwargs):
        self.logger.info("Adding %s callback to %s", function, middleware)
        getattr(self, middleware).register_callback(function, **kwargs)

