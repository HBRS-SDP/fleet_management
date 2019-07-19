"""This module contains the API class that allows components to receive and
send messages through the network using a variety of middlewares

"""

import logging

from ropod.utils.models import MessageFactory

from fleet_management.api.rest import RESTInterface
from fleet_management.api.ros import ROSInterface
from fleet_management.api.zyre import FMSZyreAPI


class API:
    """ API object serves as a facade to different middlewares

    Attributes:
        publish_dict: A dictionary that maps
        middleware_collection: A list of supported middlewares obtained from the config file
        config_params: A dictionary containing the parameters loaded from the config file
        message_factory: An object of type MessageFactory to create message templates

    """

    # pylint: disable=too-many-instance-attributes
    def __init__(self, config):
        """Initializes API with a configuration file

        Args:
            config: a dictionary containing the desired configuration
        """
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

        self.message_factory = MessageFactory()

        self.logger.info("Initialized API")

    def publish(self, msg, **kwargs):
        """Publishes a message using the configured functions per middleware

        Args:
            msg: a JSON message
            **kwargs: keyword arguments to be passed to the configured functions
        """
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
        """Create an object of type ZyreInterface

        Args:
            zyre_config: A dictionary containing the API configuration

        Returns:
            A configured ZyreInterface object

        """
        zyre_api = FMSZyreAPI(**zyre_config)
        return zyre_api

    @staticmethod
    def get_ros_api(ros_config):
        """Create an object of type ROSInterface

        Args:
            ros_config: A dictionary containing the API configuration

        Returns:
            A configured ROSInterface object
        """
        return ROSInterface(ros_config)

    @staticmethod
    def get_rest_api(rest_config):
        """Create an object of type RESTInterface

        Args:
            rest_config: A dictionary containing the API configuration

        Returns:
            A configured RESTInterface object

        """
        return RESTInterface(rest_config)

    def create_message(self, contents, **kwargs):
        """Creates a message in the right format using the MessageFactory

        Args:
            contents: message contents
            **kwargs: recipients

        Returns:
            A filled JSON message

        """
        return self.message_factory.create_message(contents, **kwargs)

    def register_callback(self, middleware, function, **kwargs):
        """Adds a callback function to the right middleware

        Args:
            middleware: a string specifying which middleware to use
            function: an instance of the function to call
            **kwargs:

        """
        self.logger.info("Adding %s callback to %s", function, middleware)
        getattr(self, middleware).register_callback(function, **kwargs)

