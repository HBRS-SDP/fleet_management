"""This module contains the API class that allows components to receive and
send messages through the network using a variety of middlewares

"""

import logging

from mrs.utils.models import MRSMessageFactory
from ropod.utils.models import MessageFactoryBase
from ropod.utils.models import RopodMessageFactory

from fleet_management.api.rest.interface import RESTInterface
from fleet_management.api.ros import ROSInterface
from fleet_management.api.zyre import FMSZyreAPI


class API:
    """ API object serves as a facade to different middlewares

    Attributes:
        publish_dict: A dictionary that maps
        middleware_collection: A list of supported middlewares obtained from the config file
        config_params: A dictionary containing the parameters loaded from the config file
        message_factory: An object of type MessageFactoryBase to create message templates

    """

    # pylint: disable=too-many-instance-attributes
    def __init__(self, middleware, **kwargs):
        """Initializes API with a configuration file

        Args:
            middleware: a list of middleware to configure.
            The keyword arguments should containing the desired configuration
            matching the middleware listed
        """
        self.logger = logging.getLogger('fms.api')

        self._version = kwargs.get('version')

        self.publish_dict = dict()
        self.interfaces = list()
        self.config_params = dict()
        self.middleware_collection = middleware
        self.__configure(kwargs)
        self.message_factory_base = MessageFactoryBase()
        self.configure_message_factory()

        self.logger.info("Initialized API")

    def configure_message_factory(self):
        ropod_message_factory = RopodMessageFactory()
        mrs_message_factory = MRSMessageFactory()

        self.message_factory_base.register_factory(RopodMessageFactory.__name__,
                                                   ropod_message_factory)
        self.message_factory_base.register_factory(MRSMessageFactory.__name__,
                                                   mrs_message_factory)

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
            config = config_params.get(option, None)
            self.config_params[option] = config
            if config is None:
                self.logger.warning("Option %s present, but no configuration was found", option)
                self.__dict__[option] = None
                continue

            self.logger.debug("Configuring %s API", option)
            interface = None
            if option == 'zyre':
                interface = API.get_zyre_api(config)
            elif option == 'ros':
                interface = API.get_ros_api(config)
            elif option == 'rest':
                interface = API.get_rest_api(config)

            self.__dict__[option] = interface
            self.interfaces.append(interface)

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
        return ROSInterface(**ros_config)

    @staticmethod
    def get_rest_api(rest_config):
        """Create an object of type RESTInterface

        Args:
            rest_config: A dictionary containing the API configuration

        Returns:
            A configured RESTInterface object

        """
        return RESTInterface(**rest_config)

    def create_message(self, contents, **kwargs):
        """Creates a message in the right format using the MessageFactory

        Args:
            contents: message contents
            **kwargs: recipients

        Returns:
            A filled JSON message

        """
        message_name = type(contents).__name__
        message_factory = self.message_factory_base.get_factory(message_name)

        return message_factory.create_message(contents, **kwargs)

    def register_callbacks(self, obj, callback_config=None):
        for option in self.middleware_collection:
            if callback_config is None:
                option_config = self.config_params.get(option, None)
            else:
                option_config = callback_config.get(option, None)

            if option_config is None:
                logging.warning("Option %s has no configuration", option)
                continue

            callbacks = option_config.get('callbacks', list())
            for callback in callbacks:
                component = callback.get('component', None)
                try:
                    function = _get_callback_function(obj, component)
                except AttributeError as err:
                    self.logger.error("%s. Skipping %s callback.", err, component)
                    continue
                self.__register_callback(option, function, **callback)

    def __register_callback(self, middleware, function, **kwargs):
        """Adds a callback function to the right middleware

        Args:
            middleware: a string specifying which middleware to use
            function: an instance of the function to call
            **kwargs:

        """
        self.logger.info("Adding %s callback to %s", function, middleware)
        getattr(self, middleware).register_callback(function, **kwargs)

    def start(self):
        """Start the API components
        """
        for interface in self.interfaces:
            interface.start()

    def shutdown(self):
        """Shutdown all API components
        """
        for interface in self.interfaces:
            interface.shutdown()

    def run(self):
        """Execute the API's specific methods
        """
        for interface in self.interfaces:
            interface.run()


def _get_callback_function(obj, component):
    objects = component.split('.')
    child = objects.pop(0)
    if child:
        parent = getattr(obj, child)
    else:
        parent = obj
    while objects:
        child = objects.pop(0)
        parent = getattr(parent, child)

    return parent
