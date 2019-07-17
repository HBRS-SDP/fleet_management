import logging
from ropod.pyre_communicator.base_class import RopodPyre


class FMSZyreAPI(RopodPyre):
    def __init__(self, zyre_config):
        super().__init__(zyre_config)
        self.logger = logging.getLogger('fms.api.zyre')
        self.callback_dict = dict()

    def add_callback(self, class_instance, msg_type, component, callback):
        obj = getattr(class_instance, component)
        callback_funct = getattr(obj, callback)
        self.__dict__[callback_funct.__name__] = callback_funct
        self.callback_dict[msg_type] = callback_funct.__name__

    def receive_msg_cb(self, msg_content):
        dict_msg = self.convert_zyre_msg_to_dict(msg_content)
        if dict_msg is None:
            self.logger.warning("Message is not a dictionary")
            return

        message_type = dict_msg['header']['type']
        payload = dict_msg.get('payload')
        self.logger.warning("Received %s message", message_type)

        try:
            callback = self.callback_dict.get(message_type, None)
            if callback is None:
                raise AttributeError
        except AttributeError:
            self.logger.error("No callback function found for %s messages. Callback dictionary: %s",
                              message_type, self.callback_dict)

        try:
            getattr(self, callback)(dict_msg)
        except Exception:
            self.logger.error("Could not execute callback %s ", callback, exc_info=True)
