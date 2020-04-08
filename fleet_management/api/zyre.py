from fmlib.api.zyre import ZyreInterface as ZyreInterfaceBase
from fmlib.utils.messages import format_document

from fleet_management.db.models.messages import Message as MessageModel


class ZyreInterface(ZyreInterfaceBase):

    def __init__(self, zyre_node, **kwargs):
        self.ccu_store = kwargs.get('ccu_store')
        super().__init__(zyre_node, **kwargs)

    def receive_msg_cb(self, msg_content):
        dict_msg = self.convert_zyre_msg_to_dict(msg_content)
        if dict_msg is None:
            self.logger.warning("Message is not a dictionary")
            return

        message_type = dict_msg['header']['type']
        # Ignore messages not declared in our message type
        if message_type not in self.message_types:
            return

        if self.ccu_store:
            header = format_document(dict_msg['header'])
            payload = format_document(dict_msg['payload'])
            document = dict(**header, payload=payload)
            msg_model = MessageModel.from_document(document)
            msg_model.save()

        super().receive_msg_cb(msg_content)

    def add_ccu_plugin(self, ccu_store):
        self.ccu_store = ccu_store
