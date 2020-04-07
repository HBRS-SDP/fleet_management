from fmlib.api.api import API as APIBase
from fleet_management.api.zyre import ZyreInterface


class API(APIBase):

    @classmethod
    def get_zyre_api(cls, zyre_config):
        """Create an object of type ZyreInterface

        Args:
            zyre_config: A dictionary containing the API configuration

        Returns:
            A configured ZyreInterface object

        """
        zyre_api = ZyreInterface(**zyre_config)
        return zyre_api

    def _configure(self, config_params):
        super()._configure(config_params)
        ccu_store = config_params.get('ccu_store')
        self.zyre.add_ccu_plugin(ccu_store)
