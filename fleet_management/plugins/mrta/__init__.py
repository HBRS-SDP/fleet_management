from mrs.config.mrta import MRTAFactory
from fmlib.config.params import ConfigParams as ConfigParamsBase
from mrs.task_allocation.auctioneer import Auctioneer
from mrs.task_execution.dispatcher import Dispatcher
from mrs.task_allocation.bidder import Bidder


class ConfigParams(ConfigParamsBase):
    default_config_module = 'fleet_management.config.default'


config = ConfigParams.default()
allocation_method = config.component('allocation_method', config)

configure = MRTAFactory(allocation_method)
configure.register_component('auctioneer', Auctioneer)
configure.register_component('dispatcher', Dispatcher)
configure.register_component('bidder', Bidder)
