import logging

from OBL import OSMBridge


class OSMBridgeBuilder:
    def __init__(self):
        self._instance = None
        self.logger = logging.getLogger(__name__)

    def __call__(self, **kwargs):
        if not self._instance:
            self.logger.debug("Configuring osm_bridge")
            try:
                self._instance = OSMBridge(**kwargs)
            except Exception as e:
                self.logger.error("There is a problem in connecting to Overpass server. Error: %s", e)
                self._instance = None
            self.logger.info("Connected to osm_bridge (%s:%s)", kwargs.get('server_ip'), kwargs.get('server_port'))
        return self._instance


configure = OSMBridgeBuilder()
