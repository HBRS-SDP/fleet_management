from OBL import OSMBridge


class OSMBridgeBuilder:
    def __init__(self):
        self._instance = None

    def __call__(self, **kwargs):
        if not self._instance:
            self._instance = OSMBridge(**kwargs)
        return self._instance


configure = OSMBridgeBuilder()
