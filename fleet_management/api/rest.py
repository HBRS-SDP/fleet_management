import falcon
import random
import os
import time
import logging

import gunicorn.app.base
from gunicorn.six import iteritems

waitTime = int(os.environ.get('WAIT_TIME', '2'))


class RandomGenerator(object):
    def on_get(self, request, response):
        time.sleep(waitTime)
        number = random.randint(0, 100)
        result = {'lowerLimit': 0, 'higherLimit': 100, 'number': number}
        response.media = result


class GunicornServer(gunicorn.app.base.BaseApplication):
    def __init__(self, app, options=None):
        self.options = options or {}
        self.application = app
        super(GunicornServer, self).__init__()
        self.logger = logging.getLogger('fms.api.rest')

    def load_config(self):
        config = dict([(key, value) for key, value in iteritems(self.options)
                       if key in self.cfg.settings and value is not None])
        for key, value in iteritems(config):
            self.cfg.set(key.lower(), value)

    def load(self):
        return self.application

    def init(self, **kwargs):
        self.logger.info("Initialised REST interface")


class RESTInterface(object):
    def __init__(self, options=None):
        self.app = falcon.API()
        self.app.add_route('/number', RandomGenerator())
        self.server = GunicornServer(self.app, options)
        self.server.init()

    def add_route(self, route, object):
        self.app.add_route(route, object)

    def start(self):
        self.server.run()


if __name__ == '__main__':
    options = {
        'bind': '%s:%s' % ('127.0.0.1', '8080'),
        'workers': 1,
    }
    api = RESTInterface(options)
    api.start()
