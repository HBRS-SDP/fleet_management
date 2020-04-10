import os
import time

from fleet_management.db.models.task import TransportationTask
from fleet_management.db.models.robot import Ropod
from fmlib.api.rest.interface import RESTInterface as RESTInterfaceBase
from fmlib.utils.messages import format_msg

waitTime = int(os.environ.get('WAIT_TIME', '2'))


class RESTInterface(RESTInterfaceBase):

    def __init__(self, server, **kwargs):
        super().__init__(server, **kwargs)
        self.ccu_store = kwargs.get('ccu_store')


class RESTResource:

    def __init__(self, **kwargs):
        self.ccu_store = kwargs.get('ccu_store')


class Task(RESTResource):

    def on_get(self, request, response):
        time.sleep(waitTime)
        tasks = list()
        for task in TransportationTask.objects.all():
            json_task = format_msg(task.to_dict())
            tasks.append(json_task)
        result = {'tasks': tasks}
        response.media = result


class Robots(RESTResource):

    def on_get(self, request, response):

        time.sleep(waitTime)
        robots = list()
        for robot in Ropod.objects.all():
            json_robot = format_msg(robot.to_dict())
            robots.append(json_robot)

        result = {'robots': robots}
        response.media = result
