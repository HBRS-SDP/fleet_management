import os
import time

import inflection
from fleet_management.db.models.robot import Ropod
from fleet_management.db.models.task import TransportationTask
from fmlib.api.rest.interface import RESTInterface as RESTInterfaceBase

waitTime = int(os.environ.get('WAIT_TIME', '2'))


class RESTInterface(RESTInterfaceBase):

    def __init__(self, server, **kwargs):
        super().__init__(server, **kwargs)
        self.ccu_store = kwargs.get('ccu_store')


def format_msg(msg_dict):
    def _format_msg_keys(value):
        return {inflection.camelize(prop, False): format_msg(value)
                for prop, value in value.items()}

    if isinstance(msg_dict, dict):
        return _format_msg_keys(msg_dict)
    elif isinstance(msg_dict, list):
        formatted_list = list()
        for item in msg_dict:
            formatted_list.append(format_msg(item))
        return formatted_list
    else:
        return str(msg_dict)


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
