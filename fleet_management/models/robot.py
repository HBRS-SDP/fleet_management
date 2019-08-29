import logging

from fleet_management.db.models.robot.robot import Position
from fleet_management.db.models.robot.status import Availability, ComponentStatus, RobotStatus
from fleet_management.db.models.ropod.ropod import BlackBoxModel, Platform, Ropod, RopodSoftwareStack, RopodVersion
from pymodm.context_managers import switch_collection
from pymongo.errors import ServerSelectionTimeoutError


class Robot:

    def __init__(self, robot_id, **kwargs):
        self.robot_id = robot_id
        self._model = Ropod(robot_id)
        component_status = ComponentStatus()
        availability = Availability()
        hw_version = Platform()
        sw_version = RopodSoftwareStack()
        black_box = BlackBoxModel('black_box_001') # TODO This shouldn't be hardcoded
        position = Position()

        self._model.status = RobotStatus(availability=availability, component_status=component_status)
        self._model.version = RopodVersion(hardware=hw_version, software=sw_version, black_box=black_box)
        self._model.position = position

        self.save()

    @classmethod
    def create_new(cls, robot_id, **kwargs):
        return cls(robot_id, **kwargs)

    def archive(self):
        with switch_collection(Ropod, Ropod.Meta.archive_collection):
            self._model.save(force_insert=True)

    def update(self):
        pass

    def save(self):
        try:
            self._model.save()
        except ServerSelectionTimeoutError:
            logging.warning('Could not save models to MongoDB')

    def update_position(self, **kwargs):
        self._model.position = Position.from_document(kwargs)
        self.save()
