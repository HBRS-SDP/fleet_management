import logging

from fleet_management.db.models.elevator import Elevator as ElevatorModel, ElevatorStatus
from fmlib.utils.messages import Document
from pymongo.errors import ServerSelectionTimeoutError


class Elevator:

    def __init__(self, elevator_id, **kwargs):
        self.elevator_id = elevator_id
        self._model = ElevatorModel(elevator_id)
        status = ElevatorStatus()
        self._model.status = status

        self.save()

    @classmethod
    def create_new(cls, robot_id, **kwargs):
        return cls(robot_id, **kwargs)

    def archive(self):
        pass

    def save(self):
        try:
            self._model.save()
        except ServerSelectionTimeoutError:
            logging.warning('Could not save models to MongoDB')

    def update_status(self, payload):
        document = Document.from_payload(payload)
        self._model.status = ElevatorStatus.from_document(document)
        self.save()

    def at_goal_floor(self):
        if self._model.status.door_open_at_goal_floor:
            return True
        else:
            return False

    def at_start_floor(self):
        if self._model.status.door_open_at_start_floor:
            return True
        else:
            return False
