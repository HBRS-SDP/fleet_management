import logging

from ropod.structs.elevator import Elevator


class ElevatorMonitor:
    def __init__(self, elevator_id, ccu_store, api):
        self.logger = logging.getLogger(__name__)
        self.id = elevator_id
        self.ccu_store = ccu_store
        self.api = api
        self.elevator = Elevator(self.id)
        self.ccu_store.add_elevator(self.elevator)

    def elevator_status_cb(self, msg):
        payload = msg.get('payload')
        self.elevator.update(payload)
        if self.elevator.at_start_floor:
            self.logger.info('Elevator reached start floor; waiting for confirmation...')
        elif self.elevator.at_goal_floor:
            self.logger.info('Elevator reached goal floor; waiting for confirmation...')
        self.ccu_store.update_elevator(self.elevator)
