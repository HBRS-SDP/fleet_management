from mrs.task_execution.dispatcher import Dispatcher as MRTADispatcher


class Dispatcher(MRTADispatcher):
    def __init__(self, robot_id, ccu_store, task_cls, stp_solver, corrective_measure, freeze_window, api, auctioneer):
        super().__init__(robot_id, ccu_store, task_cls, stp_solver, corrective_measure, freeze_window, api, auctioneer)
