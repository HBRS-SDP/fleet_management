from __future__ import print_function
import time
from fleet_management.config.config_file_reader import ConfigFileReader
from fleet_management.db.ccu_store import CCUStore
from fleet_management.task_manager import TaskManager

if __name__ == '__main__':
    config_params = ConfigFileReader.load("../config/ccu_config.yaml")
    ccu_store = CCUStore(config_params.ccu_store_db_name)
    task_manager = TaskManager(config_params, ccu_store)
    task_manager.restore_task_data()

    print("FMS initialized")

    try:
        while True:
            task_manager.dispatch_tasks()
            task_manager.resend_message_cb()
            time.sleep(0.5)
    except (KeyboardInterrupt, SystemExit):
        task_manager.shutdown()
        print('FMS interrupted; exiting')
