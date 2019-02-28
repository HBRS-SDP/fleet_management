import time
import os.path
import logging
from ropod.utils.logging.config import config_logger

from fleet_management.config.config_file_reader import ConfigFileReader
from fleet_management.db.ccu_store import CCUStore
from fleet_management.task_manager import TaskManager


if __name__ == '__main__':
    code_dir = os.path.abspath(os.path.dirname(__file__))
    main_dir = os.path.dirname(code_dir)

    log_config_file = os.path.join(main_dir, 'config/logging.yaml')
    config_logger(log_config_file)

    logging.info("Configuring FMS ...")
    config_file = os.path.join(main_dir, "config/ccu_config.yaml")

    config_params = ConfigFileReader.load(config_file)
    ccu_store = CCUStore(config_params.ccu_store_db_name)
    task_manager = TaskManager(config_params, ccu_store)
    task_manager.restore_task_data()

    logging.info("FMS initialized")

    try:
        task_manager.start()
        while True:
            task_manager.dispatch_tasks()
            task_manager.resend_message_cb()
            time.sleep(0.5)
    except (KeyboardInterrupt, SystemExit):
        task_manager.shutdown()
        logging.info('FMS interrupted; exiting')
