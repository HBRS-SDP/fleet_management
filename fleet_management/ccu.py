from __future__ import print_function
import time
import os.path
from fleet_management.config.config_file_reader import ConfigFileReader
from fleet_management.db.ccu_store import CCUStore
from fleet_management.task_manager import TaskManager
import logging
from ropod.utils.logging import ColorizingStreamHandler


if __name__ == '__main__':
    logPath = '.'
    fileName = 'fms'

    rootLogger = logging.getLogger()
    rootLogger.setLevel(logging.DEBUG)

    # Save a log to a file
    fileHandler = logging.FileHandler("{0}/{1}.log".format(logPath, fileName))
    fileHandler.setLevel(logging.DEBUG)

    # Print the log output to the console
    # consoleHandler = logging.StreamHandler()
    consoleHandler = ColorizingStreamHandler()
    consoleHandler.setLevel(logging.DEBUG)

    # Add a formatter to the handlers
    logFormatter = logging.Formatter("[%(levelname)-5.5s]  %(asctime)s [%(name)-25.25s] %(message)s")
    fileHandler.setFormatter(logFormatter)
    consoleHandler.setFormatter(logFormatter)

    # Add handlers to the logger
    rootLogger.addHandler(fileHandler)
    rootLogger.addHandler(consoleHandler)


    logging.info("Configuring FMS ...")
    code_dir = os.path.abspath(os.path.dirname(__file__))
    main_dir = os.path.dirname(code_dir)
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
        print('FMS interrupted; exiting')
