Task management
================


.. automodule:: fleet_management.task.manager
    :members:
    :undoc-members:
    :private-members:

Monitoring
------------

The FMS will receive task status updates from the robot.
The relevant ROPOD `action codes <https://git.ropod.org/ropod/communication/ropod_ros_msgs/blob/develop/task/Status.msg>`_ for the FMS are:

* Domain code: 3 (Robot)
* Module code: 7 (Task executor)
* Status code: X (Defined for now in ``ropod_common``)

.. automodule:: fleet_management.task.monitor
    :members:
    :undoc-members:
    :private-members:

Dispatching
-------------

.. automodule:: fleet_management.task.dispatcher
    :members:
    :undoc-members:
    :private-members:
