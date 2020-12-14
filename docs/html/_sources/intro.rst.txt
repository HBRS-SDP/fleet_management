Fleet Management System
=========================

The ``ropod-fms`` contains all the components for running the Fleet Management System (FMS) in ROPOD.

.. glossary::

    Fleet Management System (FMS)
        The software components that handle the planning, coordination, monitoring, and management of the fleet of robots.

    Central Control Unit (CCU)
        The central control unit is the physical device on which the FMS is deployed.
        This is a server which is located in the elevator room of Agaplesion, as described in `D1.2 <https://git.ropod.org/ropod/deliverables/d1.2>`_.

    Central Operator Console
        A part of the Graphical User Interface which is aimed at technical users, further detailed in `D5.1 <https://git.ropod.org/ropod/deliverables/d5.1>`_.
        The FMS provides an API to access the required data and is further described in `D1.4 <https://git.ropod.org/ropod/deliverables/d1.4>`_.

Architecture
-------------

.. figure:: /images/CCU.jpg
    :align: center

    FMS component diagram (Last update: 13.08.2019)

FMS core components are those that offer fundamental functionality, mostly to the user, and almost always related to tasks.
The two main components are:

Task manager
^^^^^^^^^^^^^^^^^

Handles transportation service requests from users.

* Dispatcher - responsible for sending the task to the robots, and requesting further replans as part of the system recovery.
* Task monitor - Keeps track of the task execution, in case of delays or failures reported by the robot during the task execution it triggers the task manager for recovery actions, e.g. replanning, rescheduling or reallocating tasks.

Resource manager
^^^^^^^^^^^^^^^^^

Monitors and manages the use of resources like elevators, doors, or parking spots.

* Elevator manager - Acts as an intermediary between the elevators and the elevator control.
* Fleet monitor - Keeps track of the robot status.

Task executor
^^^^^^^^^^^^^^

Dispatches the task plan one action at a time to the appropriate components in the robot. This component is actually run on the robot.
See the `task executor <https://git.ropod.org/ropod/ropod_task_executor>`_ for more information.

Plugins
^^^^^^^^^^^^^^
This are components not developed within ``ropod-fms`` but which offer some functionality required for ROPOD.

* Task planner - Decomposes a task into a list of actions along with their locations
* Task allocation - Assigns and schedules tasks to robots
* Path planner - Computes the list of waypoints between two locations using traffic rules

Please note that although we try to keep the diagram updated, it might not match the current version always.
If you find that is the case, please `open an issue <https://git.ropod.org/ropod/ccu/fleet-management/issues/new?issue%5Bassignee_id%5D=&issue%5Bmilestone_id%5D=>`_.

Task planning
-------------
A simplified version of the workflow to handle MobiDik transportation requests is shown in the figure below to illustrate the information flow between components for planning, resource management, and scheduling.


.. figure:: /images/task-planning.png
    :align: center

    FMS planning workflow for the MobiDik use case.

    FMS components are white. Blue components are run on the robot and gray components belong to a different work package.

Task execution
---------------
A simplified version of the task execution and monitoring workflow to handle MobiDik transportation requests is shown below:

.. _execution-seq-diagram:

.. figure:: /images/task-execution-monitoring.png
    :align: center

    Execution and monitoring of a transportation task.

    FMS components are white, while components running on the robot are shown in blue.
