.. _fms-tests:

FMS tests
==========

Task requests
----------------

The :ref:`test cases <test-cases>` defined in the FMS allow to test the integration with edge cases of MobiDik task requests, and are roughly summarized below.

Valid requests:

* pickup and delivery on the same floor

* pickup and delivery on different floors (2x)

* current position and pickup position are the same

* current position, pickup and delivery all on different floors

Invalid requests:

- Invalid areas (e.g. no docking or undocking)

- Pickup and delivery are the same area

.. _test-cases:

Test cases
^^^^^^^^^^^^^^

.. literalinclude:: ../fleet_management/test/fixtures/msgs/task/requests/brsu/test-cases.yaml
    :language: yaml


Running the test
^^^^^^^^^^^^^^^^^^


1. Make sure MongoDB and Overpass are running. If they are not, you can use the ``docker-compose`` services like this:
::

    docker-compose up -d mongo osm

This will start ``overpass`` with the ``brsu`` tag and MongoDB with the default configuration.

2. Run the FMS::

    python3 ccu.py


3. Launch a zyre robot::

    cd fleet_management/proxies/
    python3 robot.py ropod_001

4. Run the test using ``--case <X>`` or ``--all``::

    python3 task_request_test.py

By default, the ``task_request_test`` is using the option ``--case 4`` of the available test cases.
