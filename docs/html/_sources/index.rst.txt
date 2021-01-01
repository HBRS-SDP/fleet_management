.. ropod-fms documentation master file, created by
   sphinx-quickstart on Wed Jul 10 11:15:24 2019.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to ropod-fms's documentation!
=====================================

.. toctree::
   :name: tocmain
   :caption: Overview
   :hidden:
   :glob:
   :maxdepth: 2

    Intro <intro>
    Modules <fms>
    Plugins <plugins>
    Tests <tests>
    Limitations <limitations>

The ``ropod-fms`` contains all the components for running the Fleet Management System (FMS) in ROPOD, which are used to handle the planning, coordination, monitoring, and management of the fleet of robots.
The FMS is also the link between the users and the robots, and its main responsibility is handling task requests, dispatching them to the robots and monitoring the task progress.

For more information about the architecture and components see :doc:`intro`.


Getting started
================

To setup your development environment follow the instructions outlined in https://git.ropod.org/ropod/ccu/setup

Running tests
===============

The tests in the repository have been split into two different modules:

* ``fms``: unit tests that do not depend on any external services, e.g. pyre, mongo, overpass, etc. The structure inside should replicate that of ``fleet_management``.
* ``integration``: integration tests with different parts of the ROPOD system

For more details about what tests can be run and how to do it, see :ref:`Tests <fms-tests>`


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
