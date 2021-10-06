.. py:currentmodule:: lsst.ts.mtrotator

.. _lsst.ts.mtrotator:

#################
lsst.ts.mtrotator
#################

.. image:: https://img.shields.io/badge/Project Metadata-gray.svg
    :target: https://ts-xml.lsst.io/index.html#index-master-csc-table-mtrotator
.. image:: https://img.shields.io/badge/SAL\ Interface-gray.svg
    :target: https://ts-xml.lsst.io/sal_interfaces/MTRotator.html
.. image:: https://img.shields.io/badge/GitHub-gray.svg
    :target: https://github.com/lsst-ts/ts_mtrotator
.. image:: https://img.shields.io/badge/Jira-gray.svg
    :target: https://jira.lsstcorp.org/issues/?jql=labels+%3D+ts_mtrotator

Overview
========

The MTRotator CSC controls the camera rotator on the Simonyi Survey Telescope.

User Guide
==========

Start the MTRotator CSC as follows:

.. prompt:: bash

    run_mtrotator.py

Then check that the CSC has control of the low-level controller, as follows:

* Wait for the ``connected`` event to report ``command=True`` and ``telemetry=True``.
  This should happen quickly; if it does not then check that the low-level controller is fully booted up and configured to use the correct IP address for the CSC.
* Check the ``controllerState`` event.
  If it is ``state=Offline, offline_substate=PublishOnly``, which is the state the low-level controller wakes up in,
  then you must :ref:`use the EUI to enable DDS mode <lsst.ts.mtrotator.enable_with_eui>`.
* Check the ``commandableByDDS`` event.
  If ``state=False`` then you must :ref:`use the EUI to enable DDS mode <lsst.ts.mtrotator.enable_with_eui>`.

Notes
-----

* To recover from the ``FAULT`` state (after fixing whatever is wrong) issue the ``standby``, ``start``, and ``enable`` commands.

* Communication between the low-level controller and CSC is a bit unusual:

  * The low-level controller does not acknowledge commands in any way.
    Thus the CSC must try to predict whether the low-level controller can execute a command and reject the command if not.
    Unfortunately this prediction cannot be completely accurate.
  * The connection uses two separate sockets, one for commands and the other for telemetry and configuration.
    Both are one-directional: the low-level controller reads commands on the command socket and writes configuration and telemetry to the telemetry socket.

Simulator
---------

The CSC includes a simulation mode. To run using CSC's internal simulator:

.. prompt:: bash

    run_mtrotator.py --simulate

.. _lsst.ts.mtrotator.enable_with_eui:

Enable With the EUI
-------------------

The control mode must be ``DDS`` in order for the CSC to control the low-level controller.
If the control mode is ``GUI`` then you can use the EUI (aka GUI) to change it to ``DDS`` as follows:

* In the main panel: change the state to ``state=Offline, offline_substate=Available``.
* Go to the ``Parameters`` panel to change the control mode to ``DDS``.

Notes:

* The EUI *shows* the control mode on the main panel, but that display is read-only.
  You must use the ``Parameters`` panel to change the control mode.
* If you issue any hexapod command in the EUI, control mode will switch back to ``GUI``.
  So if you want the CSC to retain control, please be careful what you touch when using the GUI.

Developer Guide
===============

.. toctree::
    developer_guide
    :maxdepth: 1

Version History
===============

.. toctree::
    version_history
    :maxdepth: 1
