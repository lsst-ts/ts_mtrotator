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

Start the MTRotator CSC
-----------------------

Start the MTRotator CSC as follows:

.. prompt:: bash

    run_mtrotator

Enable the MTRotator CSC
------------------------

The enable sequence is slightly unusual because the MTRotator and MTMount are closely linked.
The MTRotator will go to fault if it is enabled and not receiving camera cable wrap telemetry from MTMount,
or if it is connected, but the angle difference gets too large.
But MTMount will stop following the rotator if it does not get rotation telemetry from MTRotator.

The recommended sequence is as follows:

* Send the MTRotator to disabled state.
  If this fails, fix the problem before continuing.
* Send the MTMount to enabled state.
  This will give it a chance to center the camera cable wrap on the rotator.
* Enable the MTRotator.

To Recover from Fault State
---------------------------

* To recover from the ``FAULT`` state (after fixing whatever is wrong): send the CSC to standby and then enable it.
  In detail: issue the ``standby``, ``start``, and ``enable`` commands.

Simulator
---------

The CSC includes a simulation mode. To run using CSC's internal simulator:

.. prompt:: bash

    run_mtrotator --simulate

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
