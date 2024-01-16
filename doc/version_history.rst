.. py:currentmodule:: lsst.ts.mtrotator

.. _lsst.ts.mtrotator.version_history:

###############
Version History
###############

v1.1.3
------

* Add the new commands to set the emergency acceleration and jerk at runtime.

Requires:

* ts_rotator_controller 1.6.5
* ts_xml 20.2.0

v1.1.2
------

* Support the **mypy**.

v1.1.1
------

* Fix the enumeration of fault substate.
* Improve the unit test to be more robust.

v1.1.0
------

* Updated for low-level controller 2.0.
  This includes a simpler state machine and some new configuration fields.
* Add the ``drives_enabled`` field to **Config** class.
* Add the ``motor_current`` field to **Telemetry** class.
* Publish the ``copleyFaultStatus`` data.
* Add the ``ENABLE_DRIVES`` field to **CommandCode** enum.
* Import the enums from **ts_xml** instead of **ts_idl**.
* Update the ``.ts_pre_commit_config.yaml``.
* Add the argument to bypass the check of camera cable wrapper.

Requires:

* ts_rotator_controller 1.6.1
* ts_hexrotcomm 1.1.2
* ts_salobj 8.0
* ts_idl 3.4
* ts_xml 20.1.0
* MTHexapod, MTMount, and MTRotator IDL files built from ts_xml 14.

v1.0.3
------

* Fix the test case: ``test_track_good()``.

v1.0.2
------

* Switch to python generic in conda recipe.

v1.0.1
------

* Update conda recipe.

v1.0.0
------

* Use ts_pre_commit_conf.
* ``Jenkinsfile``: use new shared library.
* Remove scons support.

Requires:

* ts_rotator_controller 1.5.0
* ts_hexrotcomm 0.29
* ts_salobj 7.1
* ts_idl 3.4
* MTHexapod, MTMount, and MTRotator IDL files built from ts_xml 14.

v0.26.1
-------

* pre-commit: update black to 23.1.0, isort to 5.12.0, mypy to 1.0.0, and pre-commit-hooks to v4.4.0.
* ``Jenkinsfile``: do not run as root.
* ``conda/meta.yaml``: remove unneeded entry_points section.

Requires:

* ts_rotator_controller 1.5.0
* ts_hexrotcomm 0.29
* ts_salobj 7.1
* ts_idl 3.4
* MTRotator and MTMount IDL files built from ts_xml 15

v0.26.0
-------

* `RotatorCsc`: write the clockOffset event. This requires ts_xml 15.

Requires:

* ts_rotator_controller 1.5.0
* ts_hexrotcomm 0.29
* ts_salobj 7.1
* ts_idl 3.4
* MTRotator and MTMount IDL files built from ts_xml 15

v0.25.2
-------

* `RotatorCommander`:

    * Avoid a flood of ``motors`` messages.
    * Improve the accuracy of the rate at which tracking commands are issued by the ``cosine`` and ``ramp`` commands.

Requires:

* ts_rotator_controller 1.5.0
* ts_hexrotcomm 0.29
* ts_salobj 7.1
* ts_idl 3.4
* MTRotator and MTMount IDL files built from ts_xml 11.1

v0.25.1
-------

* `MTRotatorCsc`: call super().start() first in the start method, for compatibility with the Kafka version of ts_salobj.
* Make test_move and test_tracking_good in tests/test_csc.py more robust by eliminating a race condition.
* Fix Jenkins CI file by changing HOME to WHOME everywhere except the cleanup section.
* Add setupRequired(ts_config_mttcs) to the ups table file.

Requires:

* ts_rotator_controller 1.5.0
* ts_hexrotcomm 0.29
* ts_salobj 7.1
* ts_idl 3.4
* MTRotator and MTMount IDL files built from ts_xml 11.1

v0.25.0
-------

* Rename command-line scripts to remove ".py" suffix.
* Remove the ``run_mock_rotator_pxi.py`` command-line script.
* Update a test to be compatible with ts_xml 12.
* Updated for ts_rotator_controller 1.5.0, which is required.
* Build with pyproject.toml.

Requires:

* ts_rotator_controller 1.5.0
* ts_hexrotcomm 0.29
* ts_salobj 7.1
* ts_idl 3.4
* MTRotator and MTMount IDL files built from ts_xml 11.1

v0.24.1
-------

* Stop publishing motors.calibrated telemetry.
  The new low-level controller won't provide the necessary information and we want to remove it from the XML.

Requires:

* ts_rotator_controller 1.4.0
* ts_hexrotcomm 0.29
* ts_salobj 7.1
* ts_idl 3.4
* MTRotator and MTMount IDL files built from ts_xml 11.1

v0.24.0
-------

* `RotatorCsc`: call ``super().start()`` at the beginning of the start method.
  This requires ts_salobj 7.1.
* ``setup.cfg``: set asyncio_mode = auto.
* git ignore .hypothesis.

Requires:

* ts_rotator_controller 1.4.0
* ts_hexrotcomm 0.29
* ts_salobj 7.1
* ts_idl 3.4
* MTRotator and MTMount IDL files built from ts_xml 11

v0.23.0
-------

* Update for ts_salobj v7, ts_xml 11, and ts_hexrotcomm 0.29, all of which are required.

Requires:

* ts_rotator_controller 1.4.0
* ts_hexrotcomm 0.29
* ts_salobj 7
* ts_idl 3.4
* MTRotator and MTMount IDL files built from ts_xml 11

v0.22.0
-------

* Update for ts_hexrotcomm 0.28 and ts_rotator_controller 1.4.0:

    * Remove support for the sync_pattern field in low-level commands.
    * Remove ``FRAME_ID`` class constants from the `Config` and `Telemetry` structs, because frame IDs are now standardized.

Requires:

* ts_rotator_controller 1.4.0
* ts_hexrotcomm 0.28
* ts_salobj 6.8
* ts_idl 3.4
* ts_xml 10.2
* MTRotator and MTMount IDL files, e.g. made using ``make_idl_files.py MTRotator MTMount``


v0.21.0
-------

* `RotatorCsc`: changed the interlock event's field from ``detail`` (a string) to ``enabled`` (a boolean).
  This change requires ts_xml 10.2.
  This change also requires ts_hexrotcomm 0.27 (only because it has other changes that require ts_xml 10.2).

Requires:

* ts_rotator_controller 1.3.0
* ts_hexrotcomm 0.27
* ts_salobj 6.8
* ts_idl 3.4
* ts_xml 10.2
* MTRotator and MTMount IDL files, e.g. made using ``make_idl_files.py MTRotator MTMount``

v0.20.0
-------

* Updated unit tests for compatibility with ts_salobj 6.8, which is now required.
* ``setup.cfg``: update to not check version.py.
* `CONFIG_SCHEMA`: update $id github link from master to main.

Requires:

* ts_rotator_controller 1.3.0
* ts_hexrotcomm 0.23
* ts_salobj 6.8
* ts_idl 3.4
* ts_xml 7.1
* MTRotator and MTMount IDL files, e.g. made using ``make_idl_files.py MTRotator MTMount``

v0.19.1
-------

* Update the default host to ``rot-pxi-controller.cp.lsst.org``.

Requires:

* ts_rotator_controller 1.3.0
* ts_hexrotcomm 0.23
* ts_salobj 6.3
* ts_idl 3.4
* ts_xml 7.1
* MTRotator and MTMount IDL files, e.g. made using ``make_idl_files.py MTRotator MTMount``

v0.19.0
-------

* Update for ts_hexrotcomm 0.23, which is required:

    * Run the TCP/IP clients in the CSC and the servers in the mock controller.
    * Disassociated controller state from CSC state.
      As part of the ``standby`` command the CSC connects to the low-level controller.
      As part of the ``enable`` command the CSC attempts to enable the low-level controller
      (including resetting errors if the low-level controller is in fault state).
    * The CSC is no longer alive in the OFFLINE state, and no longer supports the enterControl command.
    * Added ``host``, ``port``, and ``connection_timeout`` fields to the CSC configuration.

* Update to use `lsst.ts.idl.enums.MTRotator.ErrorCode`, which requires ts_idl 3.4.
* setup.cfg: add an [options] section.

Requires:

* ts_rotator_controller 1.3.0
* ts_hexrotcomm 0.23
* ts_salobj 6.3
* ts_idl 3.4
* ts_xml 7.1
* MTRotator and MTMount IDL files, e.g. made using ``make_idl_files.py MTRotator MTMount``

v0.18.0
-------

* Updated for ts_hexrotcomm v0.22.0, which is required.
  Fix a deprecation warning: stop specifying the `isbefore` argument when calling `assert_summary_state`.
* Updated to use ts_utils, which is required.

Requires:

* ts_rotator_controller 1.2.2
* ts_hexrotcomm 0.22
* ts_utils 1
* ts_salobj 6.3
* ts_simactuators 1
* ts_idl 2.2
* ts_xml 10
* MTRotator and MTMount IDL files, e.g. made using ``make_idl_files.py MTRotator MTMount`` using ts_xml 9


v0.17.0
-------

* Update for ts_xml 10.0, which is required.
* Write new motors.torque, and rotation.odometer telemetry fields.

Requires:

* ts_rotator_controller 1.2.2
* ts_hexrotcomm 0.20
* ts_salobj 6.3
* ts_simactuators 1
* ts_idl 2.2
* ts_xml 10
* MTRotator and MTMount IDL files, e.g. made using ``make_idl_files.py MTRotator MTMount`` using ts_xml 9

v0.16.1
-------

* Fix incorrect frame IDs for messages from the low-level controller.

Requires:

* ts_rotator_controller 1.2.2
* ts_hexrotcomm 0.20
* ts_salobj 6.3
* ts_simactuators 1
* ts_idl 2.2
* ts_xml 9
* MTRotator and MTMount IDL files, e.g. made using ``make_idl_files.py MTRotator MTMount`` using ts_xml 9

v0.16.0
-------

* Updated for ts_rotator_controller 1.2.2, which is required:
  the telemetry data from the low-level controller now matches what is sent to the EUI.

Requires:

* ts_rotator_controller 1.2.2
* ts_hexrotcomm 0.20
* ts_salobj 6.3
* ts_simactuators 1
* ts_idl 2.2
* ts_xml 9
* MTRotator and MTMount IDL files, e.g. made using ``make_idl_files.py MTRotator MTMount`` using ts_xml 9

v0.15.1
-------

* Improve the `inPosition` event by using more appropriate flags from the low-level controller.
* Improve robustness of test_missing_ccw_telemetry in tests/test_csc.py: cancelling the mock CCW telemetry task did not always work.
* Fix the Jenkins job by installing ts_tcpip.

Requires:

* ts_rotator_controller 1.1.6
* ts_hexrotcomm 0.20
* ts_salobj 6.3
* ts_simactuators 1
* ts_idl 2.2
* ts_xml 9
* MTRotator and MTMount IDL files, e.g. made using ``make_idl_files.py MTRotator MTMount`` using ts_xml 9

v0.15.0
-------

* Updated for ts_rotator_controller 1.1.6 and ts_hexrotcomm 0.20, both of which are required:
  messages from low-level controller now contain TAI unix time instead of UTC in the header.

Requires:

* ts_rotator_controller 1.1.6
* ts_hexrotcomm 0.20
* ts_salobj 6.3
* ts_simactuators 1
* ts_idl 2.2
* ts_xml 9
* MTRotator and MTMount IDL files, e.g. made using ``make_idl_files.py MTRotator MTMount`` using ts_xml 9

v0.14.0
-------

* Publish the new ``ccwFollowingError`` telemetry topic. This requires ts_xml 9.
* Stop publishing the deprecated ``application`` telemetry topic.
* Use `unittest.IsolatedAsyncioTestCase` instead of the abandoned ``asynctest`` package.
* Use pre-commit instead of a custom pre-commit hook; see the README.md for instructions.
* Format the code with black 20.8b1.

Requires:

* ts_hexrotcomm 0.16
* ts_salobj 6.3
* ts_simactuators 1
* ts_idl 2.2
* ts_xml 9
* MTRotator and MTMount IDL files, e.g. made using ``make_idl_files.py MTRotator MTMount`` using ts_xml 9

v0.13.0
-------

* `RotatorCsc`: save the configuration schema in code instead of a separate .yaml file.
  This requires ts_salobj 6.3 and ts_hexrotcomm 0.16.
* Delete obsolete file ``schema/MTRotator.yaml``.
* Users's Guide: improve the information for switching from GUI to DDS mode.

Requires:

* ts_hexrotcomm 0.16
* ts_salobj 6.3
* ts_simactuators 1
* ts_idl 2.2
* ts_xml 7.2
* MTRotator and MTMount IDL files, e.g. made using ``make_idl_files.py MTRotator MTMount``

v0.12.0
-------

* Added the ``fault`` command, which requires ts_xml 7.2.
* Updated to monitor camera cable wrap following error.

    * Added configuration parameters ``max_ccw_following_error`` and ``num_ccw_following_errors``.
    * Change `MTRotatorCsc` to refuse to go into the enabled state unless it is receiving telemetry from ``MTMount``.
    * Change `MTRotatorCsc` to go from ENABLED to FAULT state if the camera cable wrap has too much following error.
* `MTRotatorCsc`: set class variable ``version``, which sets the ``cscVersion`` field of the ``softwareVersions`` event.
* Fix the Jenkins job: build the ``MTMount`` IDL file.
* Modernize ``doc/conf.py`` for documenteer 0.6.

Requires:

* ts_hexrotcomm 0.14
* ts_salobj 6.1
* ts_simactuators 1
* ts_idl 2.2
* ts_xml 7.2
* MTRotator and MTMount IDL files, e.g. made using ``make_idl_files.py MTRotator MTMount``

v0.11.0
-------

* Updated to use device-specific TCP/IP ports.
  This requires ts_hexrotcomm v0.14.
* Update `RotatorCommander` to round motors data to 1 digit to reduce spurious output.

Requires:

* ts_hexrotcomm 0.14
* ts_salobj 6.1
* ts_simactuators 1
* ts_idl 2.2
* ts_xml 7.0
* MTRotator IDL files, e.g. made using ``make_idl_files.py MTRotator``

v0.10.3
-------

* Fix an error in RotatorCommander.

Requires:

* ts_hexrotcomm 0.12
* ts_salobj 6.1
* ts_simactuators 1
* ts_idl 2.2
* ts_xml 7.0
* MTRotator IDL files, e.g. made using ``make_idl_files.py MTRotator``

v0.10.2
-------

* Work around incorrectly reported time in telemetry headers (DM-28224).
* Fix incorrect values for ``actualVelocity`` and ``debugActualVelocityB`` in the ``rotation`` telemetry topic.

Requires:

* ts_hexrotcomm 0.12
* ts_salobj 6.1
* ts_simactuators 1
* ts_idl 2.2
* ts_xml 7.0
* MTRotator IDL files, e.g. made using ``make_idl_files.py MTRotator``

v0.10.1
-------

* Update Jenkinsfile.conda to use the shared library.
* Pin the versions of ts_idl and ts_salobj in conda/meta.yaml.

Requires:

* ts_hexrotcomm 0.12
* ts_salobj 6.1
* ts_simactuators 1
* ts_idl 2.2
* ts_xml 7.0
* MTRotator IDL files, e.g. made using ``make_idl_files.py MTRotator``

v0.10.0
-------

* Update to use and require ts_hexrotcomm 0.12:

    * Add argument ``curr_tai`` to `MockMTRotatorController.update_telemetry` and use it.

* Update the mock controller to report generated path data instead of target data
  in the telemetry fields used to set the demand fields of the rotation and application telemetry topics.
  This matches what the real rotator does.
* Update the unit tests to handle the new rotation and application telemetry data.
* Rename the `Telemetry` struct demand field names to clarify their content.
* Update the rotator commander to handle the rotation telemetry event better.
  Ignore the timestamp field when deciding whether the information has changed enough to justify printing the new sample.
  Update the custom motors telemetry callback to work in the same way, ignoring the raw field when deciding whether to print the data.

Requires:

* ts_hexrotcomm 0.12
* ts_salobj 6.1
* ts_simactuators 1
* ts_idl 2.2
* ts_xml 7
* MTRotator IDL files, e.g. made using ``make_idl_files.py MTRotator``

v0.9.0
------

* Updated to use and require ts_salobj 7.0, ts_idl 2.2, and ts_hexrotcomm 0.11:

    * Rename the SAL component ``Rotator`` to ``MTRotator``.
    * Rename ts_idl ``Rotator`` enum module to ``MTRotator``.

* Rename the package from ``ts_rotator`` to ``ts_mtrotator``.

Requires:

* ts_hexrotcomm 0.11
* ts_salobj 6.1
* ts_simactuators 1
* ts_idl 2.2
* ts_xml 7
* MTRotator IDL files, e.g. made using ``make_idl_files.py MTRotator``

v0.8.0
------

* Updated to use and require ts_salobj 6.1 and ts_hexrotcomm 0.10.
* Update the handling of initial_state in `RotatorCsc`:

    * If initial_state != OFFLINE then report all transitional summary states and controller states at startup.
    * Require initial_state = OFFLINE unless simulating.

Requires:

* ts_hexrotcomm 0.7
* ts_salobj 6.1
* ts_simactuators 1
* ts_idl 2
* ts_xml 6.2
* Rotator IDL files, e.g. made using ``make_idl_files.py Rotator``

v0.7.3
------

* Use the time in the telemetry header to set the ``rotation`` telemetry topic's time stamp.

Requires:

* ts_hexrotcomm 0.7
* ts_salobj 5.11 or 6.0
* ts_simactuators 1
* ts_idl 1.4, or 2 with salobj 6.0
* ts_xml 6.2
* Rotator IDL files, e.g. made using ``make_idl_files.py Rotator``


v0.7.2
------

* Fix Jenkinsfile.conda.

Requires:

* ts_hexrotcomm 0.7
* ts_salobj 5.11 or 6.0
* ts_simactuators 1
* ts_idl 1.4 with salobj 5, or 2 with salobj 6
* ts_xml 6.2
* Rotator IDL files, e.g. made using ``make_idl_files.py Rotator``

v0.7.1
------

* Fix conda build.

Requires:

* ts_hexrotcomm 0.7
* ts_salobj 5.11 or 6
* ts_simactuators 1
* ts_idl 1.4, or 2 with salobj 6
* ts_xml 6.2
* Rotator IDL files, e.g. made using ``make_idl_files.py Rotator``


v0.7.0
------

* Updated to read telemetry from a newer version of the low-level controller: changes added in https://jira.lsstcorp.org/browse/DM-25994.
* Updated to write new event and telemetry information added in ts_xml 6.2.
* Use corrected spelling of ``Rotator.ApplicationStatus.SAFETY_INTERLOCK``.
  This requires ts_idl 1.4 or later.
* Updated the git pre-commit hook to prevent the commit if black formatting needed.
  This encourages the user to properly commit the necessary reformatting.
* Modernize the documentation.

Requires:

* ts_hexrotcomm 0.7
* ts_salobj 5.11 or 6
* ts_simactuators 1
* ts_idl 1.4, or 2 with salobj 6
* ts_xml 6.2
* Rotator IDL files, e.g. made using ``make_idl_files.py Rotator``

v0.6.0
------

* Added missing ``config_dir`` constructor argument to `RotatorCsc`.
* Use `lsst.ts.salobj.BaseCscTestCase` and `lsst.ts.salobj.CscCommander` instead of the versions in ts_hexrotcomm.
* Add attribute ``position_jitter`` to `MockMTRotatorController` and update the unit tests to use it.
  Also make test_move more robust by giving the slew more time to finish.

Requires:

* ts_hexrotcomm 0.7
* ts_salobj 5.11
* ts_simactuators 1
* ts_idl 1
* ts_xml 4.6
* Rotator IDL files, e.g. made using ``make_idl_files.py Rotator``

v0.5.0
------

* Make `RotatorCsc` configurable.

Requires:

* ts_hexrotcomm 0.7
* ts_salobj 5.11
* ts_simactuators 1
* ts_idl 1
* ts_xml 4.6
* Rotator IDL files, e.g. made using ``make_idl_files.py Rotator``

v0.4.4
------

* Add ``tests/test_black.py`` to verify that files are formatted with black.
  This requires ts_salobj 5.11 or later.
* Update ``.travis.yml`` to remove ``sudo: false`` to github travis checks pass once again.

Requires:

* ts_hexrotcomm 0.2
* ts_salobj 5.11
* ts_simactuators 1
* ts_idl 1
* ts_xml 4.6
* Rotator IDL files, e.g. made using ``make_idl_files.py Rotator``

v0.4.3
------

* Fix flake8 violations.
* Improve Jenkins.conda build script so it will label PRs and branches packages as dev and upload them to anaconda.

Requires:

* ts_hexrotcomm 0.2
* ts_salobj 5
* ts_simactuators 1
* ts_idl 1
* ts_xml 4.6
* Rotator IDL files, e.g. made using ``make_idl_files.py Rotator``

v0.4.2
------

* Fix flake8 violations.
* Add Jenkinsfile for CI job.

Requires:

* ts_hexrotcomm 0.2
* ts_salobj 5
* ts_simactuators 1
* ts_idl 1
* ts_xml 4.6
* Rotator IDL files, e.g. made using ``make_idl_files.py Rotator``

v0.4.1
------

* Include conda package build configuration.
* Added a Jenkinsfile to support continuous integration and to build conda packages.
* Remove unused schema file.

Requires:

* ts_hexrotcomm 0.2
* ts_salobj 5
* ts_simactuators 1
* ts_idl 1
* ts_xml 4.6
* Rotator IDL files, e.g. made using ``make_idl_files.py Rotator``

v0.4.0
------

Update `MockMTRotatorController` to use the ``TrackingActuator`` from ts_simactuators.
Formerly `MockMTRotatorController` used a locally defined point to point actuator, which gives somewhat lower fidelity and duplicates code in ts_simactuators.

Requires:

* ts_hexrotcomm 0.2
* ts_salobj 5
* ts_simactuators 1
* ts_idl 1
* ts_xml 4.6
* Rotator IDL files, e.g. made using ``make_idl_files.py Rotator``

v0.3.0
------

Major changes:

* Added a revision history.
* Code formatted by ``black``, with a pre-commit hook to enforce this.
  See the README file for configuration instructions.

Requires:

* ts_hexrotcomm 0.2
* ts_salobj 5
* ts_idl 1
* ts_xml 4.6
* Rotator IDL files, e.g. made using ``make_idl_files.py Rotator``


v0.2.0
------

Update for changes to Rotator XML.
Tested with the rotator.

Requires:

* ts_hexrotcomm 0.2
* ts_salobj 5
* ts_idl 1
* ts_xml 4.6
* Rotator IDL files, e.g. made using ``make_idl_files.py Rotator``

v0.1.0
------

Still not fully tested with the real rotator.

Requires:

* ts_hexrotcomm 0.1
* ts_salobj 5
* ts_idl 1
* Rotator IDL files, e.g. made using ``make_idl_files.py Rotator``
