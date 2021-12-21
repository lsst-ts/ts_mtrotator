# This file is part of ts_mtrotator.
#
# Developed for the Rubin Observatory Telescope and Site System.
# This product includes software developed by the LSST Project
# (https://www.lsst.org).
# See the COPYRIGHT file at the top-level directory of this distribution
# for details of code ownership.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import asyncio
import contextlib
import unittest

from lsst.ts import utils
from lsst.ts import hexrotcomm
from lsst.ts import salobj
from lsst.ts import mtrotator
from lsst.ts.idl.enums.MTRotator import ControllerState, EnabledSubstate, ErrorCode

STD_TIMEOUT = 30  # timeout for command ack

# Standard delta for ccwFollowingError.positionError (deg)
# and velocityError (deg/sec).
STD_FOLLOWING_DELTA_POSITION = 0.1
STD_FOLLOWING_DELTA_VELOCITY = 0.1

# Time for a few telemetry updates and the mock CCW controller
# to respond to them (seconds).
WAIT_FOR_CCW_DELAY = 0.5


class TestRotatorCsc(hexrotcomm.BaseCscTestCase, unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        # The most recently seen rotation.odometer telemetry reading
        self.prev_odometer = None

        # The amount of error the mock CCW will apply
        # when following the rotator.
        # Reported CCW position = rotator position + ccw_following_error
        self.ccw_following_error = 0

        # Like ccw_following_error, but only applied once, then zeroed.
        self.ccw_transient_following_error = 0

        # Set False to stop self.mock_ccw_loop
        # (and optionally await self.mock_ccw_task).
        # Cancelling self.mock_ccw_task does not always work in Jenkins
        # (I don't know why) so use this technique instead.
        self.enable_mock_ccw_telemetry = False

    def basic_make_csc(self, initial_state, simulation_mode=1, config_dir=None):
        return mtrotator.RotatorCsc(
            initial_state=initial_state,
            simulation_mode=simulation_mode,
            config_dir=config_dir,
        )

    async def check_telemetry(self):
        """Check the the next rotation and motors telemetry messages."""
        # We need some margin, slightly more than I naively expected,
        # possibly because there is some roundoff error in converting time
        # between TAI unix seconds and astropy times.
        slop = 1e-6

        rotation_data = await self.remote.tel_rotation.next(
            flush=True, timeout=STD_TIMEOUT
        )
        path_segment = self.csc.mock_ctrl.rotator.path.at(rotation_data.timestamp)
        # Print the values to get some idea of how much slop is needed
        # and so it is easier to determine which test failed
        # in the tracking task in test_track_good.
        print(
            f"pos={rotation_data.demandPosition:0.8f}; "
            f"delta={abs(rotation_data.demandPosition - path_segment.position):0.8f}"
        )
        print(
            f"vel={rotation_data.demandVelocity:0.8f}; "
            f"delta={abs(rotation_data.demandVelocity - path_segment.velocity):0.8f}"
        )
        print(
            f"acc={rotation_data.demandAcceleration:0.8f}; "
            f"delta={abs(rotation_data.demandAcceleration - path_segment.acceleration):0.8f}"
        )
        self.assertAlmostEqual(
            rotation_data.demandPosition, path_segment.position, delta=slop
        )
        self.assertAlmostEqual(
            rotation_data.demandVelocity, path_segment.velocity, delta=slop
        )
        self.assertAlmostEqual(
            rotation_data.demandAcceleration, path_segment.acceleration, delta=slop
        )
        # actualPosition has jitter but actualVelocity does not.
        self.assertAlmostEqual(
            rotation_data.actualPosition,
            path_segment.position,
            delta=slop + self.csc.mock_ctrl.position_jitter,
        )
        self.assertAlmostEqual(
            rotation_data.actualVelocity, path_segment.velocity, delta=slop
        )

        if self.prev_odometer is not None:
            print(
                f"odometer={rotation_data.odometer:0.8f}; "
                f"prev_odometer={self.prev_odometer:0.8f}"
            )
            self.assertGreaterEqual(rotation_data.odometer, self.prev_odometer)
        self.prev_odometer = rotation_data.odometer

        motors_data = await self.remote.tel_motors.next(flush=True, timeout=STD_TIMEOUT)
        acceleration = self.csc.mock_ctrl.rotator.path.at(
            motors_data.private_sndStamp
        ).acceleration
        # DM-31447 uncomment when the low-level controller provides this data
        # desired_current = acceleration * self.csc.mock_ctrl.current_per_acceleration  # noqa
        desired_torque = acceleration * self.csc.mock_ctrl.torque_per_acceleration
        # DM-31447 uncomment when the low-level controller provides this data
        # print(
        #     f"current={motors_data.current[0]:0.8f}; "
        #     f"delta={abs(motors_data.current[0] - desired_current):0.8f}"
        # )
        print(
            f"torque={motors_data.torque[0]:0.8f}; "
            f"delta={abs(motors_data.torque[0] - desired_torque):0.8f}"
        )
        # DM-31447 uncomment when the low-level controller provides this data
        # self.assertAlmostEqual(motors_data.current[0], desired_current, delta=slop)  # noqa
        self.assertAlmostEqual(motors_data.torque[0], desired_torque, delta=0.01)
        # The mock controller publishes exactly the same current and torque
        # for both motors (though that is not realistic).
        # DM-31447 uncomment when the low-level controller provides this data
        # assert motors_data.current[0] ==  motors_data.current[1]
        assert motors_data.torque[0] == motors_data.torque[1]

    @contextlib.asynccontextmanager
    async def make_csc(
        self,
        initial_state=salobj.State.STANDBY,
        config_dir=None,
        simulation_mode=1,
        log_level=None,
        timeout=STD_TIMEOUT,
        run_mock_ccw=True,  # Set False to test failure to enable
        **kwargs,
    ):
        """Override make_csc

        This exists primarily because we need to start a mock
        camera cable wrap controller before we can enable the CSC.
        It also offers the opportunity to make better defaults.

        Parameters
        ----------
        name : `str`
            Name of SAL component.
        initial_state : `lsst.ts.salobj.State` or `int`, optional
            The initial state of the CSC. Defaults to STANDBY.
        config_dir : `str`, optional
            Directory of configuration files, or `None` (the default)
            for the standard configuration directory (obtained from
            `ConfigureCsc._get_default_config_dir`).
        simulation_mode : `int`, optional
            Simulation mode. Defaults to 0 because not all CSCs support
            simulation. However, tests of CSCs that support simulation
            will almost certainly want to set this nonzero.
        log_level : `int` or `None`, optional
            Logging level, such as `logging.INFO`.
            If `None` then do not set the log level, leaving the default
            behavior of `SalInfo`: increase the log level to INFO.
        timeout : `float`, optional
            Time limit for the CSC to start (seconds).
        run_mock_ccw : `bool`, optional
            If True then start a mock camera cable wrap controller.
        **kwargs : `dict`, optional
            Extra keyword arguments for `basic_make_csc`.
            For a configurable CSC this may include ``settings_to_apply``,
            especially if ``initial_state`` is DISABLED or ENABLED.

        """
        # We cannot transition the CSC to ENABLED
        # until the CCW following loop is running.
        # So if initial_state is ENABLED
        # start the CSC in DISABLED, start the CCW following loop,
        # then transition to ENABLED and swallow the DISABLED state
        # and associated controller state (make_csc reads all but the final
        # CSC summary state and controller state).
        if initial_state == salobj.State.ENABLED:
            modified_initial_state = salobj.State.DISABLED
        else:
            modified_initial_state = initial_state

        async with super().make_csc(
            initial_state=modified_initial_state,
            config_dir=config_dir,
            simulation_mode=simulation_mode,
            log_level=log_level,
            timeout=timeout,
            **kwargs,
        ), salobj.Controller(name="MTMount") as self.mtmount_controller:
            if run_mock_ccw:
                self.mock_ccw_task = asyncio.create_task(self.mock_ccw_loop())
            else:
                print("do not run mock_ccw_loop")
                self.mock_ccw_task = utils.make_done_future()
            if initial_state == salobj.State.ENABLED:
                await self.remote.cmd_enable.start(timeout=STD_TIMEOUT)
                await self.assert_next_summary_state(salobj.State.DISABLED)
                # Wait for and check the intermediate controller state,
                # so unit test code only needs to check the final state
                # (don't swallow the final state, for backwards compatibility).
                for controller_state in (
                    ControllerState.OFFLINE,
                    ControllerState.STANDBY,
                    ControllerState.DISABLED,
                ):
                    await self.assert_next_sample(
                        topic=self.remote.evt_controllerState,
                        controllerState=controller_state,
                    )
            try:
                yield
            finally:
                self.enable_mock_ccw_telemetry = False
                await asyncio.wait_for(self.mock_ccw_task, timeout=STD_TIMEOUT)

    async def mock_ccw_loop(self):
        """Mock the MTMount camera cable wrap system.

        Output cameraCableWrap telemetry that simulates
        the camera cable wrap following the rotator.
        """
        print("mock_ccw_loop starting")
        self.enable_mock_ccw_telemetry = True
        try:
            while self.enable_mock_ccw_telemetry:
                rotation_data = await self.remote.tel_rotation.next(
                    flush=True, timeout=STD_TIMEOUT
                )

                ccw_tai = utils.current_tai()
                dt = ccw_tai - rotation_data.timestamp
                ccw_position = (
                    rotation_data.demandPosition
                    + rotation_data.demandVelocity * dt
                    - self.ccw_following_error
                    - self.ccw_transient_following_error
                )
                print(
                    "mock_ccw_loop: put tel_cameraCableWrap; "
                    f"position={ccw_position:0.2f}; "
                    f"velocity={rotation_data.actualVelocity:0.2f}"
                )
                self.mtmount_controller.tel_cameraCableWrap.set_put(
                    actualPosition=ccw_position,
                    actualVelocity=rotation_data.actualVelocity,
                    actualAcceleration=0,
                    timestamp=ccw_tai,
                )
                self.ccw_transient_following_error = 0
            print("mock_ccw_loop ends")
        except asyncio.CancelledError:
            print("mock_ccw_loop canceled")
        except Exception as e:
            print(f"mock_ccw_loop failed: {e}")

    async def test_bin_script(self):
        """Test running from the command line script."""
        await self.check_bin_script(
            name="MTRotator",
            index=None,
            exe_name="run_mtrotator.py",
            cmdline_args=["--simulate"],
        )

    async def test_enable_no_ccw_telemetry(self):
        """Test that it is not possible to enable the CSC if it
        is not receiving MTMount cameraCableWrap telemetry.
        """
        async with self.make_csc(
            initial_state=salobj.State.DISABLED, run_mock_ccw=False
        ):
            with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_FAILED):
                await self.remote.cmd_enable.start(timeout=STD_TIMEOUT)
            await self.assert_next_summary_state(salobj.State.DISABLED)

    async def test_missing_ccw_telemetry(self):
        """Test that the CSC will fault if camera cable wrap telemetry
        disappears.

        Also test that ccwFollowingError is output even in FAULT and STANDBY
        states.
        """
        async with self.make_csc(initial_state=salobj.State.ENABLED):
            await self.assert_next_summary_state(salobj.State.ENABLED)
            await self.assert_next_sample(topic=self.remote.evt_errorCode, errorCode=0)

            # Wait a bit; the CSC should still be enabled.
            # Wait for a few rotator telemetry messages,
            # which in turn trigger CCW telemetry.
            delay = self.csc.mock_ctrl.telemetry_interval * 5
            print(f"Sleep for {delay} seconds, then cancel the mock CCW output")
            await asyncio.sleep(delay)
            assert self.csc.summary_state == salobj.State.ENABLED

            # Stop the telemetry. The CSC should soon go to FAULT.
            self.enable_mock_ccw_telemetry = False
            await asyncio.wait_for(self.mock_ccw_task, timeout=STD_TIMEOUT)
            await self.assert_next_summary_state(salobj.State.FAULT)
            await self.assert_next_sample(
                topic=self.remote.evt_errorCode, errorCode=ErrorCode.CONTROLLER_FAULT
            )

            # Test that ccwFollowingError is still output in FAULT state
            self.mock_ccw_task = asyncio.create_task(self.mock_ccw_loop())
            data = await self.remote.tel_ccwFollowingError.next(
                flush=True, timeout=STD_TIMEOUT
            )
            self.assertAlmostEqual(
                data.positionError, 0, delta=STD_FOLLOWING_DELTA_POSITION
            )
            self.assertAlmostEqual(
                data.velocityError, 0, delta=STD_FOLLOWING_DELTA_VELOCITY
            )

    async def test_excessive_ccw_following_error(self):
        async with self.make_csc(initial_state=salobj.State.ENABLED):
            await self.assert_next_summary_state(salobj.State.ENABLED)
            await self.assert_next_sample(topic=self.remote.evt_errorCode, errorCode=0)

            # Wait a bit; the CSC should still be enabled.
            # Wait for a few rotator telemetry messages,
            # which in turn trigger CCW telemetry.
            delay = self.csc.mock_ctrl.telemetry_interval * 5
            await asyncio.sleep(delay)
            await self.assert_next_ccw_following_error()
            assert self.csc.summary_state == salobj.State.ENABLED

            # Specify excessive positive following error;
            # the CSC should shortly be disabled.
            self.ccw_following_error = self.csc.config.max_ccw_following_error + 0.1
            await self.assert_next_ccw_following_error()
            await self.assert_next_summary_state(salobj.State.FAULT)
            await self.assert_next_sample(
                topic=self.remote.evt_errorCode, errorCode=ErrorCode.CONTROLLER_FAULT
            )

            # Zero the following error and re-enable the CSC.
            self.ccw_following_error = 0
            states = await salobj.set_summary_state(
                self.remote, state=salobj.State.ENABLED
            )
            for state in states[1:]:
                await self.assert_next_summary_state(state)
            await self.assert_next_ccw_following_error()
            await self.assert_next_sample(topic=self.remote.evt_errorCode, errorCode=0)

            # Wait a bit; the CSC should still be enabled
            await asyncio.sleep(WAIT_FOR_CCW_DELAY)
            await self.assert_next_ccw_following_error()
            assert self.csc.summary_state == salobj.State.ENABLED

            # Specify excessive negative following error;
            # the CSC should shortly be disabled.
            self.ccw_following_error = -(self.csc.config.max_ccw_following_error + 0.1)
            await self.assert_next_ccw_following_error()
            await self.assert_next_summary_state(salobj.State.FAULT)
            await self.assert_next_sample(
                topic=self.remote.evt_errorCode,
                errorCode=ErrorCode.CONTROLLER_FAULT,
            )

    async def test_transient_excessive_ccw_following_error(self):
        async with self.make_csc(initial_state=salobj.State.ENABLED):
            await self.assert_next_summary_state(salobj.State.ENABLED)
            await self.assert_next_sample(topic=self.remote.evt_errorCode, errorCode=0)
            self.assertGreater(self.csc.config.num_ccw_following_errors, 1)

            # Increase the following error for a single CCW telemetry message;
            # the CSC should not be disabled
            self.ccw_transient_following_error = (
                self.csc.config.max_ccw_following_error + 0.1
            )
            await asyncio.sleep(WAIT_FOR_CCW_DELAY)
            assert self.csc.summary_state == salobj.State.ENABLED

            # Set the # of fails to 1 and try again;
            # this time a single transient should cause failure.
            self.csc.config.num_ccw_following_errors = 1
            self.ccw_transient_following_error = (
                self.csc.config.max_ccw_following_error + 0.1
            )
            await self.assert_next_summary_state(salobj.State.FAULT)
            await self.assert_next_sample(
                topic=self.remote.evt_errorCode,
                errorCode=ErrorCode.CONTROLLER_FAULT,
            )

    async def test_fault(self):
        async with self.make_csc(initial_state=salobj.State.ENABLED):
            await self.assert_next_sample(topic=self.remote.evt_errorCode, errorCode=0)
            await self.assert_next_sample(
                topic=self.remote.evt_softwareVersions,
                cscVersion=mtrotator.__version__,
                subsystemVersions="",
            )
            await self.assert_next_summary_state(salobj.State.ENABLED)
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
            )

            await self.remote.cmd_fault.start(timeout=STD_TIMEOUT)
            await self.assert_next_summary_state(salobj.State.FAULT)
            await self.assert_next_sample(
                topic=self.remote.evt_errorCode,
                errorCode=ErrorCode.CONTROLLER_FAULT,
            )
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=ControllerState.FAULT,
            )

            # Make sure the fault command only works in enabled state
            with salobj.assertRaisesAckError():
                await self.remote.cmd_fault.start(timeout=STD_TIMEOUT)

            # Test recovery from fault state
            await self.remote.cmd_standby.set_start(timeout=STD_TIMEOUT)
            await self.assert_next_summary_state(salobj.State.STANDBY)
            with salobj.assertRaisesAckError():
                await self.remote.cmd_fault.start(timeout=STD_TIMEOUT)

            await self.remote.cmd_start.set_start(timeout=STD_TIMEOUT)
            await self.assert_next_summary_state(salobj.State.DISABLED)
            with salobj.assertRaisesAckError():
                await self.remote.cmd_fault.start(timeout=STD_TIMEOUT)

            await self.remote.cmd_enable.set_start(timeout=STD_TIMEOUT)
            await self.assert_next_summary_state(salobj.State.ENABLED)
            for controller_state in (
                ControllerState.OFFLINE,
                ControllerState.STANDBY,
                ControllerState.DISABLED,
            ):
                await self.assert_next_sample(
                    topic=self.remote.evt_controllerState,
                    controllerState=controller_state,
                )
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.STATIONARY,
            )

    async def test_standard_state_transitions(self):
        enabled_commands = (
            "configureVelocity",
            "configureAcceleration",
            "fault",
            "move",
            "stop",
            "trackStart",
        )
        async with self.make_csc(initial_state=salobj.State.STANDBY):
            await self.check_standard_state_transitions(
                enabled_commands=enabled_commands
            )

    async def test_configure_acceleration(self):
        """Test the configureAcceleration command."""
        async with self.make_csc(initial_state=salobj.State.ENABLED):
            data = await self.remote.evt_configuration.next(
                flush=False, timeout=STD_TIMEOUT
            )
            initial_limit = data.accelerationLimit
            print("initial_limit=", initial_limit)
            new_limit = initial_limit - 0.1
            await self.remote.cmd_configureAcceleration.set_start(
                alimit=new_limit, timeout=STD_TIMEOUT
            )
            data = await self.remote.evt_configuration.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertAlmostEqual(data.accelerationLimit, new_limit)

            for bad_alimit in (-1, 0, mtrotator.MAX_ACCEL_LIMIT + 0.001):
                with self.subTest(bad_alimit=bad_alimit):
                    with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_FAILED):
                        await self.remote.cmd_configureAcceleration.set_start(
                            alimit=bad_alimit, timeout=STD_TIMEOUT
                        )

    async def test_configure_velocity(self):
        """Test the configureVelocity command."""
        async with self.make_csc(initial_state=salobj.State.ENABLED):
            data = await self.remote.evt_configuration.next(
                flush=False, timeout=STD_TIMEOUT
            )
            initial_limit = data.velocityLimit
            new_limit = initial_limit - 0.1
            await self.remote.cmd_configureVelocity.set_start(
                vlimit=new_limit, timeout=STD_TIMEOUT
            )
            data = await self.remote.evt_configuration.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertAlmostEqual(data.velocityLimit, new_limit)

            for bad_vlimit in (0, -1, mtrotator.MAX_VEL_LIMIT + 0.001):
                with self.subTest(bad_vlimit=bad_vlimit):
                    with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_FAILED):
                        await self.remote.cmd_configureVelocity.set_start(
                            vlimit=bad_vlimit, timeout=STD_TIMEOUT
                        )

    async def test_move(self):
        """Test the move command for point to point motion."""
        destination = 2  # a small move so the test runs quickly
        # Estimated time to move; a crude estimate used for timeouts
        est_move_duration = 1

        async with self.make_csc(initial_state=salobj.State.ENABLED):
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.STATIONARY,
            )
            assert self.csc.mock_ctrl.odometer == 0
            await self.check_telemetry()
            data = await self.remote.evt_inPosition.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertFalse(data.inPosition)
            t0 = utils.current_tai()
            await self.remote.cmd_move.set_start(
                position=destination, timeout=STD_TIMEOUT
            )
            data = await self.remote.evt_target.next(flush=False, timeout=STD_TIMEOUT)
            target_event_delay = utils.current_tai() - t0
            self.assertAlmostEqual(data.position, destination)
            assert data.velocity == 0
            target_time_difference = utils.current_tai() - data.tai
            self.assertLessEqual(abs(target_time_difference), target_event_delay)

            for i in range(10):
                await self.check_telemetry()
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.MOVING_POINT_TO_POINT,
            )
            data = await self.remote.evt_inPosition.next(
                flush=False, timeout=STD_TIMEOUT + est_move_duration
            )
            self.assertTrue(data.inPosition)
            print(f"Move duration: {utils.current_tai() - t0:0.2f} seconds")
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.STATIONARY,
            )
            await self.check_telemetry()
            # The odometer is accumulated in a somewhat simplistic fashion,
            # so give some slop (though this is more than we really need)
            self.assertAlmostEqual(self.csc.mock_ctrl.odometer, destination, delta=0.01)

    async def test_stop_move(self):
        """Test stopping a point to point move."""
        destination = 20  # a large move so we have plenty of time to stop
        # Estimated time to move; a crude estimate used for timeouts
        async with self.make_csc(initial_state=salobj.State.ENABLED):
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.STATIONARY,
            )
            data = await self.remote.tel_rotation.next(flush=True, timeout=STD_TIMEOUT)
            self.assertAlmostEqual(data.demandPosition, 0)
            self.assertAlmostEqual(
                data.actualPosition, 0, delta=self.csc.mock_ctrl.position_jitter
            )
            data = await self.remote.evt_inPosition.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertFalse(data.inPosition)
            await self.remote.cmd_move.set_start(
                position=destination, timeout=STD_TIMEOUT
            )
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.MOVING_POINT_TO_POINT,
            )

            # Let the move run for a short time, then stop it.
            await asyncio.sleep(0.2)
            await self.remote.cmd_stop.start(timeout=STD_TIMEOUT)
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.STATIONARY,
            )
            data = await self.remote.tel_rotation.next(flush=True, timeout=STD_TIMEOUT)
            self.assertGreater(data.actualPosition, 0)
            self.assertLess(data.actualPosition, destination)

    async def test_track_good(self):
        """Test the trackStart and track commands."""
        pos0 = 2  # a small move so the slew ends quickly
        vel = 0.01
        # Estimated time to slew; a crude estimate used for timeouts
        est_slew_duration = 1
        async with self.make_csc(initial_state=salobj.State.ENABLED):
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.STATIONARY,
            )
            data = await self.remote.tel_rotation.next(flush=True, timeout=STD_TIMEOUT)
            self.assertAlmostEqual(data.demandPosition, 0)
            self.assertAlmostEqual(
                data.actualPosition, 0, delta=self.csc.mock_ctrl.position_jitter
            )
            data = await self.remote.evt_inPosition.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertFalse(data.inPosition)
            await self.remote.cmd_trackStart.start(timeout=STD_TIMEOUT)
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.SLEWING_OR_TRACKING,
            )
            slew_start_tai = utils.current_tai()

            while True:
                tai = utils.current_tai()
                if tai - slew_start_tai > STD_TIMEOUT + est_slew_duration:
                    self.fail("Slew did not end in time")
                dt = tai - slew_start_tai
                pos = pos0 + vel * dt
                await self.remote.cmd_track.set_start(
                    angle=pos, velocity=vel, tai=tai, timeout=STD_TIMEOUT
                )
                data = await self.remote.evt_target.next(
                    flush=False, timeout=STD_TIMEOUT
                )
                self.assertAlmostEqual(data.tai, tai)
                self.assertAlmostEqual(data.velocity, vel)
                self.assertAlmostEqual(data.position, pos)
                await self.check_telemetry()
                await asyncio.sleep(0.1)
                data = self.remote.evt_inPosition.get()
                if data.inPosition:
                    # Slew is done; success
                    break
            # Read the inPosition event from the queue, so we can await
            # the expected false value after stopping tracking.
            await self.assert_next_sample(
                topic=self.remote.evt_inPosition, inPosition=True
            )

            slew_duration = utils.current_tai() - slew_start_tai
            print(f"Slew duration: {slew_duration:0.2f} seconds")

            await self.remote.cmd_stop.start(timeout=STD_TIMEOUT)
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.STATIONARY,
            )
            data = await self.remote.evt_inPosition.next(
                flush=False, timeout=STD_TIMEOUT + est_slew_duration
            )
            self.assertFalse(data.inPosition)

    async def test_track_bad_values(self):
        """Test the track command with bad values.

        This should go into FAULT.
        """
        async with self.make_csc(initial_state=salobj.State.ENABLED):
            await self.assert_next_summary_state(salobj.State.ENABLED)
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.STATIONARY,
            )
            settings = await self.remote.evt_configuration.next(
                flush=False, timeout=STD_TIMEOUT
            )
            await self.remote.cmd_trackStart.start(timeout=STD_TIMEOUT)
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.SLEWING_OR_TRACKING,
            )

            # Run these quickly enough and the controller will still be enabled
            curr_tai = utils.current_tai()
            for pos, vel, tai in (
                # Position out of range.
                (settings.positionAngleLowerLimit - 0.001, 0, curr_tai),
                (settings.positionAngleUpperLimit + 0.001, 0, curr_tai),
                # Velocity out of range.
                (0, settings.velocityLimit + 0.001, curr_tai),
                # Current position and velocity OK but the position
                # at the specified tai is out of bounds.
                (
                    settings.positionAngleUpperLimit - 0.001,
                    settings.velocityLimit - 0.001,
                    curr_tai + 1,
                ),
            ):
                with self.subTest(pos=pos, vel=vel, tai=tai):
                    with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_FAILED):
                        await self.remote.cmd_track.set_start(
                            angle=pos, velocity=vel, tai=tai, timeout=STD_TIMEOUT
                        )
                    # Send a valid pvt to reset the tracking timer
                    # and give the controller time to deal with it.
                    await self.remote.cmd_track.set_start(
                        angle=0,
                        velocity=0,
                        tai=utils.current_tai(),
                        timeout=STD_TIMEOUT,
                    )
                    await asyncio.sleep(0.01)

    async def test_track_start_no_track(self):
        """Test the trackStart command with no track command.

        This should go into FAULT.
        """
        async with self.make_csc(initial_state=salobj.State.ENABLED):
            await self.assert_next_summary_state(salobj.State.ENABLED)
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.STATIONARY,
            )
            data = await self.remote.tel_rotation.next(flush=True, timeout=STD_TIMEOUT)
            self.assertAlmostEqual(data.demandPosition, 0)
            self.assertAlmostEqual(
                data.actualPosition, 0, delta=self.csc.mock_ctrl.position_jitter
            )
            data = await self.remote.evt_inPosition.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertFalse(data.inPosition)
            await self.remote.cmd_trackStart.start(timeout=STD_TIMEOUT)
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.SLEWING_OR_TRACKING,
            )

            # Send a tracking position
            await self.remote.cmd_track.set_start(
                angle=0, velocity=0, tai=utils.current_tai(), timeout=STD_TIMEOUT
            )

            # Wait a bit longer than usual to allow the tracking timer
            # to expire.
            await self.assert_next_summary_state(
                salobj.State.FAULT, timeout=STD_TIMEOUT + 1
            )
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=ControllerState.FAULT,
            )

    async def test_track_start_late_track(self):
        """Test tracking with a late track command.

        Also test that we can send the track command immediately after
        the trackStart command. before the controller has time to tell
        the CSC that it is in SLEWING_OR_TRACKING mode.
        This exercises special code in the CSC that checks if the
        track command is allowed.

        After the first track command send no more.
        This should send the CSC into FAULT.
        """
        async with self.make_csc(initial_state=salobj.State.ENABLED):
            await self.assert_next_summary_state(salobj.State.ENABLED)
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.STATIONARY,
            )
            await self.assert_next_sample(
                topic=self.remote.evt_tracking, tracking=False, noNewCommand=False
            )
            data = await self.remote.tel_rotation.next(flush=True, timeout=STD_TIMEOUT)
            self.assertAlmostEqual(data.demandPosition, 0)
            self.assertAlmostEqual(
                data.actualPosition, 0, delta=self.csc.mock_ctrl.position_jitter
            )
            data = await self.remote.evt_inPosition.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertFalse(data.inPosition)
            await self.remote.cmd_trackStart.start(timeout=STD_TIMEOUT)

            # Immediately send a track commands (not giving the CSC time to see
            # the changed controller state), as explained in the doc string.
            curr_tai = utils.current_tai()
            await self.remote.cmd_track.set_start(
                angle=0, velocity=0, tai=curr_tai, timeout=STD_TIMEOUT
            )

            # Now make sure the trackStart command did send the controller
            # into the state SLEWING_OR_TRACKING
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=ControllerState.ENABLED,
                enabledSubstate=EnabledSubstate.SLEWING_OR_TRACKING,
            )
            await self.assert_next_sample(
                topic=self.remote.evt_tracking, tracking=True, noNewCommand=False
            )

            # Wait for the lack of track commands to send the CSC into FAULT;
            # wait a bit longer than usual to allow the tracking timer
            # to expire.
            await self.assert_next_summary_state(
                salobj.State.FAULT, timeout=STD_TIMEOUT + 1
            )
            await self.assert_next_sample(
                topic=self.remote.evt_controllerState,
                controllerState=ControllerState.FAULT,
            )
            await self.assert_next_sample(
                topic=self.remote.evt_tracking, tracking=False, noNewCommand=True
            )

    async def assert_next_ccw_following_error(
        self,
        position_error=None,
        velocity_error=0,
        delta_position=STD_FOLLOWING_DELTA_POSITION,
        delta_velocity=STD_FOLLOWING_DELTA_VELOCITY,
        timeout=STD_TIMEOUT,
    ):
        """Assert that ccwFollowingError matches the specifications.

        Reads new samples until the position error matches,
        then checks the velocity and timestamp.
        If the correct position error is not seen in time,
        raise `asyncio.TimeoutError`.

        Parameters
        ----------
        position_error : `float` or `None`, optional
            Expected position error (deg).
            If None then use self.ccw_following_error.
            This is almost always what you want.
        velocity_error : `float`, optional
            Expected velocity error. 0 is almost always what you want,
            because the mock_ccw_loop matches the rotator velocity.
        delta_position : `float`
            Maximum allowed error in position (deg)
        delta_velocity : `float`
            Maximum allowed error in velocity (deg/sec)
        timeout : `float`
            Maximum allowed time (seconds).
        """
        if position_error is None:
            position_error = self.ccw_following_error
        await asyncio.wait_for(
            self._impl_assert_next_ccw_following_error(
                position_error=position_error,
                velocity_error=velocity_error,
                delta_position=delta_position,
                delta_velocity=delta_velocity,
            ),
            timeout=timeout,
        )

    async def _impl_assert_next_ccw_following_error(
        self, position_error, velocity_error, delta_position, delta_velocity
    ):
        """Implement assert_next_ccw_following_error without a timeout
        and without argument defaults.
        """
        while True:
            data = await self.remote.tel_ccwFollowingError.next(
                flush=True, timeout=STD_TIMEOUT
            )
            # Give enough slop for the timestamp to handle clock jitter
            # on Docker on macOS.
            self.assertAlmostEqual(data.timestamp, utils.current_tai(), delta=0.2)
            if abs(data.positionError - position_error) < delta_position:
                break
        self.assertAlmostEqual(data.velocityError, velocity_error, delta_velocity)


if __name__ == "__main__":
    unittest.main()
