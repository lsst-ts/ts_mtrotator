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

__all__ = ["RotatorCsc", "run_mtrotator"]

import argparse
import asyncio
import types
import typing
from pathlib import Path

from lsst.ts import hexrotcomm, salobj, utils
from lsst.ts.xml.enums.MTRotator import (
    ApplicationStatus,
    ControllerState,
    EnabledSubstate,
)

from . import __version__, constants, enums, mock_controller, structs
from .config_schema import CONFIG_SCHEMA
from .vibration_detector import VibrationDetector

# Approximate interval between output of the clockOffset event (seconds).
# The clockOffset event is output when the first new telemetry is received
# after the timer has expired.
CLOCK_OFFSET_EVENT_INTERVAL = 1

# Maximum allowed age of the camera cable wrap telemetry data
# when checking following error.
MAX_CCW_TELEMETRY_AGE = 1

# Telemetry rate in Hz
TELEMETRY_RATE = 20.0


class RotatorCsc(hexrotcomm.BaseCsc):
    """MTRotator CSC.

    Parameters
    ----------
    config_dir : `str`, optional
        Directory of configuration files, or None for the standard
        configuration directory (obtained from `_get_default_config_dir`).
        This is provided for unit testing.
    initial_state : `lsst.ts.salobj.State` or `int` (optional)
        The initial state of the CSC.
        Must be `lsst.ts.salobj.State.STANDBY` unless simulating
        (``simulation_mode != 0``).
    override : `str`, optional
        Configuration override file to apply if ``initial_state`` is
        `State.DISABLED` or `State.ENABLED`.
    simulation_mode : `int` (optional)
        Simulation mode. Allowed values:

        * 0: regular operation.
        * 1: simulation: use a mock low level controller.
    bypass_ccw : `bool`, optional
        Bypass the check of camera cable wrapper (CCW) or not. (the default is
        False)

    Raises
    ------
    ValueError
        If ``initial_state != lsst.ts.salobj.State.STANDBY``
        and not simulating (``simulation_mode = 0``).

    Notes
    -----
    **Error Codes**

    See `lsst.ts.xml.enums.MTRotator.ErrorCode`
    """

    valid_simulation_modes = [0, 1]
    version = __version__

    def __init__(
        self,
        config_dir: str | Path | None = None,
        initial_state: salobj.State = salobj.State.STANDBY,
        override: str = "",
        simulation_mode: int = 0,
        bypass_ccw: bool = False,
    ) -> None:
        self.client: hexrotcomm.CommandTelemetryClient | None = None
        self.mock_ctrl: mock_controller.MockMTRotatorController | None = None

        # Set this to 2 when trackStart is called, then decrement
        # when telemetry is received. If > 0 or enabled_substate is
        # SLEWING_OR_TRACKING then allow the track command.
        # This solves the problem of allowing the track command
        # immediately after the trackStart, before telemetry is received.
        self._tracking_started_telemetry_counter = 0

        self._bypass_ccw = bypass_ccw

        # The sequential number of times the camera cable wrap following error
        # has been too large.
        self._num_ccw_following_errors = 0

        self._check_ccw_following_error_task = utils.make_done_future()
        self._reported_ccw_following_error_issue = False
        self._faulting = False
        self._prev_flags_tracking_success = False
        self._prev_flags_tracking_lost = False

        # telemetry_callback should output the clockOffset event
        # for the next telemetry received when this timer expires
        self.next_clock_offset_task = utils.make_done_future()

        self._vibration_detector: VibrationDetector | None = None
        self._detect_vibration_task = utils.make_done_future()

        super().__init__(
            name="MTRotator",
            index=0,
            CommandCode=enums.CommandCode,
            ConfigClass=structs.Config,
            TelemetryClass=structs.Telemetry,
            config_schema=CONFIG_SCHEMA,
            config_dir=config_dir,
            initial_state=initial_state,
            override=override,
            simulation_mode=simulation_mode,
        )
        self.mtmount_remote = salobj.Remote(domain=self.domain, name="MTMount")

    async def start(self) -> None:
        await super().start()
        await self.mtmount_remote.start_task
        await self.evt_inPosition.set_write(inPosition=False, force_output=True)

    async def check_ccw_following_error(self) -> None:
        """Check the camera cable wrap following error.

        Publish the value, if the camera cable wrap angle can be read.
        If ENABLED and the value is too large, then go to FAULT state.

        Note: this is designed to be called by telemetry_callback.
        Thus it is called every time telemetry is read
        from the low-level controller.
        """
        try:
            # Get camera cable wrap telemetry and check its age.
            try:
                ccw_data = await self.mtmount_remote.tel_cameraCableWrap.next(
                    flush=True, timeout=MAX_CCW_TELEMETRY_AGE
                )
            except asyncio.TimeoutError:
                err_msg = "Cannot read cameraCableWrap telemetry"
                if self.summary_state == salobj.State.ENABLED:
                    self.log.error(err_msg)
                    await self.command_llv_fault()
                elif not self._reported_ccw_following_error_issue:
                    self.log.warning(err_msg)
                    self._reported_ccw_following_error_issue = True
                return

            rot_data = self.tel_rotation.data
            rot_tai = rot_data.timestamp

            dt = rot_tai - ccw_data.timestamp
            if abs(dt) > MAX_CCW_TELEMETRY_AGE:
                err_msg = (
                    "Camera cable wrap telemetry is too old: "
                    f"dt={dt}; abs(dt) > {MAX_CCW_TELEMETRY_AGE}"
                )
                if self.summary_state == salobj.State.ENABLED:
                    self.log.error(err_msg)
                    await self.command_llv_fault()
                elif not self._reported_ccw_following_error_issue:
                    self.log.warning(err_msg)
                    self._reported_ccw_following_error_issue = True
                return

            self._reported_ccw_following_error_issue = False

            # Compute and report the following error.
            corr_ccw_pos = ccw_data.actualPosition + ccw_data.actualVelocity * dt
            following_error = rot_data.actualPosition - corr_ccw_pos
            await self.tel_ccwFollowingError.set_write(
                positionError=following_error,
                velocityError=rot_data.actualVelocity - ccw_data.actualVelocity,
                timestamp=rot_tai,
            )
            # If enabled then check the error and go to FAULT if too large.
            if self.summary_state == salobj.State.ENABLED:
                if abs(following_error) <= self.config.max_ccw_following_error:
                    # The following error is acceptable.
                    self._num_ccw_following_errors = 0
                    return

                self._num_ccw_following_errors += 1
                if (
                    self._num_ccw_following_errors
                    >= self.config.num_ccw_following_errors
                ):
                    self.log.error(
                        f"Camera cable wrap not following closely enough: "
                        f"error # {self._num_ccw_following_errors} = {following_error} "
                        f"> {self.config.max_ccw_following_error} deg"
                    )
                    await self.command_llv_fault()
        except Exception:
            self.log.exception("check_ccw_following_error failed")

    async def detect_vibration(self) -> None:
        """Detect the vibration."""

        assert self._vibration_detector is not None

        frequencies = self._vibration_detector.check_vibration_frequency()

        for frequency in frequencies:
            # This is to keep the backward compatibility with ts_xml v22.0.0
            # that does not have the 'lowFrequencyVibration' defined in xml.
            if hasattr(self, "evt_lowFrequencyVibration"):
                await self.evt_lowFrequencyVibration.set_write(frequency=frequency)
                # EFD has a time window of some milliseconds. Therefore, sleep
                # a little bit to give it the enough time to avoid the lost
                # of event.
                await asyncio.sleep(0.1)

    async def close_tasks(self) -> None:
        self.next_clock_offset_task.cancel()

        if not self._detect_vibration_task.done():
            self._detect_vibration_task.cancel()

        await super().close_tasks()

    async def configure(self, config: types.SimpleNamespace) -> None:
        self.config = config

        self._vibration_detector = VibrationDetector(
            config.vibration_detection_period,
            config.vibration_max_times,
            config.vibration_range,
            config.vibration_snr,
            config.vibration_threshold,
            dt=1.0 / TELEMETRY_RATE,
        )

    async def config_callback(self, client: hexrotcomm.CommandTelemetryClient) -> None:
        """Called when the TCP/IP controller outputs configuration.

        Parameters
        ----------
        client : `lsst.ts.hexrotcomm.CommandTelemetryClient`
            TCP/IP client.
        """
        config = client.config

        configuration = dict(
            positionAngleLowerLimit=config.lower_pos_limit,
            positionAngleUpperLimit=config.upper_pos_limit,
            velocityLimit=config.velocity_limit,
            accelerationLimit=config.accel_limit,
            emergencyAccelerationLimit=config.emergency_accel_limit,
            emergencyJerkLimit=config.emergency_jerk_limit,
            positionErrorThreshold=config.pos_error_threshold,
            followingErrorThreshold=config.following_error_threshold,
            trackingSuccessPositionThreshold=config.track_success_pos_threshold,
            trackingLostTimeout=config.tracking_lost_timeout,
            disableLimitMaxTime=config.disable_limit_max_time,
            maxConfigurableVelocityLimit=config.max_velocity_limit,
            drivesEnabled=config.drives_enabled,
        )

        await self.evt_configuration.set_write(**configuration)

    async def connect_callback(self, client: hexrotcomm.CommandTelemetryClient) -> None:
        await super().connect_callback(client)
        # Always reset this counter; if newly connected then we'll start
        # accumulating the count and otherwise it makes no difference.
        self._num_ccw_following_errors = 0

    async def do_configureJerk(self, data: salobj.BaseMsgType) -> None:
        """Configure the jerk limit.

        Parameters
        ----------
        data : `salobj.BaseMsgType`
            Data of the SAL message.

        Raises
        ------
        `salobj.ExpectedError`
            When the value is 0 or negative.
        """

        self.assert_enabled_substate(EnabledSubstate.STATIONARY)

        jlimit = data.jlimit
        if jlimit <= 0.0:
            raise salobj.ExpectedError(f"{jlimit=} must be > 0")

        await self.run_command(code=enums.CommandCode.CONFIG_JERK, param1=jlimit)

    async def do_configureAcceleration(self, data: salobj.BaseMsgType) -> None:
        """Specify the acceleration limit."""
        self.assert_enabled_substate(EnabledSubstate.STATIONARY)
        if not 0 < data.alimit <= constants.MAX_ACCEL_LIMIT:
            raise salobj.ExpectedError(
                f"alimit={data.alimit} must be > 0 and <= {constants.MAX_ACCEL_LIMIT}"
            )
        await self.run_command(code=enums.CommandCode.CONFIG_ACCEL, param1=data.alimit)

    async def do_configureVelocity(self, data: salobj.BaseMsgType) -> None:
        """Specify the velocity limit."""
        self.assert_enabled_substate(EnabledSubstate.STATIONARY)
        if not 0 < data.vlimit <= constants.MAX_VEL_LIMIT:
            raise salobj.ExpectedError(
                f"vlimit={data.vlimit} must be > 0 and <= {constants.MAX_VEL_LIMIT}"
            )
        await self.run_command(code=enums.CommandCode.CONFIG_VEL, param1=data.vlimit)

    async def do_configureEmergencyAcceleration(self, data: salobj.BaseMsgType) -> None:
        """Configure the emergency acceleration limit.

        Parameters
        ----------
        data : `salobj.BaseMsgType`
            Data of the SAL message.

        Raises
        ------
        `salobj.ExpectedError`
            When the value is 0 or negative.
        """

        self.assert_enabled_substate(EnabledSubstate.STATIONARY)

        alimit = data.alimit
        if alimit <= 0.0:
            raise salobj.ExpectedError(f"Emergency {alimit=} must be > 0")

        await self.run_command(
            code=enums.CommandCode.CONFIG_ACCEL_EMERGENCY, param1=alimit
        )

    async def do_configureEmergencyJerk(self, data: salobj.BaseMsgType) -> None:
        """Configure the emergency jerk limit.

        Parameters
        ----------
        data : `salobj.BaseMsgType`
            Data of the SAL message.

        Raises
        ------
        `salobj.ExpectedError`
            When the value is 0 or negative.
        """

        self.assert_enabled_substate(EnabledSubstate.STATIONARY)

        jlimit = data.jlimit
        if jlimit <= 0.0:
            raise salobj.ExpectedError(f"Emergency {jlimit=} must be > 0")

        await self.run_command(
            code=enums.CommandCode.CONFIG_JERK_EMERGENCY, param1=jlimit
        )

    async def do_enable(self, data: salobj.BaseMsgType) -> None:
        self.assert_summary_state(salobj.State.DISABLED)

        # Make sure we wait for at least one iteration of the
        # ccw following task before proceeding.
        await self._check_ccw_following_error_task

        if (not self._bypass_ccw) and (self._reported_ccw_following_error_issue):
            raise salobj.ExpectedError(
                "Cannot enable the rotator until it receives camera cable wrap telemetry"
            )

        self._reported_ccw_following_error_issue = False
        await super().do_enable(data)

    async def do_fault(self, data: salobj.BaseMsgType) -> None:
        if self.summary_state != salobj.State.ENABLED:
            raise salobj.ExpectedError("Not enabled")
        self.log.warning(
            "fault command issued; sending low-level controller to FAULT state"
        )
        await self.command_llv_fault()

    async def do_move(self, data: salobj.BaseMsgType) -> None:
        """Go to the position specified by the most recent ``positionSet``
        command.
        """
        self.assert_enabled_substate(EnabledSubstate.STATIONARY)

        # Workaround the mypy check
        assert self.client is not None

        if (
            not self.client.config.lower_pos_limit
            <= data.position
            <= self.client.config.upper_pos_limit
        ):
            raise salobj.ExpectedError(
                f"position {data.position} not in range "
                f"[{self.client.config.lower_pos_limit}, "
                f"{self.client.config.upper_pos_limit}]"
            )
        cmd1 = self.make_command(
            code=enums.CommandCode.POSITION_SET, param1=data.position
        )
        cmd2 = self.make_command(
            code=enums.CommandCode.SET_ENABLED_SUBSTATE,
            param1=enums.SetEnabledSubstateParam.MOVE_POINT_TO_POINT,
        )
        await self.run_multiple_commands(cmd1, cmd2)
        await self.evt_target.set_write(
            position=data.position,
            velocity=0,
            tai=utils.current_tai(),
            force_output=True,
        )

    async def do_stop(self, data: salobj.BaseMsgType) -> None:
        """Halt tracking or any other motion."""
        if self.summary_state != salobj.State.ENABLED:
            raise salobj.ExpectedError("Not enabled")
        await self.run_command(
            code=enums.CommandCode.SET_ENABLED_SUBSTATE,
            param1=enums.SetEnabledSubstateParam.STOP,
        )

    async def do_track(self, data: salobj.BaseMsgType) -> None:
        """Specify a position, velocity, TAI time tracking update."""
        if self.summary_state != salobj.State.ENABLED:
            raise salobj.ExpectedError("Not enabled")

        # Workaround the mypy check
        assert self.client is not None

        if (
            self.client.telemetry.enabled_substate
            != EnabledSubstate.SLEWING_OR_TRACKING
        ):
            if self._tracking_started_telemetry_counter <= 0:
                raise salobj.ExpectedError(
                    "Low-level controller in substate "
                    f"{self.client.telemetry.enabled_substate} "
                    f"instead of {EnabledSubstate.SLEWING_OR_TRACKING}"
                )
        dt = data.tai - utils.current_tai()
        curr_pos = data.angle + data.velocity * dt
        if (
            not self.client.config.lower_pos_limit
            <= curr_pos
            <= self.client.config.upper_pos_limit
        ):
            raise salobj.ExpectedError(
                f"current position {curr_pos} not in range "
                f"[{self.client.config.lower_pos_limit}, "
                f"{self.client.config.upper_pos_limit}]"
            )
        if not abs(data.velocity) <= self.client.config.velocity_limit:
            raise salobj.ExpectedError(
                f"abs(velocity={data.velocity}) > "
                f"[{self.client.config.velocity_limit}"
            )
        await self.run_command(
            code=enums.CommandCode.TRACK_VEL_CMD,
            param1=data.tai,
            param2=data.angle,
            param3=data.velocity,
        )
        await self.evt_target.set_write(
            position=data.angle, velocity=data.velocity, tai=data.tai, force_output=True
        )

    async def do_trackStart(self, data: salobj.BaseMsgType) -> None:
        """Start tracking.

        Once this is run you must issue ``track`` commands at 10-20Hz
        until you are done tracking, then issue the ``stop`` command.
        """
        self.assert_enabled_substate(EnabledSubstate.STATIONARY)
        await self.run_command(
            code=enums.CommandCode.SET_ENABLED_SUBSTATE,
            param1=enums.SetEnabledSubstateParam.TRACK,
        )
        self._tracking_started_telemetry_counter = 2

    async def command_llv_fault(self) -> None:
        """Command the low-level controller to go to fault state."""
        if self._faulting:
            self.log.debug("command_llv_fault called, but already faulting")
            return

        self.log.debug("Sending the low-level controller to fault state")
        try:
            self._faulting = True
            await self.run_command(
                code=self.CommandCode.SET_STATE,
                param1=hexrotcomm.SetStateParam.FAULT,
            )
        finally:
            self._faulting = False

    async def telemetry_callback(
        self, client: hexrotcomm.CommandTelemetryClient
    ) -> None:
        """Called when the TCP/IP controller outputs telemetry.

        Parameters
        ----------
        client : `lsst.ts.hexrotcomm.CommandTelemetryClient`
            TCP/IP client.
        """
        tai_unix = client.header.tai_sec + client.header.tai_nsec / 1e9

        if self.next_clock_offset_task.done():
            clock_offset = tai_unix - utils.current_tai()
            await self.evt_clockOffset.set_write(offset=clock_offset)
            self.next_clock_offset_task = asyncio.create_task(
                asyncio.sleep(CLOCK_OFFSET_EVENT_INTERVAL)
            )

        if self._tracking_started_telemetry_counter > 0:
            self._tracking_started_telemetry_counter -= 1
        await self.evt_summaryState.set_write(summaryState=self.summary_state)

        # Strangely telemetry.state, fault_substate, and enabled_substate are
        # floats from the controller. But they should only have integer value,
        # so I output them as integers.
        controller_state_data = dict(
            controllerState=int(client.telemetry.state),
            enabledSubstate=int(client.telemetry.enabled_substate),
            faultSubstate=int(client.telemetry.fault_substate),
            applicationStatus=client.telemetry.application_status,
        )

        await self.evt_controllerState.set_write(**controller_state_data)

        await self.tel_rotation.set_write(
            demandPosition=client.telemetry.demand_pos,
            demandVelocity=client.telemetry.demand_vel,
            demandAcceleration=client.telemetry.demand_accel,
            actualPosition=client.telemetry.current_pos,
            actualVelocity=(
                client.telemetry.current_vel_ch_a_fb
                + client.telemetry.current_vel_ch_b_fb
            )
            / 2,
            debugActualVelocityA=client.telemetry.current_vel_ch_a_fb,
            debugActualVelocityB=client.telemetry.current_vel_ch_b_fb,
            odometer=client.telemetry.rotator_odometer,
            timestamp=tai_unix,
        )

        electrical = dict(
            copleyStatusWordDrive=[
                client.telemetry.status_word_drive0,
                client.telemetry.status_word_drive0_axis_b,
            ],
            copleyLatchingFaultStatus=[
                client.telemetry.latching_fault_status_register,
                client.telemetry.latching_fault_status_register_axis_b,
            ],
            copleyFaultStatus=client.telemetry.copley_fault_status_register,
        )

        await self.tel_electrical.set_write(**electrical)

        await self.tel_motors.set_write(
            raw=[
                client.telemetry.motor_encoder_ch_a,
                client.telemetry.motor_encoder_ch_b,
            ],
            # The torque from the low-level controller is N-m/1e6
            # (and is an integer); convert it to N-m
            torque=[
                client.telemetry.motor_torque_axis_a / 1e6,
                client.telemetry.motor_torque_axis_b / 1e6,
            ],
            current=client.telemetry.motor_current,
            busVoltage=client.telemetry.bus_voltage,
        )

        await self.evt_inPosition.set_write(
            inPosition=bool(
                client.telemetry.flags_move_success
                or client.telemetry.flags_tracking_success
            )
        )

        await self.evt_commandableByDDS.set_write(
            state=bool(
                client.telemetry.application_status
                & ApplicationStatus.DDS_COMMAND_SOURCE
            ),
        )

        await self.evt_tracking.set_write(
            tracking=bool(client.telemetry.flags_tracking_success),
            lost=bool(client.telemetry.flags_tracking_lost),
            noNewCommand=bool(client.telemetry.flags_no_new_track_cmd_error),
        )

        safety_interlock = (
            client.telemetry.application_status & ApplicationStatus.SAFETY_INTERLOCK
        )
        await self.evt_interlock.set_write(engaged=safety_interlock)

        # Check following error if enabled and if not already checking
        # following error (don't let these tasks build up).
        if self._check_ccw_following_error_task.done() and (not self._bypass_ccw):
            self._check_ccw_following_error_task = asyncio.create_task(
                self.check_ccw_following_error()
            )

        # Put the current and demand positions to the vibration detector
        assert self._vibration_detector is not None

        is_queue_full = self._vibration_detector.put_data(
            client.telemetry.current_pos, client.telemetry.demand_pos
        )
        if self._detect_vibration_task.done() and is_queue_full:
            self._detect_vibration_task = asyncio.create_task(self.detect_vibration())

    def make_mock_controller(self) -> mock_controller.MockMTRotatorController:
        return mock_controller.MockMTRotatorController(
            log=self.log,
            port=0,
            initial_state=ControllerState.STANDBY,
        )

    @classmethod
    def add_arguments(cls, parser: argparse.ArgumentParser) -> None:
        super(RotatorCsc, cls).add_arguments(parser)

        parser.add_argument(
            "--bypass-ccw",
            action="store_true",
            default=False,
            help="""
                 Bypass the check of camera cable wrapper (CCW) or not. This is
                 for test purpose only.
                 """,
        )

    @classmethod
    def add_kwargs_from_args(
        cls, args: argparse.Namespace, kwargs: typing.Dict[str, typing.Any]
    ) -> None:
        super(RotatorCsc, cls).add_kwargs_from_args(args, kwargs)

        kwargs["bypass_ccw"] = args.bypass_ccw


def run_mtrotator() -> None:
    """Run the MTRotator CSC."""
    asyncio.run(RotatorCsc.amain(index=None))
