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

__all__ = ["RotatorCsc"]

import asyncio

from lsst.ts import salobj
from lsst.ts import hexrotcomm
from lsst.ts.idl.enums.MTRotator import EnabledSubstate, ApplicationStatus
from . import __version__
from .config_schema import CONFIG_SCHEMA
from . import constants
from . import enums
from . import structs
from . import mock_controller

# Maximum allowed age of the camera cable wrap telemetry data
# when checking following error.
MAX_CCW_TELEMETRY_AGE = 1


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
        Must be `lsst.ts.salobj.State.OFFLINE` unless simulating
        (``simulation_mode != 0``).
    simulation_mode : `int` (optional)
        Simulation mode. Allowed values:

        * 0: regular operation.
        * 1: simulation: use a mock low level controller.

    Raises
    ------
    ValueError
        If ``initial_state != lsst.ts.salobj.State.OFFLINE``
        and not simulating (``simulation_mode = 0``).

    Notes
    -----
    **Error Codes**

    The error codes are described in `ErrorCode`.

    This CSC is unusual in several respect:

    * It acts as a server (not a client) for a low level controller
      (because that is how the low level controller is written).
    * The low level controller maintains the summary state and detailed state
      (that's why this code inherits from Controller instead of BaseCsc).
    * The simulation mode can only be set at construction time.
    """

    valid_simulation_modes = [0, 1]
    version = __version__

    def __init__(
        self, config_dir=None, initial_state=salobj.State.OFFLINE, simulation_mode=0
    ):
        self.server = None
        self.mock_ctrl = None

        # Set this to 2 when trackStart is called, then decrement
        # when telemetry is received. If > 0 or enabled_substate is
        # SLEWING_OR_TRACKING then allow the track command.
        # This solves the problem of allowing the track command
        # immediately after the trackStart, before telemetry is received.
        self._tracking_started_telemetry_counter = 0

        # The sequential number of times the camera cable wrap following error
        # has been too large.
        self._num_ccw_following_errors = 0

        self._check_ccw_following_error_task = salobj.make_done_future()
        self._reported_ccw_following_error_issue = False
        self._faulting = False
        self._prev_flags_tracking_success = False
        self._prev_flags_tracking_lost = False

        super().__init__(
            name="MTRotator",
            index=0,
            port=constants.TELEMETRY_PORT,
            sync_pattern=constants.ROTATOR_SYNC_PATTERN,
            CommandCode=enums.CommandCode,
            ConfigClass=structs.Config,
            TelemetryClass=structs.Telemetry,
            config_dir=config_dir,
            config_schema=CONFIG_SCHEMA,
            initial_state=initial_state,
            simulation_mode=simulation_mode,
        )
        self.mtmount_remote = salobj.Remote(domain=self.domain, name="MTMount")

    async def start(self):
        await self.mtmount_remote.start_task
        self.evt_inPosition.set_put(inPosition=False, force_output=True)
        await super().start()

    async def check_ccw_following_error(self):
        """Check the camera cable wrap following error.

        Publish the value, if the camera cable wrap angle can be read.
        If ENABLED and the value is too large, then go to FAULT state.

        Note: this is designed to be called by telemetry_callback.
        Thus it is called every time telemetry is read
        from the low-level controller.
        """
        try:
            rot_data = self.tel_rotation.data
            rot_tai = rot_data.timestamp

            # Get camera cable wrap telemetry and check its age.
            ccw_data = self.mtmount_remote.tel_cameraCableWrap.get()
            if ccw_data is None:
                err_msg = "Cannot read cameraCableWrap telemetry"
                if self.summary_state == salobj.State.ENABLED:
                    await self.afault(
                        code=enums.ErrorCode.CCW_NO_TELEMETRY, report=err_msg
                    )
                elif not self._reported_ccw_following_error_issue:
                    self.log.warning(err_msg)
                    self._reported_ccw_following_error_issue = True

                return

            dt = rot_tai - ccw_data.timestamp
            if abs(dt) > MAX_CCW_TELEMETRY_AGE:
                err_msg = (
                    "Camera cable wrap telemetry is too old: "
                    f"dt={dt}; abs(dt) > {MAX_CCW_TELEMETRY_AGE}"
                )
                if self.summary_state == salobj.State.ENABLED:
                    await self.afault(
                        code=enums.ErrorCode.CCW_NO_TELEMETRY, report=err_msg
                    )
                elif not self._reported_ccw_following_error_issue:
                    self.log.warning(err_msg)
                    self._reported_ccw_following_error_issue = True
                return

            self._reported_ccw_following_error_issue = False

            # Compute and report the following error.
            corr_ccw_pos = ccw_data.actualPosition + ccw_data.actualVelocity * dt
            following_error = rot_data.actualPosition - corr_ccw_pos
            self.tel_ccwFollowingError.set_put(
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
                    await self.afault(
                        code=enums.ErrorCode.CCW_FOLLOWING_ERROR,
                        report=f"Camera cable wrap not following closely enough: "
                        f"error # {self._num_ccw_following_errors} = {following_error} "
                        f"> {self.config.max_ccw_following_error} deg",
                    )
        except Exception:
            self.log.exception("check_ccw_following_error failed")

    async def configure(self, config):
        self.config = config

    def config_callback(self, server):
        """Called when the TCP/IP controller outputs configuration.

        Parameters
        ----------
        server : `lsst.ts.hexrotcomm.CommandTelemetryServer`
            TCP/IP server.
        """
        self.evt_configuration.set_put(
            positionAngleUpperLimit=server.config.upper_pos_limit,
            velocityLimit=server.config.velocity_limit,
            accelerationLimit=server.config.accel_limit,
            positionAngleLowerLimit=server.config.lower_pos_limit,
            followingErrorThreshold=server.config.following_error_threshold,
            trackingSuccessPositionThreshold=server.config.track_success_pos_threshold,
            trackingLostTimeout=server.config.tracking_lost_timeout,
        )

    def connect_callback(self, server):
        super().connect_callback(server)
        # Always reset this counter; if newly connected then we'll start
        # accumulating the count and otherwise it makes no difference.
        self._num_ccw_following_errors = 0

    async def do_configureAcceleration(self, data):
        """Specify the acceleration limit."""
        self.assert_enabled_substate(EnabledSubstate.STATIONARY)
        if not 0 < data.alimit <= constants.MAX_ACCEL_LIMIT:
            raise salobj.ExpectedError(
                f"alimit={data.alimit} must be > 0 and <= {constants.MAX_ACCEL_LIMIT}"
            )
        await self.run_command(code=enums.CommandCode.CONFIG_ACCEL, param1=data.alimit)

    async def do_configureVelocity(self, data):
        """Specify the velocity limit."""
        self.assert_enabled_substate(EnabledSubstate.STATIONARY)
        if not 0 < data.vlimit <= constants.MAX_VEL_LIMIT:
            raise salobj.ExpectedError(
                f"vlimit={data.vlimit} must be > 0 and <= {constants.MAX_VEL_LIMIT}"
            )
        await self.run_command(code=enums.CommandCode.CONFIG_VEL, param1=data.vlimit)

    async def do_enable(self, data):
        self.assert_summary_state(salobj.State.DISABLED, isbefore=True)

        # Make sure we have camera cable wrap telemetry (from MTMount),
        # and that it is recent enough to use for measuring following error.
        try:
            await self.mtmount_remote.tel_cameraCableWrap.next(
                flush=False, timeout=MAX_CCW_TELEMETRY_AGE * 10
            )
        except asyncio.TimeoutError:
            raise salobj.ExpectedError(
                "Cannot enable the rotator until it receives camera cable wrap telemetry"
            )
        self._reported_ccw_following_error_issue = False
        await super().do_enable(data)

    async def do_fault(self, data):
        if self.summary_state != salobj.State.ENABLED:
            raise salobj.ExpectedError("Not enabled")
        await self.afault(code=enums.ErrorCode.FAULT_COMMAND, report="fault command")

    async def do_move(self, data):
        """Go to the position specified by the most recent ``positionSet``
        command.
        """
        self.assert_enabled_substate(EnabledSubstate.STATIONARY)
        if (
            not self.server.config.lower_pos_limit
            <= data.position
            <= self.server.config.upper_pos_limit
        ):
            raise salobj.ExpectedError(
                f"position {data.position} not in range "
                f"[{self.server.config.lower_pos_limit}, "
                f"{self.server.config.upper_pos_limit}]"
            )
        cmd1 = self.make_command(
            code=enums.CommandCode.POSITION_SET, param1=data.position
        )
        cmd2 = self.make_command(
            code=enums.CommandCode.SET_ENABLED_SUBSTATE,
            param1=enums.SetEnabledSubstateParam.MOVE_POINT_TO_POINT,
        )
        await self.run_multiple_commands(cmd1, cmd2)
        self.evt_target.set_put(
            position=data.position,
            velocity=0,
            tai=salobj.current_tai(),
            force_output=True,
        )

    async def do_stop(self, data):
        """Halt tracking or any other motion."""
        if self.summary_state != salobj.State.ENABLED:
            raise salobj.ExpectedError("Not enabled")
        await self.run_command(
            code=enums.CommandCode.SET_ENABLED_SUBSTATE,
            param1=enums.SetEnabledSubstateParam.STOP,
        )

    async def do_track(self, data):
        """Specify a position, velocity, TAI time tracking update."""
        if self.summary_state != salobj.State.ENABLED:
            raise salobj.ExpectedError("Not enabled")
        if (
            self.server.telemetry.enabled_substate
            != EnabledSubstate.SLEWING_OR_TRACKING
        ):
            if self._tracking_started_telemetry_counter <= 0:
                raise salobj.ExpectedError(
                    "Low-level controller in substate "
                    f"{self.server.telemetry.enabled_substate} "
                    f"instead of {EnabledSubstate.SLEWING_OR_TRACKING}"
                )
        dt = data.tai - salobj.current_tai()
        curr_pos = data.angle + data.velocity * dt
        if (
            not self.server.config.lower_pos_limit
            <= curr_pos
            <= self.server.config.upper_pos_limit
        ):
            raise salobj.ExpectedError(
                f"current position {curr_pos} not in range "
                f"[{self.server.config.lower_pos_limit}, "
                f"{self.server.config.upper_pos_limit}]"
            )
        if not abs(data.velocity) <= self.server.config.velocity_limit:
            raise salobj.ExpectedError(
                f"abs(velocity={data.velocity}) > "
                f"[{self.server.config.velocity_limit}"
            )
        await self.run_command(
            code=enums.CommandCode.TRACK_VEL_CMD,
            param1=data.tai,
            param2=data.angle,
            param3=data.velocity,
        )
        self.evt_target.set_put(
            position=data.angle, velocity=data.velocity, tai=data.tai, force_output=True
        )

    async def do_trackStart(self, data):
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

    async def afault(self, code, report, traceback=""):
        """Enter the fault state and output the ``errorCode`` event.

        This is an asynchronous version of the standard fault method
        `lsst.ts.salobj.BaseCsc.fault`. Note that the standard version
        does not work for MTRotator (and raises `NotImplementedError`)
        because the summary state is kept in the low-level controller.

        Parameters
        ----------
        code : `int`
            Error code for the ``errorCode`` event.
            If `None` then ``errorCode`` is not output and you should
            output it yourself. Specifying `None` is deprecated;
            please always specify an integer error code.
        report : `str`
            Description of the error.
        traceback : `str`, optional
            Description of the traceback, if any.
        """
        if self._faulting:
            self.log.debug("Fault called, but already faulting")
            return

        self.log.debug("Sending the low-level controller to fault state")
        try:
            self._faulting = True
            await self.run_command(code=enums.CommandCode.FAULT)
        finally:
            self._faulting = False

        try:
            self.evt_errorCode.set_put(
                errorCode=code,
                errorReport=report,
                traceback=traceback,
                force_output=True,
            )
        except Exception:
            self.log.exception(
                f"Failed to output errorCode: code={code!r}; report={report!r}"
            )

    def telemetry_callback(self, server):
        """Called when the TCP/IP controller outputs telemetry.

        Parameters
        ----------
        server : `lsst.ts.hexrotcomm.CommandTelemetryServer`
            TCP/IP server.
        """
        tel_utc_unix = server.header.tv_sec + server.header.tv_nsec / 1e9
        tel_tai_unix = salobj.tai_from_utc(tel_utc_unix)
        if self._tracking_started_telemetry_counter > 0:
            self._tracking_started_telemetry_counter -= 1
        self.evt_summaryState.set_put(summaryState=self.summary_state)
        # Strangely telemetry.state, offline_substate and enabled_substate
        # are all floats from the controller. But they should only have
        # integer value, so I output them as integers.
        self.evt_controllerState.set_put(
            controllerState=int(server.telemetry.state),
            offlineSubstate=int(server.telemetry.offline_substate),
            enabledSubstate=int(server.telemetry.enabled_substate),
            applicationStatus=server.telemetry.application_status,
        )

        self.tel_rotation.set_put(
            demandPosition=server.telemetry.demand_pos,
            demandVelocity=server.telemetry.demand_vel,
            demandAcceleration=server.telemetry.demand_accel,
            actualPosition=server.telemetry.current_pos,
            actualVelocity=(
                server.telemetry.current_vel_ch_a_fb
                + server.telemetry.current_vel_ch_b_fb
            )
            / 2,
            debugActualVelocityA=server.telemetry.current_vel_ch_a_fb,
            debugActualVelocityB=server.telemetry.current_vel_ch_b_fb,
            timestamp=tel_tai_unix,
        )
        self.tel_electrical.set_put(
            copleyStatusWordDrive=[
                server.telemetry.status_word_drive0,
                server.telemetry.status_word_drive0_axis_b,
            ],
            copleyLatchingFaultStatus=[
                server.telemetry.latching_fault_status_register,
                server.telemetry.latching_fault_status_register_axis_b,
            ],
        )
        self.tel_motors.set_put(
            calibrated=[
                server.telemetry.state_estimation_ch_a_fb,
                server.telemetry.state_estimation_ch_b_fb,
            ],
            raw=[
                server.telemetry.state_estimation_ch_a_motor_encoder,
                server.telemetry.state_estimation_ch_b_motor_encoder,
            ],
        )

        self.evt_inPosition.set_put(
            inPosition=bool(
                server.telemetry.flags_pt2pt_move_complete
                or server.telemetry.flags_slew_complete
            )
        )

        self.evt_commandableByDDS.set_put(
            state=bool(
                server.telemetry.application_status
                & ApplicationStatus.DDS_COMMAND_SOURCE
            ),
        )

        self.evt_tracking.set_put(
            tracking=bool(server.telemetry.flags_tracking_success),
            lost=bool(server.telemetry.flags_tracking_lost),
            noNewCommand=bool(server.telemetry.flags_no_new_track_cmd_error),
        )

        safety_interlock = (
            server.telemetry.application_status & ApplicationStatus.SAFETY_INTERLOCK
        )
        self.evt_interlock.set_put(
            detail="Engaged" if safety_interlock else "Disengaged",
        )

        # Check following error if enabled and if not already checking
        # following error (don't let these tasks build up).
        if self._check_ccw_following_error_task.done():
            self._check_ccw_following_error_task = asyncio.create_task(
                self.check_ccw_following_error()
            )

    def make_mock_controller(self, initial_ctrl_state):
        return mock_controller.MockMTRotatorController(
            log=self.log,
            host=self.server.host,
            initial_state=initial_ctrl_state,
            command_port=self.server.command_port,
            telemetry_port=self.server.telemetry_port,
        )
