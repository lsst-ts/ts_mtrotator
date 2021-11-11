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

from lsst.ts import utils
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
        Must be `lsst.ts.salobj.State.STANDBY` unless simulating
        (``simulation_mode != 0``).
    settings_to_apply : `str`, optional
        Settings to apply if ``initial_state`` is `State.DISABLED`
        or `State.ENABLED`.
    simulation_mode : `int` (optional)
        Simulation mode. Allowed values:

        * 0: regular operation.
        * 1: simulation: use a mock low level controller.

    Raises
    ------
    ValueError
        If ``initial_state != lsst.ts.salobj.State.STANDBY``
        and not simulating (``simulation_mode = 0``).

    Notes
    -----
    **Error Codes**

    See `lsst.ts.idl.enums.MTRotator.ErrorCode`
    """

    valid_simulation_modes = [0, 1]
    version = __version__

    def __init__(
        self,
        config_dir=None,
        initial_state=salobj.State.STANDBY,
        settings_to_apply="",
        simulation_mode=0,
    ):
        self.client = None
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

        self._check_ccw_following_error_task = utils.make_done_future()
        self._reported_ccw_following_error_issue = False
        self._faulting = False
        self._prev_flags_tracking_success = False
        self._prev_flags_tracking_lost = False

        super().__init__(
            name="MTRotator",
            index=0,
            sync_pattern=constants.ROTATOR_SYNC_PATTERN,
            CommandCode=enums.CommandCode,
            ConfigClass=structs.Config,
            TelemetryClass=structs.Telemetry,
            config_schema=CONFIG_SCHEMA,
            config_dir=config_dir,
            initial_state=initial_state,
            settings_to_apply=settings_to_apply,
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
                    self.log.error(err_msg)
                    await self.command_llv_fault()
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
                    self.log.error(
                        f"Camera cable wrap not following closely enough: "
                        f"error # {self._num_ccw_following_errors} = {following_error} "
                        f"> {self.config.max_ccw_following_error} deg"
                    )
                    await self.command_llv_fault()
        except Exception:
            self.log.exception("check_ccw_following_error failed")

    async def configure(self, config):
        self.config = config

    def config_callback(self, client):
        """Called when the TCP/IP controller outputs configuration.

        Parameters
        ----------
        client : `lsst.ts.hexrotcomm.CommandTelemetryClient`
            TCP/IP client.
        """
        self.evt_configuration.set_put(
            positionAngleUpperLimit=client.config.upper_pos_limit,
            velocityLimit=client.config.velocity_limit,
            accelerationLimit=client.config.accel_limit,
            positionAngleLowerLimit=client.config.lower_pos_limit,
            followingErrorThreshold=client.config.following_error_threshold,
            trackingSuccessPositionThreshold=client.config.track_success_pos_threshold,
            trackingLostTimeout=client.config.tracking_lost_timeout,
        )

    def connect_callback(self, client):
        super().connect_callback(client)
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
        self.assert_summary_state(salobj.State.DISABLED)

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
        self.log.warning(
            "fault command issued; sending low-level controller to FAULT state"
        )
        await self.command_llv_fault()

    async def do_move(self, data):
        """Go to the position specified by the most recent ``positionSet``
        command.
        """
        self.assert_enabled_substate(EnabledSubstate.STATIONARY)
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
        self.evt_target.set_put(
            position=data.position,
            velocity=0,
            tai=utils.current_tai(),
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

    async def command_llv_fault(self):
        """Command the low-level controller to go to fault state."""
        if self._faulting:
            self.log.debug("command_llv_fault called, but already faulting")
            return

        self.log.debug("Sending the low-level controller to fault state")
        try:
            self._faulting = True
            await self.run_command(code=enums.CommandCode.FAULT)
        finally:
            self._faulting = False

    def telemetry_callback(self, client):
        """Called when the TCP/IP controller outputs telemetry.

        Parameters
        ----------
        client : `lsst.ts.hexrotcomm.CommandTelemetryClient`
            TCP/IP client.
        """
        tai_unix = client.header.tai_sec + client.header.tai_nsec / 1e9
        if self._tracking_started_telemetry_counter > 0:
            self._tracking_started_telemetry_counter -= 1
        self.evt_summaryState.set_put(summaryState=self.summary_state)
        # Strangely telemetry.state, offline_substate and enabled_substate
        # are all floats from the controller. But they should only have
        # integer value, so I output them as integers.
        self.evt_controllerState.set_put(
            controllerState=int(client.telemetry.state),
            offlineSubstate=int(client.telemetry.offline_substate),
            enabledSubstate=int(client.telemetry.enabled_substate),
            applicationStatus=client.telemetry.application_status,
        )

        self.tel_rotation.set_put(
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
        self.tel_electrical.set_put(
            copleyStatusWordDrive=[
                client.telemetry.status_word_drive0,
                client.telemetry.status_word_drive0_axis_b,
            ],
            copleyLatchingFaultStatus=[
                client.telemetry.latching_fault_status_register,
                client.telemetry.latching_fault_status_register_axis_b,
            ],
        )
        self.tel_motors.set_put(
            calibrated=[
                client.telemetry.ch_a_fb,
                client.telemetry.ch_b_fb,
            ],
            raw=[
                client.telemetry.motor_encoder_ch_a,
                client.telemetry.motor_encoder_ch_b,
            ],
            # DM-31447 Uncomment when the low-level controller provides
            # this data (right now the fields are always zero).
            # current=[
            #     client.telemetry.motor_current_axis_a,
            #     client.telemetry.motor_current_axis_b,
            # ],
            # The torque from the low-level controller is N-m/1e6
            # (and is an integer); convert it to N-m
            torque=[
                client.telemetry.motor_torque_axis_a / 1e6,
                client.telemetry.motor_torque_axis_b / 1e6,
            ],
        )

        self.evt_inPosition.set_put(
            inPosition=bool(
                client.telemetry.flags_move_success
                or client.telemetry.flags_tracking_success
            )
        )

        self.evt_commandableByDDS.set_put(
            state=bool(
                client.telemetry.application_status
                & ApplicationStatus.DDS_COMMAND_SOURCE
            ),
        )

        self.evt_tracking.set_put(
            tracking=bool(client.telemetry.flags_tracking_success),
            lost=bool(client.telemetry.flags_tracking_lost),
            noNewCommand=bool(client.telemetry.flags_no_new_track_cmd_error),
        )

        safety_interlock = (
            client.telemetry.application_status & ApplicationStatus.SAFETY_INTERLOCK
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
            port=0,
            initial_state=initial_ctrl_state,
        )
