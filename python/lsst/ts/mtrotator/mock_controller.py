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

__all__ = ["MockMTRotatorController", "TRACK_TIMEOUT"]

import asyncio
import math
import random

from lsst.ts import hexrotcomm, simactuators, utils
from lsst.ts.xml.enums.MTRotator import (
    ApplicationStatus,
    ControllerState,
    EnabledSubstate,
    FaultSubstate,
)

from . import constants, enums, structs

# Maximum time between track commands (seconds)
# The real controller may use 0.15
TRACK_TIMEOUT = 1


class MockMTRotatorController(hexrotcomm.BaseMockController):
    """Mock MT rotator controller that talks over TCP/IP.

    Parameters
    ----------
    log : `logging.Logger`
        Logger.
    port : `int` (optional)
        Port for telemetry and configuration;
        if nonzero then the command port will be one larger.
        Specify 0 to choose random values for both ports;
        this is recommended for unit tests, to avoid collision
        with other tests.
        Do not specify 0 with host=None (see Raises section).
    initial_state : `lsst.ts.xml.enums.MTRotator.ControllerState` (optional)
        Initial state of mock controller.

    Raises
    ------
    ValueError
        If host=None and port=0. See `CommandTelemetryServer` for details.

    Notes
    -----
    To start the mock controller:

        ctrl = MockRotatorController(...)
        await ctrl.connect_task

    To stop the server:

        await ctrl.stop()

    *Known Limitations*

    * Constant-velocity motion is not supported.
    * The odometer resets to zero each time the mock controller is constructed.
    * The motor current and torque are scaled versions of current acceleration,
      with a wild guess as to scale.
    * Motor current and torque exactly match for motors A and B.
    * Many telemetry fields are not set at all.
    """

    # Motor current (in A) per acceleration (in deg/sec^2).
    # The value is arbitrary, because I have no idea what is realistic.
    current_per_acceleration = 5

    # Motor torque (in N-m) per acceleration (in deg/sec^2).
    # Warning: the telemetry publishes an integer in units N-m/1e6
    # but it simplfies unit tests to have this scale factor produce N-m,
    # the units the CSC uses to publish the data in the motors event.
    # The value is arbitrary, because I have no idea what is realistic.
    torque_per_acceleration = 0.002

    def __init__(
        self,
        log,
        port=0,
        initial_state=ControllerState.STANDBY,
    ):
        self.encoder_resolution = 200_000  # counts/deg; arbitrary
        # Amplitude of jitter in measured position (deg),
        # to simulate encoder jitter.
        self.position_jitter = 0.000003

        # Previous position (without jitter) (deg); used to set odometer.
        self.previous_pos = None

        # Approximate cumulative rotation, in deg
        self.odometer = 0

        self.tracking_timed_out = False
        config = structs.Config()
        config.velocity_limit = 3  # deg/sec
        config.accel_limit = 1  # deg/sec^2
        config.pos_error_threshold = 0.1  # deg
        config.lower_pos_limit = -90  # deg
        config.upper_pos_limit = 90  # deg
        config.following_error_threshold = 0.1  # deg
        config.track_success_pos_threshold = 0.01  # deg
        config.tracking_lost_timeout = 5  # sec
        config.emergency_jerk_limit = 10  # m/sec^3
        config.emergency_accel_limit = 1.5  # m/sec^2
        config.disable_limit_max_time = 120  # sec
        config.max_velocity_limit = 5  # deg/sec
        config.drives_enabled = False

        telemetry = structs.Telemetry()
        telemetry.set_pos = math.nan
        self.tracking_timer_task = utils.make_done_future()

        self.rotator = simactuators.TrackingActuator(
            min_position=config.lower_pos_limit,
            max_position=config.upper_pos_limit,
            max_velocity=config.velocity_limit,
            max_acceleration=config.accel_limit,
            dtmax_track=0.25,
        )
        # Set True when the first TRACK_VEL_CMD command is received
        # and False when not tracking. Used to delay setting
        # telemetry.flags_slew_complete to 1 until we know
        # where we are going.
        self.track_vel_cmd_seen = False

        # Dict of command key: command
        extra_commands = {
            # TODO DM-39787: move this command entry (and the do_fault method)
            # to BaseMockController in ts_hexrotcomm,
            # once MTHexapod supports MTRotator's simplified states.
            (
                enums.CommandCode.SET_STATE,
                hexrotcomm.SetStateParam.FAULT,
            ): self.do_fault,
            (
                enums.CommandCode.SET_ENABLED_SUBSTATE,
                enums.SetEnabledSubstateParam.MOVE_POINT_TO_POINT,
            ): self.do_move_point_to_point,
            (
                enums.CommandCode.SET_ENABLED_SUBSTATE,
                enums.SetEnabledSubstateParam.TRACK,
            ): self.do_track,
            (
                enums.CommandCode.SET_ENABLED_SUBSTATE,
                enums.SetEnabledSubstateParam.STOP,
            ): self.do_stop,
            (
                enums.CommandCode.SET_ENABLED_SUBSTATE,
                enums.SetEnabledSubstateParam.CONSTANT_VELOCITY,
            ): self.do_constant_velocity,
            enums.CommandCode.POSITION_SET: self.do_position_set,
            enums.CommandCode.SET_CONSTANT_VEL: self.do_set_constant_vel,
            enums.CommandCode.CONFIG_VEL: self.do_config_vel,
            enums.CommandCode.CONFIG_ACCEL: self.do_config_accel,
            enums.CommandCode.TRACK_VEL_CMD: self.do_track_vel_cmd,
            enums.CommandCode.ENABLE_DRIVES: self.do_enable_drives,
        }

        super().__init__(
            log=log,
            CommandCode=enums.CommandCode,
            extra_commands=extra_commands,
            config=config,
            telemetry=telemetry,
            port=port,
            initial_state=initial_state,
        )

    async def end_run_command(self, command, cmd_method):
        if cmd_method != self.do_position_set:
            self.telemetry.set_pos = math.nan

    async def close(self):
        """Kill command and telemetry tasks and close the connections.

        Always safe to call.
        """
        self.rotator.stop()
        self.tracking_timer_task.cancel()
        await super().close()

    # TODO DM-39787: move the following state transition methods
    # to BaseMockController in ts_hexrotcomm (and delete the
    # ones that raise NotImplementedError),
    # once MTHexapod supports MTRotator's simplified states.
    async def do_enter_control(self, command):
        raise NotImplementedError()

    async def do_start(self, command):
        raise NotImplementedError()

    async def do_disable(self, command):
        raise NotImplementedError()

    async def do_exit(self, command):
        raise NotImplementedError()

    async def do_enable(self, command):
        self.assert_state(ControllerState.STANDBY)
        self.set_state(ControllerState.ENABLED)

    async def do_standby(self, command):
        self.assert_state(ControllerState.ENABLED)
        self.set_state(ControllerState.STANDBY)

    async def do_fault(self, command):
        self.set_state(ControllerState.FAULT)

    # TODO DM-39787: end of state transition methods to move.

    async def do_config_vel(self, command):
        self.assert_stationary()
        if not 0 < command.param1 <= constants.MAX_VEL_LIMIT:
            raise ValueError(
                f"Requested velocity limit {command.param1} "
                f"not in range (0, {constants.MAX_VEL_LIMIT}]"
            )
        self.config.velocity_limit = command.param1
        await self.write_config()

    async def do_config_accel(self, command):
        self.assert_stationary()
        if not 0 < command.param1 <= constants.MAX_ACCEL_LIMIT:
            raise ValueError(
                f"Requested accel limit {command.param1} "
                f"not in range (0, {constants.MAX_ACCEL_LIMIT}]"
            )
        self.config.accel_limit = command.param1
        await self.write_config()

    async def do_clearError(self, data):
        super().do_clearError(data)
        self.tracking_timed_out = False

    async def do_constant_velocity(self, command):
        raise RuntimeError("The mock controller does not support CONSTANT_VELOCITY")

    async def do_position_set(self, command):
        self.assert_stationary()
        self.telemetry.set_pos = command.param1

    async def do_track(self, command):
        self.assert_stationary()
        self.telemetry.enabled_substate = EnabledSubstate.SLEWING_OR_TRACKING
        self.tracking_timer_task.cancel()

    async def do_stop(self, command):
        self.assert_state(ControllerState.ENABLED)
        self.rotator.stop()
        self.tracking_timer_task.cancel()
        self.telemetry.enabled_substate = EnabledSubstate.STATIONARY

    async def do_move_point_to_point(self, command):
        if not math.isfinite(self.telemetry.set_pos):
            raise RuntimeError(
                "Must call POSITION_SET before calling MOVE_POINT_TO_POINT"
            )
        self.rotator.set_target(
            tai=utils.current_tai(), position=self.telemetry.set_pos, velocity=0
        )
        self.telemetry.enabled_substate = EnabledSubstate.MOVING_POINT_TO_POINT
        self.telemetry.flags_pt2pt_move_complete = 0

    async def do_set_constant_vel(self, command):
        raise RuntimeError("The mock controller does not support SET_CONSTANT_VEL")

    async def do_track_vel_cmd(self, command):
        tai, pos, vel = command.param1, command.param2, command.param3
        dt = utils.current_tai() - tai
        curr_pos = pos + vel * dt
        if not self.config.lower_pos_limit <= curr_pos <= self.config.upper_pos_limit:
            self.set_state(ControllerState.FAULT)
            raise RuntimeError(
                f"fault: commanded position {curr_pos} not in range "
                f"[{self.config.lower_pos_limit}, {self.config.upper_pos_limit}]"
            )
        self.rotator.set_target(tai=tai, position=pos, velocity=vel)
        self.tracking_timer_task.cancel()
        self.tracking_timer_task = asyncio.create_task(self.tracking_timer())
        self.track_vel_cmd_seen = True

    async def do_enable_drives(self, command):
        self.config.drives_enabled = bool(command.param1)
        await self.write_config()

    def set_state(self, state):
        # Override to stop the rotator if not enabled
        super().set_state(state)
        if state != ControllerState.ENABLED:
            self.rotator.stop()

    async def tracking_timer(self):
        """If this times out then go into a FAULT state.

        Used to make sure TRACK commands arrive often enough.
        """
        await asyncio.sleep(TRACK_TIMEOUT)
        self.log.error("Tracking timer expired; going to FAULT")
        self.tracking_timed_out = True
        self.set_state(ControllerState.FAULT)

    async def update_telemetry(self, curr_tai):
        try:
            # Add jitter to the current position, for realism
            # and to exercise RotatorCommander filtering of jitter.
            curr_segment = self.rotator.path.at(curr_tai)
            curr_pos = curr_segment.position + self.position_jitter * (
                random.random() - 0.5
            )
            if self.previous_pos is not None:
                self.odometer += abs(curr_segment.position - self.previous_pos)
            self.previous_pos = curr_segment.position
            motor_current = curr_segment.acceleration * self.current_per_acceleration
            # Motor torque in output form: an integer in N-m/1e6
            motor_torque_scaled = int(
                curr_segment.acceleration * self.torque_per_acceleration * 1e6
            )
            curr_pos_counts = self.encoder_resolution * curr_pos
            cmd_target = self.rotator.target.at(curr_tai)
            in_position = False
            self.telemetry.biss_motor_encoder_axis_a = int(curr_pos_counts)
            self.telemetry.biss_motor_encoder_axis_b = int(curr_pos_counts)
            self.telemetry.status_word_drive0 = 0
            self.telemetry.status_word_drive0_axis_b = 0
            self.telemetry.latching_fault_status_register = 0
            self.telemetry.latching_fault_status_register_axis_b = 0
            self.telemetry.input_pin_states = 0
            self.telemetry.copley_fault_status_register = (0, 0)
            self.telemetry.application_status = ApplicationStatus.DDS_COMMAND_SOURCE
            self.telemetry.demand_pos = curr_segment.position
            self.telemetry.demand_vel = curr_segment.velocity
            self.telemetry.demand_accel = curr_segment.acceleration
            self.telemetry.current_pos = curr_pos
            self.telemetry.current_vel_ch_a_fb = curr_segment.velocity
            self.telemetry.current_vel_ch_b_fb = curr_segment.velocity
            self.telemetry.motor_current[:] = [motor_current] * 2
            self.telemetry.bus_voltage = 20  # arbitrary
            self.telemetry.demand_motor_current_axis_a = motor_current
            self.telemetry.demand_motor_current_axis_b = motor_current
            self.telemetry.motor_torque_axis_a = motor_torque_scaled
            self.telemetry.motor_torque_axis_b = motor_torque_scaled
            self.telemetry.rotator_odometer = self.odometer
            if self.telemetry.state == ControllerState.ENABLED:
                # Use config parameter `track_success_pos_threshold` to
                # compute `in_position`, instead of `self.rotator.path.kind`,
                # so that tracking success varies with the config parameter.
                in_position = (
                    abs(curr_pos - cmd_target.position)
                    < self.config.track_success_pos_threshold
                )
            else:
                self.telemetry.enabled_substate = EnabledSubstate.STATIONARY
            if (
                self.telemetry.state == ControllerState.ENABLED
                and self.telemetry.enabled_substate
                == EnabledSubstate.SLEWING_OR_TRACKING
            ):
                self.telemetry.flags_slew_complete = (
                    self.track_vel_cmd_seen and in_position
                )
            else:
                self.telemetry.flags_slew_complete = 0
                self.track_vel_cmd_seen = False
            if (
                self.telemetry.state == ControllerState.ENABLED
                and self.telemetry.enabled_substate
                == EnabledSubstate.MOVING_POINT_TO_POINT
                and in_position
            ):
                self.telemetry.flags_pt2pt_move_complete = 1
                self.telemetry.flags_move_success = 1
                self.telemetry.enabled_substate = EnabledSubstate.STATIONARY
            else:
                self.telemetry.flags_pt2pt_move_complete = 0
                self.telemetry.flags_move_success = 0
            self.telemetry.flags_stop_complete = 1
            self.telemetry.flags_following_error = 0.001
            self.telemetry.flags_tracking_success = self.telemetry.flags_slew_complete
            self.telemetry.flags_position_feedback_fault = 0
            self.telemetry.flags_tracking_lost = 0
            self.telemetry.flags_no_new_track_cmd_error = self.tracking_timed_out
            self.telemetry.motor_encoder_ch_a = 0
            self.telemetry.motor_encoder_ch_b = 0
            self.telemetry.rotator_pos_deg = curr_pos

            # Assign the FAULT sub-state
            if self.telemetry.state == ControllerState.STANDBY:
                self.telemetry.fault_substate = FaultSubstate.NO_ERROR
            elif self.telemetry.state == ControllerState.FAULT:
                self.telemetry.fault_substate = FaultSubstate.WAIT_CLEAR_ERROR

        except Exception:
            self.log.exception("update_telemetry failed; output incomplete telemetry")
