# This file is part of ts_rotator.
#
# Developed for the LSST Data Management System.
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

# import asyncio
import pathlib

from lsst.ts import salobj
from lsst.ts import pymoog
from . import constants
from . import enums
from . import structs


class RotatorCSC(salobj.ConfigurableCsc):
    """MT rotator CSC.

    Notes
    -----
    **Error Codes**

    * 1: invalid data read on the telemetry socket
    """

    def __init__(self, config_dir=None, initial_state=salobj.State.STANDBY, initial_simulation_mode=0):
        schema_path = pathlib.Path(__file__).resolve().parents[4].joinpath("schema", "rotator.yaml")
        super().__init__("Rotator", index=0, schema_path=schema_path, config_dir=config_dir,
                         initial_state=initial_state, initial_simulation_mode=initial_simulation_mode)
        self.config = None
        self.server = None
        self.commands = dict()
        for cmd in enums.CommandCode:
            command = structs.Command()
            command.cmd = cmd
            command.sync_pattern = structs.ROTATOR_SYNC_PATTERN
            self.commands[cmd] = command

    @staticmethod
    def get_config_pkg():
        return "ts_config_mttcs"

    async def configure(self, config):
        self.config = config

    async def run_command(self, cmd, **kwargs):
        command = self.commands[cmd]
        for name, value in kwargs:
            if not hasattr(command, name):
                raise ValueError(f"Unknown command argument {name}")
                setattr(command, name, value)
        # Note: increment  correctly wraps around
        command.counter += 1
        self.server.run_command(command)

    async def handle_summary_state(self):
        if self.disabled_or_enabled:
            if self.server is None:
                host = None if self.initial_simulation_mode == 0 else pymoog.LOCAL_HOST
                self.server = pymoog.Server(host=host,
                                            log=self.log,
                                            ConfigClass=structs.Config,
                                            TelemetryClass=structs.Telemetry,
                                            config_callback=self.config_callback,
                                            telemetry_callback=self.telemetry_callback)
                await self.server.start_task
        else:
            if self.server is not None:
                try:
                    await self.server.close()
                finally:
                    self.server = None

    # standard commnands
    async def do_enable(self, data):
        await super().do_enable(data)
        await self.run_command(cmd=enums.CommandCode.SET_STATE,
                               param1=enums.State.ENABLED)

    async def do_disable(self, data):
        await super().do_disable(data)
        await self.run_command(cmd=enums.CommandCode.SET_STATE,
                               param1=enums.State.DISABLED)

    async def do_standby(self, data):
        if self.summary_state not in (salobj.State.DISABLED, salobj.State.FAULT):
            raise salobj.ExpectedError("CSC must be in DISABLED or FAULT state to go to STANDBY")
        if self.telemetry.state == enums.State.OFFLINE:
            if self.telemetry.substate == enums.OfflineSubState.AVAILABLE:
                await self.run_command(cmd=enums.CommandCode.SET_STATE,
                                       param1=enums.State.ENTER_CONTROL)
            else:
                raise salobj.ExpectedError("Low-level controller is OFFLINE and not AVAILABLE; "
                                           "use the engineering interface to make it AVAILABLE.")
        elif self.telemetry.state == enums.State.FAULT:
            raise salobj.ExpectedError("Low-level controller is in FAULT state; "
                                       "Use clearError or the engineering interface to reset to OFFLINE, "
                                       "then use the engineering interface to make it AVAILABLE.")
        elif self.telemetry.state == enums.State.DISABLED:
            await self.run_command(cmd=enums.CommandCode.SET_STATE,
                                   param1=enums.State.STANDBY)
        await super().do_standby(data)

    async def do_start(self, data):
        """Execute the start command."""
        if self.summary_state != salobj.State.STANDBY:
            raise salobj.ExpectedError(f"CSC is in {self.summary_state}; must be STANDBY state to start")
        if self.telemetry.state != enums.State.STANDBY:
            raise salobj.ExpectedError(f"Low-level controller in {self.telemetry.state}; "
                                       "must be in STANDBY to start")
        super().do_start(data)
        await self.run_command(cmd=enums.CommandCode.SET_STATE,
                               param1=structs.StateTriggers.START)

    async def do_clearError(self, data):
        """Reset the FAULT state.

        Unfortunately this leaves the low-level controller in
        OFFLINE/PUBLISH_ONLY state. You need the engineering interface to
        make it OFFLINE/AVAILABLE before the CSC can control it.
        """
        # Two sequential commands are needed to clear error
        await self.run_command(cmd=enums.CommandCode.SET_STATE,
                               param1=enums.State.CLEAR_ERROR)
        await self.sleep(0.9)
        await self.run_command(cmd=enums.CommandCode.SET_STATE,
                               param1=enums.State.CLEAR_ERROR)

    async def do_configureAcceleration(self, data):
        """Specify the acceleration limit."""
        self.assert_enabled_substate(enums.EnabledSubstate.STATIONARY)
        if not 0 < data.alimit <= constants.MAX_VEL_LIMIT:
            raise salobj.ExpectedError(f"alimit={data.alimit} must be > 0 and <= {constants.MAX_ACCEL_LIMIT}")
        await self.run_command(cmd=enums.CommandCode.CONFIG_ACCEL,
                               param1=data.alimit)

    async def do_configureVelocity(self, data):
        """Specify the velocity limit."""
        self.assert_enabled_substate(enums.EnabledSubstate.STATIONARY)
        if not 0 < data.vlimit <= constants.MAX_VEL_LIMIT:
            raise salobj.ExpectedError(f"vlimit={data.vlimit} must be > 0 and <= {constants.MAX_VEL_LIMIT}")
        await self.run_command(cmd=enums.CommandCode.CONFIG_VEL,
                               param1=data.vlimit)

    async def do_move(self, data):
        """Go to the position specified by the most recent ``positionSet``
        command.
        """
        self.assert_enabled_substate(enums.EnabledSubstate.STATIONARY)
        await self.run_command(cmd=enums.CommandCode.SET_SUBSTATE,
                               param1=enums.EnabledSubstate.MOVE_POINT_TO_POINT)

    async def do_moveConstantVelocity(self, data):
        """Move at the speed and for the duration specified by the most recent
        ``velocitySet`` command.
        """
        self.assert_enabled_substate(enums.EnabledSubstate.STATIONARY)
        await self.run_command(cmd=enums.CommandCode.SET_SUBSTATE,
                               param1=enums.EnabledSubstate.CONST_VEL)

    async def do_positionSet(self, data):
        """Specify a position for the ``move`` command.
        """
        self.assert_enabled_substate(enums.EnabledSubstate.STATIONARY)
        if not self.config.lower_pos_limit <= data.angle <= self.config.upper_pos_limit:
            raise salobj.ExpectedError(f"angle {data.angle} not in range "
                                       f"[{self.config.lower_pos_limit}, {self.config.upper_pos_limit}]")
        await self.run_command(cmd=enums.CommandCode.POSITION_SET,
                               param1=data.angle)

    async def do_stop(self, data):
        """Halt tracking or any other motion.
        """
        self.assert_enabled("stop")
        await self.run_command(cmd=enums.CommandCode.SET_SUBSTATE,
                               param1=enums.EnabledSubstate.STOP)

    async def do_test(self, data):
        """Execute the test command.

        This may issue the Initialization command, which measures
        backlash.
        """
        self.assert_enabled_substate(enums.EnabledSubstate.STATIONARY)
        # The test command is unique in that all fields must be left
        # at their initialized value except sync_pattern
        # (at least that is what the Vendor's code does).
        command = structs.Command()
        command.sync_pattern = structs.ROTATOR_SYNC_PATTERN
        await self.server.run_command(command)

    async def do_track(self, data):
        """Specify a position, velocity, TAI time tracking update.
        """
        self.assert_enabled_substate(enums.EnabledSubstate.TRACK)
        if abs(data.velocity) > self.config.velocity_limit:
            raise salobj.ExpectedError(f"Velocity {data.velocity} > limit {self.config.velocity_limit}")
        dt = salobj.current_tai() - data.tai
        curr_pos = data.angle + data.vel*dt
        if not self.config.lower_pos_limit <= curr_pos <= self.config.upper_pos_limit:
            raise salobj.ExpectedError(f"current position {curr_pos} not in range "
                                       f"[{self.config.lower_pos_limit}, {self.config.upper_pos_limit}]")
        await self.run_command(cmd=enums.CommandCode.TRACK_VEL_CMD,
                               param1=data.tai,
                               param2=data.angle,
                               param3=data.velocity)

    async def do_trackStart(self, data):
        """Start tracking.

        Once this is run you must issue ``track`` commands at 10-20Hz
        until you are done tracking, then issue the ``stop`` command.
        """
        self.assert_enabled_substate(enums.EnabledSubstate.STATIONARY)
        await self.run_command(cmd=enums.CommandCode.SET_SUBSTATE,
                               param1=enums.EnabledSubstate.TRACK)

    async def do_velocitySet(self, data):
        """Specify the velocity and duration for the ``moveConstantVelocity``
        command.
        """
        self.assert_enabled_substate(enums.EnabledSubstate.STATIONARY)
        if abs(data.velocity) > self.config.velocity_limit:
            raise salobj.ExpectedError(f"Velocity {data.velocity} > limit {self.config.velocity_limit}")
        await self.run_command(cmd=enums.CommandCode.SET_CONSTANT_VEL,
                               param1=data.velocity,
                               param2=data.moveDuration)

    def assert_enabled_substate(self, substate):
        """Assert the controller is enabled and in the specified substate.
        """
        if self.summary_state != salobj.State.ENABLED:
            raise salobj.ExpectedError("CSC not enabled")
        if self.telemetry.state != enums.State.Enabled:
            raise salobj.ExpectedError("Low-level controller not enabled")
        if self.telemetry.enabled_substate != substate:
            raise salobj.ExpectedError(f"Low-level controller in substate {self.telemetry.enabled_substate} "
                                       f"instead of {substate}")

    def configure_callback(self, config):
        """Called when the TCP/IP controller outputs configuration.

        Parameters
        ----------
        config : `Config`
            Configuration reported by TCP/IP controller.
        """
        pass

    def telemetry_callback(self, telemetry):
        """Called when the TCP/IP controller outputs telemetry.

        Parameters
        ----------
        telemetry : `Telemetry`
            Telemetry reported by TCP/IP controller.
        """
        pass
