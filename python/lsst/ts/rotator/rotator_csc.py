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

import asyncio
import enum
import pathlib

from lsst.ts import salobj
from . import constants
from . import stru
cts
from . import server

# Time limit for controller to connect to the CSC (sec).
CONNECT_TIMEOUT = 10

# Time limit to close the server and telemetry sockets (sec).
DISCONNECT_TIMEOUT = 5


class ServersState(enum.Enum):
    OFF = 0
    STARTING = 1
    RUNNING = 2


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

    @staticmethod
    def get_config_pkg():
        return "ts_config_mttcs"

    async def configure(self, config):
        self.config = config

    async def handle_summary_state(self):
        if self.disabled_or_enabled:
            if self.server is None:
                self.server = server.Server()
                await self.server.start_task
        else:
            if self.server is not None:
                try:
                    await self.server.close()
                finally:
                    self.server = None

    async def do_configureAcceleration(self, data):
        """Execute the configureAcceleration command."""
        raise NotImplementedError()

    async def do_configureVelocity(self, data):
        """Execute the configureVelocity command"""
        raise NotImplementedError()

    async def do_move(self, data):
        """Execute the move command."""
        raise NotImplementedError()

    async def do_track(self, data):
        """Execute the track command."""
        raise NotImplementedError()

    async def do_test(self, data):
        """Execute the test command."""
        raise NotImplementedError()

    async def do_trackStart(self, data):
        """Execute the trackStart command."""
        raise NotImplementedError()

    async def do_clearError(self, data):
        """Execute the clearError command."""
        raise NotImplementedError()

    async def do_positionSet(self, data):
        """Execute the positionSet command."""
        raise NotImplementedError()

    async def do_moveConstantVelocity(self, data):
        """Execute the moveConstantVelocity command."""
        raise NotImplementedError()

    async def do_velocitySet(self, data):
        """Execute the velocitySet command."""
        raise NotImplementedError()
