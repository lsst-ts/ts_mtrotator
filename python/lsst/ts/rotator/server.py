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

__all__ = ["Server"]

import asyncio

from . import constants
from . import structs

# Time limit for controller to connect to the CSC (sec).
CONNECT_TIMEOUT = 10

# Time limit to close the server and telemetry sockets (sec).
DISCONNECT_TIMEOUT = 5


class Server:
    """Serve command and telemetry ports for the cRIO to connect to.
    """
    def __init__(self, config_callback, telemetry_callback):
        self.config_callback = config_callback
        self.telemetry_callback = telemetry_callback

        self.command_writer_task = asyncio.Future()
        self.telemetry_reader_task = asyncio.Future()
        self.command_writer = None
        self.command_server = None
        self.telemetry_server = None
        self.start_task = asyncio.create_task(self.start())

    async def set_command_writer(self, reader, writer):
        """Send commands to the rotator controller.
        """
        if self.command_writer is not None:
            raise RuntimeError("Cannot write commands to more than one client")
        self.command_writer = writer
        self.command_writer_task.set_result(None)

    async def read_telemetry_and_config(self, reader, writer):
        """Read telemetry and configuration from the rotator controller.
        """
        if self._read_tel_running:
            raise RuntimeError("Cannot have two read_telemetry_and_config loops running")
        self._read_tel_running = True
        self.telemetry_reader_task.set_result(None)
        try:
            while True:
                header = await reader.readinto(structs.TelemetryHeader)
                if header.frame_id == structs.FrameId.CONFIG:
                    config = await reader.read_into(structs.Config)
                    try:
                        self.config_callback(config)
                    except Exception:
                        self.log.exception("config_callback failed")
                elif header.frame_id == structs.FrameId.TELEMETRY:
                    telemetry = await reader.read_into(structs.Telemetry)
                    try:
                        self.telemetry_callback(telemetry)
                    except Exception:
                        self.log.exception("telemetry_callback failed")
                else:
                    # unrecognized frame ID; we have no way to resynchronize
                    # so give up
                    self.fault(1, f"Invalid data read on the telemetry socket; frame_id={header.frame_id}")
        finally:
            self._read_tel_running = False

    async def write_command(self, cmd):
        if not self.start_task.done():
            raise RuntimeError("Server not ready.")
        if not isinstance(cmd, structs.Command):
            raise ValueError(f"cmd={cmd!r} must be an instance of structs.Command")
        self.command_writer.write(cmd)
        await self.command_writer.drain()

    async def start(self):
        """Start command and telemetry TCP/IP servers and wait for
        the controller to connect.
        """
        # Start the command and telemetry servers.
        host = None if self.simulation_mode == 0 else "127.0.0.1"
        self.command_server = await asyncio.start_server(self.set_command_writer, host=host,
                                                         port=constants.CMD_SERVER_PORT)
        self.telemetry_server = await asyncio.start_server(self.read_telemetry_and_config, host=host,
                                                           port=constants.TEL_SERVER_PORT)
        # Wait for the controller to connect to the command and telemetry
        # servers.
        await asyncio.wait_for(asyncio.gather(self.command_writer_task, self.telemetry_reader_task),
                               timeout=CONNECT_TIMEOUT)

    async def close(self):
        """Stop the command and telemetry servers.

        Raises
        ------
        asyncio.TimeoutError
            If server.wait_closed() takes longer than DISCONNECT_TIMEOUT.
        """
        self.command_writer = None
        servers_to_close = []
        if self.command_server is not None:
            servers_to_close.append(self.command_server)
            self.command_server = None
        if self.telemetry_server is not None:
            servers_to_close.append(self.telemetry_server)
            self.telemetry_server = None

        for server in servers_to_close:
            server.close()
        await asyncio.wait_for(asyncio.gather(*[server.wait_closed() for server in servers_to_close]),
                               timeout=DISCONNECT_TIMEOUT)
