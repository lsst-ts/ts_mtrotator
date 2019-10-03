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
import logging
import unittest

import asynctest

from lsst.ts import pymoog
from lsst.ts import rotator

# Standard timeout (seconds)
STD_TIMEOUT = 1


class Harness:
    def __init__(self):
        self.config_list = []
        self.telemetry_list = []
        self.config_future = asyncio.Future()
        self.telemetry_future = asyncio.Future()

        log = logging.getLogger()
        log.setLevel(logging.INFO)
        log.addHandler(logging.StreamHandler())
        self.server = pymoog.Server(host=pymoog.LOCAL_HOST,
                                    log=log,
                                    ConfigClass=rotator.Config,
                                    TelemetryClass=rotator.Telemetry,
                                    config_callback=self.config_callback,
                                    telemetry_callback=self.telemetry_callback)
        self.mock_ctrl = rotator.MockMTRotatorController(log=log)

    def config_callback(self, config):
        self.config_list.append(config)
        self.config_future.set_result(None)

    def telemetry_callback(self, telemetry):
        self.telemetry_list.append(telemetry)
        self.telemetry_future.set_result(None)

    async def __aenter__(self):
        await asyncio.gather(self.server.start_task, self.mock_ctrl.connect_task)
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await asyncio.gather(self.mock_ctrl.close(), self.server.close())


class ServerTestCase(asynctest.TestCase):
    """Test the Server by connecting it to the MockMTRotatorController.
    """
    async def test_basic_connection(self):
        async with Harness() as harness:
            await asyncio.wait_for(harness.telemetry_future, timeout=STD_TIMEOUT)
            self.assertEqual(len(harness.config_list), 1)
            self.assertEqual(len(harness.telemetry_list), 1)


if __name__ == "__main__":
    unittest.main()
