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
import unittest
import pathlib

from lsst.ts import salobj
from lsst.ts import rotator

TEST_CONFIG_DIR = pathlib.Path(__file__).resolve().parent.joinpath("data", "config")


class Harness:
    def __init__(self, initial_state=salobj.State.STANDBY,
                 config_dir=None,
                 initial_simulation_mode=0):
        salobj.test_utils.set_random_lsst_dds_domain()

        self.csc = rotator.RotatorCSC(config_dir=config_dir,
                                      initial_state=initial_state,
                                      initial_simulation_mode=initial_simulation_mode)

        self.remote = salobj.Remote(self.csc.domain, "Rotator")\


    async def __aenter__(self):
        await asyncio.gather(self.csc.start_task, self.remote.start_task)
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await asyncio.gather(self.csc.close(), self.remote.close())


class TestRotatorCSC(unittest.TestCase):

    def test_basic_state_transitions(self):
        async def doit():
            async with Harness(config_dir=TEST_CONFIG_DIR) as harness:

                evt_timeout = 5.
                state = await harness.remote.evt_summaryState.next(flush=False, timeout=evt_timeout)
                self.assertEqual(salobj.State(state.summaryState), salobj.State.STANDBY)

        asyncio.get_event_loop().run_until_complete(doit())


if __name__ == "__main__":
    unittest.main()
