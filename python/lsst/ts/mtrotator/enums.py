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

__all__ = ["CommandCode", "SetEnabledSubstateParam"]

import enum


class CommandCode(enum.IntEnum):
    """Values for ``Command.code``.


    In the Moog code these are defined in enum cmdType.
    I have reworded them for clarity.
    """

    ENABLE_DRIVES = 0x7000
    FAULT = 0x8001
    SET_STATE = 0x8000
    SET_ENABLED_SUBSTATE = 0x8002
    POSITION_SET = 0x8007
    SET_CONSTANT_VEL = 0x800B
    CONFIG_VEL = 0x9001
    CONFIG_ACCEL = 0x9002
    CONFIG_ACCEL_EMERGENCY = 0x9003
    CONFIG_JERK_EMERGENCY = 0x9004
    TRACK_VEL_CMD = 0x9031


class SetEnabledSubstateParam(enum.IntEnum):
    """Enabled substate parameters.

    Values for ``Command.param1`` when
    ``Command.code = CommandCode.SET_ENABLED_SUBSTATE``
    and the current state is ``ENABLED``.

    Called ``EnabledSubStateTriggers`` in Moog code.
    """

    ENABLED_INVALID = 0
    MOVE_POINT_TO_POINT = enum.auto()
    TRACK = enum.auto()
    STOP = enum.auto()
    INITIALIZE = enum.auto()
    RELATIVE = enum.auto()
    CONSTANT_VELOCITY = enum.auto()
    SPARE2 = enum.auto()
    MOVE_LUT = enum.auto()
