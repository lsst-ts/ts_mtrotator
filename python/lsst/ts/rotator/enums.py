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

__all__ = ["CommandCode", "State", "EnabledSubstate", "OfflineSubState"]

import enum


class CommandCode(enum.IntEnum):
    """Value of ``command.cmd``.
    """
    SET_STATE = 0x8000
    SET_SUBSTATE = 0x8002
    POSITION_SET = 0x8007
    SET_CONSTANT_VEL = 0x800B
    CONFIG_VEL = 0x9001
    CONFIG_ACCEL = 0x9002
    TRACK_VEL_CMD = 0x9031


class State(enum.IntEnum):
    """Values for ``command.param1`` when command.cmd = ``STATE_TRIGGER``.
    """
    INVALID = 0
    START = enum.auto()
    ENABLED = enum.auto()
    STANDBY = enum.auto()
    DISABLED = enum.auto()
    EXIT = enum.auto()
    CLEAR_ERROR = enum.auto()
    ENTER_CONTROL = enum.auto()


class EnabledSubstate(enum.IntEnum):
    """Substates for the ENABLED state.
    """
    ENABLED_INVALID = 0
    MOVE_POINT_TO_POINT = enum.auto()
    TRACK = enum.auto()
    STOP = enum.auto()
    INITIALIZE = enum.auto()
    RELATIVE = enum.auto()
    CONST_VEL = enum.auto()
    SPARE2 = enum.auto()
    MOVE_LUT = enum.auto()


class OfflineSubState(enum.IntEnum):
    """Substates for the OFFLINE state.
    """
    PUBLISH_ONLY = 0
    AVAILABLE = enum.auto()
