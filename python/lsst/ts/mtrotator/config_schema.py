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

__all__ = ["CONFIG_SCHEMA"]

import yaml

CONFIG_SCHEMA = yaml.safe_load(
    """
$schema: http://json-schema.org/draft-07/schema#
$id: https://github.com/lsst-ts/ts_mtrotator/blob/master/python/lsst/ts/mtrotator/config_schema.py
title: MTRotator v1
description: Configuration for the MTRotator CSC.
type: object
properties:
  max_ccw_following_error:
    description: >-
      The maximum difference (in degrees) between the camera rotator actual position and the
      camera cable wrap actual position, beyond which the rotator is halted and sent to FAULT state.
    type: number
    exclusiveMinimum: 0
    default: 2.2
  num_ccw_following_errors:
    description: >-
      The number of sequential camera cable wrap following errors required before halting the rotator.
      A value of 1 will fail at the first error. A value of 3 avoids triggering on an outlier.
      Note that the value is tested each time rotator telemetry arrives.
    type: integer
    exclusiveMinimum: 0
    default: 3
required: [max_ccw_following_error, num_ccw_following_errors]
additionalProperties: false
"""
)
