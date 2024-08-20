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
$id: https://github.com/lsst-ts/ts_mtrotator/blob/main/python/lsst/ts/mtrotator/config_schema.py
title: MTRotator v2
description: Configuration for the MTRotator CSC.
type: object
properties:
  max_ccw_following_error:
    description: >-
      The maximum difference (in degrees) between the camera rotator actual position and the
      camera cable wrap actual position, beyond which the rotator is halted and sent to FAULT state.
    type: number
    exclusiveMinimum: 0
  num_ccw_following_errors:
    description: >-
      The number of sequential camera cable wrap following errors required before halting the rotator.
      A value of 1 will fail at the first error. A value of 3 avoids triggering on an outlier.
      Note that the value is tested each time rotator telemetry arrives.
    type: integer
    exclusiveMinimum: 0
  host:
    description: >-
      IP address of the TCP/IP interface.
    type: string
    format: hostname
  port:
    description: >-
      Telemetry port number of the TCP/IP interface.
      The command port is one larger.
    type: integer
  connection_timeout:
    description: Time limit for connecting to the TCP/IP interface (sec)
    type: number
    exclusiveMinimum: 0
  vibration_detection_period:
    description: >-
      Detection period of the low-frequency vibration in seconds. The data collected in this
      period will be used to do the fast Fourier transform (FFT).
    type: number
    exclusiveMinimum: 0
  vibration_range:
    description: >-
      Detection range [low, high] of the low-frequency vibration in Hz.
    type: array
    minItems: 2
    maxItems: 2
    items:
      type: number
  vibration_max_times:
    description: >-
      If the times of low-frequency vibration exceed this value contiguously, it will
      be identified as a vibration event.
    type: integer
    exclusiveMinimum: 0
  vibration_snr:
    description: >-
      Signal-to-noise ratio (SNR) of the low-frequency vibration (peak in the frequency
      diagram after FFT) to be identified as a vibration.
    type: number
    exclusiveMinimum: 0
  vibration_threshold:
    description: >-
      Threshold to decide the post-processing data of rotator's position can be used
      to do FFT or not (in degrees).
    type: number
    exclusiveMinimum: 0
required: [max_ccw_following_error, num_ccw_following_errors, host, port, connection_timeout,
           vibration_detection_period, vibration_range, vibration_max_times, vibration_snr,
           vibration_threshold]
additionalProperties: false
"""
)
