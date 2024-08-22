# This file is part of ts_mtrotator.
#
# Developed for the LSST Telescope and Site Systems.
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

import unittest

import jsonschema
from lsst.ts import mtrotator, salobj


class ValidationTestCase(unittest.TestCase):
    """Test validation of the config schema."""

    def setUp(self) -> None:
        self.schema = mtrotator.CONFIG_SCHEMA
        self.validator = salobj.StandardValidator(schema=self.schema)
        self.default = dict(
            max_ccw_following_error=2.2,
            num_ccw_following_errors=3,
            host="rot-pxi-controller.cp.lsst.org",
            port=5570,
            connection_timeout=10,
            vibration_detection_period=15.0,
            vibration_range=[0.3, 0.7],
            vibration_max_times=4,
            vibration_snr=10.0,
            vibration_threshold=0.001,
        )

    def test_basics(self) -> None:
        config = dict(
            max_ccw_following_error=1.5,
            num_ccw_following_errors=1,
            host="foo.bar",
            port=25,
            connection_timeout=0.5,
            vibration_detection_period=15.0,
            vibration_range=[0.3, 0.7],
            vibration_max_times=4,
            vibration_snr=10.0,
            vibration_threshold=0.001,
        )
        self.validator.validate(config)

    def test_invalid_configs(self) -> None:
        for name, badval in (
            ("max_ccw_following_error", "oops"),  # Wrong type
            ("max_ccw_following_error", 0),  # Not positive
            ("num_ccw_following_error", "oops"),  # Wrong type
            ("num_ccw_following_error", 0),  # Not positive
        ):
            bad_data = {name: badval}
            with self.subTest(bad_data=bad_data):
                with self.assertRaises(jsonschema.exceptions.ValidationError):
                    self.validator.validate(bad_data)


if __name__ == "__main__":
    unittest.main()
