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

import numpy as np
import numpy.typing
from lsst.ts.mtrotator import VibrationDetector


class VibrationDetectorTestCase(unittest.TestCase):
    """Test the vibration detector class."""

    def setUp(self) -> None:

        self.detector = VibrationDetector(
            15.0,
            4,
            [0.3, 0.7],
            10.0,
            0.001,
        )

    def test_put_data(self) -> None:

        # First data
        self.assertFalse(self.detector.put_data(1.0, 0.3))
        self.assertEqual(self.detector._queue.qsize(), 1)

        data = self.detector._queue.get_nowait()
        self.assertEqual(data, 0.7)

        # Put until full
        for _ in range(self.detector._queue.maxsize):
            is_full = self.detector.put_data(1.0, 0.3)

        self.assertTrue(is_full)

        # Put one more but no change actually
        self.assertTrue(self.detector.put_data(1.0, 0.3))
        self.assertEqual(self.detector._queue.qsize(), self.detector._queue.maxsize)

    def test_clear_vibration_frequencies(self) -> None:

        self.detector._current_times = 1
        self.detector._vibration_frequencies.add(0.1)

        self.detector._clear_vibration_frequencies()

        self.assertEqual(self.detector._current_times, 0)
        self.assertEqual(len(self.detector._vibration_frequencies), 0)

    def test_evaluate_vibration_frequency(self) -> None:

        # No vibration
        data_no_vibration = np.zeros(self.detector._queue.maxsize)

        self.assertEqual(
            self.detector._evaluate_vibration_frequency(data_no_vibration), 0.0
        )

        # Vibration
        amplitude = 0.0001
        frequency = 0.4
        data_vibration = self._prepare_vibration_data(amplitude, frequency)[0]

        self.assertEqual(
            self.detector._evaluate_vibration_frequency(data_vibration), frequency
        )

    def _prepare_vibration_data(
        self, amplitude: float, frequency: float
    ) -> tuple[numpy.typing.NDArray[np.float64], numpy.typing.NDArray[np.float64]]:

        times = np.linspace(0.0, 15.0, num=self.detector._queue.maxsize, endpoint=False)
        data = amplitude * np.sin(2.0 * np.pi * frequency * times)

        return data, times

    def test_check_vibration_frequency(self) -> None:

        # Prepare the data
        amplitude = 0.0001
        frequency = 0.4
        data, times = self._prepare_vibration_data(amplitude, frequency)

        slope = 1.45
        noise = np.random.normal(loc=0.0, scale=amplitude * 0.01, size=len(data))
        data += slope * times + noise

        # Put the first data
        offset = 1.2
        self.detector.put_data(data[0] + offset, offset)

        self.assertEqual(len(self.detector.check_vibration_frequency()), 0)

        # Put the left data as the first run
        for idx in range(1, len(data)):
            self.detector.put_data(data[idx] + offset, offset)

        self.assertEqual(len(self.detector.check_vibration_frequency()), 0)
        self.assertEqual(self.detector._current_times, 1)
        self.assertEqual(len(self.detector._vibration_frequencies), 1)

        # For the second run
        for data_point in data:
            self.detector.put_data(data_point + offset, offset)

        self.assertEqual(len(self.detector.check_vibration_frequency()), 0)
        self.assertEqual(self.detector._current_times, 2)
        self.assertEqual(len(self.detector._vibration_frequencies), 1)

        # Run to be the maximum times
        for _ in range(2):
            for data_point in data:
                self.detector.put_data(data_point + offset, offset)

            vibration_frequencies = self.detector.check_vibration_frequency()

        self.assertEqual(len(vibration_frequencies), 1)
        self.assertEqual(vibration_frequencies.pop(), frequency)

        self.assertEqual(self.detector._current_times, 0)
        self.assertEqual(len(self.detector._vibration_frequencies), 0)


if __name__ == "__main__":
    unittest.main()
