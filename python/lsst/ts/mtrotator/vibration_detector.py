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

__all__ = ["VibrationDetector"]

import asyncio

import numpy as np
import numpy.typing
from scipy.fft import fft, fftfreq
from scipy.signal import detrend


class VibrationDetector:
    """Vibration detector to monitor the low-frequency vibration.

    Parameters
    ----------
    period : `float`
        Detection period of the low-frequency vibration in seconds. The data
        collected in this period will be used to do the fast Fourier transform
        (FFT).
    max_times : `int`
        If the times of low-frequency vibration exceed this value contiguously,
        it will be identified as a vibration event.
    frequency_range : `list` [`float`]
        Frequency range [low, high] to detect in Hz.
    snr : `float`
        Signal-to-noise ratio (SNR) of the low-frequency vibration (peak in the
        frequency diagram after FFT) to be identified as a vibration.
    threshold : `float`
        Threshold to decide the post-processing data of rotator's position can
        be used to do FFT or not (in degrees).
    dt : `float`, optional
        Delta time between the data points in seconds. (the default is 0.05)
    """

    def __init__(
        self,
        period: float,
        max_times: int,
        frequency_range: list[float],
        snr: float,
        threshold: float,
        dt: float = 0.05,
    ) -> None:
        # Queue of the data.
        self._queue: asyncio.Queue = asyncio.Queue(maxsize=int(period / dt))

        self._max_times = max_times

        # Current times to find the continuous vibration.
        self._current_times = 0

        # Set of the vibration frequencies in Hz.
        self._vibration_frequencies: set[float] = set()

        self._frequency_range = frequency_range
        self._snr = snr
        self._threshold = threshold
        self._dt = dt

    def put_data(self, data: float, reference: float) -> bool:
        """Put the data to the queue.

        Parameters
        ----------
        data : `float`
            Data.
        reference : `float`
            Reference of the data.

        Returns
        -------
        `bool`
            If the queue is full, return True. Otherwise, return False.
        """

        if not self._queue.full():
            self._queue.put_nowait(data - reference)

        return self._queue.full()

    def check_vibration_frequency(self) -> set:
        """Check the vibration frequency in the detection range.

        Returns
        -------
        `set`
            Frequencies of the detected vibration.
        """

        # There is no enough data to do the FFT yet
        if not self._queue.full():
            return set()

        # Get all data
        data = list()
        for _ in range(self._queue.maxsize):
            data.append(self._queue.get_nowait())

        # Do the detrend and judge the data is available or not
        data_detrend = detrend(data, type="linear")
        is_data_detrend_ok = np.all(np.abs(data_detrend) <= self._threshold)

        # Data is not good. Re-detect the vibration.
        if not is_data_detrend_ok:
            self._clear_vibration_frequencies()
            return set()

        # Do the FFT to check the frequency
        frequency = round(self._evaluate_vibration_frequency(data_detrend), ndigits=2)
        if frequency == 0.0:
            # Nothing. Re-detect.
            self._clear_vibration_frequencies()
            return set()
        else:
            # Accumulate the vibration frequency
            self._current_times += 1
            self._vibration_frequencies.add(frequency)

        # If the times of the vibration exceed the maximum times, return the
        # vibration frequencies.
        if self._current_times >= self._max_times:
            vibration_frequencies = self._vibration_frequencies.copy()

            # Clear the internal states and redo the detection.
            self._clear_vibration_frequencies()

            return vibration_frequencies
        else:
            return set()

    def _clear_vibration_frequencies(self) -> None:
        """Clear the vibration frequencies and reset the current times to 0."""

        self._current_times = 0
        self._vibration_frequencies.clear()

    def _evaluate_vibration_frequency(self, data: numpy.typing.NDArray[np.float64]) -> float:
        """Evaluate the vibration frequency in the detection range.

        Parameters
        ----------
        data : `numpy.ndarray` [`float`]
            Data.

        Returns
        -------
        `float`
            If no low-frequency vibration is detected, return 0.0. Otherwise,
            return the maximum vibration frequency in the detection range.
        """

        data_fft, freq_fft = self._calculate_fft(data)
        data_positive = data_fft[0 : len(data) // 2]

        idx_low = (np.abs(freq_fft - self._frequency_range[0])).argmin()
        idx_high = (np.abs(freq_fft - self._frequency_range[1])).argmin()

        data_in_range = data_positive[idx_low:idx_high]
        data_out_range = np.append(data_positive[0:idx_low], data_positive[idx_high:])
        if np.all(data_in_range <= np.mean(data_out_range) * self._snr):
            return 0.0
        else:
            return freq_fft[np.argmax(data_in_range) + idx_low]

    def _calculate_fft(
        self, data: numpy.typing.NDArray[np.float64]
    ) -> tuple[numpy.typing.NDArray[np.float64], numpy.typing.NDArray[np.float64]]:
        """Calculate the fast Fourier transform (FFT).

        Parameters
        ----------
        data : `numpy.ndarray` [`float`]
            Data.

        Returns
        -------
        `numpy.ndarray` [`float`]
            FFT data.
        frequency_fft : `numpy.ndarray` [`float`]
            Frequency in FFT data (in Hz).
        """

        data_fft = fft(data)

        num = len(data)
        frequency_fft = fftfreq(num, self._dt)[: num // 2]

        return np.abs(data_fft), frequency_fft
