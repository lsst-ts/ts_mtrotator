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

__all__ = ["RotatorCommander", "command_mtrotator"]

import asyncio

from lsst.ts import salobj, simactuators, utils

STD_TIMEOUT = 5  # timeout for command ack

# How far in advance to set the time field of tracking commands (seconds)
TRACK_ADVANCE_TIME = 0.15

# Interval between tracking commands (seconds). 0.05 matches MTPtg.
TRACK_INTERVAL = 0.05


class RotatorCommander(salobj.CscCommander):
    """Command the MTRotator CSC from the command-line.

    This is only intended for engineering.

    Parameters
    ----------
    enable : `bool`
        Enable the CSC when first connecting?
    """

    def __init__(self, enable: bool) -> None:
        self.tracking_task: asyncio.Future = asyncio.Future()
        super().__init__(
            name="MTRotator",
            index=0,
            enable=enable,
        )
        self.help_dict[
            "ramp"
        ] = "start_position end_position speed  # track a path of constant"
        self.help_dict[
            "cosine"
        ] = "center_position, amplitude, max_speed  # track one cycle of a cosine wave"
        for command_to_ignore in ("abort", "setValue"):
            self.command_dict.pop(command_to_ignore, None)

    async def close(self) -> None:
        self.tracking_task.cancel()
        await super().close()

    async def do_ramp(self, args: list[str]) -> None:
        """Track from start_position to end_position at the specified speed."""
        self.tracking_task.cancel()
        kwargs = self.check_arguments(args, "start_position", "end_position", "speed")
        self.tracking_task = asyncio.ensure_future(self._ramp(**kwargs))

    async def do_cosine(self, args: list[str]) -> None:
        """Track along a cosine wave (one full cycle)."""
        self.tracking_task.cancel()
        kwargs = self.check_arguments(args, "center_position", "amplitude", "max_speed")
        self.tracking_task = asyncio.ensure_future(self._cosine(**kwargs))

    def _special_telemetry_callback(
        self,
        data: salobj.BaseMsgType,
        name: str,
        omit_fields: list[str],
        digits: int = 2,
    ) -> None:
        """Callback for telemetry omitting one specified field
        from the comparison, but printing it.

        Parameters
        ----------
        data : `object`
            Telemetry data.
        name : `str`
            Name of telemetry topic, without the `tel_` prefix.
        omit_fields : `list` [`str`]
            All fields to omit from the comparison.
            Note that the CscCommander constructor arguments
            that specify fields to ignore are ignored.
        """
        prev_value_name = f"previous_tel_{name}"
        public_data = self.get_rounded_public_data(data, digits=digits)
        trimmed_data = public_data.copy()
        for omit_field in omit_fields:
            trimmed_data.pop(omit_field)
        if trimmed_data == getattr(self, prev_value_name):
            return
        setattr(self, prev_value_name, trimmed_data)
        formatted_data = self.format_dict(public_data)
        self.output(f"{data.private_sndStamp:0.3f}: {name}: {formatted_data}")

    async def tel_motors_callback(self, data: salobj.BaseMsgType) -> None:
        """Don't print if only the raw or busVoltage fields have changed.

        Parameters
        ----------
        data : `object`
            MTRotator motors telemetry data.
        """
        # Only pay attention to changes in the "raw" and "torque" fields,
        # and round to -5 digits to ignore large jitter in "raw".
        self._special_telemetry_callback(
            data=data, name="motors", omit_fields=["busVoltage", "current"], digits=-5
        )

    async def tel_rotation_callback(self, data: salobj.BaseMsgType) -> None:
        self._special_telemetry_callback(
            data=data, name="rotation", omit_fields=["odometer", "timestamp"], digits=2
        )

    async def _ramp(
        self, start_position: float, end_position: float, speed: float
    ) -> None:
        """Track a linear ramp.

        Parameters
        ----------
        start_position : `float`
            Starting position of ramp (deg).
        end_position : `float`
            Ending position of ramp (deg).
        speed : `float`
            Speed of motion along the ramp (deg/sec).
        """
        try:
            ramp_generator = simactuators.RampGenerator(
                start_positions=[start_position],
                end_positions=[end_position],
                speeds=[speed],
                advance_time=TRACK_ADVANCE_TIME,
            )
            print(
                f"Track a ramp from {start_position} to {end_position} at speed {speed}; "
                f"this will take {ramp_generator.duration:0.2f} seconds"
            )
            await self.remote.cmd_trackStart.start(timeout=STD_TIMEOUT)
            for positions, velocities, tai in ramp_generator():
                t0 = utils.current_tai()
                await self.remote.cmd_track.set_start(
                    angle=positions[0],
                    velocity=velocities[0],
                    tai=tai,
                    timeout=STD_TIMEOUT,
                )
                dt = utils.current_tai() - t0
                sleep_duration = max(0, TRACK_INTERVAL - dt)
                await asyncio.sleep(sleep_duration)
        except asyncio.CancelledError:
            print("Ramp cancelled")
        except Exception as e:
            print(f"Ramp failed: {e!r}")
        finally:
            print("Stop tracking")
            await self.remote.cmd_stop.start(timeout=STD_TIMEOUT)
            print("Ramp finished")

    async def _cosine(
        self, center_position: float, amplitude: float, max_speed: float
    ) -> None:
        """Track one sine wave of specified amplitude and period.

        The range of motion is period - amplitude to period + amplitude,
        plus whatever motion is required to slew to the path.

        Parameters
        ----------
        center_position : `float`
            Midpoint of cosine wave (deg).
        amplitude : `float`
            Amplitude of cosine wave (deg).
        max_speed : `float`
            Maximum speed of motion (deg/sec).
        """
        try:
            settings = self.remote.evt_configuration.get()
            if settings is None:
                raise RuntimeError(
                    "Must wait until configuration seen so we can check max velocity"
                )
            if abs(max_speed) > settings.velocityLimit:
                raise ValueError(
                    f"maximum speed {max_speed} > allowed {settings.velocityLimit}"
                )
            cosine_generator = simactuators.CosineGenerator(
                center_positions=[center_position],
                amplitudes=[amplitude],
                max_speeds=[max_speed],
                advance_time=TRACK_ADVANCE_TIME,
            )
            print(
                f"Tracking one cycle of a cosine wave centered at {center_position} "
                f"with amplitude {amplitude}; "
                f"this will take {cosine_generator.duration:0.2f} seconds"
            )
            await self.remote.cmd_trackStart.start(timeout=STD_TIMEOUT)
            for positions, velocities, tai in cosine_generator():
                t0 = utils.current_tai()
                await self.remote.cmd_track.set_start(
                    angle=positions[0],
                    velocity=velocities[0],
                    tai=tai,
                    timeout=STD_TIMEOUT,
                )
                dt = utils.current_tai() - t0
                sleep_duration = max(0, TRACK_INTERVAL - dt)
                await asyncio.sleep(sleep_duration)
        except asyncio.CancelledError:
            print("sine cancelled")
        except Exception as e:
            print(f"sine failed: {e}")
        finally:
            await self.remote.cmd_stop.start(timeout=STD_TIMEOUT)


def command_mtrotator() -> None:
    """Run the MTRotator commander."""
    asyncio.run(RotatorCommander.amain(index=None))
