# Line Following

The GIGA implements line following as a separate drive mode. Pressing the
remote joystick button toggles the mode. A deliberate joystick movement,
remote timeout, emergency stop, wall warning, or missing-floor warning
cancels it immediately.

The controller independently normalizes the two analog sensors, applies
filtered proportional and derivative steering, slows for larger errors, and
keeps corrections within configured drive limits. If the line disappears, it
briefly continues at reduced speed, then pivots only toward the last observed
line direction. It stops if the line is not reacquired before the search
timeout.

## Sensor Placement

Place the two sensors close enough that the centered line produces a useful
signal on at least one sensor, preferably both. With only two sensors, a line
that passes invisibly through a wide gap between them cannot be distinguished
from a completely lost line.

## Calibration

Keep `kLineSensorCalibrationConfigured` set to `false` until calibration is
complete. The firmware will reject line-following activation in this state.

1. Disconnect motor power or raise the wheels clear of the work surface.
2. Set `kLineSensorCalibrationLoggingEnabled` to `true` and upload the GIGA
   firmware.
3. Hold both sensors over the normal floor. After the filtered readings settle,
   record several `Line sensors raw` values and average each sensor.
4. Repeat with both sensors over the line.
5. Put the four averages into the floor/line reading constants in the GIGA
   configuration. The line may read higher or lower than the floor.
6. Set `kLineSensorCalibrationConfigured` to `true` and calibration logging
   back to `false`.
7. With the robot supported and launcher power disconnected, verify that the
   Line Detected LED responds to the line before conducting a low-speed floor
   test.

The configured floor-to-line span must be at least
`kLineSensorMinimumCalibrationSpan` for each sensor. After basic operation is
verified, steering gains, cruise speed, loss timing, and detection strength can
be tuned in `Config.h` without changing the controller logic.
