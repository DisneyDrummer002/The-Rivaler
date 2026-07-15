# Safety Requirements

The remote's emergency-stop button on `D12` sends a wireless software stop.
It stops drive motion and disarms the launcher when the GIGA receives it, but
it is not a substitute for a physical emergency stop.

Install a separate normally closed, latching emergency-stop circuit that
removes power from the drive and launcher motor supplies. The circuit must
remain effective if any controller, radio link, or firmware component fails.

The launcher magazine uses positional commands across its configured
-180-degree through 180-degree range. It homes to 0 degrees at startup and
advances through the configured shooting positions. Verify its mechanical
alignment and full travel with launcher motor power disconnected before using
the firing controls.
