# Firmware

Firmware for the BatteryCharger_24V controller.

## Development Environment

The firmware is developed for the ATmega328P using the
Arduino / VisualMicro development environment.

## Main Firmware

- BatteryCharger_24V.ino

The charging parameters, source limits and hardware limits are defined
in parameter blocks at the beginning of the source code.

No EEPROM-based configuration menu is used. The charger is intended
for fixed installation, and charging parameters can be adapted in the
firmware for different battery types and applications.

The firmware is not tied to a specific battery chemistry.

## Charging Control

The charger uses the following basic charging sequence:

- Bulk charging
- Absorption charging
- Full detection
- Optional float charging

If float charging is disabled, the charger disconnects after a full
charge and observes the battery voltage after a rest period. A new
charging cycle starts when the configured restart voltage is reached.

Minimum and maximum absorption times are used to provide defined
charge-cycle behavior even when external loads influence the measured
battery current.

## Power Source Management

Three DC power inputs are supported:

- LOW_POWER
- MID_POWER
- HIGH_POWER

Each input has a configurable current limit.

The charger automatically selects the available source according to
the configured source priority.

The source current limit, battery current limit, DC/DC power limit and
thermal power limit are evaluated independently. The lowest applicable
limit determines the permitted charging power.

## DC/DC Protection

Before the battery is connected to the DC/DC converter, the firmware
performs a pre-charge verification of the converter output.

The DC/DC output is first matched to the measured battery voltage and
then changed by a small defined amount. The measured response is used
to verify that the converter output can be controlled correctly before
the battery connection is enabled.

Reverse current is treated as an operating condition rather than a
latched fault. If another source charges the battery at a higher
voltage, the local DC/DC converter is disconnected and charging is
evaluated again later.

## Thermal Management

The temperatures of the MOSFET area and the DC/DC inductor are
monitored independently.

The fan operates whenever the DC/DC converter is actively charging.
After charging stops, the fan continues to operate until both monitored
temperatures have fallen below the configured fan-off temperature.

The maximum DC/DC power is reduced progressively at elevated
temperatures. Charging is stopped if the configured thermal shutdown
temperature is reached.

## Diagnostics

The UART interface remains available for monitoring and diagnostics.

Available commands:

- `D0` - Disable continuous debug output
- `D1` - Enable continuous debug output
- `P`  - Print current charger status once
- `?`  - Show command help
- `R`  - Reboot the controller

Diagnostic messages distinguish between normal operating states,
warnings and protection errors.

Warnings indicate conditions that should be investigated but do not
necessarily stop the charger. Protection errors may interrupt charging
when required to protect the battery or DC/DC converter.

## Parameter Verification

Parameters that have been changed or still require practical
verification are marked in the source code with:

`Fix!Me`

This marker is intentionally retained until the corresponding value
has been verified under practical operating conditions.

Power and thermal limits in particular should be verified with the
actual hardware, since DC/DC converter losses depend on operating
conditions such as input voltage, battery voltage and converter ratio.
