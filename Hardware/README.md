# Hardware

Hardware documentation for the BatteryCharger_24V.

## Hardware Platform

BatteryCharger_24V uses the same PCB and basic hardware design as
BatteryCharger_12V.

The 24 V version differs in component values required for the higher
battery and DC/DC operating voltage.

## 24 V Component Changes

The following components differ from the BatteryCharger_12V assembly:

| Component | BatteryCharger_24V |
|-----------|--------------------|
| R48       | 180 kOhm           |
| R45       | 33 kOhm            |

All other component values correspond to the common hardware design
unless explicitly documented otherwise.

The BOM for BatteryCharger_24V must therefore be used when assembling
the 24 V version.

## Firmware Calibration

The different component values change the DC/DC output-voltage
characteristic.

BatteryCharger_24V therefore uses its own measured DAC calibration
parameters and voltage limits in the firmware.

## Related Hardware

The PCB layout and general circuit documentation are shared with the
BatteryCharger_12V project.

See the BatteryCharger_12V repository for the common PCB and schematic
documentation.
