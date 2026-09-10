# BatteryCharger_24V

24 V DC battery charger with configurable charge profile, power-source management and protected DC/DC control.

## Project Overview

BatteryCharger_24V is a microcontroller-controlled DC battery charger developed as the 24 V variant of the modular battery charger family.

The charger is intended for general DC charging applications using available DC power sources such as power supplies, converters or other suitable DC sources.

Charging parameters are defined directly in the firmware and can therefore be adapted to different battery types and applications without requiring a configuration menu during normal operation.

The project combines battery charging, source management, DC/DC converter protection and thermal management in one controller.

## Main Features

- 24 V battery charging
- Configurable battery charging parameters
- Bulk, absorption, full and optional float charging
- Three DC power inputs:
  - Low Power: up to 2 A
  - Mid Power: up to 6 A
  - High Power: up to 9 A
- Automatic input priority: High → Mid → Low
- Configurable battery current limit
- Maximum DC/DC converter power: 350 W
- DC/DC pre-charge and output verification
- Reverse-current detection
- Temperature monitoring of MOSFETs and inductor
- Temperature-dependent power reduction
- Active cooling during charging with temperature-controlled fan run-on
- UART monitoring and diagnostics

## Current Default Charging Parameters

- Absorption voltage: 28.4 V
- Float voltage: 27.8 V
- Restart voltage: 25.2 V
- Battery undervoltage limit: 21.0 V
- Battery overvoltage limit: 28.8 V
- Maximum battery charge current: 15 A
- Maximum DC/DC power: 350 W

These values are configurable in the firmware and are not tied to a specific battery chemistry.

## Project Status

Development and testing in progress.

Some power, thermal and charge-end parameters are marked with `Fix!Me` in the source code where practical verification is still required.

BatteryCharger_24V uses the same basic software structure and operating concept as BatteryCharger_12V, with hardware-specific voltage, current and calibration parameters adapted for the 24 V power stage.
