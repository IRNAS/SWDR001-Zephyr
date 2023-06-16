# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/) format.

## [Unreleased]

## [0.2.1] - 2023-06-16

### Changed

-   The RF frequency has been switched from 490 MHz to 868 MHz (which is a default EU frequency for LoRa) which applies to all the samples.

## [1.2.0] - 2023-05-31

### Added

-   Firmware update sample

## [1.1.0] - 2023-02-07

### Changed

-   Update headers and device getters in accordance with NCS 2.2.

## [1.0.0] - 2023-02-07

### Added

-   Semtech SWDR001 LR11XX driver.
-   Zephyr compatible HAL implementation.
-   Zephyr compatible yaml bindings.
-   Zephyr compatible context and peripheral implementation.
-   Common sample file with initialization functions and event handler.
-   Almanac update sample.
-   CAD sample.
-   GNSS sample.
-   PER sample.
-   ping-pong sample.
-   spectral scan sample.
-   spectral display sample.
-   TX continuos wave sample.
-   TX infinite preamble sample.
-   WiFi scan sample.
-   Modified ping-pong sample with signal reporting for LR 868 MHz, LR 2.4 GHz and BT signal strength.

[Unreleased]: https://github.com/IRNAS/SWDR001-Zephyr/compare/v0.2.1...HEAD

[0.2.1]: https://github.com/IRNAS/SWDR001-Zephyr/compare/v1.2.0...v0.2.1

[1.2.0]: https://github.com/IRNAS/SWDR001-Zephyr/compare/v1.1.0...v1.2.0

[1.1.0]: https://github.com/IRNAS/SWDR001-Zephyr/compare/v1.0.0...v1.1.0

[1.0.0]: https://github.com/IRNAS/SWDR001-Zephyr/compare/da01832ec757744cf488c648f9006b4e671e6e5d...v1.0.0
