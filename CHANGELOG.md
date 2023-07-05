# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/) format.

## [Unreleased]

## [1.5.1] - 2023-07-05

### Fixed

-   Fixed nrf52840DK overlay in all samples, adding missing bindings.

## [1.5.0] - 2023-07-04

### Added

-   Added additional device tree properties for TXCO, LF clock and regulator configuration.

## [1.4.0] - 2023-06-21

### Added

-   Mandatory `lf-tx-path` dts entry for LR11XX. This is required for proper TX path configuration.

## [1.3.0] - 2023-06-19

### Changed

-   Update to NCS v2.2.0.
-   Disable compiler warnings for Semtech's code.

## [1.2.1] - 2023-06-16

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

[Unreleased]: https://github.com/IRNAS/SWDR001-Zephyr/compare/v1.5.1...HEAD

[1.5.1]: https://github.com/IRNAS/SWDR001-Zephyr/compare/v1.5.0...v1.5.1

[1.5.0]: https://github.com/IRNAS/SWDR001-Zephyr/compare/v1.4.0...v1.5.0

[1.4.0]: https://github.com/IRNAS/SWDR001-Zephyr/compare/v1.3.0...v1.4.0

[1.3.0]: https://github.com/IRNAS/SWDR001-Zephyr/compare/v1.2.1...v1.3.0

[1.2.1]: https://github.com/IRNAS/SWDR001-Zephyr/compare/v1.2.0...v1.2.1

[1.2.0]: https://github.com/IRNAS/SWDR001-Zephyr/compare/v1.1.0...v1.2.0

[1.1.0]: https://github.com/IRNAS/SWDR001-Zephyr/compare/v1.0.0...v1.1.0

[1.0.0]: https://github.com/IRNAS/SWDR001-Zephyr/compare/da01832ec757744cf488c648f9006b4e671e6e5d...v1.0.0
