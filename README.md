# SWDR001-Zephyr
IRNAS port of the Semtech SWDR001 LR11XX driver to Zephyr. Original package proposes an implementation in C of the driver for **LR11XX** radio component.

## Folder structure
Driver is located in the `drivers/radio` folder where `lr11xx_driver` contains Semtech SWDR001 LR11XX driver files and Zephy compatible hal implementation is contained in the `radio_drivers_hal` folder. Board specific interface functions and defined in the `lr11xx_board.h` and `lr11xx_board.c` files. 

Compatible Device Tree bing is contained in the `dts` folder. 

`samples` folder contains functionality samples. Only `tx_cw` if functional at the moment. 

## Installation
This driver was written and tested for nrf-sdk v2.0.1

To install, first modify `.../ncs/nrf/west.yml` and add the following sections:

1. In `remotes`, add the following if not already added:

```yaml
 - name: irnas
   url-base: https://github.com/irnas
```

2. In the `projects` section add at the bottom (select revision you need):

```
- name: SWDR001-Zephyr
      repo-path: SWDR001-Zephyr
      path: irnas/SWDR001-Zephyr
      remote: irnas
      revision: dev
```

Then run `west update` in your freshly created bash/command prompt session.

Above command will clone `SWDR001-Zephyr` repository inside of `ncs/irnas/`. You can now use the driver in your application projects.
## SWDR001 LR11XX driver Components

The driver is split in several components:

- Bootloader

### Bootloader

This component is used to update the firmware.

### Register / memory access

This component is used to read / write data from registers or internal memory.

### System configuration

This component is used to interact with system-wide parameters like clock sources, integrated RF switches, etc.

### Radio

This component is used to send / receive data through the different modems (LoRa and GFSK) or perform a LoRa CAD (Channel Activity Detection). Parameters like power amplifier selection, output power and fallback modes are also accessible through this component.

### Wi-Fi Passive Scanning

This component is used to configure and initiate the passive scanning of the Wi-Fi signals that can be shared to request a geolocation.

### GNSS Scanning

This component is used to configure and initiate the acquisition of GNSS signals that can be shared to request a geolocation.

### Crypto engine

This component is used to set and derive keys in the internal keychain and perform cryptographic operations with the integrated hardware accelerator.

## Structure

Each component is based on different files:

- lr11xx_component.c: implementation of the functions related to component
- lr11xx_component.h: declarations of the functions related to component
- lr11xx_component_types.h: type definitions related to components

## HAL

The HAL (Hardware Abstraction Layer) is a collection of functions that the user shall implement to write platform-dependent calls to the host. The list of functions is the following:

- lr11xx_hal_reset()
- lr11xx_hal_wakeup()
- lr11xx_hal_write()
- lr11xx_hal_read()
- lr11xx_hal_direct_read()

The following driver contains Zephyr-compatible implementation.
