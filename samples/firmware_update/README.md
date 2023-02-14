# LR11xx Firmware update example

Based on [SWTL001](https://github.com/Lora-net/SWTL001).

## Description

In `prj.conf`, select either `CONFIG_UPDATE_LR1110` or `CONFIG_UPDATE_LR1120`.

The sample will update the connected lr11xx to the latest transceiver firmware available for the selected chip type. See `src/firmware_images`
and the top of `main.c` to see which firmware will be flashed.

The sample will print a success or error message at the end.
