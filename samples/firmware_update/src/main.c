#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/types.h>

#include <lr11xx_firmware_update.h>

#include "apps_common.h"

LOG_MODULE_REGISTER(main);

#if CONFIG_UPDATE_LR1110
#include "lr1110_transceiver_0307.h"
#elif CONFIG_UPDATE_LR1120
#include "lr1120_transceiver_0101.h"
#else
#error "Unsupported update target. Select either CONFIG_UPDATE_LR1110 or CONFIG_UPDATE_LR1120."

/* Bellow line is here so that only the above error is printed */
#include "lr1110_transceiver_0303.h"
#endif

const struct device *context = DEVICE_DT_GET_ONE(irnas_lr11xx);

void main(void)
{
	apps_common_lr11xx_system_init(context);

	if (LR11XX_FIRMWARE_UPDATE_TO == LR1110_FIRMWARE_UPDATE_TO_TRX) {
		LOG_INF("Updating LR1110 to transceiver firmware 0x%04x ", LR11XX_FIRMWARE_VERSION);
	} else if (LR11XX_FIRMWARE_UPDATE_TO == LR1120_FIRMWARE_UPDATE_TO_TRX) {
		LOG_INF("Updating LR1120 to transceiver firmware 0x%04x ", LR11XX_FIRMWARE_VERSION);
	} else {
		LOG_ERR("Unsupported update type: 0x%04x ", LR11XX_FIRMWARE_VERSION);
		return;
	}

	const lr11xx_fw_update_status_t status =
		lr11xx_update_firmware(context, LR11XX_FIRMWARE_UPDATE_TO, LR11XX_FIRMWARE_VERSION,
				       lr11xx_firmware_image, (uint32_t)LR11XX_FIRMWARE_IMAGE_SIZE);

	switch (status) {
	case LR11XX_FW_UPDATE_OK:
		LOG_INF("LR11XX firmware was updated successfully!");
		break;
	case LR11XX_FW_UPDATE_NOT_IN_PROD_MODE:
		LOG_ERR("LR11XX did not enter bootloader mode!");
		break;
	case LR11XX_FW_UPDATE_UNSUPPORTED:
		LOG_ERR("Selected firmware is not supproted on this lr11xx chip! (You might have "
			"selected a lr1110 firmware but have a lr1120 chip, or vice versa)");
		break;
	case LR11XX_FW_UPDATE_ERROR:
		LOG_ERR("Firmware version after chip update does not match the selected firmware.");
		break;
	}
}
