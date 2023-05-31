/*!
 * @file      lr11xx_firmware_update.c
 *
 * @brief     LR11XX firmware update implementation
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2022. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "lr11xx_firmware_update.h"

#include "lr11xx_board.h"
#include "lr11xx_bootloader.h"
#include "lr11xx_system.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(lr11xx_firmware_update);

/** This value is reported in version_bootloader.type if lr11xx entered production mode */
#define LR11XX_TYPE_PRODUCTION_MODE 0xDF

/**
 * @brief Check if lr11xx is in production/bootloader mode
 *
 * @param[in] type bootloader type (obtained from lr11xx_bootloader_get_version)
 * @retval true If chip is in production mode
 * @retval false If chip is not in production mode
 */
static bool prv_lr11xx_is_chip_in_production_mode(uint8_t type)
{
	return (type == LR11XX_TYPE_PRODUCTION_MODE) ? true : false;
}

/**
 * @brief Check if desired new firmware is compatible with the chip
 *
 * @param[in] update The update target
 * @param[in] bootloader_version The bootloader version of the lr11xx (obtained from
 * lr11xx_bootloader_get_version)
 * @retval true If firmware is compatible
 * @retval false If firmware is not compatible
 */
static bool prv_lr11xx_is_fw_compatible_with_chip(lr11xx_fw_update_t update,
						  uint16_t bootloader_version)
{
	if ((update == LR1110_FIRMWARE_UPDATE_TO_TRX) && (bootloader_version != 0x6500)) {
		return false;
	} else if ((update == LR1120_FIRMWARE_UPDATE_TO_TRX) && (bootloader_version != 0x2000)) {
		return false;
	}

	return true;
}

lr11xx_fw_update_status_t lr11xx_update_firmware(const struct device *context,
						 lr11xx_fw_update_t fw_update, uint32_t fw_expected,
						 const uint32_t *buffer, uint32_t length)
{
	lr11xx_bootloader_version_t version_bootloader = {0};

	const struct lr11xx_hal_context_cfg_t *config = context->config;

	LOG_DBG("Reseting the chip...");

	/* Hold the BUSY pin low when resetting lr11xx. This will put it into bootloader mode */
	gpio_pin_configure_dt(&config->busy, GPIO_OUTPUT_INACTIVE);

	lr11xx_system_reset(context);

	k_sleep(K_MSEC(500));
	/* Configure back to input */
	gpio_pin_configure_dt(&config->busy, GPIO_INPUT);
	k_sleep(K_MSEC(100));

	LOG_DBG("Reset done");

	lr11xx_bootloader_get_version(context, &version_bootloader);
	LOG_DBG("Chip in bootloader mode: ");
	LOG_DBG(" - Chip type               = 0x%02X (0xDF for production) ",
		version_bootloader.type);
	LOG_DBG(" - Chip hardware version   = 0x%02X (0x22 for V2C) ", version_bootloader.hw);
	LOG_DBG(" - Chip bootloader version = 0x%04X  ", version_bootloader.fw);

	if (prv_lr11xx_is_chip_in_production_mode(version_bootloader.type) == false) {
		LOG_ERR("LR11XX did not enter production mode");
		return LR11XX_FW_UPDATE_NOT_IN_PROD_MODE;
	}

	if (prv_lr11xx_is_fw_compatible_with_chip(fw_update, version_bootloader.fw) == false) {
		LOG_ERR("Privded firmware is not supported on this chip");
		return LR11XX_FW_UPDATE_UNSUPPORTED;
	};

	lr11xx_bootloader_pin_t pin = {0x00};
	lr11xx_bootloader_chip_eui_t chip_eui = {0x00};
	lr11xx_bootloader_join_eui_t join_eui = {0x00};

	lr11xx_bootloader_read_pin(context, pin);
	lr11xx_bootloader_read_chip_eui(context, chip_eui);
	lr11xx_bootloader_read_join_eui(context, join_eui);

	LOG_DBG("Start flash erase... ");
	lr11xx_bootloader_erase_flash(context);
	LOG_DBG("Flash erase done");

	LOG_DBG("Start flashing firmware... ");
	lr11xx_bootloader_write_flash_encrypted_full(context, 0, buffer, length);
	LOG_DBG("Flashing done");

	LOG_DBG("Rebooting... ");
	lr11xx_bootloader_reboot(context, false);
	LOG_DBG("Reboot done");

	lr11xx_system_version_t version_trx = {0x00};
	lr11xx_system_uid_t uid = {0x00};

	lr11xx_system_get_version(context, &version_trx);
	LOG_DBG("Chip in transceiver mode: ");
	LOG_DBG(" - Chip type             = 0x%02X ", version_trx.type);
	LOG_DBG(" - Chip hardware version = 0x%02X ", version_trx.hw);
	LOG_DBG(" - Chip firmware version = 0x%04X ", version_trx.fw);

	lr11xx_system_read_uid(context, uid);

	if (version_trx.fw == fw_expected) {
		return LR11XX_FW_UPDATE_OK;
	} else {
		return LR11XX_FW_UPDATE_ERROR;
	}
}