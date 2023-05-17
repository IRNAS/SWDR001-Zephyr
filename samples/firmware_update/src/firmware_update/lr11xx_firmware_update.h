/*!
 * @file      lr11xx_firmware_update.h
 *
 * @brief     LR11XX firmware update definition
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
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

#ifndef LR11XX_FIRMWARE_UPDATE_H
#define LR11XX_FIRMWARE_UPDATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/device.h>
#include <zephyr/types.h>

typedef enum {
	/** Update LR1110 to transciever */
	LR1110_FIRMWARE_UPDATE_TO_TRX,
	/** Update LR1120 to transciever */
	LR1120_FIRMWARE_UPDATE_TO_TRX,
} lr11xx_fw_update_t;

typedef enum {
	/** Firmware was successfully updated */
	LR11XX_FW_UPDATE_OK,
	/** LR11XX did not enter production/bootloader mode successfully */
	LR11XX_FW_UPDATE_NOT_IN_PROD_MODE,
	/** Provided firmware image is not supported by this LR11XX chip */
	LR11XX_FW_UPDATE_UNSUPPORTED,
	/** Firmware did not update successfully */
	LR11XX_FW_UPDATE_ERROR,
} lr11xx_fw_update_status_t;

/**
 * @brief Update lr11xx chip with the provided firmware
 *
 * @param[in] context The lr11xx radio context
 * @param[in] fw_update Update type
 * @param[in] fw_expected Expected firmware version after the update
 * @param[in] buffer The buffer containing the firmware image
 * @param[in] length The length of the buffer, in bytes
 *
 * @return See enum definition above
 */
lr11xx_fw_update_status_t lr11xx_update_firmware(const struct device *context,
						 lr11xx_fw_update_t fw_update, uint32_t fw_expected,
						 const uint32_t *buffer, uint32_t length);

#ifdef __cplusplus
}
#endif

#endif // LR11XX_FIRMWARE_UPDATE_H
