/** @file lr11xx_board.h
 *
 * @brief 
 * 
 * @par
 * COPYRIGHT NOTICE: (c) 2022 Irnas. All rights reserved.
 */


#ifndef LR11XX_BOARD_H
#define LR11XX_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr.h>
#include "lr11xx_hal_context.h"
#include "lr11xx_types.h"
#include "lr11xx_radio_types.h"

typedef struct
{
    bool                                has_tcxo;
    lr11xx_system_tcxo_supply_voltage_t supply;
    uint32_t                            timeout_ms;
} lr11xx_board_tcxo_cfg_t;

typedef struct
{
    lr11xx_system_lfclk_cfg_t lf_clk_cfg;
    bool                      wait_32k_ready;
} lr11xx_board_lf_clck_cfg_t;

typedef struct lr11xx_board_pa_pwr_cfg_t
{
    int8_t                power;
    lr11xx_radio_pa_cfg_t pa_config;
} lr11xx_board_pa_pwr_cfg_t;

/**
 * @brief Create lr11xx_hal_context_t from Device Tree binding.
 * 
 * @return lr11xx_hal_context_t* 
 */
lr11xx_hal_context_t *lr11xx_board_context_get(void);

/**
 * @brief Init GPIO pins in lr11xx_hal_context_t context.
 * 
 * @param context 
 */
void lr11xx_board_context_init(lr11xx_hal_context_t * context);

/**
 * @brief Return HW specific reg mode.
 * 
 * @return lr11xx_system_reg_mode_t 
 */
lr11xx_system_reg_mode_t lr11xx_board_get_reg_mode( void );

/*!
 * @brief Get the TCXO configuration compatible with the board
 *
 * @return TCXO configuration compatible with the board
 */
lr11xx_board_tcxo_cfg_t lr11xx_board_get_tcxo_cfg( void );

/*!
 * @brief Get a LF clock configuration compatible with the board
 *
 * @return LF clock configuration compatible with the board
 */
lr11xx_board_lf_clck_cfg_t lr11xx_board_get_lf_clk_cfg( void );

/*!
 * @brief Return the RSSI calibration table corresponding to a given RF frequency
 *
 * @param [in] freq_in_hz RF frequence in Hz
 *
 * @return The RSSI calibration table
 */
const lr11xx_radio_rssi_calibration_table_t* lr11xx_board_get_rssi_calibration_table( const uint32_t freq_in_hz );

/*!
 * @brief Get the power amplifier configuration given a RF frequency and output power
 *
 * @param [in] rf_freq_in_hz RF frequence in Hz
 * @param [in] expected_output_pwr_in_dbm TX output power in dBm
 *
 * @returns Pointer to a structure holding the expected configuration.
 * Can be NULL if no configuration found for given arguments.
 */
const lr11xx_board_pa_pwr_cfg_t* lr11xx_board_get_pa_pwr_cfg( const uint32_t rf_freq_in_hz,
                                                          int8_t         expected_output_pwr_in_dbm );

#ifdef __cplusplus
}
#endif

#endif  // LR11XX_BOARD_H
