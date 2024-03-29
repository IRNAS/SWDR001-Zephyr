/*!
 * @file      main_spectrum_display.c
 *
 * @brief     Spectrum-display example for LR11xx chip
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

#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include "apps_common.h"
#include "lr11xx_board.h"
#include "lr11xx_radio.h"
#include "lr11xx_system.h"
#include "lr11xx_system_types.h"
#include "main_spectrum_display.h"
#include "curve_plot.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/**
 * @brief Duration of the wait between setting to RX mode and valid instant RSSI value available
 *
 * Expressed in milliseconds
 *
 * @warning If switching from StandbyRC mode this delay is recommended to set to 30ms; if switching from StandbyXOSC,
 * 1ms.
 */
#define DELAY_BETWEEN_SET_RX_AND_VALID_RSSI_MS ( 1 )

/**
 * @brief Duration of the wait between each instant RSSI fetch. This is to make sure that the RSSI value is stable
 * before fetching
 */
#define DELAY_BETWEEN_EACH_INST_RSSI_FETCH_US ( 800 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define APP_PARTIAL_SLEEP true

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

const struct device *context = DEVICE_DT_GET(DT_NODELABEL(lr11xx));

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static void spectrum_display_start( uint32_t freq_hz );
static void print_configuration( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/**
 * @brief Main application entry point.
 */
int main( void )
{
    int ret = 0;

    LOG_INF( "===== LR11xx Spectrum Display example =====\n" );

    apps_common_lr11xx_system_init( ( void* ) context );

    apps_common_lr11xx_fetch_and_print_version( ( void* ) context );

    apps_common_lr11xx_radio_init( ( void* ) context );

    print_configuration( );

    create_canvas( );

    while( 1 )
    {
        int8_t         result;
        static uint8_t freq_chan_index = 0;
        const uint32_t freq_hz         = FREQ_START_HZ + ( freq_chan_index * WIDTH_CHAN_HZ );

        /* Start Spectral scan */
        spectrum_display_start( freq_hz );

        k_sleep(K_USEC( DELAY_BETWEEN_EACH_INST_RSSI_FETCH_US ));
        ret = lr11xx_radio_get_rssi_inst( context, &result );
        if(ret)
        {
            LOG_ERR("Failed to get rssi.");
        }
        plot_curve( ( freq_chan_index + 1 ), ( abs( result ) / RSSI_SCALE ) );

        /* Switch to next channel */
        ret = lr11xx_system_set_standby( context, LR11XX_SYSTEM_STANDBY_CFG_XOSC );
        if(ret)
        {
            LOG_ERR("Failed to set standby.");
        }

        freq_chan_index++;
        if( freq_chan_index >= NB_CHAN )
        {
            freq_chan_index = 0;

            /* Pace the scan speed (1 sec min) */
            for( uint16_t i = 0; i < ( int ) ( PACE_S ? PACE_S : 1 ); i++ )
            {
                k_sleep(K_MSEC( 1000 ));
            }
        }
    }
}

void spectrum_display_start( uint32_t freq_hz )
{
    int ret = 0;
    /* Set frequency */
    ret = lr11xx_radio_set_rf_freq( context, freq_hz );
    if(ret)
    {
        LOG_ERR("Failed to set RF frequency.");
    }

    /* Set Radio in Rx continuous mode */
    ret = lr11xx_radio_set_rx_with_timeout_in_rtc_step( context, RX_CONTINUOUS );
    if(ret)
    {
        LOG_ERR("Failed to set RX ctn mode.");
    }

    k_sleep(K_MSEC( DELAY_BETWEEN_SET_RX_AND_VALID_RSSI_MS ));
}

void print_configuration( void )
{
    LOG_INF( "Spectral Scan configuration:" );
    LOG_INF( "  - Number of channels need to scan: %d", NB_CHAN );
    LOG_INF( "  - Time delay between 2 scans: %d S", PACE_S );
    LOG_INF( "  - Start frequency: %lld Hz", FREQ_START_HZ );
    LOG_INF( "  - Frequency step of scan channels: %d Hz", WIDTH_CHAN_HZ );
    LOG_INF( "Start Spectrum Display:" );
}
