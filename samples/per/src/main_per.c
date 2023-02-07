/*!
 * @file      main_per.c
 *
 * @brief     Packet Error Rate (PER) example for LR11xx chip
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

#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include "apps_common.h"
#include "lr11xx_board.h"
#include "lr11xx_radio.h"
#include "lr11xx_regmem.h"
#include "lr11xx_system.h"
#include "main_per.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/**
 * @brief LR11xx interrupt mask used by the application
 */
#define IRQ_MASK                                                                          \
    ( LR11XX_SYSTEM_IRQ_TX_DONE | LR11XX_SYSTEM_IRQ_RX_DONE | LR11XX_SYSTEM_IRQ_TIMEOUT | \
      LR11XX_SYSTEM_IRQ_HEADER_ERROR | LR11XX_SYSTEM_IRQ_FSK_LEN_ERROR | LR11XX_SYSTEM_IRQ_CRC_ERROR )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define APP_PARTIAL_SLEEP true

#if( RECEIVER == 1 )
const char* mode = "Receiver";
#else
const char* mode = "Transmitter";
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

const struct device *context = DEVICE_DT_GET(DT_NODELABEL(lr1120));

static uint8_t buffer[PAYLOAD_LENGTH];

static uint64_t timestamp_start = 0;
static uint32_t timestamp_stop  = 0;

static uint16_t nb_ok            = 0;
static uint16_t nb_rx_timeout    = 0;
static uint16_t nb_rx_error      = 0;
static uint16_t nb_fsk_len_error = 0;

static uint8_t rolling_counter = 0;

static uint16_t per_index      = 0;
static bool     first_pkt_flag = false;

static uint8_t per_msg[PAYLOAD_LENGTH];

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Handle reception failure for PER example
 *
 * @param [in] failure_counter pointer to the counter for each type of reception failure
 */
static void per_reception_failure_handling( uint16_t* failure_counter );

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

    LOG_INF( "===== LR11xx PER example - %s =====\n", mode );

    apps_common_lr11xx_system_init( ( void* ) context );

    apps_common_lr11xx_fetch_and_print_version( ( void* ) context );

    apps_common_lr11xx_radio_init( ( void* ) context );

    LOG_INF("Set dio irq mask");
    ret = lr11xx_system_set_dio_irq_params( context, IRQ_MASK, 0 );
    if(ret)
    {
        LOG_ERR("Failed to set dio irq params.");
    }

    LOG_INF("Clear irq status");
    ret = lr11xx_system_clear_irq_status( context, LR11XX_SYSTEM_IRQ_ALL_MASK );
    if(ret)
    {
        LOG_ERR("Failed to set dio irq params.");
    }

    apps_common_lr11xx_enable_irq(context);

    timestamp_start = k_uptime_get( );

    for( int i = 1; i < PAYLOAD_LENGTH; i++ )
    {
        buffer[i] = i;
    }
#if RECEIVER == 1

    ret = lr11xx_radio_set_rx( context, RX_TIMEOUT_VALUE );
    if(ret)
    {
        LOG_ERR("Failed to set RX mode.");
    }
    memcpy( per_msg, &buffer[1], PAYLOAD_LENGTH - 1 );
#else
    buffer[0] = 0;
    ret = lr11xx_regmem_write_buffer8( context, buffer, PAYLOAD_LENGTH );
    if(ret)
    {
        LOG_INF("Failed to write buffer");
    }
    ret = lr11xx_radio_set_tx( context, 0 );
    if(ret)
    {
        LOG_INF("Failed to set TX mode.");
    }
#endif

    while( per_index < NB_FRAME )
    {
        apps_common_lr11xx_irq_process( context, IRQ_MASK );
        k_sleep(K_MSEC(1));
    }

    if( per_index > NB_FRAME )  // The last validdated packet should not be counted in this case
    {
        nb_ok--;
    }
    /* Display PER*/
    LOG_INF( "PER = %d \n", 100 - ( ( nb_ok * 100 ) / NB_FRAME ) );

    LOG_INF( "Final PER index: %d", per_index );
    LOG_INF( "Valid reception amount: %d", nb_ok );
    LOG_INF( "Timeout reception amount: %d", nb_rx_timeout );
    LOG_INF( "CRC Error reception amount: %d", nb_rx_error );
#if( PACKET_TYPE == LR11XX_RADIO_PKT_TYPE_GFSK )
    LOG_INF( "FSK Length Error reception amount: %d", nb_fsk_len_error );
#endif

    timestamp_stop = (uint32_t)( ( k_uptime_get( ) - timestamp_start ) / 60000 );

    LOG_INF( "TEST END %d min elapsed \n", timestamp_stop );

    while( 1 )
    {
        k_sleep(K_MSEC(100));
    }
}

void on_tx_done( void )
{
    int ret = 0;

    k_sleep(K_MSEC( TX_TO_TX_DELAY_IN_MS ));

    buffer[0]++;
    LOG_INF( "Counter value: %d", buffer[0] );
    ret = lr11xx_regmem_write_buffer8( context, buffer, PAYLOAD_LENGTH );
    if(ret)
    {
        LOG_INF("Failed to write buffer");
    }
    ret = lr11xx_radio_set_tx( context, 0 );
    if(ret)
    {
        LOG_INF("Failed to set TX mode.");
    }
}

void on_rx_done( void )
{
    int ret = 0;

    uint8_t size;

    apps_common_lr11xx_receive( context, buffer, &size );

    if( memcmp( &buffer[1], per_msg, PAYLOAD_LENGTH - 1 ) == 0 )
    {
        // Let's start counting after the first received packet
        if( first_pkt_flag == true )
        {
            uint8_t rolling_counter_gap = ( uint8_t )( buffer[0] - rolling_counter );
            nb_ok++;
            per_index += rolling_counter_gap;
            if( rolling_counter_gap > 1 )
            {
                LOG_WRN( "%d packet(s) missed", ( rolling_counter_gap - 1 ) );
            }
            rolling_counter = buffer[0];
        }
        else
        {
            first_pkt_flag  = true;
            rolling_counter = buffer[0];
        }
        LOG_INF( "Counter value: %d, PER index: %d", buffer[0], per_index );
    }
    if( per_index < NB_FRAME )  // Re-start Rx only if the expected number of frames is not reached
    {
        ret = lr11xx_radio_set_rx( context, RX_TIMEOUT_VALUE );
        if(ret)
        {
            LOG_INF("Failed to set RX mode.");
        }
    }
}

void on_rx_timeout( void )
{
    per_reception_failure_handling( &nb_rx_timeout );
}

void on_rx_crc_error( void )
{
    per_reception_failure_handling( &nb_rx_error );
}

void on_fsk_len_error( void )
{
    per_reception_failure_handling( &nb_fsk_len_error );
}

static void per_reception_failure_handling( uint16_t* failure_counter )
{
    int ret = 0;

    // Let's start counting after the first received packet
    if( first_pkt_flag == true )
    {
        ( *failure_counter )++;
    }

    ret =  lr11xx_radio_set_rx( context, RX_TIMEOUT_VALUE );
    if(ret)
    {
        LOG_INF("Failed to set RX mode.");
    }
}