/*!
 * @file      main_gnss.c
 *
 * @brief     GNSS scan example for LR11xx chip
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
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include "apps_common.h"
#include "lr11xx_system.h"
#include "main_gnss.h"
#include "lr11xx_gnss.h"
#include "lr11xx_board.h"

#include "gnss_example_api.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define LR11XX_NODE           DT_NODELABEL(lr1120)

#define APP_PARTIAL_SLEEP true
#define NAV_MAX_LENGTH ( 300 )

/**
 * @brief LR11xx interrupt mask used by the application
 */
#define IRQ_MASK ( LR11XX_SYSTEM_IRQ_GNSS_SCAN_DONE )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

const struct device *context;
static uint32_t              number_of_scan = 0;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Handle the start of a new GNSS scan
 *
 * It also check that a new Wi-Fi scan can start
 *
 */
static void start_scan( );

/**
 * @brief Handle the printing of results
 *
 * It includes the printing of detected SVs and cumulative timings.
 *
 */
static void fetch_and_print_results( void );

/**
 * @brief Helper function that indicate if a new scan can be started or if the demo is terminated
 *
 * @return true New scan can start
 * @return false The demo is terminated
 */
static bool can_execute_next_scan( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/**
 * @brief Main application entry point.
 */
int main( void )
{
    LOG_INF( "===== %s =====\n", gnss_get_example_name( ) );

    context = device_get_binding(DT_LABEL(LR11XX_NODE));

    apps_common_lr11xx_system_init( context );

    apps_common_lr11xx_fetch_and_print_version( context );

    int ret = 0;

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

    gnss_init( context );
    start_scan( );

    while( 1 )
    {
        apps_common_lr11xx_irq_process( context, IRQ_MASK );
        k_sleep(K_MSEC(1));
    }
}

void start_scan( void )
{
    if( can_execute_next_scan( ) )
    {
        gnss_call_scan( context );
        k_sleep(K_SECONDS(5));
    }
}

static void fetch_and_print_results( void )
{
    LOG_INF( "== Scan #%d ==", number_of_scan );

    lr11xx_gnss_timings_t gnss_timing = { 0 };
    int ret = lr11xx_gnss_get_timings( context, &gnss_timing );
    if(ret)
    {
        LOG_ERR("Failed to get timings.");
    }

    LOG_INF( "Timings:" );
    LOG_INF( "  - radio: %u ms", gnss_timing.radio_ms );
    LOG_INF( "  - computation: %u ms", gnss_timing.computation_ms );

    uint8_t                          n_sv_detected            = 0;
    lr11xx_gnss_detected_satellite_t sv_detected[GNSS_MAX_SV] = { 0 };
    ret = lr11xx_gnss_get_nb_detected_satellites( context, &n_sv_detected );
    if(ret)
    {
        LOG_ERR("Failed to get nr. of satellites.");
    }

    if( n_sv_detected > GNSS_MAX_SV )
    {
        LOG_ERR( "More SV detected than configured (detected %u, max is %u)", n_sv_detected,
                             GNSS_MAX_SV );
        return;
    }

    ret = lr11xx_gnss_get_detected_satellites( context, n_sv_detected, sv_detected );
    if(ret)
    {
        LOG_ERR("Failed to get detected satellites.");
    }

    uint16_t result_size         = 0;
    uint8_t  nav[NAV_MAX_LENGTH] = { 0 };
    ret =  lr11xx_gnss_get_result_size( context, &result_size );
    if(ret)
    {
        LOG_ERR("Failed to get result size.");
    }

    if( result_size > NAV_MAX_LENGTH )
    {
        LOG_ERR( "Result size too long (size %u, max is %u)", result_size, NAV_MAX_LENGTH );
        return;
    }

    ret = lr11xx_gnss_read_results( context, nav, result_size );
    if(ret)
    {
        LOG_ERR("Failed to read results.");
    }

    LOG_INF( "Detected %u SV(s):", n_sv_detected );
    for( uint8_t index_sv = 0; index_sv < n_sv_detected; index_sv++ )
    {
        const lr11xx_gnss_detected_satellite_t* local_sv = &sv_detected[index_sv];
        LOG_INF( "  - SV %u: CNR: %i, doppler: %i", local_sv->satellite_id, local_sv->cnr,
                            local_sv->doppler );
    }

    //Print nav message
    LOG_INF( "NAV message:");
    for(uint8_t i = 0; i < result_size; i++)
    {
        printk("%x ", nav[i]);
    }
    printk("\n");
}

bool can_execute_next_scan( void )
{
    const uint32_t max_n_scans = GNSS_MAX_SCANS;
    if( max_n_scans == 0 )
    {
        // If the maximum number of scan is set to 0, the example run indefinitely
        return true;
    }
    else
    {
        return ( number_of_scan < max_n_scans ) ? true : false;
    }
}

void on_gnss_scan_done( void )
{
    number_of_scan++;

    fetch_and_print_results( );

    start_scan( );
}
