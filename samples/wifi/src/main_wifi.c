/*!
 * @file      main_wifi.c
 *
 * @brief     Wi-Fi scan example for LR11xx chip
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
#include <zephyr.h>
#include <zephyr/types.h>
#include <device.h>
#include <devicetree.h>

#include "main_wifi.h"
#include "apps_common.h"
#include "lr11xx_board.h"
#include "lr11xx_wifi.h"
#include "lr11xx_system.h"
#include "wifi_configuration_base.h"
#include "lr11xx_wifi_types_str.h"
#include "wifi_result_printers.h"

#include "wifi_scan.h"
#include "wifi_scan_time_limit.h"
#include "wifi_scan_country_code.h"
#include "wifi_scan_country_code_time_limit.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(main);

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define LR11XX_NODE           DT_NODELABEL(lr1120)

/**
 * @brief Duration of the wait after printing example configuration on the serial port
 *
 * Expressed in milliseconds
 */
#define DELAY_AFTER_PRINT_CONFIGURATION_MS ( 800 )

/**
 * @brief LR11xx interrupt mask used by the application
 */
#define IRQ_MASK ( LR11XX_SYSTEM_IRQ_WIFI_SCAN_DONE )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef enum
{
    WIFI_SCAN,
    WIFI_SCAN_TIME_LIMIT,
    WIFI_SCAN_COUNTRY_CODE,
    WIFI_SCAN_COUNTRY_CODE_TIME_LIMIT,
} wifi_demo_to_run_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define APP_PARTIAL_SLEEP true

const wifi_demo_to_run_t          wifi_demo_to_run        = WIFI_DEMO_TO_RUN;
const lr11xx_wifi_result_format_t wifi_scan_result_format = WIFI_RESULT_FORMAT;

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
 * @brief Handle the start of a new Wi-Fi scan
 *
 * It also check that a new Wi-Fi scan can start
 *
 */
static void start_scan( void );

/**
 * @brief Handle the printing of results
 *
 * It includes the printing of cumulative timings.
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

/**
 * @brief Get the interface name depending on example selected
 *
 * @return const char* The interface name
 */
const char* get_interface_name( void );

/**
 * @brief Print the configuration depending on the example selected
 */
void print_configuration( void );

/**
 * @brief Call the scan API depending on the example selected
 *
 * @param context Chip implementation context
 */
void call_scan( const void* context );

/**
 * @brief Print the Wi-Fi scan result depending on the example and the result format selected
 *
 * @param context Chip implementation context
 */
void result_printer( const void* context );

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
    
    LOG_INF( "===== LR11xx %s example =====\n", get_interface_name( ) );

    context = device_get_binding(DT_LABEL(LR11XX_NODE));

    apps_common_lr11xx_system_init( ( void* ) context );

    apps_common_lr11xx_fetch_and_print_version( ( void* ) context );

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

    // Check that the Wi-Fi format and Wi-Fi scan mode are compatible before starting the example
    const bool is_compatible = lr11xx_wifi_are_scan_mode_result_format_compatible( WIFI_SCAN_MODE, WIFI_RESULT_FORMAT );
    if( is_compatible == false )
    {
        LOG_ERR( "Wi-Fi Scan mode %s and Wi-Fi format %s are not compatible",
                             lr11xx_wifi_mode_to_str( WIFI_SCAN_MODE ),
                             lr11xx_wifi_result_format_to_str( WIFI_RESULT_FORMAT ) );
    }
    print_configuration( );

    // This delay leaves some time to read the configuration on the terminal emulator
    k_sleep( K_MSEC(DELAY_AFTER_PRINT_CONFIGURATION_MS) );

    start_scan( );

    while( 1 )
    {
        apps_common_lr11xx_irq_process( context, IRQ_MASK );
        k_sleep(K_MSEC(1));
    }
}

void start_scan( void )
{
    int ret = 0;

    if( can_execute_next_scan( ) )
    {
        ret = lr11xx_wifi_reset_cumulative_timing( context );
        if(ret)
        {
            LOG_ERR("Failed to reset cumulative timing.");
        }

        call_scan( context );
    }
};

void fetch_and_print_results( void )
{
    int ret = 0;

    lr11xx_wifi_cumulative_timings_t cumulative_timings = { 0 };
    ret = lr11xx_wifi_read_cumulative_timing( context, &cumulative_timings );
    if(ret)
    {
        LOG_ERR("Failed to read cumulative timing.");
    }

    LOG_INF( "== Scan #%u ==", number_of_scan );
    LOG_INF( "Cummulative timings:" );
    LOG_INF( "  -> Demodulation: %u us", cumulative_timings.demodulation_us );
    LOG_INF( "  -> Capture: %u us", cumulative_timings.rx_capture_us );
    LOG_INF( "  -> Correlation: %u us", cumulative_timings.rx_correlation_us );
    LOG_INF( "  -> Detection: %u us", cumulative_timings.rx_detection_us );
    LOG_INF( "  => Total : %u us\n",
                          cumulative_timings.demodulation_us + cumulative_timings.rx_capture_us +
                              cumulative_timings.rx_correlation_us + cumulative_timings.rx_detection_us );
    result_printer( context );
}

bool can_execute_next_scan( void )
{
    const uint32_t max_number_of_scan = WIFI_MAX_NUMBER_OF_SCAN;
    if( max_number_of_scan == 0 )
    {
        // If the maximum number of scan is set to 0, the example run indefinitely
        return true;
    }
    else
    {
        if( number_of_scan < max_number_of_scan )
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}

void on_wifi_scan_done( void )
{
    number_of_scan++;

    fetch_and_print_results( );
    start_scan( );
}

const char* get_interface_name( void )
{
    switch( wifi_demo_to_run )
    {
    case WIFI_SCAN:
    {
        return wifi_scan_get_example_name( );
    }

    case WIFI_SCAN_TIME_LIMIT:
    {
        return wifi_scan_time_limit_get_example_name( );
    }

    case WIFI_SCAN_COUNTRY_CODE:
    {
        return wifi_scan_country_code_get_example_name( );
    }

    case WIFI_SCAN_COUNTRY_CODE_TIME_LIMIT:
    {
        return wifi_scan_country_code_time_limit_get_example_name( );
    }

    default:
    {
        LOG_ERR( "Unknown interface: 0x%02x", wifi_demo_to_run );
        return ( const char* ) "unknown";
    }
    }
}

void print_configuration( void )
{
    switch( wifi_demo_to_run )
    {
    case WIFI_SCAN:
    {
        return wifi_scan_print_configuration( );
    }

    case WIFI_SCAN_TIME_LIMIT:
    {
        return wifi_scan_time_limit_print_configuration( );
    }

    case WIFI_SCAN_COUNTRY_CODE:
    {
        return wifi_scan_country_code_print_configuration( );
    }

    case WIFI_SCAN_COUNTRY_CODE_TIME_LIMIT:
    {
        return wifi_scan_country_code_time_limit_print_configuration( );
    }

    default:
    {
        LOG_ERR( "Unknown interface: 0x%02x", wifi_demo_to_run );
    }
    }
}

void call_scan( const void* context )
{

    switch( wifi_demo_to_run )
    {
    case WIFI_SCAN:
    {
        return wifi_scan_call_api_scan( context );
    }

    case WIFI_SCAN_TIME_LIMIT:
    {
        return wifi_scan_time_limit_call_api_scan( context );
    }

    case WIFI_SCAN_COUNTRY_CODE:
    {
        return wifi_scan_country_code_call_api_scan( context );
    }

    case WIFI_SCAN_COUNTRY_CODE_TIME_LIMIT:
    {
        return wifi_scan_country_code_time_limit_call_api_scan( context );
    }

    default:
    {
        LOG_ERR( "Unknown interface: 0x%02x", wifi_demo_to_run );
    }
    }
}

void result_printer( const void* context )
{
    switch( wifi_demo_to_run )
    {
    case WIFI_SCAN:
    case WIFI_SCAN_TIME_LIMIT:
    {
        switch( wifi_scan_result_format )
        {
        case LR11XX_WIFI_RESULT_FORMAT_BASIC_COMPLETE:
        {
            wifi_fetch_and_print_scan_basic_complete_results( context );
            break;
        }
        case LR11XX_WIFI_RESULT_FORMAT_BASIC_MAC_TYPE_CHANNEL:
        {
            wifi_fetch_and_print_scan_basic_mac_type_channel_results( context );
            break;
        }
        case LR11XX_WIFI_RESULT_FORMAT_EXTENDED_FULL:
        {
            wifi_fetch_and_print_scan_extended_complete_results( context );
            break;
        }
        }
        break;
    }

    case WIFI_SCAN_COUNTRY_CODE:
    case WIFI_SCAN_COUNTRY_CODE_TIME_LIMIT:
    {
        wifi_fetch_and_print_scan_country_code_results( context );
        break;
    }

    default:
    {
        LOG_ERR( "Unknown interface: 0x%02x", wifi_demo_to_run );
    }
    }
}