/*!
 * @file      wifi_scan_country_code.c
 *
 * @brief     Wi-Fi scan country code example for LR11xx chip
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

#include "main_wifi.h"
#include "wifi_scan_country_code.h"
#include "lr11xx_wifi.h"
#include "lr11xx_wifi_types_str.h"
#include "wifi_configuration_base.h"
#include "wifi_result_printers.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wifi_scan_country_code);

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef struct
{
    wifi_configuration_scan_base_t base;
    uint8_t                        nb_scan_per_channel;
    uint16_t                       timeout_per_scan;
    bool                           abort_on_timeout;
} wifi_configuration_scan_country_code_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static const wifi_configuration_scan_country_code_t wifi_configuration = {
    .base.channel_mask       = WIFI_CHANNEL_MASK,
    .base.max_result         = WIFI_MAX_RESULTS,
    .nb_scan_per_channel     = WIFI_NB_SCAN_PER_CHANNEL,
    .timeout_per_scan        = WIFI_TIMEOUT_PER_SCAN,
    .abort_on_timeout        = WIFI_ABORT_ON_TIMEOUT,
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

const char* wifi_scan_country_code_get_example_name( void ) { return ( const char* ) "Wi-Fi scan country-code"; }

void wifi_scan_country_code_call_api_scan( const void* context )
{
    int ret = lr11xx_wifi_search_country_code( context, wifi_configuration.base.channel_mask,
                                         wifi_configuration.base.max_result, wifi_configuration.nb_scan_per_channel,
                                         wifi_configuration.timeout_per_scan, wifi_configuration.abort_on_timeout );
    if(ret)
    {
        LOG_ERR("Failed passive wifi scan for country codes.");
    }
}

void wifi_scan_country_code_print_configuration( void )
{
    LOG_INF( "Wi-Fi example configuration:" );
    LOG_INF( "  -> channel mask: 0x%04X", wifi_configuration.base.channel_mask );
    LOG_INF( "  -> max result: %u", wifi_configuration.base.max_result );
    LOG_INF( "  -> nb scan per channel: %u", wifi_configuration.nb_scan_per_channel );
    LOG_INF( "  -> timeout per scan: %u", wifi_configuration.timeout_per_scan );
    LOG_INF( "  -> abort on timeout: %s", ( wifi_configuration.abort_on_timeout == true ) ? "True" : "False" );
    LOG_INF( "\n" );
}