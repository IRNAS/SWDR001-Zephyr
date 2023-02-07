/*!
 * @file      wifi_result_printers.c
 *
 * @brief     Wi-Fi result printers
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
#include <zephyr/kernel.h>
#include <zephyr/types.h>

#include "lr11xx_wifi.h"
#include "lr11xx_wifi_types_str.h"
#include "wifi_result_printers.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wifi);

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

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

void print_mac_address( const char* prefix, lr11xx_wifi_mac_address_t mac );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void wifi_fetch_and_print_scan_basic_mac_type_channel_results( const void* context )
{
    uint8_t n_results = 0;
    int ret = 0;
    ret = lr11xx_wifi_get_nb_results( context, &n_results );
    if(ret)
    {
        LOG_ERR("Failed to obtain number of results.");
    }

    for( uint8_t result_index = 0; result_index < n_results; result_index++ )
    {
        lr11xx_wifi_basic_mac_type_channel_result_t local_result = { 0 };
        ret = lr11xx_wifi_read_basic_mac_type_channel_results( context, result_index, 1, &local_result );
        if(ret)
        {
            LOG_ERR("Failed to read basic mac type channel results.");
        }

        lr11xx_wifi_mac_origin_t mac_origin    = LR11XX_WIFI_ORIGIN_BEACON_FIX_AP;
        lr11xx_wifi_channel_t    channel       = LR11XX_WIFI_NO_CHANNEL;
        bool                     rssi_validity = false;
        lr11xx_wifi_parse_channel_info( local_result.channel_info_byte, &channel, &rssi_validity, &mac_origin );

        LOG_INF( "Result %u/%u", result_index + 1, n_results );
        print_mac_address( "  -> MAC address: ", local_result.mac_address );
        LOG_INF( "  -> Channel: %s", lr11xx_wifi_channel_to_str( channel ) );
        LOG_INF( "  -> MAC origin: %s", ( rssi_validity ? "From gateway" : "From end device" ) );
        LOG_INF( "  -> Signal type: %s\n",
            lr11xx_wifi_signal_type_result_to_str(
                lr11xx_wifi_extract_signal_type_from_data_rate_info( local_result.data_rate_info_byte ) ) );
    }
}

void wifi_fetch_and_print_scan_basic_complete_results( const void* context )
{
    uint8_t n_results = 0;
    int ret = 0;
    ret = lr11xx_wifi_get_nb_results( context, &n_results );
    if(ret)
    {
        LOG_ERR("Failed to obtain number of results.");
    }

    for( uint8_t result_index = 0; result_index < n_results; result_index++ )
    {
        lr11xx_wifi_basic_complete_result_t local_result = { 0 };
        ret = lr11xx_wifi_read_basic_complete_results( context, result_index, 1, &local_result );
        if(ret)
        {
            LOG_ERR("Failed to read basic complete results.");
        }

        lr11xx_wifi_mac_origin_t mac_origin    = LR11XX_WIFI_ORIGIN_BEACON_FIX_AP;
        lr11xx_wifi_channel_t    channel       = LR11XX_WIFI_NO_CHANNEL;
        bool                     rssi_validity = false;
        lr11xx_wifi_parse_channel_info( local_result.channel_info_byte, &channel, &rssi_validity, &mac_origin );

        lr11xx_wifi_frame_type_t     frame_type     = LR11XX_WIFI_FRAME_TYPE_MANAGEMENT;
        lr11xx_wifi_frame_sub_type_t frame_sub_type = 0;
        bool                         to_ds          = false;
        bool                         from_ds        = false;
        lr11xx_wifi_parse_frame_type_info( local_result.frame_type_info_byte, &frame_type, &frame_sub_type, &to_ds,
                                           &from_ds );

        LOG_INF( "Result %u/%u", result_index + 1, n_results );
        print_mac_address( "  -> MAC address: ", local_result.mac_address );
        LOG_INF( "  -> Channel: %s", lr11xx_wifi_channel_to_str( channel ) );
        LOG_INF( "  -> MAC origin: %s", ( rssi_validity ? "From gateway" : "From end device" ) );
        LOG_INF(
            "  -> Signal type: %s",
            lr11xx_wifi_signal_type_result_to_str(
                lr11xx_wifi_extract_signal_type_from_data_rate_info( local_result.data_rate_info_byte ) ) );
        LOG_INF( "  -> Frame type: %s", lr11xx_wifi_frame_type_to_str( frame_type ) );
        LOG_INF( "  -> Frame sub-type: 0x%02X", frame_sub_type );
        LOG_INF( "  -> FromDS/ToDS: %s / %s", ( ( from_ds == true ) ? "true" : "false" ),
                              ( ( to_ds == true ) ? "true" : "false" ) );
        LOG_INF( "  -> Phi Offset: %i", local_result.phi_offset );
        LOG_INF( "  -> Timestamp: %llu us", local_result.timestamp_us );
        LOG_INF( "  -> Beacon period: %u TU\n", local_result.beacon_period_tu );
    }
}

void wifi_fetch_and_print_scan_extended_complete_results( const void* context )
{
    uint8_t n_results = 0;
    int ret = 0;
    ret = lr11xx_wifi_get_nb_results( context, &n_results );
    if(ret)
    {
        LOG_ERR("Failed to obtain number of results.");
    }

    for( uint8_t result_index = 0; result_index < n_results; result_index++ )
    {
        lr11xx_wifi_extended_full_result_t local_result = { 0 };
        ret = lr11xx_wifi_read_extended_full_results( context, result_index, 1, &local_result );
        if(ret)
        {
            LOG_ERR("Failed to read extended full results.");
        }

        lr11xx_wifi_mac_origin_t mac_origin    = LR11XX_WIFI_ORIGIN_BEACON_FIX_AP;
        lr11xx_wifi_channel_t    channel       = LR11XX_WIFI_NO_CHANNEL;
        bool                     rssi_validity = false;
        lr11xx_wifi_parse_channel_info( local_result.channel_info_byte, &channel, &rssi_validity, &mac_origin );

        lr11xx_wifi_signal_type_result_t wifi_signal_type = { 0 };
        lr11xx_wifi_datarate_t           wifi_data_rate   = { 0 };
        lr11xx_wifi_parse_data_rate_info( local_result.data_rate_info_byte, &wifi_signal_type, &wifi_data_rate );

        LOG_INF( "Result %u/%u", result_index + 1, n_results );
        print_mac_address( "  -> MAC address 1: ", local_result.mac_address_1 );
        print_mac_address( "  -> MAC address 2: ", local_result.mac_address_2 );
        print_mac_address( "  -> MAC address 3: ", local_result.mac_address_3 );
        LOG_INF( "  -> Country code: %c%c", ( uint8_t ) ( local_result.country_code[0] ),
                              ( uint8_t ) ( local_result.country_code[1] ) );
        LOG_INF( "  -> Channel: %s", lr11xx_wifi_channel_to_str( channel ) );
        LOG_INF( "  -> Signal type: %s", lr11xx_wifi_signal_type_result_to_str( wifi_signal_type ) );
        LOG_INF( "  -> RSSI: %i dBm", local_result.rssi );
        LOG_INF( "  -> Rate index: 0x%02x", local_result.rate );
        LOG_INF( "  -> Service: 0x%04x", local_result.service );
        LOG_INF( "  -> Length: %u", local_result.length );
        LOG_INF( "  -> Frame control: 0x%04X", local_result.frame_control );
        LOG_INF( "  -> Data rate: %s", lr11xx_wifi_datarate_to_str( wifi_data_rate ) );
        LOG_INF( "  -> MAC origin: %s", ( rssi_validity ? "From gateway" : "From end device" ) );
        LOG_INF( "  -> Phi Offset: %i", local_result.phi_offset );
        LOG_INF( "  -> Timestamp: %llu us", local_result.timestamp_us );
        LOG_INF( "  -> Beacon period: %u TU", local_result.beacon_period_tu );
        LOG_INF( "  -> Sequence control: 0x%04x", local_result.seq_control );
        LOG_INF( "  -> IO regulation: 0x%02x", local_result.io_regulation );
        LOG_INF( "  -> Current channel: %s",
                              lr11xx_wifi_channel_to_str( local_result.current_channel ) );
        LOG_INF( "  -> FCS status:\n    - %s",
                              ( local_result.fcs_check_byte.is_fcs_checked ) ? "Is present" : "Is not present" );
        LOG_INF( "    - %s", ( local_result.fcs_check_byte.is_fcs_ok ) ? "Valid" : "Not valid" );
    }
}

void wifi_fetch_and_print_scan_country_code_results( const void* context )
{
    uint8_t n_results = 0;
    int ret = 0;
    ret = lr11xx_wifi_get_nb_results( context, &n_results );
    if(ret)
    {
        LOG_ERR("Failed to obtain number of results.");
    }

    for( uint8_t result_index = 0; result_index < n_results; result_index++ )
    {
        lr11xx_wifi_country_code_t local_result = { 0 };
        ret = lr11xx_wifi_read_country_code_results( context, result_index, 1, &local_result );
        if(ret)
        {
            LOG_ERR("Failed to read country code results.");
        }

        lr11xx_wifi_mac_origin_t mac_origin    = LR11XX_WIFI_ORIGIN_BEACON_FIX_AP;
        lr11xx_wifi_channel_t    channel       = LR11XX_WIFI_NO_CHANNEL;
        bool                     rssi_validity = false;
        lr11xx_wifi_parse_channel_info( local_result.channel_info_byte, &channel, &rssi_validity, &mac_origin );

        LOG_INF( "Result %u/%u\n", result_index + 1, n_results );
        print_mac_address( "  -> MAC address: ", local_result.mac_address );
        LOG_INF( "  -> Country code: %c%c\n", local_result.country_code[0], local_result.country_code[1] );
        LOG_INF( "  -> Channel: %s\n", lr11xx_wifi_channel_to_str( channel ) );
        LOG_INF( "  -> MAC origin: %s\n", ( rssi_validity ? "From gateway" : "From end device" ) );
    }
}

void print_mac_address( const char* prefix, lr11xx_wifi_mac_address_t mac )
{
    LOG_INF( "%s%02x:%02x:%02x:%02x:%02x:%02x", prefix, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5] );
}