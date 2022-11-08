/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <zephyr.h>
#include <device.h>
#include <zephyr/types.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>

#include "apps_common.h"
#include "lr11xx_regmem.h"
#include "lr11xx_system.h"
#include "lr11xx_radio.h"
#include "lr11xx_radio_types_str.h"
#include "lr11xx_board.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(lr11xx_common, CONFIG_LR11XX_LOG_LEVEL);

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

static volatile bool irq_fired = false;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Print the common configuration on the debug interface
 */
void print_common_configuration( void );

/*!
 * @brief Print the LoRa configuration on the debug interface
 */
void print_lora_configuration( void );

/*!
 * @brief Print the GFSK configuration on the debug interface
 */
void print_gfsk_configuration( void );

void radio_on_dio_irq( void );
void on_tx_done( void ) __attribute__( ( weak ) );
void on_rx_done( void ) __attribute__( ( weak ) );
void on_rx_timeout( void ) __attribute__( ( weak ) );
void on_preamble_detected( void ) __attribute__( ( weak ) );
void on_syncword_header_valid( void ) __attribute__( ( weak ) );
void on_header_error( void ) __attribute__( ( weak ) );
void on_fsk_len_error( void ) __attribute__( ( weak ) );
void on_rx_crc_error( void ) __attribute__( ( weak ) );
void on_cad_done_undetected( void ) __attribute__( ( weak ) );
void on_cad_done_detected( void ) __attribute__( ( weak ) );
void on_wifi_scan_done( void ) __attribute__( ( weak ) );
void on_gnss_scan_done( void ) __attribute__( ( weak ) );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC VARIABLES --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/* Set interrupt callback - EvaTODO*/
// context->event_interrupt_cb = radio_on_dio_irq;

void apps_common_lr11xx_system_init( const struct device* context )
{
    int ret = 0;
    const struct lr11xx_hal_context_cfg_t *config = context->config;

    LOG_INF("Reset system");
    ret = lr11xx_system_reset( ( void* ) context );
    if(ret)
    {
        LOG_ERR("System reset failed.");
    }

    // Configure the regulator
    const lr11xx_system_reg_mode_t regulator = lr11xx_board_get_reg_mode();
    ret = lr11xx_system_set_reg_mode( ( void* ) context, regulator );
    if(ret)
    {
        LOG_ERR("Failed to config regulator.");
    }

    // Configure RF switch
    const lr11xx_system_rfswitch_cfg_t rf_switch_setup = config->rf_switch_cfg;
    ret = lr11xx_system_set_dio_as_rf_switch( context, &rf_switch_setup );
    if(ret)
    {
        LOG_ERR("Failed to config rf switch.");
    }

    // Configure the 32MHz TCXO if it is available on the board
    const struct lr11xx_hal_context_tcxo_cfg_t tcxo_cfg; // EvaTODO - get from device structure = lr11xx_board_get_tcxo_cfg( );
    if( tcxo_cfg.has_tcxo == true )
    {
        const uint32_t timeout_rtc_step = lr11xx_radio_convert_time_in_ms_to_rtc_step( tcxo_cfg.timeout_ms );
        ret = lr11xx_system_set_tcxo_mode( context, tcxo_cfg.supply, timeout_rtc_step );
        if(ret)
        {
            LOG_ERR("Failed to configure TCXO.");
        }
    }

    // Configure the Low Frequency Clock
    const struct lr11xx_hal_context_lf_clck_cfg_t lf_clk_cfg; // EvaTODO - get from device structure = lr11xx_board_get_lf_clk_cfg( );
    ret = lr11xx_system_cfg_lfclk( context, lf_clk_cfg.lf_clk_cfg, lf_clk_cfg.wait_32k_ready );
    if(ret)
    {
        LOG_ERR("Failed to configure Configure the Low Frequency Clock.");
    }

    ret = lr11xx_system_clear_errors( context );
    if(ret)
    {
        LOG_ERR("Failed to clear errors.");
    }

    ret = lr11xx_system_calibrate( context, 0x3F );
    if(ret)
    {
        LOG_ERR("Failed to calibrate.");
    }

    uint16_t errors;
    ret = lr11xx_system_get_errors( context, &errors );
    if(ret)
    {
        LOG_ERR("Failed to get errors.");
    }
    LOG_INF("LR init errors: %d", errors);

    ret = lr11xx_system_clear_errors( context );
    if(ret)
    {
        LOG_ERR("Failed to clear errors.");
    }

    ret = lr11xx_system_clear_irq_status( context, LR11XX_SYSTEM_IRQ_ALL_MASK );
    if(ret)
    {
        LOG_ERR("Failed to clear irq status.");
    }
}

void apps_common_lr11xx_fetch_and_print_version( const struct device* context )
{
    lr11xx_system_version_t version;

    lr11xx_system_get_version( ( void* ) context, &version );

    LOG_INF( "LR11xx information:" );
    LOG_INF( "  - Firmware = 0x%04X", version.fw );
    LOG_INF( "  - Hardware = 0x%02X", version.hw );
    LOG_INF( "  - Type     = 0x%02X (0x01 for LR1110, 0x02 for LR1120)", version.type );
}

void apps_common_lr11xx_radio_init( const void* context )
{
    int ret;
    
    const lr11xx_board_pa_pwr_cfg_t* pa_pwr_cfg = lr11xx_board_get_pa_pwr_cfg( RF_FREQ_IN_HZ, TX_OUTPUT_POWER_DBM );
    if( pa_pwr_cfg == NULL )
    {
        LOG_ERR( "Invalid target frequency or power level" );
        while( true )
        {
        }
    }
    
    print_common_configuration( );
    
    ret = lr11xx_radio_set_pkt_type( context, PACKET_TYPE );
    if(ret)
    {
        LOG_ERR("Failed pkt pkt type.");
    }

    ret = lr11xx_radio_set_rf_freq( context, RF_FREQ_IN_HZ );
    if(ret)
    {
        LOG_ERR("Failed set RF freq.");
    }

    ret = lr11xx_radio_set_rssi_calibration( context, lr11xx_board_get_rssi_calibration_table( RF_FREQ_IN_HZ ) );
    if(ret)
    {
        LOG_ERR("Failed set RSSI calibration.");
    }

    ret = lr11xx_radio_set_pa_cfg( context, &( pa_pwr_cfg->pa_config ) );
    if(ret)
    {
        LOG_ERR("Failed set PA config.");
    }
    ret = lr11xx_radio_set_tx_params( context, pa_pwr_cfg->power, PA_RAMP_TIME );
    if(ret)
    {
        LOG_ERR("Failed set RSSI calibration.");
    }

    ret = lr11xx_radio_set_rx_tx_fallback_mode( context, FALLBACK_MODE );
    if(ret)
    {
        LOG_ERR("Failed set fallback mode.");
    }
    ret = lr11xx_radio_cfg_rx_boosted( context, ENABLE_RX_BOOST_MODE );
    if(ret)
    {
        LOG_ERR("Failed set rx boosted.");
    }

    if( PACKET_TYPE == LR11XX_RADIO_PKT_TYPE_LORA )
    {
        print_lora_configuration( );

        const lr11xx_radio_mod_params_lora_t lora_mod_params = {
            .sf   = LORA_SPREADING_FACTOR,
            .bw   = LORA_BANDWIDTH,
            .cr   = LORA_CODING_RATE,
            .ldro = apps_common_compute_lora_ldro( LORA_SPREADING_FACTOR, LORA_BANDWIDTH ),
        };

        const lr11xx_radio_pkt_params_lora_t lora_pkt_params = {
            .preamble_len_in_symb = LORA_PREAMBLE_LENGTH,
            .header_type          = LORA_PKT_LEN_MODE,
            .pld_len_in_bytes     = PAYLOAD_LENGTH,
            .crc                  = LORA_CRC,
            .iq                   = LORA_IQ,
        };

        ret = lr11xx_radio_set_lora_mod_params( context, &lora_mod_params);
        if(ret)
        {
            LOG_ERR("Failed set lora mod params.");
        }
        ret = lr11xx_radio_set_lora_pkt_params( context, &lora_pkt_params );
        if(ret)
        {
            LOG_ERR("Failed set lora pkt params.");
        }
        ret = lr11xx_radio_set_lora_sync_word( context, LORA_SYNCWORD );
        if(ret)
        {
            LOG_ERR("Failed set lora sync word.");
        }
    }
    else if( PACKET_TYPE == LR11XX_RADIO_PKT_TYPE_GFSK )
    {
        print_gfsk_configuration( );

        const lr11xx_radio_mod_params_gfsk_t gfsk_mod_params = {
            .br_in_bps    = FSK_BITRATE,
            .pulse_shape  = FSK_PULSE_SHAPE,
            .bw_dsb_param = FSK_BANDWIDTH,
            .fdev_in_hz   = FSK_FDEV,
        };

        const lr11xx_radio_pkt_params_gfsk_t gfsk_pkt_params = {
            .preamble_len_in_bits  = FSK_PREAMBLE_LENGTH,
            .preamble_detector     = FSK_PREAMBLE_DETECTOR,
            .sync_word_len_in_bits = FSK_SYNCWORD_LENGTH,
            .address_filtering     = FSK_ADDRESS_FILTERING,
            .header_type           = FSK_HEADER_TYPE,
            .pld_len_in_bytes      = PAYLOAD_LENGTH,
            .crc_type              = FSK_CRC_TYPE,
            .dc_free               = FSK_DC_FREE,
        };

        ret = lr11xx_radio_set_gfsk_mod_params( context, &gfsk_mod_params );
        if(ret)
        {
            LOG_ERR("Failed set gfsk mod params.");
        }
        ret = lr11xx_radio_set_gfsk_pkt_params( context, &gfsk_pkt_params );
        if(ret)
        {
            LOG_ERR("Failed set gfck pkt params.");
        }
        ret = lr11xx_radio_set_gfsk_sync_word( context, gfsk_sync_word );
        if(ret)
        {
            LOG_ERR("Failed set gfsk sync word.");
        }

        if( FSK_DC_FREE != LR11XX_RADIO_GFSK_DC_FREE_OFF )
        {
            ret = lr11xx_radio_set_gfsk_whitening_seed( context, FSK_WHITENING_SEED );
            if(ret)
            {
                LOG_ERR("Failed set gfsk whitening seed.");
            }
        }

        if( FSK_CRC_TYPE != LR11XX_RADIO_GFSK_CRC_OFF )
        {
            ret = lr11xx_radio_set_gfsk_crc_params( context, FSK_CRC_SEED, FSK_CRC_POLYNOMIAL );
            if(ret)
            {
                LOG_ERR("Failed set gfsk crc params.");
            }
        }

        if( FSK_ADDRESS_FILTERING != LR11XX_RADIO_GFSK_ADDRESS_FILTERING_DISABLE )
        {
            ret = lr11xx_radio_set_pkt_address( context, FSK_NODE_ADDRESS, FSK_BROADCAST_ADDRESS );
            if(ret)
            {
                LOG_ERR("Failed set gfsk pkt address.");
            }
        }
    }
}

void apps_common_lr11xx_receive( const void* context, uint8_t* buffer, uint8_t* size )
{
    lr11xx_radio_rx_buffer_status_t rx_buffer_status;
    lr11xx_radio_pkt_status_lora_t  pkt_status_lora;
    lr11xx_radio_pkt_status_gfsk_t  pkt_status_gfsk;

    lr11xx_radio_get_rx_buffer_status( context, &rx_buffer_status );
    lr11xx_regmem_read_buffer8( context, buffer, rx_buffer_status.buffer_start_pointer,
                                rx_buffer_status.pld_len_in_bytes );
    *size = rx_buffer_status.pld_len_in_bytes;

    LOG_INF( "Packet content: ");
    for(uint8_t i=0; i < *size; i++)
    {
        LOG_INF("%x", buffer[i]);
    }

    LOG_INF( "Packet status:" );
    if( PACKET_TYPE == LR11XX_RADIO_PKT_TYPE_LORA )
    {
        lr11xx_radio_get_lora_pkt_status( context, &pkt_status_lora );
        LOG_INF( "  - RSSI packet = %i dBm", pkt_status_lora.rssi_pkt_in_dbm );
        LOG_INF( "  - Signal RSSI packet = %i dBm", pkt_status_lora.signal_rssi_pkt_in_dbm );
        LOG_INF( "  - SNR packet = %i dB", pkt_status_lora.snr_pkt_in_db );
    }
    else if( PACKET_TYPE == LR11XX_RADIO_PKT_TYPE_GFSK )
    {
        lr11xx_radio_get_gfsk_pkt_status( context, &pkt_status_gfsk );
        LOG_INF( "  - RSSI average = %i dBm", pkt_status_gfsk.rssi_avg_in_dbm );
        LOG_INF( "  - RSSI sync = %i dBm", pkt_status_gfsk.rssi_sync_in_dbm );
    }
}

void apps_common_lr11xx_irq_process( const void* context, lr11xx_system_irq_mask_t irq_filter_mask )
{
    if( irq_fired == true )
    {
        irq_fired = false;

        lr11xx_system_irq_mask_t irq_regs;
        lr11xx_system_get_and_clear_irq_status( context, &irq_regs );

        LOG_DBG( "Interrupt flags = 0x%08X", irq_regs );

        irq_regs &= irq_filter_mask;

        LOG_DBG( "Interrupt flags (after filtering) = 0x%08X", irq_regs );

        if( ( irq_regs & LR11XX_SYSTEM_IRQ_TX_DONE ) == LR11XX_SYSTEM_IRQ_TX_DONE )
        {
            LOG_INF( "Tx done" );
            on_tx_done( );
        }

        if( ( irq_regs & LR11XX_SYSTEM_IRQ_PREAMBLE_DETECTED ) == LR11XX_SYSTEM_IRQ_PREAMBLE_DETECTED )
        {
            LOG_INF( "Preamble detected" );
            on_preamble_detected( );
        }

        if( ( irq_regs & LR11XX_SYSTEM_IRQ_HEADER_ERROR ) == LR11XX_SYSTEM_IRQ_HEADER_ERROR )
        {
            LOG_INF( "Header error" );
            on_header_error( );
        }

        if( ( irq_regs & LR11XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID ) == LR11XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID )
        {
            LOG_INF( "Syncword or header valid" );
            on_syncword_header_valid( );
        }

        if( ( irq_regs & LR11XX_SYSTEM_IRQ_RX_DONE ) == LR11XX_SYSTEM_IRQ_RX_DONE )
        {
            if( ( irq_regs & LR11XX_SYSTEM_IRQ_CRC_ERROR ) == LR11XX_SYSTEM_IRQ_CRC_ERROR )
            {
                LOG_INF( "CRC error" );
                on_rx_crc_error( );
            }
            else if( ( irq_regs & LR11XX_SYSTEM_IRQ_FSK_LEN_ERROR ) == LR11XX_SYSTEM_IRQ_FSK_LEN_ERROR )
            {
                LOG_INF( "FSK length error" );
                on_fsk_len_error( );
            }
            else
            {
                LOG_INF( "Rx done" );
                on_rx_done( );
            }
        }

        if( ( irq_regs & LR11XX_SYSTEM_IRQ_CAD_DONE ) == LR11XX_SYSTEM_IRQ_CAD_DONE )
        {
            LOG_INF( "CAD done" );
            if( ( irq_regs & LR11XX_SYSTEM_IRQ_CAD_DETECTED ) == LR11XX_SYSTEM_IRQ_CAD_DETECTED )
            {
                LOG_INF( "Channel activity detected" );
                on_cad_done_detected( );
            }
            else
            {
                LOG_INF( "No channel activity detected" );
                on_cad_done_undetected( );
            }
        }

        if( ( irq_regs & LR11XX_SYSTEM_IRQ_TIMEOUT ) == LR11XX_SYSTEM_IRQ_TIMEOUT )
        {
            LOG_INF( "Rx timeout" );
            on_rx_timeout( );
        }

        if( ( irq_regs & LR11XX_SYSTEM_IRQ_WIFI_SCAN_DONE ) == LR11XX_SYSTEM_IRQ_WIFI_SCAN_DONE )
        {
            LOG_INF( "Wi-Fi scan done" );
            on_wifi_scan_done( );
        }

        if( ( irq_regs & LR11XX_SYSTEM_IRQ_GNSS_SCAN_DONE ) == LR11XX_SYSTEM_IRQ_GNSS_SCAN_DONE )
        {
            LOG_INF( "GNSS scan done" );
            on_gnss_scan_done( );
        }

        LOG_INF( " " );
    }
}


/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void print_common_configuration( void )
{
    LOG_INF( "Common parameters:" );
    LOG_INF( "   Packet type   = %s", lr11xx_radio_pkt_type_to_str( PACKET_TYPE ) );
    LOG_INF( "   RF frequency  = %u Hz", RF_FREQ_IN_HZ );
    LOG_INF( "   Output power  = %i dBm", TX_OUTPUT_POWER_DBM );
    LOG_INF( "   Fallback mode = %s", lr11xx_radio_fallback_modes_to_str( FALLBACK_MODE ) );
    LOG_INF( "   Rx boost activated: %d", ENABLE_RX_BOOST_MODE);
    LOG_INF( " " );
}

void print_lora_configuration( void )
{
    LOG_INF( "LoRa modulation parameters:" );
    LOG_INF( "   Spreading factor = %s", lr11xx_radio_lora_sf_to_str( LORA_SPREADING_FACTOR ) );
    LOG_INF( "   Bandwidth        = %s", lr11xx_radio_lora_bw_to_str( LORA_BANDWIDTH ) );
    LOG_INF( "   Coding rate      = %s", lr11xx_radio_lora_cr_to_str( LORA_CODING_RATE ) );
    LOG_INF( " " );

    LOG_INF( "LoRa packet parameters:" );
    LOG_INF( "   Preamble length = %d symbol(s)", LORA_PREAMBLE_LENGTH );
    LOG_INF( "   Header mode     = %s", lr11xx_radio_lora_pkt_len_modes_to_str( LORA_PKT_LEN_MODE ) );
    LOG_INF( "   Payload length  = %d byte(s)", PAYLOAD_LENGTH );
    LOG_INF( "   CRC mode        = %s", lr11xx_radio_lora_crc_to_str( LORA_CRC ) );
    LOG_INF( "   IQ              = %s", lr11xx_radio_lora_iq_to_str( LORA_IQ ) );
    LOG_INF( " " );

    LOG_INF( "LoRa syncword = 0x%02X", LORA_SYNCWORD );
    LOG_INF( " " );
}

void print_gfsk_configuration( void )
{
    LOG_INF( "GFSK modulation parameters:" );
    LOG_INF( "   Bitrate             = %u bps", FSK_BITRATE );
    LOG_INF( "   Pulse shape         = %s", lr11xx_radio_gfsk_pulse_shape_to_str( FSK_PULSE_SHAPE ) );
    LOG_INF( "   Bandwidth           = %s", lr11xx_radio_gfsk_bw_to_str( FSK_BANDWIDTH ) );
    LOG_INF( "   Frequency deviation = %u Hz", FSK_FDEV );
    LOG_INF( " " );

    LOG_INF( "GFSK packet parameters:" );
    LOG_INF( "   Preamble length   = %d bit(s)", FSK_PREAMBLE_LENGTH );
    LOG_INF( "   Preamble detector = %s",
                        lr11xx_radio_gfsk_preamble_detector_to_str( FSK_PREAMBLE_DETECTOR ) );
    LOG_INF( "   Syncword length   = %d bit(s)", FSK_SYNCWORD_LENGTH );
    LOG_INF( "   Address filtering = %s",
                        lr11xx_radio_gfsk_address_filtering_to_str( FSK_ADDRESS_FILTERING ) );
    if( FSK_ADDRESS_FILTERING != LR11XX_RADIO_GFSK_ADDRESS_FILTERING_DISABLE )
    {
        LOG_INF( "     (Node address      = 0x%02X)", FSK_NODE_ADDRESS );
        if( FSK_ADDRESS_FILTERING == LR11XX_RADIO_GFSK_ADDRESS_FILTERING_NODE_AND_BROADCAST_ADDRESSES )
        {
            LOG_INF( "     (Broadcast address = 0x%02X)", FSK_BROADCAST_ADDRESS );
        }
    }
    LOG_INF( "   Header mode       = %s", lr11xx_radio_gfsk_pkt_len_modes_to_str( FSK_HEADER_TYPE ) );
    LOG_INF( "   Payload length    = %d byte(s)", PAYLOAD_LENGTH );
    LOG_INF( "   CRC mode          = %s", lr11xx_radio_gfsk_crc_type_to_str( FSK_CRC_TYPE ) );
    if( FSK_CRC_TYPE != LR11XX_RADIO_GFSK_CRC_OFF )
    {
        LOG_INF( "     (CRC seed       = 0x%08X)", FSK_CRC_SEED );
        LOG_INF( "     (CRC polynomial = 0x%08X)", FSK_CRC_POLYNOMIAL );
    }
    LOG_INF( "   DC free           = %s", lr11xx_radio_gfsk_dc_free_to_str( FSK_DC_FREE ) );
    if( FSK_DC_FREE != LR11XX_RADIO_GFSK_DC_FREE_OFF )
    {
        LOG_INF( "     (Whitening seed = 0x%04X)", FSK_WHITENING_SEED );
    }
    LOG_INF( " " );
}

void radio_on_dio_irq( void )
{
    irq_fired = true;
    LOG_DBG("Irq fired");
}
void on_tx_done( void )
{
    LOG_INF( "No IRQ routine defined" );
}
void on_rx_done( void )
{
    LOG_INF( "No IRQ routine defined" );
}
void on_rx_timeout( void )
{
    LOG_INF( "No IRQ routine defined" );
}
void on_preamble_detected( void )
{
    LOG_INF( "No IRQ routine defined" );
}
void on_syncword_header_valid( void )
{
    LOG_INF( "No IRQ routine defined" );
}
void on_header_error( void )
{
    LOG_INF( "No IRQ routine defined" );
}
void on_fsk_len_error( void )
{
    LOG_INF( "No IRQ routine defined" );
}
void on_rx_crc_error( void )
{
    LOG_INF( "No IRQ routine defined" );
}
void on_cad_done_undetected( void )
{
    LOG_INF( "No IRQ routine defined" );
}
void on_cad_done_detected( void )
{
    LOG_INF( "No IRQ routine defined" );
}
void on_wifi_scan_done( void )
{
    LOG_INF( "No IRQ routine defined" );
}
void on_gnss_scan_done( void )
{
    LOG_INF( "No IRQ routine defined" );
}

/* --- EOF ------------------------------------------------------------------ */
