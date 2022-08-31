/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <zephyr.h>
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

static lr11xx_hal_context_t context;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC VARIABLES --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr11xx_hal_context_t* apps_common_lr11xx_get_context( )
{
    lr11xx_board_context_create(&context);
    lr11xx_board_context_init(&context);
    return &context;
}

void apps_common_lr11xx_system_init( const lr11xx_hal_context_t* context )
{
    int ret = 0;

    
    //ret = lr11xx_system_reset( ( void* ) context );
    if(ret)
    {
        LOG_ERR("System reset failed.");
    }

    // Configure the regulator
    //const lr11xx_system_reg_mode_t regulator = lr11xx_board_get_reg_mode();
    //ret = lr11xx_system_set_reg_mode( ( void* ) context, regulator );
    if(ret)
    {
        LOG_ERR("Failed to config regulator.");
    }

    // Configure RF switch
    //const lr11xx_system_rfswitch_cfg_t rf_switch_setup = context->rf_switch_cfg;
    //ret = lr11xx_system_set_dio_as_rf_switch( context, &rf_switch_setup );
    if(ret)
    {
        LOG_ERR("Failed to config rf switch.");
    }

    // Configure the 32MHz TCXO if it is available on the board
    /*
    const lr11xx_board_tcxo_cfg_t tcxo_cfg = lr11xx_board_get_tcxo_cfg( );
    if( tcxo_cfg.has_tcxo == true )
    {
        const uint32_t timeout_rtc_step = lr11xx_radio_convert_time_in_ms_to_rtc_step( tcxo_cfg.timeout_ms );
        ret = lr11xx_system_set_tcxo_mode( context, tcxo_cfg.supply, timeout_rtc_step );
        if(ret)
        {
            LOG_ERR("Failed to configure TCXO.");
        }
    }
    */

    // Configure the Low Frequency Clock
    //const lr11xx_board_lf_clck_cfg_t lf_clk_cfg = lr11xx_board_get_lf_clk_cfg( );
    //ret = lr11xx_system_cfg_lfclk( context, lf_clk_cfg.lf_clk_cfg, lf_clk_cfg.wait_32k_ready );
    if(ret)
    {
        LOG_ERR("Failed to configure Configure the Low Frequency Clock.");
    }

    //ret = lr11xx_system_clear_errors( context );
    if(ret)
    {
        LOG_ERR("Failed to clear errors.");
    }

    //ret = lr11xx_system_calibrate( context, 0x3F );
    if(ret)
    {
        LOG_ERR("Failed to calibrate.");
    }

    uint16_t errors;
    //ret = lr11xx_system_get_errors( context, &errors );
    if(ret)
    {
        LOG_ERR("Failed to get errors.");
    }
    LOG_INF("LR init errors: %d", errors);

    //ret = lr11xx_system_clear_errors( context );
    if(ret)
    {
        LOG_ERR("Failed to clear errors.");
    }

    //ret = lr11xx_system_clear_irq_status( context, LR11XX_SYSTEM_IRQ_ALL_MASK );
    if(ret)
    {
        LOG_ERR("Failed to clear irq status.");
    }
}

void apps_common_lr11xx_fetch_and_print_version( const lr11xx_hal_context_t* context )
{
    lr11xx_system_version_t version;

    //lr11xx_system_get_version( ( void* ) context, &version );

    LOG_INF( "LR11xx information:" );
    LOG_INF( "  - Firmware = 0x%04X", version.fw );
    LOG_INF( "  - Hardware = 0x%02X", version.hw );
    LOG_INF( "  - Type     = 0x%02X (0x01 for LR1110, 0x02 for LR1120)", version.type );
}

void apps_common_lr11xx_radio_init( const void* context )
{
    int ret;

    //ret = lr11xx_radio_set_pkt_type( context, PACKET_TYPE );
    if(ret)
    {
        LOG_ERR("Failed pkt pkt type.");
    }

    //ret = lr11xx_radio_set_rf_freq( context, RF_FREQ_IN_HZ );
    if(ret)
    {
        LOG_ERR("Failed set RF freq.");
    }

    //ret = lr11xx_radio_set_rssi_calibration( context, lr11xx_board_get_rssi_calibration_table( RF_FREQ_IN_HZ ) );
    if(ret)
    {
        LOG_ERR("Failed set RSSI calibration.");
    }

    

/* Not implemented - do we need out power config?
    const smtc_board_pa_pwr_cfg_t* pa_pwr_cfg = smtc_board_get_pa_pwr_cfg( RF_FREQ_IN_HZ, TX_OUTPUT_POWER_DBM );

    if( pa_pwr_cfg == NULL )
    {
        LOG_ERR( "Invalid target frequency or power level" );
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
*/
/*
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
        //print_lora_configuration( );

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
        ret = lr11xx_radio_set_lora_pkt_params( context, &lora_pkt_params );
        ret = lr11xx_radio_set_lora_sync_word( context, LORA_SYNCWORD );
    }
    else if( PACKET_TYPE == LR11XX_RADIO_PKT_TYPE_GFSK )
    {
        //print_gfsk_configuration( );

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
        ret = lr11xx_radio_set_gfsk_pkt_params( context, &gfsk_pkt_params );
        ret = lr11xx_radio_set_gfsk_sync_word( context, gfsk_sync_word );

        if( FSK_DC_FREE != LR11XX_RADIO_GFSK_DC_FREE_OFF )
        {
            ret = lr11xx_radio_set_gfsk_whitening_seed( context, FSK_WHITENING_SEED );
        }

        if( FSK_CRC_TYPE != LR11XX_RADIO_GFSK_CRC_OFF )
        {
            ret = lr11xx_radio_set_gfsk_crc_params( context, FSK_CRC_SEED, FSK_CRC_POLYNOMIAL );
        }

        if( FSK_ADDRESS_FILTERING != LR11XX_RADIO_GFSK_ADDRESS_FILTERING_DISABLE )
        {
            ret = lr11xx_radio_set_pkt_address( context, FSK_NODE_ADDRESS, FSK_BROADCAST_ADDRESS );
        }
    }

    */
}