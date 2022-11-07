/** @file lr11xx_board.c
 *
 * @brief 
 * 
 * @par
 * COPYRIGHT NOTICE: (c) 2022 Irnas. All rights reserved.
 */

#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <device.h>
#include <zephyr.h>
#include <string.h>

#include "lr11xx_board.h"
#include "lr11xx_hal_context.h"
#include "lr11xx_system_types.h"
#include "lr11xx_radio_types.h"


/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define DT_DRV_COMPAT irnas_lr11xx

#define LR11XX_SPI_OPERATION (SPI_WORD_SET(8) |			\
				SPI_OP_MODE_MASTER |			\
				SPI_TRANSFER_MSB)

#define BOARD_TCXO_WAKEUP_TIME     (5)

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

static const lr11xx_radio_rssi_calibration_table_t rssi_calibration_table_below_600mhz = {
    .gain_offset = 0,
    .gain_tune   = { .g4     = 12,
                     .g5     = 12,
                     .g6     = 14,
                     .g7     = 0,
                     .g8     = 1,
                     .g9     = 3,
                     .g10    = 4,
                     .g11    = 4,
                     .g12    = 3,
                     .g13    = 6,
                     .g13hp1 = 6,
                     .g13hp2 = 6,
                     .g13hp3 = 6,
                     .g13hp4 = 6,
                     .g13hp5 = 6,
                     .g13hp6 = 6,
                     .g13hp7 = 6 },
};

static const lr11xx_radio_rssi_calibration_table_t rssi_calibration_table_from_600mhz_to_2ghz = {
    .gain_offset = 0,
    .gain_tune   = { .g4     = 2,
                     .g5     = 2,
                     .g6     = 2,
                     .g7     = 3,
                     .g8     = 3,
                     .g9     = 4,
                     .g10    = 5,
                     .g11    = 4,
                     .g12    = 4,
                     .g13    = 6,
                     .g13hp1 = 5,
                     .g13hp2 = 5,
                     .g13hp3 = 6,
                     .g13hp4 = 6,
                     .g13hp5 = 6,
                     .g13hp6 = 7,
                     .g13hp7 = 6 },
};

static const lr11xx_radio_rssi_calibration_table_t rssi_calibration_table_above_2ghz = {
    .gain_offset = 2030,
    .gain_tune   = { .g4     = 6,
                     .g5     = 7,
                     .g6     = 6,
                     .g7     = 4,
                     .g8     = 3,
                     .g9     = 4,
                     .g10    = 14,
                     .g11    = 12,
                     .g12    = 14,
                     .g13    = 12,
                     .g13hp1 = 12,
                     .g13hp2 = 12,
                     .g13hp3 = 12,
                     .g13hp4 = 8,
                     .g13hp5 = 8,
                     .g13hp6 = 9,
                     .g13hp7 = 9 },
};

/**
 * @brief DIO5 - DIO8 pins
 */
enum lr11xx_dio_pin_e
{
    LR11XX_DIO5 = 5,
    LR11XX_DIO6 = 6,
    LR11XX_DIO7 = 7,
    LR11XX_DIO8 = 8,
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
static lr11xx_hal_context_t lr11xx_context = {
    .spi = SPI_DT_SPEC_INST_GET(0, LR11XX_SPI_OPERATION, 0),
    .busy = GPIO_DT_SPEC_INST_GET(0, busy_gpios),
    .reset = GPIO_DT_SPEC_INST_GET(0, reset_gpios),   
    .event = GPIO_DT_SPEC_INST_GET(0, event_gpios),
#if DT_INST_NODE_HAS_PROP(0, pwr_en_gpios)
    .pwr_en = GPIO_DT_SPEC_INST_GET(0, pwr_en_gpios),
#endif //DT_INST_NODE_HAS_PROP(inst, pwr_en_gpios)

#if DT_INST_NODE_HAS_PROP(0, gps_lna_en_gpios)
    .lna_en = GPIO_DT_SPEC_INST_GET(0, gps_lna_en_gpios),
#endif //DT_INST_NODE_HAS_PROP(inst, gps_lna_en_gpios)
};

#if DT_INST_NODE_HAS_PROP(0, rf_sw_enable)
static int rf_sw_enable[] = DT_INST_PROP(0, rf_sw_enable);
static size_t rf_sw_enable_len = DT_INST_PROP_LEN(0, rf_sw_enable);
#else
static int rf_sw_enable[];
static size_t rf_sw_enable_len = 0;
#endif //DT_INST_NODE_HAS_PROP(0, rf_sw_enable)

#if DT_INST_NODE_HAS_PROP(0, rf_sw_standby_mode)
static int rf_sw_standby_mode[] = DT_INST_PROP(0, rf_sw_standby_mode);
static size_t rf_sw_standby_mode_len = DT_INST_PROP_LEN(0, rf_sw_standby_mode);
#else
static int *rf_sw_standby_mode;
static size_t rf_sw_standby_mode_len = 0;
#endif //DT_INST_NODE_HAS_PROP(0, rf_sw_standby_mode)

#if DT_INST_NODE_HAS_PROP(0, rf_sw_rx_mode)
static int rf_sw_rx_mode[] = DT_INST_PROP(0, rf_sw_rx_mode);
static size_t rf_sw_rx_mode_len = DT_INST_PROP_LEN(0, rf_sw_rx_mode);
#else
static int *rf_sw_rx_mode;
static size_t rf_sw_rx_mode_len = 0;
#endif //DT_INST_NODE_HAS_PROP(0, rf_sw_rx_mode)

#if DT_INST_NODE_HAS_PROP(0, rf_sw_tx_mode)
static int rf_sw_tx_mode[] = DT_INST_PROP(0, rf_sw_tx_mode);
static size_t rf_sw_tx_mode_len = DT_INST_PROP_LEN(0, rf_sw_tx_mode);
#else
static int *rf_sw_tx_mode;
static size_t rf_sw_tx_mode_len = 0;
#endif //DT_INST_NODE_HAS_PROP(0, rf_sw_tx_mode)

#if DT_INST_NODE_HAS_PROP(0, rf_sw_tx_hp_mode)
static int rf_sw_tx_hp_mode[] = DT_INST_PROP(0, rf_sw_tx_hp_mode);
static size_t rf_sw_tx_hp_mode_len = DT_INST_PROP_LEN(0, rf_sw_tx_hp_mode);
#else
static int *rf_sw_tx_hp_mode;
static size_t rf_sw_tx_hp_mode_len = 0;
#endif //DT_INST_NODE_HAS_PROP(0, rf_sw_tx_hp_mode)

#if DT_INST_NODE_HAS_PROP(0, rf_sw_tx_hf_mode)
static int rf_sw_tx_hf_mode[] = DT_INST_PROP(0, rf_sw_tx_hf_mode);
static size_t rf_sw_tx_hf_mode_len = DT_INST_PROP_LEN(0, rf_sw_tx_hf_mode);
#else
static int *rf_sw_tx_hf_mode;
static size_t rf_sw_tx_hf_mode_len = 0;
#endif //DT_INST_NODE_HAS_PROP(0, rf_sw_tx_hf_mode)

#if DT_INST_NODE_HAS_PROP(0, rf_sw_wifi_mode)
static int rf_sw_wifi_mode[] = DT_INST_PROP(0, rf_sw_wifi_mode);
static size_t rf_sw_wifi_mode_len = DT_INST_PROP_LEN(0, rf_sw_wifi_mode);
#else
static int *rf_sw_wifi_mode;
static size_t rf_sw_wifi_mode_len = 0;
#endif //DT_INST_NODE_HAS_PROP(0, rf_sw_wifi_mode)

#if DT_INST_NODE_HAS_PROP(0, rf_sw_gnss_mode)
static int rf_sw_gnss_mode[] = DT_INST_PROP(0, rf_sw_gnss_mode);
static size_t rf_sw_gnss_mode_len = DT_INST_PROP_LEN(0, rf_sw_gnss_mode);
#else
static int *rf_sw_gnss_mode;
static size_t rf_sw_gnss_mode_len = 0;
#endif //DT_INST_NODE_HAS_PROP(0, rf_sw_gnss_mode)

#else 
static lr11xx_hal_context_t lr11xx_context;
#endif //DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* Function callback that will be used inside event pin interrupt*/
void (*event_interrupt_cb)();

/*!
 * @brief           Creates rf switch settings from given list of strings that
 *                  were fetched from DTS.
 *
 * @param[in] list  List of strings
 * @param[in] len   Length of list
 *
 * @return  rf switch setting
 */
static uint8_t create_rf_sw_setting(int *list, size_t len)
{
    uint8_t rf_sw = 0;

    printk("We got rf len: %d with first entry: %d\n", len, list[0]);

    for (uint8_t i = 0; i < len; i++) {
        if (list[i] == LR11XX_DIO5) {
            rf_sw |= LR11XX_SYSTEM_RFSW0_HIGH;
        }
        else if (list[i] == LR11XX_DIO6) {
            rf_sw |= LR11XX_SYSTEM_RFSW1_HIGH;
        }
        else if (list[i] == LR11XX_DIO7) {
            rf_sw |= LR11XX_SYSTEM_RFSW2_HIGH;
        }
        else if (list[i] == LR11XX_DIO8) {
            rf_sw |= LR11XX_SYSTEM_RFSW3_HIGH;
        }
        else {
            rf_sw = 0;
        }
    }
    return rf_sw;
}

static void lr11xx_board_event_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    lr11xx_hal_context_t* lr11xx_context = CONTAINER_OF(cb, lr11xx_hal_context_t, event_cb);

    if ((pins & BIT(lr11xx_context->event.pin)) == 0U) 
    {
		return;
	}
    
    if (gpio_pin_get_dt(&lr11xx_context->event))
    {
        /* Wait for value to drop */
        gpio_pin_interrupt_configure_dt(&lr11xx_context->event, GPIO_INT_LEVEL_INACTIVE);
        /* Call provided callback */
        if (event_interrupt_cb != NULL)
        {
            event_interrupt_cb();
        }
    }
    else
    {
        gpio_pin_interrupt_configure_dt(&lr11xx_context->event, GPIO_INT_LEVEL_ACTIVE);
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr11xx_hal_context_t *lr11xx_board_context_get(void)
{
    return &lr11xx_context;
}

void lr11xx_board_context_init(lr11xx_hal_context_t * context)
{

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
	context->rf_switch_cfg.enable  = create_rf_sw_setting(rf_sw_enable, rf_sw_enable_len);
	context->rf_switch_cfg.standby = create_rf_sw_setting(rf_sw_standby_mode, rf_sw_standby_mode_len);
	context->rf_switch_cfg.rx      = create_rf_sw_setting(rf_sw_rx_mode, rf_sw_rx_mode_len);
	context->rf_switch_cfg.tx      = create_rf_sw_setting(rf_sw_tx_mode, rf_sw_tx_mode_len);
	context->rf_switch_cfg.tx_hp   = create_rf_sw_setting(rf_sw_tx_hp_mode, rf_sw_tx_hp_mode_len);
	context->rf_switch_cfg.tx_hf   = create_rf_sw_setting(rf_sw_tx_hf_mode, rf_sw_tx_hf_mode_len);
	context->rf_switch_cfg.wifi    = create_rf_sw_setting(rf_sw_wifi_mode, rf_sw_wifi_mode_len);
	context->rf_switch_cfg.gnss    = create_rf_sw_setting(rf_sw_gnss_mode, rf_sw_gnss_mode_len);
#endif

    // Busy pin
    gpio_pin_configure_dt(&context->busy, GPIO_INPUT);
    // Reset pin
    gpio_pin_configure_dt(&context->reset, GPIO_OUTPUT_INACTIVE);
    
#if DT_INST_NODE_HAS_PROP(0, pwr_en_gpios)
    gpio_pin_configure_dt(&context->reset, GPIO_OUTPUT_ACTIVE);
#endif //DT_INST_NODE_HAS_PROP(inst, pwr_en_gpios)

#if DT_INST_NODE_HAS_PROP(0, gps_lna_en_gpios)
    gpio_pin_configure_dt(&context->reset, GPIO_OUTPUT_INACTIVE);
#endif //DT_INST_NODE_HAS_PROP(inst, gps_lna_en_gpios)

    // Event pin
    gpio_pin_configure_dt(&context->event, GPIO_INPUT);

    // Configure interrupt
    //gpio_pin_interrupt_configure_dt(&context->event, GPIO_INT_LEVEL_ACTIVE); 
    // Init callback
    gpio_init_callback(&context->event_cb, lr11xx_board_event_callback, BIT(context->event.pin));
    // Add callback
    gpio_add_callback(context->event.port, &context->event_cb); 
    
    //Copy function pointer from context to internal function pointer variable
    if (context->event_interrupt_cb != NULL) {
        event_interrupt_cb = context->event_interrupt_cb;
    }
    else {
        event_interrupt_cb = NULL;
    } 
}

void lr11xx_board_enable_interrupt(lr11xx_hal_context_t * context)
{
    // Configure interrupt
    gpio_pin_interrupt_configure_dt(&context->event, GPIO_INT_LEVEL_ACTIVE); 
}

void lr11xx_board_disable_interrupt(lr11xx_hal_context_t * context)
{
    // Configure interrupt
    gpio_pin_interrupt_configure_dt(&context->event, GPIO_INT_DISABLE); 
}

lr11xx_system_reg_mode_t lr11xx_board_get_reg_mode( void )
{
    return LR11XX_SYSTEM_REG_MODE_DCDC;
}

lr11xx_board_tcxo_cfg_t lr11xx_board_get_tcxo_cfg( void )
{
    return ( lr11xx_board_tcxo_cfg_t ){
        .has_tcxo   = true,
        .supply     = LR11XX_SYSTEM_TCXO_CTRL_1_8V,
        .timeout_ms = BOARD_TCXO_WAKEUP_TIME,
    };
}

lr11xx_board_lf_clck_cfg_t lr11xx_board_get_lf_clk_cfg( void )
{
    return ( lr11xx_board_lf_clck_cfg_t ){
        .lf_clk_cfg     = LR11XX_SYSTEM_LFCLK_RC,
        .wait_32k_ready = true,
    };
}

const lr11xx_radio_rssi_calibration_table_t* lr11xx_board_get_rssi_calibration_table( const uint32_t freq_in_hz )
{
    if( freq_in_hz < 600000000 )
    {
        return &rssi_calibration_table_below_600mhz;
    }
    else if( ( 600000000 <= freq_in_hz ) && ( freq_in_hz <= 2000000000 ) )
    {
        return &rssi_calibration_table_from_600mhz_to_2ghz;
    }
    else
    {
        return &rssi_calibration_table_above_2ghz;
    }
}