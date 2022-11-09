/** @file lr11xx_board.c
 *
 * @brief 
 * 
 * @par
 * COPYRIGHT NOTICE: (c) 2022 Irnas. All rights reserved.
 */

#include <zephyr.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include <device.h>
#include <pm/device.h>
#include <devicetree.h>
#include <string.h>

#include "lr11xx_board.h"
#include "lr11xx_hal_context.h"
#include "lr11xx_system_types.h"
#include "lr11xx_radio_types.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(lr11xx_board);


#define DT_DRV_COMPAT irnas_lr11xx

#define LR11XX_SPI_OPERATION (SPI_WORD_SET(8) |			\
				SPI_OP_MODE_MASTER |			        \
				SPI_TRANSFER_MSB)

#define BOARD_TCXO_WAKEUP_TIME     (5)

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

static void lr11xx_board_event_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    struct lr11xx_hal_context_data_t *data = CONTAINER_OF(cb, struct lr11xx_hal_context_data_t, event_cb);
    const struct lr11xx_hal_context_cfg_t *config = data->lr11xx_dev->config;

    if ((pins & BIT(config->event.pin)) == 0U) 
    {
        return;
	}
    
    if (gpio_pin_get_dt(&config->event))
    {
        /* Wait for value to drop */
        gpio_pin_interrupt_configure_dt(&config->event, GPIO_INT_LEVEL_INACTIVE);
        /* Call provided callback */
#ifdef CONFIG_LR11XX_EVENT_TRIGGER_OWN_THREAD
	    k_sem_give(&data->gpio_sem);
#elif CONFIG_LR11XX_EVENT_TRIGGER_GLOBAL_THREAD
        k_work_submit(&data->work);
#endif 
    }
    else
    {
        gpio_pin_interrupt_configure_dt(&config->event, GPIO_INT_LEVEL_ACTIVE);
    }
}

#ifdef CONFIG_LR11XX_EVENT_TRIGGER_OWN_THREAD
static void lr11xx_thread(struct lr11xx_hal_context_data_t *data)
{
	while (1) {
		k_sem_take(&data->gpio_sem, K_FOREVER);
		if(data->event_interrupt_cb)
        {
            data->event_interrupt_cb(data->lr11xx_dev);
        }
	}
}
#endif /* CONFIG_LR11XX_EVENT_TRIGGER_OWN_THREAD */

#ifdef CONFIG_LR11XX_EVENT_TRIGGER_GLOBAL_THREAD
static void lr11xx_work_cb(struct k_work *work)
{
	struct lr11xx_hal_context_data_t *data = CONTAINER_OF(work, struct lr11xx_hal_context_data_t, work);
	if(data->event_interrupt_cb)
    {
        data->event_interrupt_cb(data->lr11xx_dev);
    }
}
#endif /* CONFIG_LR11XX_EVENT_TRIGGER_GLOBAL_THREAD */

void lr11xx_board_attach_interrupt(const struct device *dev, lr11xx_event_cb_t cb)
{
#ifdef CONFIG_LR11XX_EVENT_TRIGGER
    struct lr11xx_hal_context_data_t *data = dev->data;
    data->event_interrupt_cb = cb;
#else
    LOG_ERR("Event trigger not supported!");
#endif //CONFIG_LR11XX_EVENT_TRIGGER  
}

void lr11xx_board_enable_interrupt(const struct device *dev)
{
#ifdef CONFIG_LR11XX_EVENT_TRIGGER
    const struct lr11xx_hal_context_cfg_t *config = dev->config;
    gpio_pin_interrupt_configure_dt(&config->event, GPIO_INT_LEVEL_ACTIVE);
#else
    LOG_ERR("Event trigger not supported!");
#endif //CONFIG_LR11XX_EVENT_TRIGGER  
}

void lr11xx_board_disable_interrupt(const struct device *dev)
{
#ifdef CONFIG_LR11XX_EVENT_TRIGGER
    const struct lr11xx_hal_context_cfg_t *config = dev->config;
    gpio_pin_interrupt_configure_dt(&config->event, GPIO_INT_DISABLE); 
#else
    LOG_ERR("Event trigger not supported!");
#endif //CONFIG_LR11XX_EVENT_TRIGGER  
}



/**
 * @brief Initialise lr11xx.
 * Initialise all GPIOs and configure interrupt on event pin.
 * 
 * @param dev 
 * @return int 
 */
static int lr11xx_init(const struct device *dev)
{
    const struct lr11xx_hal_context_cfg_t *config = dev->config;
    struct lr11xx_hal_context_data_t *data = dev->data;
    int ret;

    /* Check the SPI device */
	if (!device_is_ready(config->spi.bus) ) {
		LOG_ERR("Could not find SPI device");
		return -EINVAL;
	}
    
    // Busy pin
    ret = gpio_pin_configure_dt(&config->busy, GPIO_INPUT);
    if (ret < 0) {
		LOG_ERR("Could not configure busy gpio");
		return ret;
	}

    // Reset pin
    ret = gpio_pin_configure_dt(&config->reset, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
		LOG_ERR("Could not configure reset gpio");
		return ret;
	}

    // Power enable pin
    if(config->pwr_en.port)
    {
        ret = gpio_pin_configure_dt(&config->pwr_en, GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
            LOG_ERR("Could not configure power enable gpio");
            return ret;
	    }
    }

    // LNA enable pin
    if(config->lna_en.port)
    {
        ret = gpio_pin_configure_dt(&config->lna_en, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_ERR("Could not configure lna enable gpio");
            return ret;
	    }
    }

    // Event pin
    ret = gpio_pin_configure_dt(&config->event, GPIO_INPUT);
    if (ret < 0) {
		LOG_ERR("Could not configure event gpio");
		return ret;
	}

    data->lr11xx_dev = dev;

    //Event pin trigger config
#ifdef CONFIG_LR11XX_EVENT_TRIGGER   
#ifdef CONFIG_LR11XX_EVENT_TRIGGER_GLOBAL_THREAD
    LOG_INF("Event trigger set in global thread.");
    data->work.handler = lr11xx_work_cb;
#elif CONFIG_LR11XX_EVENT_TRIGGER_OWN_THREAD
    LOG_INF("Event trigger set in own thread.");
    k_sem_init(&data->trig_sem, 0, K_SEM_MAX_LIMIT);

	k_thread_create(&data->thread, data->thread_stack,
		       CONFIG_LR11XX_THREAD_STACK_SIZE,
		       (k_thread_entry_t)lr11xx_thread, data,
		       NULL, NULL, K_PRIO_COOP(CONFIG_LR11XX_THREAD_PRIORITY),
		       0, K_NO_WAIT);
#endif //CONFIG_LR11XX_EVENT_TRIGGER_OWN_THREAD
    
    // Init callback
    gpio_init_callback(&data->event_cb, lr11xx_board_event_callback, BIT(config->event.pin));
    // Add callback
    if (gpio_add_callback(config->event.port, &data->event_cb))
    {
        LOG_ERR("Could not set event pin callback");
		return -EIO;
    }
#endif //CONFIG_LR11XX_EVENT_TRIGGER
   return ret;
    
}

#if IS_ENABLED(CONFIG_PM_DEVICE)
static int lr11xx_pm_action(const struct device *dev, enum pm_device_action action)
{
    int ret = 0;
    switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		/* Put the lr11xx into normal operation mode */
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		/* Put the lr11xx into sleep mode */
		break;
	default:
		return -ENOTSUP;
	}

	return ret;
}
#endif //IS_ENABLED(CONFIG_PM_DEVICE)

/*
 * Device creation macro.
 */
#define LR11XX_DEVICE_INIT(inst)                                  \
    DEVICE_DT_INST_DEFINE(inst,                                   \
        lr11xx_init,                                              \
        PM_DEVICE_DT_INST_GET(inst),                              \
        &lr11xx_data_##inst,                                      \
        &lr11xx_config_##inst,                                    \
        POST_KERNEL,                                              \
        CONFIG_LR11XX_INIT_PRIORITY,                              \
        NULL);

// EvaTODO move to DT values
#define LR11XX_CFG_TCXO(inst)                                     \
    .tcxo_cfg = {                                                 \
        .has_tcxo   = true,                                       \
        .supply     = LR11XX_SYSTEM_TCXO_CTRL_1_8V,               \
        .timeout_ms = BOARD_TCXO_WAKEUP_TIME,                     \
    },

#define LR11XX_CFG_PWR_EN(inst)                                   \
	.pwr_en = GPIO_DT_SPEC_INST_GET(inst, pwr_en_gpios),		  

#define LR11XX_CFG_LNA_EN(inst)                                   \
	.lna_en = GPIO_DT_SPEC_INST_GET(inst, gps_lna_en_gpios),

#define LR11XX_CFG_RF_SW_EN(inst)                                 \
    .enable = DT_INST_PROP(inst, rf_sw_enable),                  

#define LR11XX_CFG_RF_STANDBY_MODE(inst)                          \
    .standby = DT_INST_PROP(inst, rf_sw_standby_mode), 

#define LR11XX_CFG_RF_SW_RX_MODE(inst)                            \
    .rx = DT_INST_PROP(inst, rf_sw_rx_mode),

#define LR11XX_CFG_RF_SW_TX_MODE(inst)                            \
    .tx = DT_INST_PROP(inst, rf_sw_tx_mode),

#define LR11XX_CFG_RF_SW_TX_HP_MODE(inst)                         \
    .tx_hp = DT_INST_PROP(inst, rf_sw_tx_hp_mode),

#define LR11XX_CFG_RF_SW_TX_HF_MODE(inst)                         \
    .tx_hf = DT_INST_PROP(inst, rf_sw_tx_hf_mode),

#define LR11XX_CFG_RF_SW_WIFI_MODE(inst)                          \
    .wifi = DT_INST_PROP(inst, rf_sw_wifi_mode),

#define LR11XX_CFG_RF_SW_GNSS_MODE(inst)                          \
    .gnss = DT_INST_PROP(inst, rf_sw_gnss_mode),

#define LR11XX_CFG_RF_SW(inst)                                    \
    .rf_switch_cfg =                                              \
    {                                                             \
        COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, rf_sw_enable),	  \
			(LR11XX_CFG_RF_SW_EN(inst)), ())			          \
        COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, rf_sw_standby_mode),	  \
			(LR11XX_CFG_RF_STANDBY_MODE(inst)), ())			      \
        COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, rf_sw_rx_mode),	  \
			(LR11XX_CFG_RF_SW_RX_MODE(inst)), ())    	          \
        COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, rf_sw_tx_mode),	  \
			(LR11XX_CFG_RF_SW_TX_MODE(inst)), ())    	          \
        COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, rf_sw_tx_hp_mode),	  \
			(LR11XX_CFG_RF_SW_TX_HP_MODE(inst)), ())    	          \
        COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, rf_sw_tx_hf_mode),	  \
			(LR11XX_CFG_RF_SW_TX_HF_MODE(inst)), ())    	          \
        COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, rf_sw_wifi_mode),	  \
			(LR11XX_CFG_RF_SW_WIFI_MODE(inst)), ())    	          \
        COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, rf_sw_gnss_mode), \
			(LR11XX_CFG_RF_SW_GNSS_MODE(inst)), ())    	          \
    }

// EvaTODO move to DT values
#define LR11XX_CFG_LF_CLCK(inst)                                  \
    .lf_clck_cfg = {                                              \
        .lf_clk_cfg     = LR11XX_SYSTEM_LFCLK_RC,                 \
        .wait_32k_ready = true,                                   \
    },

#define LR11XX_CONFIG(inst)                                       \
    {                                                             \
    .spi = SPI_DT_SPEC_INST_GET(inst, LR11XX_SPI_OPERATION, 0),   \
    .busy = GPIO_DT_SPEC_INST_GET(inst, busy_gpios),              \
    .reset = GPIO_DT_SPEC_INST_GET(inst, reset_gpios),            \
    .event = GPIO_DT_SPEC_INST_GET(inst, event_gpios),            \
    .reg_mode = LR11XX_SYSTEM_REG_MODE_DCDC,                      \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, pwr_en_gpios),	      \
			(LR11XX_CFG_PWR_EN(inst)), ())			              \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, gps_lna_en_gpios),    \
			(LR11XX_CFG_LNA_EN(inst)), ())			              \
    LR11XX_CFG_TCXO(inst)                                         \
    LR11XX_CFG_LF_CLCK(inst)                                      \
    LR11XX_CFG_RF_SW(inst)                                        \
    }

#define LR11XX_DEFINE(inst)                                       \
    static struct lr11xx_hal_context_data_t lr11xx_data_##inst;   \
    static const struct lr11xx_hal_context_cfg_t lr11xx_config_##inst =  \
    LR11XX_CONFIG(inst);                                          \
    PM_DEVICE_DT_INST_DEFINE(inst, lr11xx_pm_action);             \
    LR11XX_DEVICE_INIT(inst)                                      

DT_INST_FOREACH_STATUS_OKAY(LR11XX_DEFINE)