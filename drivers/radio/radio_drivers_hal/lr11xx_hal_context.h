/** @file lr11xx_hal_context.h
 *
 * @brief 
 * 
 * @par
 * COPYRIGHT NOTICE: (c) 2022 Irnas. All rights reserved.
 */


#ifndef LR11XX_HAL_CONTEXT_H
#define LR11XX_HAL_CONTEXT_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <zephyr.h>
#include <zephyr/types.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>

#include "lr11xx_system_types.h"


/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/**
 * @brief Callback upon firing event trigger
 * 
 */
typedef void (*lr11xx_event_cb_t)(const struct device *dev);

struct lr11xx_hal_context_tcxo_cfg_t
{
    bool                                has_tcxo;
    lr11xx_system_tcxo_supply_voltage_t supply;
    uint32_t                            timeout_ms;
};

struct lr11xx_hal_context_lf_clck_cfg_t
{
    lr11xx_system_lfclk_cfg_t lf_clk_cfg;
    bool                      wait_32k_ready;
};

/**
 * @brief lr11xx context device config structure
 * 
 */
struct lr11xx_hal_context_cfg_t
{
    struct spi_dt_spec spi;      /* spi peripheral */

    struct gpio_dt_spec busy;    /* busy pin */
    struct gpio_dt_spec reset;   /* reset pin */
    struct gpio_dt_spec event;   /* event pin */
    struct gpio_dt_spec pwr_en;  /* enable pin */
    struct gpio_dt_spec lna_en;  /* lna pin */

    lr11xx_system_rfswitch_cfg_t rf_switch_cfg;  /* RF switch config */
    struct lr11xx_hal_context_tcxo_cfg_t tcxo_cfg;
    struct lr11xx_hal_context_lf_clck_cfg_t lf_clck_cfg;
    lr11xx_system_reg_mode_t reg_mode;
};

struct lr11xx_hal_context_data_t
{
#ifdef CONFIG_LR11XX_EVENT_TRIGGER
    const struct device *lr11xx_dev; 
    struct gpio_callback event_cb;          /* event callback structure */
    lr11xx_event_cb_t event_interrupt_cb;   /* event interrupt provided callback */
#ifdef CONFIG_LR11XX_EVENT_TRIGGER_GLOBAL_THREAD
    struct k_work work;
#endif //LR11XX_EVENT_TRIGGER_GLOBAL_THREAD
#ifdef CONFIG_LR11XX_EVENT_TRIGGER_OWN_THREAD
    K_THREAD_STACK_MEMBER(thread_stack, CONFIG_LR11XX_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem trig_sem;
#endif //CONFIG_LR11XX_EVENT_TRIGGER_OWN_THREAD
#endif //CONFIG_LR11XX_EVENT_TRIGGER
};

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */


#ifdef __cplusplus
}
#endif

#endif  // LR11XX_HAL_CONTEXT_H
