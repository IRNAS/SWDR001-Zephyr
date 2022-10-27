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

typedef void (*lr11xx_event_cb_t)(void);

/**
 * @brief lr11xx context structure
 * 
 */
typedef struct
{
    struct spi_dt_spec spi;      /* spi peripheral */

    struct gpio_dt_spec busy;    /* busy pin */
    struct gpio_dt_spec reset;   /* reset pin */
    struct gpio_dt_spec event;   /* event pin */
    struct gpio_dt_spec pwr_en;  /* enable pin */
    struct gpio_dt_spec lna_en;  /* lna pin */

    struct gpio_callback event_cb;          /* event callback structure */
    lr11xx_event_cb_t event_interrupt_cb;   /* event interrupt provided callback */

    lr11xx_system_rfswitch_cfg_t rf_switch_cfg;  /* RF switch config */

} lr11xx_hal_context_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */


#ifdef __cplusplus
}
#endif

#endif  // LR11XX_HAL_CONTEXT_H
