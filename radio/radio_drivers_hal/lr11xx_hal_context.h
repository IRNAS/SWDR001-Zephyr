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

typedef struct
{
    const struct  spi_dt_spec spi;

    struct gpio_dt_spec nss;
    struct gpio_dt_spec busy;
    struct gpio_dt_spec reset;

} lr11xx_hal_context_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

#ifdef __cplusplus
}
#endif

#endif  // LR11XX_HAL_CONTEXT_H
