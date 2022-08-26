
#include <zephyr.h>
#include <zephyr/types.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>

#include "lr11xx_hal.h"
#include "lr11xx_hal_context.h"


#define LR11XX_HAL_WAIT_ON_BUSY_TIMEOUT_SEC  10

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
*/

/**
 * @brief Wait until radio busy pin returns to 0
 */
lr11xx_hal_status_t lr11xx_hal_wait_on_busy( struct gpio_dt_spec *busy_pin );

lr11xx_hal_status_t lr11xx_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length )
{
#if defined( USE_LR11XX_CRC_OVER_SPI )
    // Compute the CRC over command array first and over data array then
    uint8_t cmd_crc = lr11xx_hal_compute_crc( 0xFF, command, command_length );
    cmd_crc         = lr11xx_hal_compute_crc( cmd_crc, data, data_length );
#endif
    
    const lr11xx_hal_context_t* lr11xx_context = ( const lr11xx_hal_context_t* ) context;


}

lr11xx_hal_status_t lr11xx_hal_direct_read( const void* context, uint8_t* data, const uint16_t data_length )
{
#if defined( USE_LR11XX_CRC_OVER_SPI )
    // Compute the CRC over command array first and over data array then
    uint8_t cmd_crc = lr11xx_hal_compute_crc( 0xFF, command, command_length );
    cmd_crc         = lr11xx_hal_compute_crc( cmd_crc, data, data_length );
#endif
    
    const lr11xx_hal_context_t* lr11xx_context = ( const lr11xx_hal_context_t* ) context;


}


lr11xx_hal_status_t lr11xx_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length )
{
#if defined( USE_LR11XX_CRC_OVER_SPI )
    // Compute the CRC over command array first and over data array then
    uint8_t cmd_crc = lr11xx_hal_compute_crc( 0xFF, command, command_length );
    cmd_crc         = lr11xx_hal_compute_crc( cmd_crc, data, data_length );
#endif
    
    const lr11xx_hal_context_t* lr11xx_context = ( const lr11xx_hal_context_t* ) context;


}

lr11xx_hal_status_t lr11xx_hal_reset( const void* context )
{
    const lr11xx_hal_context_t* lr11xx_context = ( const lr11xx_hal_context_t* ) context;
}

lr11xx_hal_status_t lr11xx_hal_wakeup( const void* context )
{
    const lr11xx_hal_context_t* lr11xx_context = ( const lr11xx_hal_context_t* ) context;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static bool lr11xx_hal_busy_timeout = false;

static void lr11xx_hal_wait_on_busy_timer_handler(struct k_timer *dummy)
{
    lr11xx_hal_busy_timeout = true;
}

K_TIMER_DEFINE(lr11xx_hal_wait_on_busy_timer, lr11xx_hal_wait_on_busy_timer_handler, )

lr11xx_hal_status_t lr11xx_hal_wait_on_busy( struct gpio_dt_spec *busy_pin )
{
    lr11xx_hal_busy_timeout = false;
    k_timer_start(&lr11xx_hal_wait_on_busy_timer, K_SECONDS(LR11XX_HAL_WAIT_ON_BUSY_TIMEOUT_SEC), K_NO_WAIT);

    while(gpio_pin_get_dt(busy_pin) && !lr11xx_hal_busy_timeout )
    {
        k_sleep(K_MSEC(1));
    }

    k_timer_stop(&lr11xx_hal_wait_on_busy_timer);

    if(lr11xx_hal_busy_timeout)
    {
        return LR11XX_HAL_STATUS_ERROR;
    }
    
    return LR11XX_HAL_STATUS_OK;
}
