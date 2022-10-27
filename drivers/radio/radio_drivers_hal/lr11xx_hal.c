
#include <zephyr.h>
#include <zephyr/types.h>
#include <drivers/gpio.h>
#include <drivers/spi.h>

#include "lr11xx_hal.h"
#include "lr11xx_hal_context.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define LR11XX_HAL_WAIT_ON_BUSY_TIMEOUT_SEC  10

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef enum
{
    RADIO_SLEEP,
    RADIO_AWAKE
} radio_mode_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static volatile radio_mode_t radio_mode = RADIO_AWAKE;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
*/

/**
 * @brief Wait until radio busy pin returns to 0
 */
lr11xx_hal_status_t lr11xx_hal_wait_on_busy( const struct gpio_dt_spec *busy_pin );

/**
 * @brief Check if device is ready to receive spi transaction.
 * @remark If the device is in sleep mode, it will awake it and wait until it is ready
 */
void lr11xx_hal_check_device_ready( const lr11xx_hal_context_t* lr11xx_context );


/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

lr11xx_hal_status_t lr11xx_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length )
{
#if defined( USE_LR11XX_CRC_OVER_SPI )
    // Compute the CRC over command array first and over data array then
    uint8_t cmd_crc = lr11xx_hal_compute_crc( 0xFF, command, command_length );
    cmd_crc         = lr11xx_hal_compute_crc( cmd_crc, data, data_length );
#endif //defined( USE_LR11XX_CRC_OVER_SPI )

    const lr11xx_hal_context_t* lr11xx_context = ( const lr11xx_hal_context_t* ) context;

    int ret;

    lr11xx_hal_check_device_ready( lr11xx_context );
    const struct spi_buf tx_buf[] = {
        {
			.buf = (uint8_t *)command,
			.len = command_length,
		},
		{
			.buf = (uint8_t *)data,
			.len = data_length
		},
#if defined( USE_LR11XX_CRC_OVER_SPI )
        {
			.buf = &cmd_crc,
			.len = 1
		}
#endif //defined( USE_LR11XX_CRC_OVER_SPI )
    };

    const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = ARRAY_SIZE(tx_buf),
	};

    ret = spi_write_dt(&lr11xx_context->spi, &tx);
    if(ret)
    {
        return LR11XX_HAL_STATUS_ERROR;
    }

    // LR11XX_SYSTEM_SET_SLEEP_OC=0x011B opcode. In sleep mode the radio busy line is held at 1 => do not test it
    if( ( command[0] == 0x01 ) && ( command[1] == 0x1B ) )
    {
        radio_mode = RADIO_SLEEP;

        // add a incompressible delay to prevent trying to wake the radio before it is full asleep
        k_sleep(K_MSEC(1));
        return LR11XX_HAL_STATUS_OK;
    }

    return lr11xx_hal_wait_on_busy(&lr11xx_context->busy);
}

lr11xx_hal_status_t lr11xx_hal_direct_read( const void* context, uint8_t* data, const uint16_t data_length )
{ 
    const lr11xx_hal_context_t* lr11xx_context = ( const lr11xx_hal_context_t* ) context;

    lr11xx_hal_check_device_ready( lr11xx_context );

    int ret;
    
#if defined( USE_LR11XX_CRC_OVER_SPI )
    uint8_t rx_crc;
#endif //defined( USE_LR11XX_CRC_OVER_SPI )

    const struct spi_buf rx_buf[] = {
    {
        .buf = data,
        .len = data_length
    },
#if defined( USE_LR11XX_CRC_OVER_SPI )
    // read crc sent by lr11xx at the end of the transaction
    {
        .buf = &rx_crc,
        .len = 1
    }
#endif //defined( USE_LR11XX_CRC_OVER_SPI )
    };

    const struct spi_buf_set rx = {
        .buffers = rx_buf,
        .count = ARRAY_SIZE(rx_buf),
    };

    ret = spi_read_dt(&lr11xx_context->spi, &rx);
    if(ret)
    {
        return LR11XX_HAL_STATUS_ERROR;
    }

#if defined( USE_LR11XX_CRC_OVER_SPI )
    // check crc value
    uint8_t computed_crc = lr11xx_hal_compute_crc( 0xFF, data, data_length );
    if( rx_crc != computed_crc )
    {
        return LR11XX_HAL_STATUS_ERROR;
    }
#endif //defined( USE_LR11XX_CRC_OVER_SPI )

    return LR11XX_HAL_STATUS_OK;
}


lr11xx_hal_status_t lr11xx_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length )
{
#if defined( USE_LR11XX_CRC_OVER_SPI )
    // Compute the CRC over command array first and over data array then
    uint8_t cmd_crc = lr11xx_hal_compute_crc( 0xFF, command, command_length );
#endif
    
    const lr11xx_hal_context_t* lr11xx_context = ( const lr11xx_hal_context_t* ) context;

    lr11xx_hal_check_device_ready( lr11xx_context );
        int ret;

    const struct spi_buf tx_buf[] = {
        {
			.buf = (uint8_t *)command,
			.len = command_length,
		},
#if defined( USE_LR11XX_CRC_OVER_SPI )
        {
			.buf = &cmd_crc,
			.len = 1
		}
#endif //defined( USE_LR11XX_CRC_OVER_SPI )
    };

    

    const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = ARRAY_SIZE(tx_buf),
	};

    ret = spi_write_dt(&lr11xx_context->spi, &tx);
    if(ret)
    {
        return LR11XX_HAL_STATUS_ERROR;
    }

    if(data_length > 0)
    {
        lr11xx_hal_check_device_ready( lr11xx_context );

        uint8_t dummy_byte;
    
        const struct spi_buf rx_buf[] = {
		// save dummy for crc calculation
        {
			.buf = &dummy_byte,
			.len = 1,
		},
		{
			.buf = data,
			.len = data_length
		},
#if defined( USE_LR11XX_CRC_OVER_SPI )
        // read crc sent by lr11xx at the end of the transaction
        {
			.buf = &cmd_crc,
			.len = 1
		}
#endif //defined( USE_LR11XX_CRC_OVER_SPI )
        };

        const struct spi_buf_set rx = {
            .buffers = rx_buf,
            .count = ARRAY_SIZE(rx_buf),
	    };

        ret = spi_read_dt(&lr11xx_context->spi, &rx);
        if(ret)
        {
            return LR11XX_HAL_STATUS_ERROR;
        }

#if defined( USE_LR11XX_CRC_OVER_SPI )
        // Check CRC value
        uint8_t computed_crc = lr11xx_hal_compute_crc( 0xFF, &dummy_byte, 1 );
        computed_crc         = lr11xx_hal_compute_crc( computed_crc, data, data_length );
        if( cmd_crc != computed_crc )
        {
            return LR11XX_HAL_STATUS_ERROR;
        }
#endif //defined( USE_LR11XX_CRC_OVER_SPI )

    }

    return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_reset( const void* context )
{
    const lr11xx_hal_context_t* lr11xx_context = ( const lr11xx_hal_context_t* ) context;
    gpio_pin_set_dt(&lr11xx_context->reset, 1);
    k_sleep(K_MSEC(5));
    gpio_pin_set_dt(&lr11xx_context->reset, 0);
    k_sleep(K_MSEC(5));

    // Wait 200ms until internal lr11xx fw is ready
    k_sleep(K_MSEC(200));
    radio_mode = RADIO_AWAKE;

    return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_wakeup( const void* context )
{
    const lr11xx_hal_context_t* lr11xx_context = ( const lr11xx_hal_context_t* ) context;
    lr11xx_hal_check_device_ready(lr11xx_context);

    return LR11XX_HAL_STATUS_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static bool lr11xx_hal_busy_timeout = false;

/**
 * @brief timer handler, sets timeout to true when timer expires
 * 
 * @param dummy 
 */
static void lr11xx_hal_wait_on_busy_timer_handler(struct k_timer *dummy)
{
    lr11xx_hal_busy_timeout = true;
}

/**
 * @brief Construct a new k timer define object for busy pin timeout
 * 
 */
K_TIMER_DEFINE(lr11xx_hal_wait_on_busy_timer, lr11xx_hal_wait_on_busy_timer_handler, NULL);

lr11xx_hal_status_t lr11xx_hal_wait_on_busy( const struct gpio_dt_spec *busy_pin )
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
        printk("Busy pin timeout\n");
        return LR11XX_HAL_STATUS_ERROR;
    }
    
    return LR11XX_HAL_STATUS_OK;
}

void lr11xx_hal_check_device_ready( const lr11xx_hal_context_t* lr11xx_context )
{
    if( radio_mode != RADIO_SLEEP )
    {
        lr11xx_hal_wait_on_busy( &lr11xx_context->busy );
    }
    else
    {
        // Busy is HIGH in sleep mode, wake-up the device with a small glitch on NSS
        const struct spi_cs_control *ctrl = lr11xx_context->spi.config.cs;
        printk("CS: %d bus: %s\n",ctrl->gpio.pin, ctrl->gpio.port->name);


        gpio_pin_set_dt(&ctrl->gpio, 1);
        gpio_pin_set_dt(&ctrl->gpio, 0);
        lr11xx_hal_wait_on_busy( &lr11xx_context->busy );
        radio_mode = RADIO_AWAKE;
    }
}