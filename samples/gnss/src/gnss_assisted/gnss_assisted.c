/*!
 * @file      gnss_assisted.c
 *
 * @brief     GNSS assisted scan example for LR11xx chip
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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/devicetree.h>

#include "gnss_example_api.h"
#include "lr11xx_gnss.h"
#include "main_gnss.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gnss_assisted);

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/**
 * @brief UART peripheral
 *
 */

#define MAX_BUF_SIZE 255

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef struct
{
    lr11xx_gnss_solver_assistance_position_t assistance_position;
    lr11xx_gnss_scan_mode_t                  scan_mode;
    lr11xx_gnss_search_mode_t                effort_mode;
    uint8_t                                  input_parameters;
    uint8_t                                  max_sv;
} gnss_configuration_t;

typedef struct {
	void  *fifo_reserved;
	uint8_t    data[MAX_BUF_SIZE];
	uint16_t   len;
} uart_data_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

const gnss_configuration_t gnss_configuration = {
    .assistance_position = { .latitude  = GNSS_ASSISTANCE_POSITION_LATITUDE,
                             .longitude = GNSS_ASSISTANCE_POSITION_LONGITUDE },
    .scan_mode           = GNSS_SCAN_MODE,
    .effort_mode         = GNSS_EFFORT_MODE,
    .input_parameters    = GNSS_INPUT_PARAMETERS,
    .max_sv              = GNSS_MAX_SV,
};

const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart0));

const struct uart_config uart_cfg = {
    .baudrate = 115200,
    .parity = UART_CFG_PARITY_NONE,
    .stop_bits = UART_CFG_STOP_BITS_1,
    .data_bits = UART_CFG_DATA_BITS_8,
    .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static uint32_t gps_local_delay_s = 0;

// FIFO for storing incoming messages
static K_FIFO_DEFINE(fifo_uart_rx_data);

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

static uint32_t get_gps_delay_from_serial( void );
static uint32_t get_gps_time_now( void );

/**
 * @brief Initialise UART RX callback.
 *
 */
static void init_uart(void);

/**
 * @brief UART RX callback.
 *
 * @param dev
 * @param user_data
 */
static void rc_uart_cb(const struct device *dev, void *user_data);

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void gnss_init( const void* context )
{
    gps_local_delay_s = get_gps_delay_from_serial( );

    int ret = lr11xx_gnss_set_scan_mode( context, gnss_configuration.scan_mode );
    if(ret)
    {
        LOG_ERR("Failed to init assisted gnss scan.");
    }
    ret = lr11xx_gnss_set_assistance_position( context, &gnss_configuration.assistance_position );
    if(ret)
    {
        LOG_ERR("Failed to init assistance position.");
    }
}

void gnss_call_scan( const void* context )
{
    int ret = lr11xx_gnss_scan_assisted( context, get_gps_time_now( ), gnss_configuration.effort_mode,
                                                 gnss_configuration.input_parameters, gnss_configuration.max_sv );
    if(ret)
    {
        LOG_ERR("Failed to call assisted gnss scan.");
    }
}

const char* gnss_get_example_name( void ) { return ( const char* ) "LR11xx GNSS Assisted scan example"; }

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

uint32_t get_gps_delay_from_serial( )
{
    init_uart();

    uint32_t gps_time = 0;

    if (uart_dev)
    {
        //Enable callback
        uart_irq_rx_enable(uart_dev);

        LOG_INF("Send gps time!");

        uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data, K_FOREVER);
        if(buf)
        {
            gps_time = strtol(buf->data, NULL, 10);
            LOG_INF("Got GPS time: %d", gps_time);
            k_free(buf);
        }
    }
    else
    {
        LOG_ERR("UART not available!");
        return 0;

    }

    uint64_t local_rtc = k_uptime_get();

    const uint32_t gps_local_delay_s = gps_time - (uint32_t)(local_rtc/1000);
    return gps_local_delay_s;
}

uint32_t get_gps_time_now( void )
{
    uint64_t local_rtc = k_uptime_get();
    return (uint32_t)(local_rtc/1000) + gps_local_delay_s;

}

static void init_uart(void)
{
    //Init UART callback
    if (!device_is_ready(uart_dev))
    {
        LOG_ERR("Failed to init GNSS UART!");
        return;
    }

    if (uart_configure(uart_dev, &uart_cfg))
    {
        LOG_ERR("Failed to configure rf UART!");
        return;
    }

    //Ad RX callback
    uart_irq_callback_set(uart_dev, rc_uart_cb);

    //Enable callback
    uart_irq_rx_enable(uart_dev);
}

static void rc_uart_cb(const struct device *dev, void *user_data)
{
	static uart_data_t *rx;
	ARG_UNUSED(user_data);

	uart_irq_update(dev);

	if (uart_irq_rx_ready(dev))
	{
		int data_length;
		if (!rx)
		{
			rx = k_malloc(sizeof(*rx));
			if (rx)
			{
				rx->len = 0;
			}
			else {
				/* Disable UART interface, it will be
				 * enabled again after releasing the buffer.
				 */
				uart_irq_rx_disable(dev);

				LOG_ERR("Not able to allocate UART receive buffer\n");
				return;
			}
		}


		data_length = uart_fifo_read(dev, &rx->data[rx->len], MAX_BUF_SIZE-rx->len);

		rx->len += data_length;

		if (rx->len > 0) {
            if ((rx->len == MAX_BUF_SIZE) ||
			   ((rx->data[rx->len - 2] == '\r') &&
			   (rx->data[rx->len - 1] == '\n')))
			{
                //If only new line is obtained, skip
                if(rx->len > 1)
                {
                    rx->data[rx->len - 2] = 0;
                    rx->data[rx->len - 1] = 0;

                    rx->len -= 2;

                    k_fifo_put(&fifo_uart_rx_data, rx);
                }
                else
                {
                    k_free(rx);
                }
				rx = NULL;
			}
		}
	}
}
