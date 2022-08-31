#include <stdio.h>
#include <string.h>
#include <zephyr.h>
#include <zephyr/types.h>

#include "apps_common.h"
#include "lr11xx_radio.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(main);


static lr11xx_hal_context_t* context;

/**
 * @brief Main application entry point.
 */
void main( void )
{
    LOG_INF( "===== LR11xx TX CW example =====" );

    context = apps_common_lr11xx_get_context( );

    apps_common_lr11xx_system_init( context );
    apps_common_lr11xx_fetch_and_print_version( context );

    apps_common_lr11xx_radio_init( context );

    lr11xx_radio_set_tx_cw( context );

    while( 1 )
    {
        k_sleep(K_MSEC(1));
    }
}
