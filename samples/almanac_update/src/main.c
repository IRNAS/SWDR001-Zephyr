#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include "apps_common.h"
#include "lr11xx_radio.h"
#include "lr11xx_gnss.h"
#include "lr11xx_gnss_types.h"
#include "lr11xx_almanac.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

#define LR11XX_NODE           DT_NODELABEL(lr1120)

const struct device *context;
static lr11xx_gnss_context_status_bytestream_t context_status_buffer;
lr11xx_gnss_context_status_t context_status;

/**
 * @brief Main application entry point.
 */
void main( void )
{
    int ret = 0;

    LOG_INF( "===== LR11xx Almanac Update =====" );

    context = device_get_binding(DT_LABEL(LR11XX_NODE));

    apps_common_lr11xx_system_init( context );
    apps_common_lr11xx_fetch_and_print_version( context );

    apps_common_lr11xx_radio_init( context );
    k_sleep(K_SECONDS(2));

    /* Get gnss context status */
    ret = lr11xx_gnss_get_context_status( context, context_status_buffer );
    if(ret)
    {
        LOG_ERR("Failed to get context status");
    }

    /* Parse status */
    ret = lr11xx_gnss_parse_context_status_buffer( context_status_buffer, &context_status );
    if(ret)
    {
        LOG_ERR("Failed to parse gnss context status");
    }
    LOG_INF("Almanac error code: %02X", context_status.error_code);

    /* Get almanac age */
    uint16_t almanac_age;
    ret = lr11xx_gnss_get_almanac_age_for_satellite( context, 0, &almanac_age );
    if(ret)
    {
        LOG_ERR("Failed to get almanac age");
    }

    LOG_INF("Current almanac age: %d", almanac_age);

    /* Check available age */
    uint16_t available_age = (((uint16_t)lr11xx_full_almanac[2]) << 8) + (((uint16_t)lr11xx_full_almanac[1]) << 0);

    LOG_INF("Available almanac age: %d", available_age);

    if (almanac_age < available_age)
    {
        LOG_INF("New Almanac available, start update!");
        ret = lr11xx_gnss_set_almanac_update( context, LR11XX_GNSS_GPS_MASK | LR11XX_GNSS_BEIDOU_MASK);
        if (ret)
        {
            LOG_ERR("Set update almanac err!");
        }
        else
        {
            ret = lr11xx_gnss_almanac_update( context, lr11xx_full_almanac, LR11XX_GNSS_FULL_UPDATE_N_ALMANACS + 1);
            if (ret)
            {
                LOG_ERR("Almanac update error!");
            }
            else
            {
                LOG_INF("Almanac update successful!");
                ret = lr11xx_gnss_get_almanac_age_for_satellite( context, 0, &almanac_age );
                if(ret)
                {
                    LOG_ERR("Failed to get almanac age");
                }

                LOG_INF("New almanac age: %d", almanac_age);
            }
        }
    }

    while( 1 )
    {
        k_sleep(K_MSEC(1));
    }
}
