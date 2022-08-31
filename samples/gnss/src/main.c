#include <stdio.h>
#include <string.h>
#include <zephyr.h>
#include <zephyr/types.h>

#include "apps_common.h"
#include "lr11xx_system.h"
#include "smtc_board.h"
#include "smtc_hal.h"
#include "main_gnss.h"
#include "lr11xx_gnss.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(main);


/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define APP_PARTIAL_SLEEP true
#define NAV_MAX_LENGTH ( 300 )

/**
 * @brief LR11xx interrupt mask used by the application
 */
#define IRQ_MASK ( LR11XX_SYSTEM_IRQ_GNSS_SCAN_DONE )

#define LATITUDE  46.452761f
#define LONGITUDE 15.641854f


/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

typedef struct
{
    lr11xx_gnss_solver_assistance_position_t assistance_position;
    lr11xx_gnss_scan_mode_t   scan_mode;
    lr11xx_gnss_search_mode_t effort_mode;
    uint8_t                   input_parameters;
    uint8_t                   max_sv;
} gnss_configuration_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

const gnss_configuration_t gnss_configuration = {
    .assistance_position = { .latitude  = LATITUDE,
                             .longitude = LONGITUDE },
    .scan_mode        = LR11XX_GNSS_SCAN_MODE_3_SINGLE_SCAN_AND_5_FAST_SCANS,
    .effort_mode      = LR11XX_GNSS_OPTION_BEST_EFFORT,
    .input_parameters = 0x07,
    .max_sv           = 20,
};


/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static lr11xx_hal_context_t* context;
static uint32_t              number_of_scan = 0;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Main application entry point.
 */
void main( void )
{
    int ret;
    
    LOG_INF( "===== LR11xx GNSS example =====" );

    context = apps_common_lr11xx_get_context( );

    apps_common_lr11xx_system_init( context );
    apps_common_lr11xx_fetch_and_print_version( context );

    ret = lr11xx_system_set_dio_irq_params( context, IRQ_MASK, 0 )
    if(ret)
    {
        LOG_ERR("Failed to set irq params");
    }

    ret = lr11xx_system_clear_irq_status( context, LR11XX_SYSTEM_IRQ_ALL_MASK );
    if(ret)
    {
        LOG_ERR("Failed to clear irq status");
    }

    ret = lr11xx_gnss_set_scan_mode( context, gnss_configuration.scan_mode);
    if(ret)
    {
        LOG_ERR("Failed to set scan mode");
    }

    ret = 


    while( 1 )
    {
        k_sleep(K_MSEC(1));
    }
}
