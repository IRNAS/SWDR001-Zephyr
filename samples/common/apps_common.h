#ifndef APPS_COMMON_H
#define APPS_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include "apps_configuration.h"
#include "lr11xx_hal_context.h"
#include "lr11xx_system_types.h"
#include "lr11xx_radio_types.h"
#include "lr11xx_radio.h"


/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */
#define RX_CONTINUOUS 0xFFFFFF

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Initialize the system configuration of the transceiver
 *
 * @param [in] context  Pointer to the radio context
 */
void apps_common_lr11xx_system_init( const struct device* context );

/*!
 * @brief Initialize the radio configuration of the transceiver
 *
 * @param [in] context  Pointer to the radio context
 */
void apps_common_lr11xx_radio_init( const void* context );

/**
 * @brief Fetch the version from the LR11xx and print it on the log interface
 *
 * @param [in] context  Pointer to the radio context
 */
void apps_common_lr11xx_fetch_and_print_version( const struct device* context );

/*!
 * @brief Interface to read bytes from rx buffer
 *
 * @param [in] context  Pointer to the radio context
 * @param [in] buffer Pointer to a byte array to be filled with content from memory. Its size must be enough to contain
 * at least length bytes
 * @param [in] size Number of bytes to read from memory
 */
void apps_common_lr11xx_receive( const void* context, uint8_t* buffer, uint8_t* size );

/*!
 * @brief Interface to enable hardware interruption
 *
 * @param [in] context  Pointer to the radio context
 *
 * @warning This function must be called to enable hardware interruption and 'irq_fired' flag will be triggered to
 * indicate interruption, then 'apps_common_lr11xx_irq_process' in main loop will be executed.
 */
void apps_common_lr11xx_enable_irq( const void* context );

/*!
 * @brief Interface to lr11xx interrupt processing routine
 *
 * This function fetched the IRQ mask from the lr11xx and process the raised IRQ if the corresponding bit is also set in irq_filter.
 * The argument irq_filter allows to not process an IRQ even if it is raised by the lr11xx.
 *
 * @warning This function must be called from the main loop of project to dispense all the lr11xx interrupt routine
 *
 * @param [in] context  Pointer to the radio context
 * @param [in] irq_filter_mask  Mask of IRQ to process
 */
void apps_common_lr11xx_irq_process( const void* context, lr11xx_system_irq_mask_t irq_filter_mask );

/*!
 * @brief A function to get the value for low data rate optimization setting
 *
 * @param [in] sf  LoRa Spreading Factor
 * @param [in] bw  LoRa Bandwidth
 */
inline static uint8_t apps_common_compute_lora_ldro( const lr11xx_radio_lora_sf_t sf, const lr11xx_radio_lora_bw_t bw )
{
    switch( bw )
    {
    case LR11XX_RADIO_LORA_BW_500:
        return 0;

    case LR11XX_RADIO_LORA_BW_250:
        if( sf == LR11XX_RADIO_LORA_SF12 )
        {
            return 1;
        }
        else
        {
            return 0;
        }

    case LR11XX_RADIO_LORA_BW_800:
    case LR11XX_RADIO_LORA_BW_400:
    case LR11XX_RADIO_LORA_BW_200:
    case LR11XX_RADIO_LORA_BW_125:
        if( ( sf == LR11XX_RADIO_LORA_SF12 ) || ( sf == LR11XX_RADIO_LORA_SF11 ) )
        {
            return 1;
        }
        else
        {
            return 0;
        }

    case LR11XX_RADIO_LORA_BW_62:
        if( ( sf == LR11XX_RADIO_LORA_SF12 ) || ( sf == LR11XX_RADIO_LORA_SF11 ) || ( sf == LR11XX_RADIO_LORA_SF10 ) )
        {
            return 1;
        }
        else
        {
            return 0;
        }

    case LR11XX_RADIO_LORA_BW_41:
        if( ( sf == LR11XX_RADIO_LORA_SF12 ) || ( sf == LR11XX_RADIO_LORA_SF11 ) || ( sf == LR11XX_RADIO_LORA_SF10 ) ||
            ( sf == LR11XX_RADIO_LORA_SF9 ) )
        {
            return 1;
        }
        else
        {
            return 0;
        }

    case LR11XX_RADIO_LORA_BW_31:
    case LR11XX_RADIO_LORA_BW_20:
    case LR11XX_RADIO_LORA_BW_15:
    case LR11XX_RADIO_LORA_BW_10:
        // case LR11XX_RADIO_LORA_BW_7:
        return 1;

    default:
        return 0;
    }
}

#ifdef __cplusplus
}
#endif

#endif  // APPS_COMMON_H

/* --- EOF ------------------------------------------------------------------ */
