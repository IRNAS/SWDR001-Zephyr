#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/random/rand32.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

#include "apps_common.h"
#include "lr11xx_radio.h"
#include "lr11xx_system.h"
#include "lr11xx_system_types.h"
#include "lr11xx_regmem.h"
#include "lr11xx_board.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

#define RX_TIMEOUT_VALUE 600

/**
 * @brief Size of ping-pong message prefix
 *
 * Expressed in bytes
 */
#define PING_PONG_PREFIX_SIZE 6

/**
 * @brief Duration of the wait before packet transmission to assure reception status is ready on the other side
 *
 * Expressed in milliseconds
 */
#define DELAY_BEFORE_TX_MS 20

/**
 * @brief Duration of the wait between each ping-pong activity, can be used to adjust ping-pong speed
 *
 * Expressed in milliseconds
 */
#define DELAY_PING_PONG_PACE_MS 200

/**
 * @brief LR11xx interrupt mask used by the application
 */
#define IRQ_MASK                                                                          \
    ( LR11XX_SYSTEM_IRQ_TX_DONE | LR11XX_SYSTEM_IRQ_RX_DONE | LR11XX_SYSTEM_IRQ_TIMEOUT | \
      LR11XX_SYSTEM_IRQ_HEADER_ERROR | LR11XX_SYSTEM_IRQ_CRC_ERROR | LR11XX_SYSTEM_IRQ_FSK_LEN_ERROR )


/* Two sets of LoRa settings for reception testing */
/* CONFIG_1 */
#define RF_FREQ_IN_HZ_1 868000000U
#define TX_OUTPUT_POWER_DBM_1 10  // range [-17, +22] for sub-G, range [-18, 13] for 2.4G ( HF_PA )
#define LORA_SPREADING_FACTOR_1 LR11XX_RADIO_LORA_SF7
#define LORA_BANDWIDTH_1 LR11XX_RADIO_LORA_BW_125
#define LORA_CODING_RATE_1 LR11XX_RADIO_LORA_CR_4_5

/* CONFIG_2 */
#define RF_FREQ_IN_HZ_2 2400000000U
#define TX_OUTPUT_POWER_DBM_2 10  // range [-17, +22] for sub-G, range [-18, 13] for 2.4G ( HF_PA )
#define LORA_SPREADING_FACTOR_2 LR11XX_RADIO_LORA_SF10
#define LORA_BANDWIDTH_2 LR11XX_RADIO_LORA_BW_400
#define LORA_CODING_RATE_2 LR11XX_RADIO_LORA_CR_4_5

/**
 * @brief BT advertisement name prefix
 *
 */
#define DEVICE_NAME  "Mopper-"

/**
 * @brief Use nordic BT_MANUFACTURER_DATA for test
 *
 */
#define BT_MANUFACTURER_DATA  0x0059

#define BT_SCAN_TIMEOUT 500
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define APP_PARTIAL_SLEEP true

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/* Configuration label */
enum {
    CONFIG_1 = 0,
    CONFIG_2 = 1
};

/**
 * @brief radio config structure, including all configs we weill modify during the example.
 *
 */
typedef struct radio_config {
    lr11xx_board_pa_pwr_cfg_t* pa_pwr_cfg;
    int8_t output_p_dbm;
    uint32_t freq_in_hz;
    lr11xx_radio_mod_params_lora_t radio_mod;
} radio_config;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

const struct device *context = DEVICE_DT_GET(DT_NODELABEL(lr11xx));

static radio_config config; //LR modifiable config options

static uint8_t buffer_tx[PAYLOAD_LENGTH]; //TX buffer
static uint8_t buffer_rx[PAYLOAD_LENGTH]; //RX buffer
static bool    is_master = true; //Master - slave status
static bool    is_status = false; //Master - slave status set

static uint32_t counter = 0; //Message counter
static int current_config = CONFIG_1; //Current config choice

/* Ping - pong message prefix */
static const uint8_t ping_msg[PING_PONG_PREFIX_SIZE] = "M-PING";
static const uint8_t pong_msg[PING_PONG_PREFIX_SIZE] = "S-PONG";

static const uint8_t prefix_size = ( PING_PONG_PREFIX_SIZE <= PAYLOAD_LENGTH ? PING_PONG_PREFIX_SIZE : PAYLOAD_LENGTH );

/**
 * @brief BT advertisement parameters
 *
 */
static struct bt_le_adv_param *adv_param =
    BT_LE_ADV_PARAM(BT_LE_ADV_OPT_USE_IDENTITY,
                    320,             //(N * 0.625 = 500ms)
                    352,             //(N * 0.625 = 520ms)
                    NULL);

/**
 * @brief BT scan parameters - use list fillter for scanning to only scan for particular device in BT example
 *
 */
struct bt_le_scan_param scan_param = {
		.type       = BT_HCI_LE_SCAN_PASSIVE,
		.options    = BT_LE_SCAN_OPT_FILTER_ACCEPT_LIST | BT_LE_SCAN_OPT_FILTER_DUPLICATE,
		.interval   = 32,  //(N * 0.625 ms)
		.window     = 32,  //(N * 0.625 ms)
	};

static char device_name[13]; //Device BT name

static uint8_t adv_data[6]; //Device adv data

static bt_addr_le_t bt_addr; //Local BT MAC addr
static bt_addr_le_t bt_addr_remote; //Remote BT MAC addr

static bool bt_scan_done = false; //Scan results obtained flag
static bool bt_scan_timeout = false; //BT scan timeout flag
/**
 * @brief Advertisement data structure.
 *
 */
static const struct bt_data ad[] = {
	BT_DATA(BT_DATA_MANUFACTURER_DATA, adv_data, sizeof(adv_data)),
    BT_DATA(BT_DATA_NAME_COMPLETE, device_name, sizeof(device_name))
};


/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/**
 * @brief Handle reception failure for ping-pong example
 */
static void ping_pong_reception_failure_handling( void );

/**
 * @brief Update radio_config config structure with new values based on the configuration choice.
 *
 * @param config_label CONFIG_1 / CONFIG_2
 * @return int - 0 on success or negative error code
 */
static int ping_pong_change_config( int config_label);

/**
 * @brief Change initialisation of some radio parameters,a s set in the radio_config config structure.
 *
 * @return int - 0 on success or negative error code
 */
static int ping_pong_radio_reinit( void );

/*!
 * @brief Interface to read bytes from rx buffer
 *
 * @param [in] context  Pointer to the radio context
 * @param [in] buffer Pointer to a byte array to be filled with content from memory. Its size must be enough to contain
 * at least length bytes
 * @param [in] size Number of bytes to read from memory
 * @param [in] counter_rx  Received message counter
 */
static void ping_pong_lr11xx_receive( const void* context, uint8_t* buffer, uint8_t* size, uint32_t* counter_rx );

/**
 * @brief Initialise BT advertisement
 * Enable and init BT.
 * Construct device name by attaching 3 significant bytes of MAC address to pre-defined name.
 * Initialize advertisement data and start advertising.
 *
 */
static void initialize_bt_adv(void);

/**
 * @brief BT scan cb
 *
 * @param addr
 * @param rssi
 * @param adv_type
 * @param buf
 */
static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type,
		    struct net_buf_simple *buf);

/**
 * @brief timer handler, sets timeout to true when timer expires
 *
 * @param dummy
 */
static void bt_scan_timer_handler(struct k_timer *dummy);

/**
 * @brief Perform BT scan, start timer and wait for result or timeout.
 *
 */
static void ping_pong_wait_on_bt_scan( void );


/**
 * @brief Construct a new k timer define object for bt scan timeout
 *
 */
K_TIMER_DEFINE(bt_scan_timer, bt_scan_timer_handler, NULL);

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/**
 * @brief Main application entry point.
 */
void main( void )
{
    LOG_INF( "===== LR11xx IRNAS ping-pong example =====" );

    /* Init BLE adv */
    initialize_bt_adv();

    /* General system init */
    apps_common_lr11xx_system_init( context );

    /* Print LR11xx HW and FW version */
    apps_common_lr11xx_fetch_and_print_version( context );

    /* Start with defout LR configuration as defined for all samples */
    apps_common_lr11xx_radio_init( context );

    /* Set initial config */
    current_config = CONFIG_1;
    ping_pong_change_config(current_config);

    /* Apply config */
    ping_pong_radio_reinit();

    int ret = 0;

    /* Define event trigger mask */
    LOG_INF("Set dio irq mask");
    ret = lr11xx_system_set_dio_irq_params( context, IRQ_MASK, 0 );
    if(ret)
    {
        LOG_ERR("Failed to set dio irq params.");
    }

    /* Clear IRQ status */
    LOG_INF("Clear irq status");
    ret = lr11xx_system_clear_irq_status( context, LR11XX_SYSTEM_IRQ_ALL_MASK );
    if(ret)
    {
        LOG_ERR("Failed to set dio irq params.");
    }

    apps_common_lr11xx_enable_irq(context);

    /* Setup TX buffer  */
    /* Add ping message */
    memcpy( buffer_tx, ping_msg, prefix_size );
    /* Add counter */
    memcpy( buffer_tx+prefix_size, &counter, sizeof(counter) );
    /* Add MAC address */
    memcpy( buffer_tx+prefix_size+sizeof(counter), bt_addr.a.val, 6 );

    /* Write buffer */
    LOG_INF("Write TX buffer");
    ret = lr11xx_regmem_write_buffer8( context, buffer_tx, PAYLOAD_LENGTH );
    if(ret)
    {
        LOG_ERR("Failed to write buffer.");
    }

    /* Set in TX mode */
    LOG_INF("Set TX mode");
    ret = lr11xx_radio_set_tx( context, 0 );
    if(ret)
    {
        LOG_ERR("Failed to set TX mode.");
    }

    /* Remain in the original setting while status is not set yet */
    while(!is_status)
    {
        apps_common_lr11xx_irq_process( context, IRQ_MASK );
        k_sleep(K_MSEC(1));
    }

    LOG_INF("Status set to master: %d", is_master);
    /* Read remote MAC */
    memcpy(bt_addr_remote.a.val, buffer_rx+prefix_size+sizeof(counter), 6 );
    bt_addr_remote.type = 1;
    LOG_INF("Remote MAC addr: %x:%x:%x:%x:%x:%x", bt_addr_remote.a.val[0], bt_addr_remote.a.val[1], bt_addr_remote.a.val[2], bt_addr_remote.a.val[3], bt_addr_remote.a.val[4], bt_addr_remote.a.val[5]);

    const bt_addr_le_t *accept_addr = &bt_addr_remote;
    ret = bt_le_filter_accept_list_add(accept_addr);
    LOG_INF("Added MAC to accept list: %d", ret);

    /* Main loop */
    while( 1 )
    {
        apps_common_lr11xx_irq_process( context, IRQ_MASK );
        k_sleep(K_MSEC(1));
    }
}

void on_tx_done( void )
{
    /* Slave changes config after sending */
    if(!is_master && is_status)
    {
        /* If config is CONFIG_2 perform BT scan first */
        if(current_config == CONFIG_2)
        {
            ping_pong_wait_on_bt_scan();
        }

        /* Change config */
        current_config = (current_config + 1) % 2;
        ping_pong_change_config(current_config);

        /* Apply config */
        ping_pong_radio_reinit();
    }

    k_sleep(K_MSEC( DELAY_PING_PONG_PACE_MS ));

    int ret = lr11xx_radio_set_rx( context, RX_TIMEOUT_VALUE + sys_rand32_get() % 500 );  // Random delay to avoid unwanted synchronization
    if(ret)
    {
        LOG_ERR("Failed to set RX mode.");
    }
}

void on_rx_done( void )
{
    uint8_t size;
    uint32_t rx_count;

    ping_pong_lr11xx_receive( context, buffer_rx, &size, &rx_count );

    if( is_master == true )
    {
        /* Check if we got pong message */
        if( memcmp( buffer_rx, pong_msg, prefix_size ) == 0 )
        {
            /* If status is not determined yet, set it */
            if(!is_status)
            {
                is_status = true;
                /* Read other device MAC */
            }

            /* If config is CONFIG_2 perform BT scan first */
            if(current_config == CONFIG_2)
            {
                ping_pong_wait_on_bt_scan();
            }
            counter++;

            /* Set config */
            current_config = (current_config + 1) % 2;
            ping_pong_change_config(current_config);

            /* Apply config */
            ping_pong_radio_reinit();
        }
        else if( memcmp( buffer_rx, ping_msg, prefix_size ) == 0 )
        {
            /* If status is not determined yet, set it */
            if(!is_status)
            {
                is_status = true;
                is_master = false;
                memcpy( buffer_tx, pong_msg, prefix_size );
            }
            else
            {
                /* Wrong message to receive if we are master with set status - what to do? */
            }
        }
        else
        {
            LOG_ERR( "Unexpected message" );
        }
    }
    else
    {
        if( memcmp( buffer_rx, ping_msg, prefix_size ) == 0 )
        {
            /* If status is not determined yet, set it */
            if(!is_status)
            {
                is_status = true;
            }
            /* In normal operation change setting */
            else
            {
                counter = rx_count;
            }
        }
        else
        {
            LOG_ERR( "Unexpected message" );
        }
    }

    k_sleep(K_MSEC( DELAY_PING_PONG_PACE_MS ));

    k_sleep(K_MSEC( DELAY_BEFORE_TX_MS ));

    memcpy(buffer_tx+prefix_size, &counter, sizeof(counter));

    int ret = lr11xx_regmem_write_buffer8( context, buffer_tx, PAYLOAD_LENGTH );
    if(ret)
    {
        LOG_ERR("Failed to write buffer.");
    }

    ret = lr11xx_radio_set_tx( context, 0 );
    if(ret)
    {
        LOG_ERR("Failed to set TX mode.");
    }
}

void on_rx_timeout( void )
{
    ping_pong_reception_failure_handling( );
}

void on_rx_crc_error( void )
{
    ping_pong_reception_failure_handling( );
}

void on_fsk_len_error( void )
{
    ping_pong_reception_failure_handling( );
}

static void ping_pong_reception_failure_handling( void )
{
    /* Go to original setting */
    current_config = CONFIG_1;
    ping_pong_change_config(current_config);
    ping_pong_radio_reinit();

    //If master, resend message with same count
    if(is_master)
    {
        int ret = lr11xx_regmem_write_buffer8( context, buffer_tx, PAYLOAD_LENGTH );
        if(ret)
        {
            LOG_ERR("Failed to write buffer.");
        }

        ret = lr11xx_radio_set_tx( context, 0 );
        if(ret)
        {
            LOG_ERR("Failed to set TX mode.");
        }
    }
    /* If slave go to reception mode */
    else
    {
        int ret = lr11xx_radio_set_rx( context, RX_TIMEOUT_VALUE + sys_rand32_get() % 500 );  // Random delay to avoid unwanted synchronization
        if(ret)
        {
            LOG_ERR("Failed to set RX mode.");
        }
    }
}

static int ping_pong_change_config( int config_label)
{
    int ret = 0;

    LOG_INF("Change to config: %d", config_label);

    if(config_label == CONFIG_1)
    {
        config.pa_pwr_cfg = (lr11xx_board_pa_pwr_cfg_t*) lr11xx_board_get_pa_pwr_cfg( RF_FREQ_IN_HZ_1, TX_OUTPUT_POWER_DBM_1 );
        config.output_p_dbm = TX_OUTPUT_POWER_DBM_1;
        config.freq_in_hz = RF_FREQ_IN_HZ_1;
        config.radio_mod.sf = LORA_SPREADING_FACTOR_1;
        config.radio_mod.bw = LORA_BANDWIDTH_1;
        config.radio_mod.cr = LORA_CODING_RATE_1;
        config.radio_mod.ldro = apps_common_compute_lora_ldro( LORA_SPREADING_FACTOR_1, LORA_BANDWIDTH_1 );
    }
    else if (config_label == CONFIG_2)
    {
        config.pa_pwr_cfg = (lr11xx_board_pa_pwr_cfg_t*) lr11xx_board_get_pa_pwr_cfg( RF_FREQ_IN_HZ_2, TX_OUTPUT_POWER_DBM_2 );
        config.output_p_dbm = TX_OUTPUT_POWER_DBM_2;
        config.freq_in_hz = RF_FREQ_IN_HZ_2;
        config.radio_mod.sf = LORA_SPREADING_FACTOR_2;
        config.radio_mod.bw = LORA_BANDWIDTH_2;
        config.radio_mod.cr = LORA_CODING_RATE_2;
        config.radio_mod.ldro = apps_common_compute_lora_ldro( LORA_SPREADING_FACTOR_2, LORA_BANDWIDTH_2 );
    }
    else
    {
        ret = -EIO;
    }

    return ret;
}

static int ping_pong_radio_reinit( void )
{
    int ret = 0;

    if( config.pa_pwr_cfg == NULL )
    {
        LOG_ERR( "Invalid target frequency or power level" );
        return -EIO;
    }

    ret = lr11xx_radio_set_rf_freq( context, config.freq_in_hz );
    if(ret)
    {
        LOG_ERR("Failed set RF freq.");
    }

    ret = lr11xx_radio_set_rssi_calibration( context, lr11xx_board_get_rssi_calibration_table( config.freq_in_hz ) );
    if(ret)
    {
        LOG_ERR("Failed set RSSI calibration.");
    }

    ret = lr11xx_radio_set_pa_cfg( context, &( config.pa_pwr_cfg->pa_config ) );
    if(ret)
    {
        LOG_ERR("Failed set PA config.");
    }

    ret = lr11xx_radio_set_lora_mod_params( context, &config.radio_mod);
    if(ret)
    {
        LOG_ERR("Failed set lora mod params.");
    }

    return ret;
}

static void ping_pong_lr11xx_receive( const void* context, uint8_t* buffer, uint8_t* size, uint32_t* rx_count )
{
    lr11xx_radio_rx_buffer_status_t rx_buffer_status;
    lr11xx_radio_pkt_status_lora_t  pkt_status_lora;

    lr11xx_radio_get_rx_buffer_status( context, &rx_buffer_status );
    lr11xx_regmem_read_buffer8( context, buffer, rx_buffer_status.buffer_start_pointer,
                                rx_buffer_status.pld_len_in_bytes );
    *size = rx_buffer_status.pld_len_in_bytes;

    memcpy(rx_count, buffer+prefix_size, sizeof(*rx_count));

    lr11xx_radio_get_lora_pkt_status( context, &pkt_status_lora );

    printk( "{\"type\": \"lr\", \"config_id\": \"%02X\", \"freq\": %u, \"P\": %i, \"SF\": \"%02X\", \"BW\": \"%02X\", \"CR\": \"%02X\", \"RSSI\": %i, \"S_RSSI\": %i, \"SNR\": %i, \"count\": %d}\n",
                                                                                                     current_config,
                                                                                                     config.freq_in_hz,
                                                                                                     config.output_p_dbm,
                                                                                                     config.radio_mod.sf,
                                                                                                     config.radio_mod.bw,
                                                                                                     config.radio_mod.cr,
                                                                                                     pkt_status_lora.rssi_pkt_in_dbm,
                                                                                                     pkt_status_lora.signal_rssi_pkt_in_dbm,
                                                                                                     pkt_status_lora.snr_pkt_in_db,
                                                                                                     *rx_count);
}

static void initialize_bt_adv(void)
{
    int err;

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return;
	}

    /* Init device name */
    const char _name[] = DEVICE_NAME;
    uint8_t name_idx = 0;
    for (int i = 0; i < sizeof(DEVICE_NAME) - 1; i++)
    {
        device_name[i] = _name[i];
        name_idx++;
    }
    // Get advertisement mac address
    size_t count = 6;
    bt_id_get(&bt_addr, &count);
    LOG_INF("MAC addr: %x:%x:%x:%x:%x:%x", bt_addr.a.val[0], bt_addr.a.val[1], bt_addr.a.val[2], bt_addr.a.val[3], bt_addr.a.val[4], bt_addr.a.val[5]);

    for (int i = 2; i >=0 ; i--)
    {
        char tmp[3];
        sprintf(&tmp[0], "%02x", bt_addr.a.val[i]);
        device_name[name_idx] = tmp[0];
        name_idx++;
        device_name[name_idx] = tmp[1];
        name_idx++;
    }

    /* Init adv data */
    adv_data[0] = BT_MANUFACTURER_DATA & 0x00ff;
    adv_data[1] = BT_MANUFACTURER_DATA >> 8;

    /* Start advertising */
    err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)\n", err);
        return;
    }
}

static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type,
		    struct net_buf_simple *buf)
{
	counter++;
    LOG_INF("BT scan MAC: %x:%x:%x:%x:%x:%x rssi: %d", addr->a.val[0], addr->a.val[1], addr->a.val[2], addr->a.val[3], addr->a.val[4], addr->a.val[5], rssi);
    printk( "{\"type\": \"bt\",  \"RSSI\": %i, \"count\": %d}\n", rssi, counter);

    bt_le_scan_stop();
    bt_scan_done = true;
}

static void bt_scan_timer_handler(struct k_timer *dummy)
{
    bt_scan_timeout = true;
}

static void ping_pong_wait_on_bt_scan( void )
{
    bt_scan_timeout = false;
    bt_scan_done = false;
    k_timer_start(&bt_scan_timer, K_MSEC(BT_SCAN_TIMEOUT), K_NO_WAIT);

    bt_le_scan_start(&scan_param, scan_cb);

    while(!bt_scan_done && !bt_scan_timeout)
    {
        k_sleep(K_MSEC(1));
    }

    if(bt_scan_timeout)
    {
        bt_le_scan_stop();
        LOG_INF("BT scan timeout");
    }

    k_timer_stop(&bt_scan_timer);
}