# Signal mapper example

The following example is a custom implementation of the classic ping-pong example between two LR11xx shields. In example we support classical random arraignment of roles between two boards: master and slave. Once roles are determined, ping messages and pong responses are send between two parties, switching up the configuration on each cycle. After both config cycles are done, BT scan is performed on both ends and loop repeats. 

## Installation
You will need two `nrf52840dk_nrf52840` boards along with two `lr1120` development kit shields with appropriate LoRa, WiFi and BlueTooth antennas. You will also need `SWDR001-Zephyr` driver - follow the main README for installation instructions. 

Once you have repository cloned and driver added to the `west.yaml` file, `cd` into the sample folder `samples\ping_pong_irnas`. To build the program for the `nrf52840dk_nrf52840` board:

```
west build -b nrf52840dk_nrf52840
```

and flash the board:

```
west flash
```

Once you flashed both boards, open two separate terminal and observe uart report on received packets via two LR configurations and BT scan.

## Debug

By defoult, config setting:

```
CONFIG_LOG_DEFAULT_LEVEL=0
```

is recommended for scanning as only result strings will be displayed. For development purposes set the desired log level. 

## Operation

Main operation functionality:
1. Start BLE advertisement.
2. Initialize LR1120 to `CONFIG_1` option.
3. Start ping-pong procedure to determine master-slave status.
4. Store remote MAC address and configure BLE scanning options.
5. Loop over ping-pong cycle:
    * Send - receive on CONFIG_1
    * Send - receive on CONFIG_2
    * Perform BT scan


### LR1120 Init
Example will initialize the LR1120 chip using standard protocol and default settings that can be observed in `samples\common` folder. During operation config will be changed after ech sucessful send\receive cycle. 

Two config options are labeled wit valid enum options:

```
enum {
    CONFIG_1 = 0,
    CONFIG_2 = 1
};
```

### LR1120 configuration change 
Variable config options are defined in two sets with macros:

```
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
```

and are stored in the private type variable:

```
typedef struct radio_config {
    lr11xx_board_pa_pwr_cfg_t* pa_pwr_cfg;
    int8_t output_p_dbm;
    uint32_t freq_in_hz;
    lr11xx_radio_mod_params_lora_t radio_mod;
} radio_config;
```

When configuration is changed, function `ping_pong_change_config( int config_label)` switches up configuration in the `radio_config config` variable, then `ping_pong_radio_reinit( void )` reinits the LR1120 chip.

### BLE advertisement and scanning
Upon booting, BT advertisement will start. Device will setup its scan parameters after its status is determined and first lora message, containing remote device MAC address is received. `BT_LE_SCAN_OPT_FILTER_ACCEPT_LIST` option is used for scanning and remote MAC is added on the accept list, after it is received. 
