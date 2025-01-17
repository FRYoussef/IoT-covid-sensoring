/**
 * Author: Youssef El Faqir El Rhazoui
 * Date: January 26th 2021
 **/

#ifndef _GATT_TABLE_
#define _GATT_TABLE_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "si7021_sensor/si7021.h"
#include "ccs811_sensor/ccs811.h"
#include "ble_beacon_counter/ble_beacon_counter.h"

/* Attributes State Machine */
enum GattAttr{
    IDX_SVC,

    IDX_CHAR_CO2, // air CO2 characteristic  
    IDX_CHAR_CO2_VAL, // air CO2 value
    IDX_CHAR_CO2_T_CFG, // Time sensoring configuration in s
    IDX_CHAR_CO2_ENB, // CO2 enable/disable

    IDX_CHAR_TEMP, // temperature characteristic  
    IDX_CHAR_TEMP_VAL, // temperature value
    IDX_CHAR_TEMP_T_CFG, // Time sensoring configuration in s
    IDX_CHAR_TEMP_ENB, // temperature enable/disable

    IDX_CHAR_HUM, // humidity characteristic  
    IDX_CHAR_HUM_VAL, // humidity value
    IDX_CHAR_HUM_T_CFG, // Time sensoring configuration in s
    IDX_CHAR_HUM_ENB, // humidity enable/disable

    IDX_CHAR_DEV, // number of devices characteristic
    IDX_CHAR_DEV_VAL, // number of devices value
    IDX_CHAR_DEV_D_CFG, // distance sensoring configuration in meters
    IDX_CHAR_DEV_ENB, // number of devices enable/disable

    SEN_IDX_NB, // table elements
};

#define PROFILE_NUM                 1 // number of profiles
#define PROFILE_APP_IDX             0 // profile index
#define ESP_APP_ID                  0x55 // Application Profile ID
#define DEVICE_NAME                 "ESP_GATT_SENSORING"
#define SVC_INST_ID                 0

/* The max length of characteristic value. When the GATT client performs a write or prepare write operation,
*  the data length must be less than GATT_CHAR_VAL_LEN_MAX. 
*/
#define GATT_CHAR_VAL_LEN_MAX 500
#define PREPARE_BUF_MAX_SIZE        1024
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)

static uint8_t adv_config_done       = 0;

uint16_t sensoring_handle_table[SEN_IDX_NB];

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static uint8_t raw_adv_data[] = {
        /* flags */
        0x02, 0x01, 0x06,
        /* tx power*/
        0x02, 0x0a, 0xeb,
        /* service uuid */
        0x03, 0x03, 0xFF, 0x00,
        /* device name */
        0x0f, 0x09, 'E', 'S', 'P', '_', 'S','E','N','S','O','R','I','N','G', '_'
};
static uint8_t raw_scan_rsp_data[] = {
        /* flags */
        0x02, 0x01, 0x06,
        /* tx power */
        0x02, 0x0a, 0xeb,
        /* service uuid */
        0x03, 0x03, 0xFF,0x00
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min         = 0x20,
    .adv_int_max         = 0x40,
    .adv_type            = ADV_TYPE_IND,
    .own_addr_type       = BLE_ADDR_TYPE_PUBLIC,
    .channel_map         = ADV_CHNL_ALL,
    .adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
					esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst sensoring_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/* control args and queue events */
static si7021_control_args_t *si7021_control;
static si7021_event_t si7021_ev;
static ccs811_control_args_t *ccs811_control;
static ccs811_event_t ccs811_ev;
static beacon_control_args_t *beacon_control;

/* Service */
static const uint16_t GATTS_SERVICE_UUID           = 0x00FF;
static const uint16_t GATTS_CHAR_UUID_CO2          = 0xFF01;
static const uint16_t GATTS_CHAR_UUID_TEMP         = 0xFF02;
static const uint16_t GATTS_CHAR_UUID_HUM          = 0xFF03;
static const uint16_t GATTS_CHAR_UUID_DEV          = 0xFF04;

static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_read                = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_write               = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static uint8_t co2_char_value[2]                   = {0x00, 0x00};
static uint8_t temp_char_value[2]                  = {0x00, 0x00}; // temp_char_value[0] = int part; temp_char_value[1] decimal part
static uint8_t hum_char_value[2]                   = {0x00, 0x00};
static uint8_t dev_char_value[2]                   = {0x00, 0x00};
static uint8_t co2_ccc[2]                          = {0x05, 0x00};
static uint8_t temp_ccc[2]                         = {0x05, 0x00};
static uint8_t hum_ccc[2]                          = {0x05, 0x00};
static uint8_t dev_ccc[2]                          = {0x05, 0x00};
static uint8_t co2_enb[1]                          = {0x01};
static uint8_t temp_enb[1]                         = {0x01};
static uint8_t hum_enb[1]                          = {0x01};
static uint8_t dev_enb[1]                          = {0x01};

/* Full Database Description - Used to add attributes into the database */
static const esp_gatts_attr_db_t gatt_db[SEN_IDX_NB] =
{
    // Service Declaration
    [IDX_SVC]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID), (uint8_t *)&GATTS_SERVICE_UUID}},

    /* Characteristic Declaration */
    [IDX_CHAR_CO2]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    /* Characteristic Value */
    [IDX_CHAR_CO2_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_CO2, ESP_GATT_PERM_READ,
      GATT_CHAR_VAL_LEN_MAX, sizeof(co2_char_value), (uint8_t *)co2_char_value}},

    /* Client Characteristic Configuration Descriptor */
    [IDX_CHAR_CO2_T_CFG]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(co2_ccc), (uint8_t *)co2_ccc}},

    /* Client Characteristic Configuration Descriptor */
    [IDX_CHAR_CO2_ENB]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(co2_enb), (uint8_t *)co2_enb}},

    /* Characteristic Declaration */
    [IDX_CHAR_TEMP]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    /* Characteristic Value */
    [IDX_CHAR_TEMP_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEMP, ESP_GATT_PERM_READ,
      GATT_CHAR_VAL_LEN_MAX, sizeof(temp_char_value), (uint8_t *)temp_char_value}},

    /* Client Characteristic Configuration Descriptor */
    [IDX_CHAR_TEMP_T_CFG]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(temp_ccc), (uint8_t *)temp_ccc}},

    /* Client Characteristic Configuration Descriptor */
    [IDX_CHAR_TEMP_ENB]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(temp_enb), (uint8_t *)temp_enb}},

    /* Characteristic Declaration */
    [IDX_CHAR_HUM]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    /* Characteristic Value */
    [IDX_CHAR_HUM_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_HUM, ESP_GATT_PERM_READ,
      GATT_CHAR_VAL_LEN_MAX, sizeof(hum_char_value), (uint8_t *)hum_char_value}},

    /* Client Characteristic Configuration Descriptor */
    [IDX_CHAR_HUM_T_CFG]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(hum_ccc), (uint8_t *)hum_ccc}},

    /* Client Characteristic Configuration Descriptor */
    [IDX_CHAR_HUM_ENB]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(hum_enb), (uint8_t *)hum_enb}},

    /* Characteristic Declaration */
    [IDX_CHAR_DEV]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    /* Characteristic Value */
    [IDX_CHAR_DEV_VAL] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_DEV, ESP_GATT_PERM_READ,
      GATT_CHAR_VAL_LEN_MAX, sizeof(dev_char_value), (uint8_t *)dev_char_value}},

    /* Client Characteristic Configuration Descriptor */
    [IDX_CHAR_DEV_D_CFG]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(dev_ccc), (uint8_t *)dev_ccc}},

    /* Client Characteristic Configuration Descriptor */
    [IDX_CHAR_DEV_ENB]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(dev_enb), (uint8_t *)dev_enb}},
};

void configure_gatt_server(si7021_control_args_t *c1, ccs811_control_args_t *c2, beacon_control_args_t *c3);

#endif