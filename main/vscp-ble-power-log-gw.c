/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/****************************************************************************
*
* Built on
* https://github.com/espressif/esp-idf/blob/master/examples/bluetooth/bluedroid/ble/gatt_server_service_table/tutorial/Gatt_Server_Service_Table_Example_Walkthrough.md
*
****************************************************************************/


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gptimer.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_task_wdt.h"
#include "driver/temperature_sensor.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "vscp-ble-power-log-gw.h"
#include "esp_gatt_common_api.h"

#define TAG "VSCP_PWR_LOG_GW"

#define PROFILE_NUM                 1
#define PROFILE_APP_IDX             0
/*
  The ESP_APP_ID is application ID. It is used to distinguish between application layer callbacks. 
  We will fill in more details in doc later.
  https://github.com/espressif/esp-idf/issues/7838
*/
#define ESP_APP_ID                  0x55
#define SAMPLE_DEVICE_NAME          "PwrLogGw"
#define SVC_INST_ID                 0

/* The max length of characteristic value. When the GATT client performs a write or prepare write operation,
*  the data length must be less than PWRLOG_CHAR_VAL_LEN_MAX.
*/
#define PWRLOG_CHAR_VAL_LEN_MAX  500
#define PREPARE_BUF_MAX_SIZE          1024
#define CHAR_DECLARATION_SIZE         (sizeof(uint8_t))

#define ADV_CONFIG_FLAG               (1 << 0)
#define SCAN_RSP_CONFIG_FLAG          (1 << 1)

static uint8_t adv_config_done  = 0;

// Table with char handles
uint16_t attr_table[HRS_IDX_NB]; 

typedef struct {
  uint8_t        *prepare_buf;
  int             prepare_len;
} prepare_type_env_t;

static prepare_type_env_t prepare_write_env;

#define CONFIG_SET_RAW_ADV_DATA

#ifdef CONFIG_SET_RAW_ADV_DATA

static uint8_t raw_adv_data[] = {
  /* flags  (0x01) */
  0x02, 0x01, 0x06,
  /* tx power (0x0a) */
  0x02, 0x0a, 0xeb,
  /* service uuid  (00x03) */
  0x03, 0x03, 0xFF, 0x00,
  /* device name (0x09) */
  0x09, 0x09, 'P', 'w', 'r', 'L', 'o', 'g', 'G', 'w'
};
static uint8_t raw_scan_rsp_data[] = {
  /* flags  (0x01) */
  0x02, 0x01, 0x06,
  /* tx power (0x0a) */
  0x02, 0x0a, 0xeb,
  /* service uuid  (0x03 )*/
  0x03, 0x03, 0xFF,0x00
};

#else

static uint8_t service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

/* The length of adv data must be less than 31 bytes */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006,  // slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval        = 0x0010,  // slave connection max interval, Time = max_interval * 1.25 msec
    .appearance          = 0x00,
    .manufacturer_len    = 0,       // TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL,    // test_manufacturer,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp        = true,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006,
    .max_interval        = 0x0010,
    .appearance          = 0x00,
    .manufacturer_len    = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

#endif // CONFIG_SET_RAW_ADV_DATA 

static esp_ble_adv_params_t adv_params = {
    .adv_int_min         = 0x20,  // Minimum advertising interval for undirected and low duty cycle directed advertising.
    .adv_int_max         = 0x40,  // Maximum advertising interval for undirected and low duty cycle directed advertising.
    .adv_type            = ADV_TYPE_IND,
    .own_addr_type       = BLE_ADDR_TYPE_PUBLIC,
    .channel_map         = ADV_CHNL_ALL,
    .adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst 
{
    esp_gatts_cb_t gatts_cb;    // Callback function
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
					                                esp_gatt_if_t gatts_if, 
                                          esp_ble_gatts_cb_param_t *param);

/* 
  One gatt-based profile one app_id and one gatts_if, this array will store 
  the gatts_if returned by ESP_GATTS_REG_EVT 
*/
static struct gatts_profile_inst heart_rate_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

uint16_t mcu_temp_value;

/* Service */
static const uint16_t GATTS_SERVICE_UUID_APP      = 0x00FF;
static const uint16_t GATTS_CHAR_UUID_A           = 0xFF01;
static const uint16_t GATTS_CHAR_UUID_B           = 0xFF02;
static const uint16_t GATTS_CHAR_UUID_C           = 0xFF03;
static const uint16_t GATTS_CHAR_UUID_TEMP       = 0xFF04;

static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_read                = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_write               = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_notify         = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t interval_measurement_ccc[2]   = {0x00, 0x00};
static const uint8_t char_value[4]                 = {0x21, 0x22, 0x23, 0x24};
static const uint8_t char_value_temp[2]            = {0x00, 0x00}; 



/* Full Database Description - Used to add attributes into the database */
static const esp_gatts_attr_db_t gatt_db[HRS_IDX_NB] =
{
    // Service Declaration
    [IDX_SVC]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_APP), (uint8_t *)&GATTS_SERVICE_UUID_APP}},
    
    // ------------------------------------------------------------------------

    /* Characteristic Declaration */
    [IDX_CHAR_A]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_A] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_A, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      PWRLOG_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

    

    // ------------------------------------------------------------------------

    /* Characteristic Declaration */
    [IDX_CHAR_B]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_B]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_B, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      PWRLOG_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

    // ------------------------------------------------------------------------

    /* Characteristic Declaration */
    [IDX_CHAR_C]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write}},

    /* Characteristic Value */
    [IDX_CHAR_VAL_C]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_C, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      PWRLOG_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

    // ------------------------------------------------------------------------

    // Characteristic Declaration for MCU temperature 
    [IDX_CHAR_TEMP]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    /* Characteristic Value ESP_GATT_RSP_BY_APP*/
    [IDX_CHAR_VAL_TEMP]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEMP, ESP_GATT_PERM_READ,
      PWRLOG_CHAR_VAL_LEN_MAX, sizeof(char_value_temp), (uint8_t *)char_value_temp}},

    /* Client Characteristic Configuration Descriptor */
    [IDX_CHAR_CFG_TEMP]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(interval_measurement_ccc), (uint8_t *)interval_measurement_ccc}},  
};

static void gap_event_handler( esp_gap_ble_cb_event_t event, 
                                esp_ble_gap_cb_param_t *param)
{
    switch (event) {

#ifdef CONFIG_SET_RAW_ADV_DATA
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0) {
              esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0) {
               esp_ble_gap_start_advertising(&adv_params);
            }
            break;
#else
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0) {
              esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0) {
              esp_ble_gap_start_advertising(&adv_params);
            }
            break;
#endif
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            /* advertising start complete event to indicate advertising start successfully or failed */
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
              ESP_LOGE(TAG, "advertising start failed");
            }
            else{
              ESP_LOGI(TAG, "advertising start successfully");
            }
            break;

        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
              ESP_LOGE(TAG, "Advertising stop failed");
            }
            else {
              ESP_LOGI(TAG, "Stop adv successfully\n");
            }
            break;

        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
            break;

        default:
            break;
    }
}

///////////////////////////////////////////////////////////////////////////////
// prepare_write_event_env
//

void prepare_write_event_env( esp_gatt_if_t gatts_if, 
                                prepare_type_env_t *prepare_write_env, 
                                esp_ble_gatts_cb_param_t *param )
{
    ESP_LOGI(TAG, "prepare write, handle = %d, value len = %d", param->write.handle, param->write.len);
    esp_gatt_status_t status = ESP_GATT_OK;
    if (prepare_write_env->prepare_buf == NULL) {
        prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
        prepare_write_env->prepare_len = 0;
        if (prepare_write_env->prepare_buf == NULL) {
            ESP_LOGE(TAG, "%s, Gatt_server prep no mem", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    } 
    else {
        if(param->write.offset > PREPARE_BUF_MAX_SIZE) {
            status = ESP_GATT_INVALID_OFFSET;
        } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
            status = ESP_GATT_INVALID_ATTR_LEN;
        }
    }

    /* Send response when param->write.need_rsp is true */
    if (param->write.need_rsp) {
        esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
        if (gatt_rsp != NULL) {
          gatt_rsp->attr_value.len = param->write.len;
          gatt_rsp->attr_value.handle = param->write.handle;
          gatt_rsp->attr_value.offset = param->write.offset;
          gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
          memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
          esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
          if (response_err != ESP_OK) {
              ESP_LOGE(TAG, "Send response error");
          }
          free(gatt_rsp);
        }
        else{
          ESP_LOGE(TAG, "%s, malloc failed", __func__);
        }
    }

    if (status != ESP_GATT_OK) {
      return;
    }
    
    memcpy(prepare_write_env->prepare_buf + param->write.offset,
           param->write.value,
           param->write.len);

    prepare_write_env->prepare_len += param->write.len;

}

///////////////////////////////////////////////////////////////////////////////
// exec_write_event_env
//

void exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
  if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_write_env->prepare_buf) {
      esp_log_buffer_hex(TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
  }
  else{
      ESP_LOGI(TAG,"ESP_GATT_PREP_WRITE_CANCEL");
  }

  if (prepare_write_env->prepare_buf) {
      free(prepare_write_env->prepare_buf);
      prepare_write_env->prepare_buf = NULL;
  }
  
  prepare_write_env->prepare_len = 0;
}

///////////////////////////////////////////////////////////////////////////////
// gatts_profile_event_handler
//

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, 
                                          esp_gatt_if_t gatts_if, 
                                          esp_ble_gatts_cb_param_t *param)
{
    switch (event) {

        case ESP_GATTS_REG_EVT:{
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
            if (set_dev_name_ret) {
                ESP_LOGE(TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }

#ifdef CONFIG_SET_RAW_ADV_DATA
            esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
            if (raw_adv_ret) {
                ESP_LOGE(TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
            if (raw_scan_ret) {
                ESP_LOGE(TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
#else
            // config adv data
            esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
            if (ret) {
                ESP_LOGE(TAG, "config adv data failed, error code = %x", ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            // config scan response data
            ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
            if (ret) {
                ESP_LOGE(TAG, "config scan response data failed, error code = %x", ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
#endif
            esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, HRS_IDX_NB, SVC_INST_ID);
            if (create_attr_ret) {
                ESP_LOGE(TAG, "create attr table failed, error code = %x", create_attr_ret);
            }
          }
       	  break;

        case ESP_GATTS_READ_EVT: {

          ESP_LOGI(TAG, "GATT_READ_EVT, conn_id %u, trans_id %lu, handle %d\n", 
                      param->read.conn_id, 
                      param->read.trans_id, 
                      param->read.handle);

          //int attrIndex;
          // uint16_t getAttributeIndexByWifiHandle(uint16_t attributeHandle);
          // if( ( attrIndex = getAttributeIndexByMyHandle(param->read.handle)) < MY_SERV_NUM_ATTR ) {
            
          // }

          // esp_ble_gatts_set_attr_value(param->read.handle, 
          //                          2, 
          //                          (const uint8_t *)&mcu_temp_value);
          
          // esp_gatt_rsp_t rsp;
          // memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
          // if (attr_table[IDX_CHAR_VAL_TEMP] == param->read.handle ) {
          //   rsp.attr_value.handle = param->read.handle;
          //   rsp.attr_value.len = 2;
          //   rsp.attr_value.offset = param->read.offset;
          //   rsp.attr_value.value[0] = mcu_temp_value >> 8;
          //   rsp.attr_value.value[1] = mcu_temp_value & 0xFF;
          // }
          // esp_ble_gatts_send_response(gatts_if, 
          //                               param->read.conn_id, 
          //                               param->read.trans_id,
          //                               ESP_GATT_OK, &rsp);
          break;
        }

        case ESP_GATTS_SET_ATTR_VAL_EVT:
          // Called everytime a nw value is set
          ESP_LOGI(TAG, "GATT_SET_ATTR_VAL_EVT");
          break;
            
        case ESP_GATTS_WRITE_EVT:

          if ( !param->write.is_prep ) {              
            
            // the data length of gattc write  must be less than PWRLOG_CHAR_VAL_LEN_MAX.
            ESP_LOGI(TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);
            esp_log_buffer_hex(TAG, param->write.value, param->write.len);
            
            if (attr_table[IDX_CHAR_CFG_A] == param->write.handle && param->write.len == 2) {

              uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];

              if (descr_value == 0x0001) {

                ESP_LOGI(TAG, "notify enable");
                
                uint8_t notify_data[15];
                for (int i = 0; i < sizeof(notify_data); ++i) {
                    notify_data[i] = i % 0xff;
                }
                // the size of notify_data[] need less than MTU size
                esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, attr_table[IDX_CHAR_VAL_A],
                                        sizeof(notify_data), notify_data, false);

              } 
              else if (descr_value == 0x0002) {

                ESP_LOGI(TAG, "indicate enable");
                
                uint8_t indicate_data[15];
                for (int i = 0; i < sizeof(indicate_data); ++i) {
                  indicate_data[i] = i % 0xff;
                }
                // the size of indicate_data[] need less than MTU size
                esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, attr_table[IDX_CHAR_VAL_A],
                                    sizeof(indicate_data), indicate_data, true);

              }
              else if (descr_value == 0x0000) {
                ESP_LOGI(TAG, "notify/indicate disable ");
              } 
              else {
                ESP_LOGE(TAG, "unknown descr value");
                esp_log_buffer_hex(TAG, param->write.value, param->write.len);
              }

            }

            // send response when param->write.need_rsp is true 
            if (param->write.need_rsp) {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
            }
          }
          else{
              // handle prepare write 
              prepare_write_event_env(gatts_if, &prepare_write_env, param);
          }
          break;

        case ESP_GATTS_EXEC_WRITE_EVT:
          // the length of gattc prepare write data must be less than PWRLOG_CHAR_VAL_LEN_MAX.
          ESP_LOGI(TAG, "ESP_GATTS_EXEC_WRITE_EVT");
          exec_write_event_env(&prepare_write_env, param);
          break;

        case ESP_GATTS_MTU_EVT:
          ESP_LOGI(TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
          break;

        case ESP_GATTS_CONF_EVT:
          ESP_LOGI(TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
          break;

        case ESP_GATTS_START_EVT:
          ESP_LOGI(TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
          break;

        case ESP_GATTS_CONNECT_EVT:
          ESP_LOGI(TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
          esp_log_buffer_hex(TAG, param->connect.remote_bda, 6);
          esp_ble_conn_update_params_t conn_params = {0};
          memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
          /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
          conn_params.latency = 0;
          conn_params.max_int = 0x20;     // max_int = 0x20*1.25ms = 40ms
          conn_params.min_int = 0x10;     // min_int = 0x10*1.25ms = 20ms
          conn_params.timeout = 400;      // timeout = 400*10ms = 4000ms
          // start sent the update connection parameters to the peer device.
          esp_ble_gap_update_conn_params(&conn_params);
          break;

        case ESP_GATTS_DISCONNECT_EVT:
          ESP_LOGI(TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
          esp_ble_gap_start_advertising(&adv_params);
          break;

        case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
          if (param->add_attr_tab.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
          }
          else if (param->add_attr_tab.num_handle != HRS_IDX_NB) {
            ESP_LOGE(TAG, "create attribute table abnormally, num_handle (%d) \
                    doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, HRS_IDX_NB);
          }
          else {
            ESP_LOGI(TAG, "create attribute table successfully, the number handle = %d\n",param->add_attr_tab.num_handle);
            memcpy(attr_table, param->add_attr_tab.handles, sizeof(attr_table));
            esp_ble_gatts_start_service(attr_table[IDX_SVC]);
          }
          break;
        }
        case ESP_GATTS_STOP_EVT:
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        case ESP_GATTS_UNREG_EVT:
        case ESP_GATTS_DELETE_EVT:
        default:
          break;
    }
}

///////////////////////////////////////////////////////////////////////////////
// gatts_event_handler
//

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
  /* If event is register event, store the gatts_if for each profile */
  if (event == ESP_GATTS_REG_EVT) {
    if (param->reg.status == ESP_GATT_OK) {
      heart_rate_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
    } else {
      ESP_LOGE(TAG, "reg app failed, app_id %04x, status %d",
              param->reg.app_id,
              param->reg.status);
      return;
    }
  }
  do {
    int idx;
    for (idx = 0; idx < PROFILE_NUM; idx++) {
      /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
      if (gatts_if == ESP_GATT_IF_NONE || gatts_if == heart_rate_profile_tab[idx].gatts_if) {
          if (heart_rate_profile_tab[idx].gatts_cb) {
              heart_rate_profile_tab[idx].gatts_cb(event, gatts_if, param);
          }
      }
    }
  } while (0);
}

static bool IRAM_ATTR main_timer_on_alarm_cb( gptimer_handle_t timer, 
                                                const gptimer_alarm_event_data_t *edata, 
                                                void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
    QueueHandle_t queue = (QueueHandle_t)user_data;
    
    // stop timer immediately
    gptimer_stop(timer);
    
    // Retrieve count value and send to queue
    timing_queue_element_t ele = {
        .event_count = edata->count_value
    };
    
    xQueueSendFromISR(queue, &ele, &high_task_awoken);

    // reconfigure alarm value
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = edata->alarm_value + 1000000, // alarm in next 1s
    };
    gptimer_set_alarm_action(timer, &alarm_config);

    // return whether we need to yield at the end of ISR
    return (high_task_awoken == pdTRUE);
}

///////////////////////////////////////////////////////////////////////////////
// app_main
//

void app_main(void)
{
  esp_err_t ret;
  timing_queue_element_t ele;

  /* Initialize NVS. */
  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK( ret );

  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

  QueueHandle_t queue = xQueueCreate(10, sizeof(timing_queue_element_t));
  if (!queue) {
      ESP_LOGE(TAG, "Creating queue failed");
      return;
  }

  // Timer

  ESP_LOGI(TAG, "Create timer handle");
  gptimer_handle_t gptimer = NULL;
  gptimer_config_t timer_config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1000000, // 1MHz, 1 tick=1us
  };
  ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

  gptimer_event_callbacks_t cbs = {
      .on_alarm = main_timer_on_alarm_cb,
  };
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, queue));

  ESP_LOGI(TAG, "Enable timer");
  ESP_ERROR_CHECK(gptimer_enable(gptimer));

  ESP_LOGI(TAG, "Install temperature sensor, expected temp ranger range: 10~50 ℃");
  temperature_sensor_handle_t temp_sensor = NULL;
  /*
     50 ~ 125 < 3
     20 ~ 100 < 2
    -10 ~ 80 < 1
    -30 ~ 50 < 2
    -40 ~ 20 < 3
  */  
  temperature_sensor_config_t temp_sensor_config = TEMPERAUTRE_SENSOR_CONFIG_DEFAULT(10, 50);
  ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_sensor));

  ESP_LOGI(TAG, "Enable temperature sensor");
  ESP_ERROR_CHECK(temperature_sensor_enable(temp_sensor));

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ret = esp_bt_controller_init(&bt_cfg);
  if (ret) {
    ESP_LOGE(TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
    return;
  }

  ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  if (ret) {
    ESP_LOGE(TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
    return;
  }

  ret = esp_bluedroid_init();
  if (ret) {
    ESP_LOGE(TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
    return;
  }

  ret = esp_bluedroid_enable();
  if (ret) {
    ESP_LOGE(TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
    return;
  }

  ret = esp_ble_gatts_register_callback(gatts_event_handler);
  if (ret) {
    ESP_LOGE(TAG, "gatts register error, error code = %x", ret);
    return;
  }

  ret = esp_ble_gap_register_callback(gap_event_handler);
  if (ret) {
    ESP_LOGE(TAG, "gap register error, error code = %x", ret);
    return;
  }

  ret = esp_ble_gatts_app_register(ESP_APP_ID);
  if (ret) {
    ESP_LOGE(TAG, "gatts app register error, error code = %x", ret);
    return;
  }

  esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
  if (local_mtu_ret) {
    ESP_LOGE(TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
  }

  //esp_task_wdt_init();

  float value;

  while(1) {
    ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_sensor, &value));
    mcu_temp_value = (uint16_t)(value * 100);
    ESP_LOGI(TAG, "Temperature value %.02f %X ℃", value, mcu_temp_value);
    //char_value_temp[0] = mcu_temp_value >> 8;
    //char_value_temp[1] = mcu_temp_value & 0xFF;
    
    //ESP_LOGI(TAG, "heart rate profile: handle %u", ttt);
    esp_ble_gatts_set_attr_value(attr_table[IDX_CHAR_VAL_TEMP], 
                                  2, 
                                  (uint8_t *)&mcu_temp_value);

    esp_ble_gatts_send_indicate(heart_rate_profile_tab[PROFILE_APP_IDX].gatts_if , 
                                  heart_rate_profile_tab[IDX_CHAR_TEMP].conn_id, 
                                  attr_table[IDX_CHAR_VAL_TEMP],
                                  sizeof(mcu_temp_value), 
                                  (uint8_t *)&mcu_temp_value, false);
    vTaskDelay(pdMS_TO_TICKS(1000));
    //esp_task_wdt_reset();
  };
}
