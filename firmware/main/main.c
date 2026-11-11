extern void
twai_transmit_task(void *arg);
/*
  File: main.c

  VSCP Wireless CAN4VSCP Gateway

  Main application entry point and system initialization for ESP32-C3 based
  VSCP CAN gateway with WiFi connectivity. This module handles:

  - System initialization and boot sequence
  - NVS (Non-Volatile Storage) configuration management
  - WiFi provisioning (BLE or SoftAP based)
  - CAN/TWAI driver initialization and message routing
  - Network service initialization (MQTT, TCP, UDP, Multicast, WebSockets)
  - Web server for configuration interface
  - Event handling for WiFi and provisioning events
  - Main application loop for message distribution

  This file is part of the VSCP (https://www.vscp.org)

  The MIT License (MIT)
  Copyright (C) 2021-2026 Ake Hedman, the VSCP project <info@vscp.org>

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#include <stdio.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include "freertos/semphr.h"

#include <driver/gpio.h>
#include "esp_task_wdt.h"
#include <esp_twai.h>
#include <esp_event.h>
#include <esp_http_server.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <esp_timer.h>
#include <esp_tls_crypto.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#include <lwip/sockets.h>
#include <lwip/ip4_addr.h>
#include <nvs_flash.h>

#include <network_provisioning/manager.h>

// Components: LED indicator
#include "led_indicator_blink_default.h"
#include "led_indicator_gpio.h"

// Components: Buttons
#include "iot_button.h"
#include "button_gpio.h"

#ifdef CONFIG_WCANG_PROV_TRANSPORT_BLE
#include <network_provisioning/scheme_ble.h>
#endif /* CONFIG_WCANG_PROV_TRANSPORT_BLE */

#ifdef CONFIG_WCANG_PROV_TRANSPORT_SOFTAP
#include <network_provisioning/scheme_softap.h>
#endif /* CONFIG_WCANG_PROV_TRANSPORT_SOFTAP */
#include "qrcode.h"

#include <vscp.h>
#include <vscp-firmware-helper.h>
#include <vscp-crc.h>

#include "can4vscp.h"
#include "tcpsrv.h"
#include "udpsrv.h"
#include "multicastsrv.h"
#include "websrv.h"
#include "websocksrv.h"
#include "mqtt.h"

#include "main.h"

#include "vscp-compiler.h"
#include "vscp-projdefs.h"

static const char *TAG          = "main";
static esp_netif_t *s_sta_netif = NULL;

// ============================================================================
//                           Global Variables
// ============================================================================

// NVS (Non-Volatile Storage) handle for persistent configuration
nvs_handle_t g_nvsHandle;

/**
 * @brief Global persistent configuration structure
 *
 * Contains all device configuration parameters including network settings,
 * CAN parameters, MQTT/UDP/TCP settings, and user credentials. Values are
 * initialized with defaults and loaded from NVS during startup.
 *
 * Modified via web interface and saved to NVS for persistence across reboots.
 */
node_persistent_config_t g_persistent = {
  .nodeName    = DEFAULT_NODE_NAME,
  .nodeZone    = DEFAULT_MODULE_ZONE,
  .nodeSubzone = DEFAULT_MODULE_SUBZONE,
  .guid        = { 0 }, // Default GUID is constructed from MAC address
  .bootCnt     = 0,
  .pmkLen      = 16,                       // AES128 (for future use)
  .pmk         = { 0 },                    // Default key is all nills
  .encryptLvl  = DEFAULT_ENCRYPTION_LEVEL, // 0 = none, 1 = AES128, 2 = AES192, 3 = AES256

  // CAN/TWAI
  .canSpeed  = DEFAULT_CAN_SPEED,
  .canMode   = DEFAULT_CAN_MODE,
  .canFilter = DEFAULT_CAN_FILTER,

  .logType      = DEFAULT_LOG_TYPE,               // Log type
  .logLevel     = DEFAULT_LOG_LEVEL,              // Log level
  .logRetries   = DEFAULT_LOG_RETRIES,            // Number of retries for log message send
  .logPort      = DEFAULT_LOG_PORT,               // Log server port
  .logUrl       = DEFAULT_LOG_URL,                // Log server address
  .logMqttTopic = DEFAULT_MQTT_LOG_PUBLISH_TOPIC, // MQTT topic for log messages

  .webPort     = DEFAULT_WEBSERVER_PORT,
  .webUser     = DEFAULT_WEBSERVER_USER,
  .webPassword = DEFAULT_WEBSERVER_PASSWORD,

  // WiFi
  .wifiPrimarySsid       = "",
  .wifiPrimaryPassword   = "",
  .wifiSecondarySsid     = "",
  .wifiSecondaryPassword = "",
  .wifiStaticEnable      = DEFAULT_WIFI_STATIC_ENABLE,
  .wifiStaticIp          = DEFAULT_WIFI_STATIC_IP,
  .wifiStaticNetmask     = DEFAULT_WIFI_STATIC_NETMASK,
  .wifiStaticGateway     = DEFAULT_WIFI_STATIC_GATEWAY,
  .wifiStaticDns         = DEFAULT_WIFI_STATIC_DNS,

  .enableVscpLink = DEFAULT_VSCP_LINK_ENABLE,
  .vscplinkPort   = DEFAULT_VSCP_LINK_PORT,
  .vscplinkUser   = DEFAULT_VSCP_LINK_USER,
  .vscplinkPw     = DEFAULT_VSCP_LINK_PASSWORD,

  .enableMqtt      = DEFAULT_MQTT_ENABLE,
  .enableMqttTls   = DEFAULT_MQTT_TLS_ENABLE,
  .mqttUrl         = DEFAULT_MQTT_URL,
  .mqttPort        = DEFAULT_MQTT_PORT,
  .mqttUser        = DEFAULT_MQTT_USER,
  .mqttPw          = DEFAULT_MQTT_PASSWORD,
  .mqttPubTopic    = DEFAULT_MQTT_PUBLISH,
  .mqttSubTopic    = DEFAULT_MQTT_SUBSCRIBE,
  .mqttPubLogTopic = DEFAULT_MQTT_LOG_PUBLISH_TOPIC, // Set in logging configuration
  .mqttClientId    = DEFAULT_MQTT_CLIENT_ID,
  .mqttCaCert      = DEFAULT_MQTT_CA_CERT,
  .mqttClientCert  = DEFAULT_MQTT_CLIENT_CERT,
  .mqttClientKey   = DEFAULT_MQTT_CLIENT_KEY,
  .mqttQos         = DEFAULT_MQTT_QOS,
  .mqttRetain      = DEFAULT_MQTT_RETAIN,
  .mqttFormat      = DEFAULT_MQTT_FORMAT,

  // Multicast
  .enableMulticast = DEFAULT_MULTICAST_ENABLE,
  .multicastUrl    = DEFAULT_MULTICAST_URL,
  .multicastPort   = DEFAULT_MULTICAST_PORT,
  .multicastTtl    = DEFAULT_MULTICAST_TTL,
  .bMcastEncrypt   = DEFAULT_MULTICAST_ENCRYPTION,
  .bHeartbeat      = DEFAULT_MULTICAST_HEARTBEAT,

  // UDP
  .enableUdpRx = DEFAULT_UDP_RX_ENABLE,
  .enableUdpTx = DEFAULT_UDP_TX_ENABLE,
  .udpUrl      = DEFAULT_UDP_URL,
  .udpPort     = DEFAULT_UDP_PORT,
  .bUdpEncrypt = DEFAULT_UDP_ENCRYPTION,

  // Websocket server protocol
  .enableWebsock = DEFAULT_WEBSOCKETS_ENABLE,
  .websockPort   = DEFAULT_WEBSOCKETS_PORT,
  .websockUser   = DEFAULT_WEBSOCKETS_USER,
  .websockPw     = DEFAULT_WEBSOCKETS_PASSWORD,
};

static led_indicator_handle_t led_handle_wifi   = NULL;
static led_indicator_handle_t led_handle_status = NULL;

/**
 * @brief Transport layer message queue structures
 *
 * Each transport mechanism (TCP, MQTT, Multicast, UDP, WebSockets) has its own
 * message queue for receiving CAN messages. Messages are distributed from the
 * CAN receive queue to all active transport queues.
 */
transport_t tr_tcpsrv[MAX_TCP_CONNECTIONS] = {}; // TCP/IP VSCP link protocol connections
transport_t tr_mqtt                        = {}; // MQTT client transport
transport_t tr_multicast                   = {}; // Multicast UDP transport
transport_t tr_udp                         = {}; // UDP broadcast transport
transport_t tr_websockets                  = {}; // WebSocket server transport

// Dedicated monitor/command-reply transport - always fed by twai_receive_task
transport_t tr_canapi  = {};
transport_t tr_monitor = {};

// Semaphore for controlling access to main event distribution task
SemaphoreHandle_t ctrl_task_sem;

// ============================================================================
//                        CAN/TWAI Message Queues
// ============================================================================

/**
 * @brief CAN message queues for transmit and receive
 *
 * tr_twai_tx: Outgoing CAN messages from all network interfaces
 * tr_twai_rx: Incoming CAN messages distributed to network interfaces
 */
static QueueHandle_t tr_twai_tx;
static QueueHandle_t tr_twai_rx;

// ============================================================================
//                           Web Server Handle
// ============================================================================

// HTTP server handle for configuration web interface
static httpd_handle_t server_httpd = NULL;

// ============================================================================
//                    WiFi Provisioning Security (Version 2)
// ============================================================================

#if CONFIG_WCANG_PROV_SECURITY_VERSION_2

#if CONFIG_WCANG_PROV_SEC2_DEV_MODE
#define WCANG_PROV_SEC2_USERNAME "testuser"
#define WCANG_PROV_SEC2_PWD      "testpassword"

/**
 * Salt and verifier for SEC2 provisioning authentication
 *
 * This salt/verifier pair has been generated for username = "testuser" and password = "testpassword"
 * IMPORTANT NOTE: For production cases, this must be unique to every device
 * and should come from device manufacturing partition.*/
static const char sec2_salt[] = { 0x2f, 0x3d, 0x3c, 0xf8, 0x0d, 0xbd, 0x0c, 0xa9,
                                  0x6f, 0x30, 0xb4, 0x4d, 0x89, 0xd5, 0x2f, 0x0e };

// 24*16 = 384 * 8 = 3072
static const char sec2_verifier[] = {
  0xf2, 0x9f, 0xc1, 0xf5, 0x28, 0x4a, 0x11, 0x74, 0xb4, 0x24, 0x09, 0x23, 0xd8, 0x27, 0xb7, 0x5a, 0x95, 0x3a, 0x99,
  0xed, 0xf4, 0x6e, 0xe9, 0x8c, 0x4f, 0x07, 0xf2, 0xf5, 0x43, 0x3d, 0x7f, 0x9a, 0x11, 0x60, 0x66, 0xaf, 0xcd, 0xa5,
  0xf6, 0xfa, 0xcb, 0x06, 0xe9, 0xc5, 0x3f, 0x4d, 0x77, 0x16, 0x4c, 0x68, 0x6d, 0x7f, 0x7c, 0xd7, 0xc7, 0x5a, 0x83,
  0xc0, 0xfb, 0x94, 0x2d, 0xa9, 0x60, 0xf0, 0x09, 0x11, 0xa0, 0xe1, 0x95, 0x33, 0xd1, 0x30, 0x7f, 0x82, 0x1b, 0x1b,
  0x0f, 0x6d, 0xf1, 0xdc, 0x93, 0x1c, 0x20, 0xa7, 0xc0, 0x8d, 0x48, 0x38, 0xff, 0x46, 0xb9, 0xaf, 0xf7, 0x93, 0x78,
  0xae, 0xff, 0xb8, 0x3b, 0xdf, 0x99, 0x7b, 0x64, 0x47, 0x02, 0xba, 0x01, 0x39, 0x0f, 0x5c, 0xd8, 0x4e, 0x6f, 0xc8,
  0xd0, 0x82, 0x7f, 0x2d, 0x33, 0x1a, 0x09, 0x65, 0x77, 0x85, 0xbc, 0x8a, 0x84, 0xe0, 0x46, 0x7e, 0x3b, 0x0e, 0x6e,
  0x3b, 0xdf, 0x70, 0x17, 0x70, 0x0a, 0xbc, 0x84, 0x67, 0xfa, 0xf9, 0x84, 0x53, 0xda, 0xb4, 0xca, 0x38, 0x71, 0xe4,
  0x06, 0xf6, 0x7d, 0xc8, 0x32, 0xbb, 0x91, 0x0c, 0xe7, 0xd3, 0x59, 0xb6, 0x03, 0xed, 0x8e, 0x0d, 0x91, 0x9c, 0x09,
  0xd7, 0x6f, 0xd5, 0xca, 0x55, 0xc5, 0x58, 0x0f, 0x95, 0xb5, 0x83, 0x65, 0x6f, 0x2d, 0xbc, 0x94, 0x0f, 0xbb, 0x0f,
  0xd3, 0x42, 0xa5, 0xfe, 0x15, 0x7f, 0xf9, 0xa8, 0x16, 0xe6, 0x58, 0x9b, 0x4c, 0x0f, 0xd3, 0x83, 0x2c, 0xac, 0xe4,
  0xbf, 0xa3, 0x96, 0x1e, 0xb6, 0x6f, 0x59, 0xe6, 0xd1, 0x0e, 0xd4, 0x27, 0xb6, 0x05, 0x34, 0xec, 0x8c, 0xf8, 0x72,
  0xbb, 0x04, 0x7b, 0xa4, 0x49, 0x3d, 0x6d, 0xa9, 0x99, 0xfc, 0x0a, 0x2b, 0xd8, 0x46, 0xa8, 0xd1, 0x46, 0x61, 0x5c,
  0x96, 0xd2, 0x43, 0xcd, 0xea, 0x7f, 0x6a, 0x50, 0x59, 0x0d, 0x0e, 0xa1, 0xb3, 0x94, 0x5a, 0x34, 0xe0, 0x1e, 0x95,
  0x56, 0x68, 0xb4, 0xbc, 0xf1, 0x08, 0x54, 0xcb, 0x42, 0x41, 0xc6, 0x78, 0xad, 0x71, 0x84, 0x1c, 0x29, 0xb8, 0x33,
  0x79, 0x1c, 0x10, 0xdd, 0x07, 0xc8, 0x91, 0x21, 0x85, 0x89, 0x76, 0xd7, 0x37, 0xdf, 0x5b, 0x19, 0x33, 0x4e, 0x17,
  0x67, 0x02, 0x0f, 0x1b, 0xb9, 0x2f, 0xa4, 0xdc, 0xdd, 0x75, 0x32, 0x96, 0x87, 0xdd, 0x66, 0xc3, 0x33, 0xc1, 0xfc,
  0x4c, 0x27, 0x63, 0xb9, 0x14, 0x72, 0x76, 0x65, 0xb8, 0x90, 0x2b, 0xeb, 0x7a, 0xde, 0x71, 0x97, 0xf3, 0x6b, 0xc9,
  0x8e, 0xdf, 0xfc, 0x6e, 0x13, 0xcc, 0x1b, 0x2b, 0x54, 0x1a, 0x6e, 0x3d, 0xe6, 0x1c, 0xec, 0x5d, 0xa1, 0xf1, 0xd4,
  0x86, 0x9d, 0xcd, 0xb9, 0xe8, 0x98, 0xf1, 0xe5, 0x16, 0xa5, 0x48, 0xe5, 0xec, 0x12, 0xe8, 0x17, 0xe2, 0x55, 0xb5,
  0xb3, 0x7c, 0xce, 0xfd
};
#endif

// ============================================================================
//                  Provisioning Security Helper Functions
// ============================================================================

/**
 * @brief Get SEC2 provisioning salt value
 *
 * Returns the salt value used for SEC2 (SRP6a) authentication during WiFi
 * provisioning. In development mode, returns hardcoded test salt. In production
 * mode, should retrieve device-specific salt from secure storage.
 *
 * @param[out] salt      Pointer to store salt buffer address
 * @param[out] salt_len  Pointer to store salt length
 *
 * @return ESP_OK on success, ESP_FAIL if not implemented
 */
static esp_err_t
wcang_get_sec2_salt(const char **salt, uint16_t *salt_len)
{
#if CONFIG_WCANG_PROV_SEC2_DEV_MODE
  ESP_LOGI(TAG, "Development mode: using hard coded salt");
  *salt     = sec2_salt;
  *salt_len = sizeof(sec2_salt);
  return ESP_OK;
#elif CONFIG_WCANG_PROV_SEC2_PROD_MODE
  ESP_LOGE(TAG, "Not implemented!");
  return ESP_FAIL;
#endif
}

/**
 * @brief Get SEC2 provisioning verifier value
 *
 * Returns the verifier value used for SEC2 (SRP6a) authentication during WiFi
 * provisioning. In development mode, returns hardcoded test verifier. In production
 * mode, should retrieve device-specific verifier from secure storage.
 *
 * The verifier is derived from the username, password, and salt using SRP6a protocol.
 *
 * @param[out] verifier      Pointer to store verifier buffer address
 * @param[out] verifier_len  Pointer to store verifier length
 *
 * @return ESP_OK on success, ESP_FAIL if not implemented
 */
static esp_err_t
wcang_get_sec2_verifier(const char **verifier, uint16_t *verifier_len)
{
#if CONFIG_WCANG_PROV_SEC2_DEV_MODE
  ESP_LOGI(TAG, "Development mode: using hard coded verifier");
  *verifier     = sec2_verifier;
  *verifier_len = sizeof(sec2_verifier);
  return ESP_OK;
#elif CONFIG_WCANG_PROV_SEC2_PROD_MODE
  /* This code needs to be updated with appropriate implementation to provide verifier */
  ESP_LOGE(TAG, "Not implemented!");
  return ESP_FAIL;
#endif
}
#endif

static void
init_watchdog_timer(void)
{
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms     = 10000, // 10-second timeout
    .idle_core_mask = 0,     // Monitor all cores
    .trigger_panic  = true   // Trigger panic on timeout
  };

  esp_err_t err = esp_task_wdt_init(&wdt_config);
  if (err == ESP_OK) {
    printf("Task Watchdog Timer initialized successfully.\n");
  }
  else {
    printf("Failed to initialize Task Watchdog Timer: %s\n", esp_err_to_name(err));
  }
}

// ============================================================================
//                    NVS Configuration Management
// ============================================================================

/**
 * @brief Initialize and load persistent configuration from NVS
 *
 * Opens the NVS namespace "config" and loads all device configuration parameters.
 * If a parameter doesn't exist in NVS, it's initialized with the default value
 * and written to NVS. The boot counter is incremented on each call.
 *
 * Configuration includes:
 * - Node name and GUID
 * - CAN/TWAI settings (mode, speed, filter)
 * - WiFi credentials (primary and secondary)
 * - Web server settings
 * - VSCP link protocol settings
 * - MQTT configuration (URL, credentials, TLS certificates)
 * - Multicast/UDP/WebSocket settings
 *
 * @note This function should be called once during system initialization
 * @note GUID is constructed from MAC address if not previously set
 */
static void
initPersistentStorage(void)
{
  esp_err_t rv;
  size_t length;

  rv = nvs_open("config", NVS_READWRITE, &g_nvsHandle);
  if (rv != ESP_OK) {
    ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(rv));
  }
  else {

    // * * * Module persistent configuration * * *

    // Load boot counter and increment it
    NVS_GET_OR_SET_DEFAULT(u32, nvs_get_u32, nvs_set_u32, g_nvsHandle, "bootCnt", g_persistent.bootCnt, 0, TAG, "%d");

    ESP_LOGD(TAG, "Updating restart counter in NVS ... ");
    g_persistent.bootCnt++;

    // Write updated counter value to nvs
    rv = nvs_set_u32(g_nvsHandle, "bootCnt", g_persistent.bootCnt);
    if (rv != ESP_OK) {
      ESP_LOGI(TAG, "Failed to read restart counter!");
    }

    // Load or set default node name
    NVS_GET_OR_SET_DEFAULT_STR(g_nvsHandle, "nodeName", g_persistent.nodeName, DEFAULT_NODE_NAME, TAG);

    // Load GUID, or construct from MAC address if not set
    length = 16;
    rv     = nvs_get_blob(g_nvsHandle, "guid", g_persistent.guid, &length);
    switch (rv) {
      case ESP_OK:
        ESP_LOGI(TAG,
                 "GUID: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                 g_persistent.guid[0],
                 g_persistent.guid[1],
                 g_persistent.guid[2],
                 g_persistent.guid[3],
                 g_persistent.guid[4],
                 g_persistent.guid[5],
                 g_persistent.guid[6],
                 g_persistent.guid[7],
                 g_persistent.guid[8],
                 g_persistent.guid[9],
                 g_persistent.guid[10],
                 g_persistent.guid[11],
                 g_persistent.guid[12],
                 g_persistent.guid[13],
                 g_persistent.guid[14],
                 g_persistent.guid[15]);
        break;

      case ESP_ERR_NVS_NOT_FOUND:
        ESP_LOGI(TAG, "GUID not found in nvs, writing default\n");
        memset(g_persistent.guid, 0, 16);
        break;

      default:
        ESP_LOGI(TAG, "Error (%s) reading GUID from nvs!\n", esp_err_to_name(rv));
        break;
    }

    // If GUID is all zero construct GUID
    if (!(g_persistent.guid[0] | g_persistent.guid[1] | g_persistent.guid[2] | g_persistent.guid[3] |
          g_persistent.guid[4] | g_persistent.guid[5] | g_persistent.guid[6] | g_persistent.guid[7] |
          g_persistent.guid[8] | g_persistent.guid[9] | g_persistent.guid[10] | g_persistent.guid[11] |
          g_persistent.guid[12] | g_persistent.guid[13] | g_persistent.guid[14] | g_persistent.guid[15])) {
      g_persistent.guid[0] = 0xff;
      g_persistent.guid[1] = 0xff;
      g_persistent.guid[2] = 0xff;
      g_persistent.guid[3] = 0xff;
      g_persistent.guid[4] = 0xff;
      g_persistent.guid[5] = 0xff;
      g_persistent.guid[6] = 0xff;
      g_persistent.guid[7] = 0xfe;
      rv                   = esp_efuse_mac_get_default(g_persistent.guid + 8);
      rv                   = nvs_set_blob(g_nvsHandle, "guid", g_persistent.guid, 16);
      ESP_LOGI(TAG,
               "Constructed GUID: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
               g_persistent.guid[0],
               g_persistent.guid[1],
               g_persistent.guid[2],
               g_persistent.guid[3],
               g_persistent.guid[4],
               g_persistent.guid[5],
               g_persistent.guid[6],
               g_persistent.guid[7],
               g_persistent.guid[8],
               g_persistent.guid[9],
               g_persistent.guid[10],
               g_persistent.guid[11],
               g_persistent.guid[12],
               g_persistent.guid[13],
               g_persistent.guid[14],
               g_persistent.guid[15]);
    }
  }

  // Primary key pmk
  length = 16;
  rv     = nvs_get_blob(g_nvsHandle, "pmk", g_persistent.pmk, &length);
  switch (rv) {
    case ESP_OK:
      ESP_LOGI(TAG, "Primary key: xxxxxxxxx reading from nvs");
      break;

    case ESP_ERR_NVS_NOT_FOUND:
      ESP_LOGI(TAG, "Primary key not found in nvs, writing default\n");
      memset(g_persistent.pmk, 0, 16);
      if (-1 != vscp_fwhlp_hex2bin(g_persistent.pmk, DEFAULT_KEY_LEN, VSCP_DEFAULT_KEY16)) {
        rv = nvs_set_blob(g_nvsHandle, "pmk", g_persistent.pmk, 16);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to write primary key to nvs!");
        }
        else {
          ESP_LOGI(TAG, "Primary key set to default value");
        }
      }
      break;

    default:
      ESP_LOGI(TAG, "Error (%s) reading Primary key from nvs!\n", esp_err_to_name(rv));
      break;
  }

  NVS_GET_OR_SET_DEFAULT(u8,
                         nvs_get_u8,
                         nvs_set_u8,
                         g_nvsHandle,
                         "encryptLvl",
                         g_persistent.encryptLvl,
                         DEFAULT_ENCRYPTION_LEVEL,
                         TAG,
                         "%d");

  NVS_GET_OR_SET_DEFAULT(u8,
                         nvs_get_u8,
                         nvs_set_u8,
                         g_nvsHandle,
                         "zone",
                         g_persistent.nodeZone,
                         DEFAULT_MODULE_ZONE,
                         TAG,
                         "%d");

  NVS_GET_OR_SET_DEFAULT(u8,
                         nvs_get_u8,
                         nvs_set_u8,
                         g_nvsHandle,
                         "subzone",
                         g_persistent.nodeSubzone,
                         DEFAULT_MODULE_SUBZONE,
                         TAG,
                         "%d");

  // * * * Log persistent configuration * * *

  NVS_GET_OR_SET_DEFAULT(u8,
                         nvs_get_u8,
                         nvs_set_u8,
                         g_nvsHandle,
                         "canSpeed",
                         g_persistent.canSpeed,
                         DEFAULT_CAN_SPEED,
                         TAG,
                         "%d");

  NVS_GET_OR_SET_DEFAULT(u8,
                         nvs_get_u8,
                         nvs_set_u8,
                         g_nvsHandle,
                         "canMode",
                         g_persistent.canMode,
                         DEFAULT_CAN_MODE,
                         TAG,
                         "%d");

  NVS_GET_OR_SET_DEFAULT(u32,
                         nvs_get_u32,
                         nvs_set_u32,
                         g_nvsHandle,
                         "canFilter",
                         g_persistent.canFilter,
                         DEFAULT_CAN_FILTER,
                         TAG,
                         "%d");

  NVS_GET_OR_SET_DEFAULT(u8,
                         nvs_get_u8,
                         nvs_set_u8,
                         g_nvsHandle,
                         "logType",
                         g_persistent.logType,
                         DEFAULT_LOG_TYPE,
                         TAG,
                         "%d");

  if (g_persistent.logType > LOG_TYPE_VSCP) {
    g_persistent.logType = DEFAULT_LOG_TYPE;
    nvs_set_u8(g_nvsHandle, "logType", g_persistent.logType);
  }

  NVS_GET_OR_SET_DEFAULT(u8,
                         nvs_get_u8,
                         nvs_set_u8,
                         g_nvsHandle,
                         "logRetries",
                         g_persistent.logRetries,
                         DEFAULT_LOG_RETRIES,
                         TAG,
                         "%d");

  NVS_GET_OR_SET_DEFAULT(u8,
                         nvs_get_u8,
                         nvs_set_u8,
                         g_nvsHandle,
                         "logLevel",
                         g_persistent.logLevel,
                         DEFAULT_LOG_LEVEL,
                         TAG,
                         "%d");

  NVS_GET_OR_SET_DEFAULT(u16,
                         nvs_get_u16,
                         nvs_set_u16,
                         g_nvsHandle,
                         "logPort",
                         g_persistent.logPort,
                         DEFAULT_LOG_PORT,
                         TAG,
                         "%d");

  NVS_GET_OR_SET_DEFAULT_STR(g_nvsHandle, "logUrl", g_persistent.logUrl, DEFAULT_LOG_URL, TAG);
  NVS_GET_OR_SET_DEFAULT_STR(g_nvsHandle,
                             "logMqttTopic",
                             g_persistent.logMqttTopic,
                             DEFAULT_MQTT_LOG_PUBLISH_TOPIC,
                             TAG);

  NVS_GET_OR_SET_DEFAULT(u8,
                         nvs_get_u8,
                         nvs_set_u8,
                         g_nvsHandle,
                         "enableMqttLog",
                         g_persistent.enableMqttLog,
                         DEFAULT_MQTT_LOGGING_ENABLE,
                         TAG,
                         "%d");

  // * * * Web server persistent configuration * * *

  NVS_GET_OR_SET_DEFAULT(u16,
                         nvs_get_u16,
                         nvs_set_u16,
                         g_nvsHandle,
                         "webPort",
                         g_persistent.webPort,
                         DEFAULT_WEBSERVER_PORT,
                         TAG,
                         "%d");

  NVS_GET_OR_SET_DEFAULT_STR(g_nvsHandle, "webUser", g_persistent.webUser, DEFAULT_WEBSERVER_USER, TAG);
  NVS_GET_OR_SET_DEFAULT_STR(g_nvsHandle, "webPassword", g_persistent.webPassword, DEFAULT_WEBSERVER_PASSWORD, TAG);

  // * * * WiFi configuration * * *
  NVS_GET_OR_SET_DEFAULT_STR(g_nvsHandle, "wifiPriSsid", g_persistent.wifiPrimarySsid, "", TAG);
  NVS_GET_OR_SET_DEFAULT_STR(g_nvsHandle, "wifiPriPass", g_persistent.wifiPrimaryPassword, "", TAG);
  NVS_GET_OR_SET_DEFAULT_STR(g_nvsHandle, "wifiSecSsid", g_persistent.wifiSecondarySsid, "", TAG);
  NVS_GET_OR_SET_DEFAULT_STR(g_nvsHandle, "wifiSecPass", g_persistent.wifiSecondaryPassword, "", TAG);
  NVS_GET_OR_SET_DEFAULT(u8,
                         nvs_get_u8,
                         nvs_set_u8,
                         g_nvsHandle,
                         "wifiStaIpEn",
                         g_persistent.wifiStaticEnable,
                         DEFAULT_WIFI_STATIC_ENABLE,
                         TAG,
                         "%d");
  NVS_GET_OR_SET_DEFAULT_STR(g_nvsHandle, "wifiStaIp", g_persistent.wifiStaticIp, DEFAULT_WIFI_STATIC_IP, TAG);
  NVS_GET_OR_SET_DEFAULT_STR(g_nvsHandle,
                             "wifiStaMask",
                             g_persistent.wifiStaticNetmask,
                             DEFAULT_WIFI_STATIC_NETMASK,
                             TAG);
  NVS_GET_OR_SET_DEFAULT_STR(g_nvsHandle,
                             "wifiStaGw",
                             g_persistent.wifiStaticGateway,
                             DEFAULT_WIFI_STATIC_GATEWAY,
                             TAG);
  NVS_GET_OR_SET_DEFAULT_STR(g_nvsHandle, "wifiStaDns", g_persistent.wifiStaticDns, DEFAULT_WIFI_STATIC_DNS, TAG);

  // * * * VSCP link persistent configuration * * *
  NVS_GET_OR_SET_DEFAULT(u8,
                         nvs_get_u8,
                         nvs_set_u8,
                         g_nvsHandle,
                         "enableVscpLink",
                         g_persistent.enableVscpLink,
                         DEFAULT_VSCP_LINK_ENABLE,
                         TAG,
                         "%d");

  NVS_GET_OR_SET_DEFAULT(u16,
                         nvs_get_u16,
                         nvs_set_u16,
                         g_nvsHandle,
                         "vscplinkPort",
                         g_persistent.vscplinkPort,
                         DEFAULT_VSCP_LINK_PORT,
                         TAG,
                         "%d");

  NVS_GET_OR_SET_DEFAULT_STR(g_nvsHandle, "vscplinkUser", g_persistent.vscplinkUser, DEFAULT_VSCP_LINK_USER, TAG);
  NVS_GET_OR_SET_DEFAULT_STR(g_nvsHandle, "vscplinkPw", g_persistent.vscplinkPw, DEFAULT_VSCP_LINK_PASSWORD, TAG);

  // * * * MQTT persistent configuration * * *

  NVS_GET_OR_SET_DEFAULT(u8,
                         nvs_get_u8,
                         nvs_set_u8,
                         g_nvsHandle,
                         "enableMqtt",
                         g_persistent.enableMqtt,
                         DEFAULT_MQTT_ENABLE,
                         TAG,
                         "%d");

  NVS_GET_OR_SET_DEFAULT(u8,
                         nvs_get_u8,
                         nvs_set_u8,
                         g_nvsHandle,
                         "enableMqttTls",
                         g_persistent.enableMqttTls,
                         DEFAULT_MQTT_TLS_ENABLE,
                         TAG,
                         "%d");

  NVS_GET_OR_SET_DEFAULT(u16,
                         nvs_get_u16,
                         nvs_set_u16,
                         g_nvsHandle,
                         "mqttPort",
                         g_persistent.mqttPort,
                         DEFAULT_MQTT_PORT,
                         TAG,
                         "%d");

  NVS_GET_OR_SET_DEFAULT_STR(g_nvsHandle, "mqttUrl", g_persistent.mqttUrl, DEFAULT_MQTT_URL, TAG);
  NVS_GET_OR_SET_DEFAULT_STR(g_nvsHandle, "mqttUser", g_persistent.mqttUser, DEFAULT_MQTT_USER, TAG);
  NVS_GET_OR_SET_DEFAULT_STR(g_nvsHandle, "mqttPw", g_persistent.mqttPw, DEFAULT_MQTT_PASSWORD, TAG);

  NVS_GET_OR_SET_DEFAULT_STR(g_nvsHandle, "mqttPubTopic", g_persistent.mqttPubTopic, DEFAULT_MQTT_PUBLISH, TAG);
  NVS_GET_OR_SET_DEFAULT_STR(g_nvsHandle, "mqttSubTopic", g_persistent.mqttSubTopic, DEFAULT_MQTT_SUBSCRIBE, TAG);
  NVS_GET_OR_SET_DEFAULT_STR(g_nvsHandle,
                             "mqttPubLogTopic",
                             g_persistent.mqttPubLogTopic,
                             DEFAULT_MQTT_LOG_PUBLISH_TOPIC,
                             TAG);
  NVS_GET_OR_SET_DEFAULT_STR(g_nvsHandle, "mqttClientId", g_persistent.mqttClientId, DEFAULT_MQTT_CLIENT_ID, TAG);
  NVS_GET_OR_SET_DEFAULT_STR(g_nvsHandle, "mqttCaCert", g_persistent.mqttCaCert, DEFAULT_MQTT_CA_CERT, TAG);
  NVS_GET_OR_SET_DEFAULT_STR(g_nvsHandle, "mqttClientCert", g_persistent.mqttClientCert, DEFAULT_MQTT_CLIENT_CERT, TAG);
  NVS_GET_OR_SET_DEFAULT_STR(g_nvsHandle, "mqttClientKey", g_persistent.mqttClientKey, DEFAULT_MQTT_CLIENT_KEY, TAG);
  NVS_GET_OR_SET_DEFAULT(u8,
                         nvs_get_u8,
                         nvs_set_u8,
                         g_nvsHandle,
                         "mqttQos",
                         g_persistent.mqttQos,
                         DEFAULT_MQTT_QOS,
                         TAG,
                         "%d");
  NVS_GET_OR_SET_DEFAULT(u8,
                         nvs_get_u8,
                         nvs_set_u8,
                         g_nvsHandle,
                         "mqttRetain",
                         g_persistent.mqttRetain,
                         DEFAULT_MQTT_RETAIN,
                         TAG,
                         "%d");
  NVS_GET_OR_SET_DEFAULT(u8,
                         nvs_get_u8,
                         nvs_set_u8,
                         g_nvsHandle,
                         "mqttFormat",
                         g_persistent.mqttFormat,
                         DEFAULT_MQTT_FORMAT,
                         TAG,
                         "%d");

  // * * * Multicast persistent configuration * * *
  NVS_GET_OR_SET_DEFAULT(u8,
                         nvs_get_u8,
                         nvs_set_u8,
                         g_nvsHandle,
                         "enableMulticast",
                         g_persistent.enableMulticast,
                         DEFAULT_MULTICAST_ENABLE,
                         TAG,
                         "%d");

  NVS_GET_OR_SET_DEFAULT(u16,
                         nvs_get_u16,
                         nvs_set_u16,
                         g_nvsHandle,
                         "multicastPort",
                         g_persistent.multicastPort,
                         DEFAULT_MULTICAST_PORT,
                         TAG,
                         "%d");

  NVS_GET_OR_SET_DEFAULT(u8,
                         nvs_get_u8,
                         nvs_set_u8,
                         g_nvsHandle,
                         "multicastTtl",
                         g_persistent.multicastTtl,
                         DEFAULT_MULTICAST_TTL,
                         TAG,
                         "%d");

  NVS_GET_OR_SET_DEFAULT_STR(g_nvsHandle, "multicastUrl", g_persistent.multicastUrl, DEFAULT_MULTICAST_URL, TAG);

  NVS_GET_OR_SET_DEFAULT(u8,
                         nvs_get_u8,
                         nvs_set_u8,
                         g_nvsHandle,
                         "bMcastEncrypt",
                         g_persistent.bMcastEncrypt,
                         DEFAULT_MULTICAST_ENCRYPTION,
                         TAG,
                         "%d");

  NVS_GET_OR_SET_DEFAULT(u8,
                         nvs_get_u8,
                         nvs_set_u8,
                         g_nvsHandle,
                         "bMcastEncrypt",
                         g_persistent.bMcastEncrypt,
                         DEFAULT_MULTICAST_ENCRYPTION,
                         TAG,
                         "%d");

  NVS_GET_OR_SET_DEFAULT(u8,
                         nvs_get_u8,
                         nvs_set_u8,
                         g_nvsHandle,
                         "bMcastHbeat",
                         g_persistent.bHeartbeat,
                         DEFAULT_MULTICAST_HEARTBEAT,
                         TAG,
                         "%d");

  // * * * UDP persistent configuration * * *

  NVS_GET_OR_SET_DEFAULT(u8,
                         nvs_get_u8,
                         nvs_set_u8,
                         g_nvsHandle,
                         "enableUdpRx",
                         g_persistent.enableUdpRx,
                         DEFAULT_UDP_RX_ENABLE,
                         TAG,
                         "%d");

  NVS_GET_OR_SET_DEFAULT(u8,
                         nvs_get_u8,
                         nvs_set_u8,
                         g_nvsHandle,
                         "enableUdpTx",
                         g_persistent.enableUdpTx,
                         DEFAULT_UDP_TX_ENABLE,
                         TAG,
                         "%d");

  NVS_GET_OR_SET_DEFAULT(u16,
                         nvs_get_u16,
                         nvs_set_u16,
                         g_nvsHandle,
                         "udpPort",
                         g_persistent.udpPort,
                         DEFAULT_UDP_PORT,
                         TAG,
                         "%d");

  NVS_GET_OR_SET_DEFAULT_STR(g_nvsHandle, "udpUrl", g_persistent.udpUrl, "255.255.255.255", TAG);

  NVS_GET_OR_SET_DEFAULT(u8,
                         nvs_get_u8,
                         nvs_set_u8,
                         g_nvsHandle,
                         "bUdpEncrypt",
                         g_persistent.bUdpEncrypt,
                         DEFAULT_UDP_ENCRYPTION,
                         TAG,
                         "%d");

  // * * * Websockets server configuration * * *

  NVS_GET_OR_SET_DEFAULT(u8,
                         nvs_get_u8,
                         nvs_set_u8,
                         g_nvsHandle,
                         "enableWebsock",
                         g_persistent.enableWebsock,
                         DEFAULT_WEBSOCKETS_ENABLE,
                         TAG,
                         "%d");

  NVS_GET_OR_SET_DEFAULT(u16,
                         nvs_get_u16,
                         nvs_set_u16,
                         g_nvsHandle,
                         "websockPort",
                         g_persistent.websockPort,
                         DEFAULT_WEBSOCKETS_PORT,
                         TAG,
                         "%d");

  NVS_GET_OR_SET_DEFAULT_STR(g_nvsHandle, "websockUser", g_persistent.websockUser, DEFAULT_WEBSOCKETS_USER, TAG);
  NVS_GET_OR_SET_DEFAULT_STR(g_nvsHandle, "websockPw", g_persistent.websockPw, DEFAULT_WEBSOCKETS_PASSWORD, TAG);

  //////////////////////////////////////////////////////////////////////////////////////
  // Commit written value.
  // After setting any values, nvs_commit() must be called to ensure changes are written
  // to flash storage. Implementations may write to storage at other times,
  // but this is not guaranteed.
  /////////////////////////////////////////////////////////////////////////////////////
  printf("Committing updates in NVS ... ");
  rv = nvs_commit(g_nvsHandle);
  if (rv != ESP_OK) {
    ESP_LOGE(TAG, "NVS commit faild at startup (%s)!", esp_err_to_name(rv));
  }
}

// ============================================================================
//                      WiFi Event Synchronization
// ============================================================================

/**
 * @brief Event group for WiFi connection status
 *
 * WIFI_CONNECTED_EVENT bit is set when device successfully connects to WiFi
 * and obtains an IP address. Used to synchronize network service initialization.
 */
const int WIFI_CONNECTED_EVENT = BIT0;
static EventGroupHandle_t wifi_event_group;

#define WIFI_PRIMARY_RETRY_LIMIT 5

static bool s_wifi_use_secondary    = false;
static uint8_t s_wifi_retry_counter = 0;

#define PROV_QR_VERSION       "v1"
#define PROV_TRANSPORT_SOFTAP "softap"
#define PROV_TRANSPORT_BLE    "ble"
#define QRCODE_BASE_URL       "https://espressif.github.io/esp-jumpstart/qrcode.html"

// ============================================================================
//                      Firmware Update Functions
// ============================================================================

/**
 * @brief Initiate OTA (Over-The-Air) firmware update
 *
 * Starts firmware update process using default firmware server URL.
 * Currently placeholder for future OTA implementation.
 */
void
startOTA(void)
{
  ESP_LOGI(TAG, "Starting OTA firmware update...");

  // vscp_fwhlp_initiate_ota_update("http://firmware.vscp.org/firmware/vscp-din-wireless-esp32-can-z102-latest.bin");
}

/**
 * @brief Initiate firmware update from specific URL
 *
 * Starts OTA firmware update using the provided URL. Currently placeholder
 * for future implementation.
 *
 * @param url  HTTP(S) URL pointing to firmware binary file
 *
 * @return VSCP_ERROR_SUCCESS on success, error code otherwise
 */
int
app_initiate_firmware_upload(const char *url)
{
  ESP_LOGI(TAG, "Starting firmware update from URL: %s", url);

  // vscp_fwhlp_initiate_ota_update(url);

  return VSCP_ERROR_SUCCESS;
}

// ============================================================================
//                        Utility Functions
// ============================================================================

/**
 * @brief Get current time in milliseconds
 *
 * Returns system uptime in milliseconds since boot using ESP timer.
 * Used for timestamping and timeout calculations.
 *
 * @return Milliseconds since system boot
 */
uint32_t
getMilliSeconds(void)
{
  return (esp_timer_get_time() / 1000);
};

/**
 * @brief Validate user credentials against stored values
 *
 * Checks provided username and password against credentials stored in NVS.
 * Used for VSCP link protocol authentication and web interface access.
 *
 * @param user  Username string to validate
 * @param pw    Password string to validate
 *
 * @return true if credentials match, false otherwise
 */
bool
validate_user(const char *user, const char *pw)
{
  esp_err_t rv;
  size_t length;
  char username[VSCP_LINK_MAX_USER_NAME_LENGTH];
  char password[VSCP_LINK_MAX_PASSWORD_LENGTH];

  length = sizeof(username);
  rv     = nvs_get_str(g_nvsHandle, "vscplinkUser", username, &length);
  switch (rv) {

    case ESP_OK:
      break;

    case ESP_ERR_NVS_NOT_FOUND:
      ESP_LOGI(TAG, "Username not found in nvs\n");
      return false;

    default:
      ESP_LOGI(TAG, "Error (%s) reading username from nvs!\n", esp_err_to_name(rv));
      return false;
  }

  length = sizeof(password);
  rv     = nvs_get_str(g_nvsHandle, "vscplinkPw", password, &length);
  switch (rv) {

    case ESP_OK:
      break;

    case ESP_ERR_NVS_NOT_FOUND:
      ESP_LOGI(TAG, "Password not found in nvs\n");
      return false;

    default:
      ESP_LOGI(TAG, "Error (%s) reading password from nvs!\n", esp_err_to_name(rv));
      return false;
  }

  ESP_LOGI(TAG, "Credentials: %s=%s - %s=%s", username, user, password, pw);
  if (0 == strcmp(username, user) && 0 == strcmp(password, pw)) {
    return true;
  }

  return false;
}

/**
 * @brief Retrieve device GUID from NVS
 *
 * Reads the 16-byte GUID from persistent storage. GUID is used as unique
 * identifier for VSCP node addressing.
 *
 * @param[out] pguid  Buffer to store 16-byte GUID (must be pre-allocated)
 *
 * @return true if GUID successfully retrieved, false on error
 */
bool
get_device_guid(uint8_t *pguid)
{
  esp_err_t rv;
  size_t length = 16;

  // Check pointer
  if (NULL == pguid) {
    return false;
  }

  rv = nvs_get_blob(g_nvsHandle, "guid", pguid, &length);
  switch (rv) {

    case ESP_OK:
      break;

    case ESP_ERR_NVS_NOT_FOUND:
      ESP_LOGW(TAG, "guid not found in nvs\n");
      return false;

    default:
      ESP_LOGI(TAG, "Error (%s) reading guid from nvs!\n", esp_err_to_name(rv));
      return false;
  }

  return true;
}

// ============================================================================
//                       System Event Handler
// ============================================================================

/**
 * @brief Main event handler for WiFi and provisioning events
 *
 * Handles various system events:
 * - WIFI_PROV_EVENT: Provisioning lifecycle (start, credentials, success, fail, end)
 * - WIFI_EVENT: WiFi state changes (STA start, connect, disconnect)
 * - IP_EVENT: IP address assignment (GOT_IP)
 *
 * When credentials are received via provisioning, they are automatically saved
 * as the primary WiFi configuration in NVS.
 *
 * @param arg        User data (unused)
 * @param event_base Event base identifier (WIFI_PROV_EVENT, WIFI_EVENT, IP_EVENT)
 * @param event_id   Specific event ID within the base
 * @param event_data Event-specific data structure
 */
static void
event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
#ifdef CONFIG_WCANG_RESET_PROV_MGR_ON_FAILURE
  static int retries;
#endif
  if (event_base == NETWORK_PROV_EVENT) {
    switch (event_id) {
      case NETWORK_PROV_START:
        ESP_LOGI(TAG, "Provisioning started");
        break;
      case NETWORK_PROV_WIFI_CRED_RECV: {
        wifi_sta_config_t *wifi_sta_cfg = (wifi_sta_config_t *) event_data;
        ESP_LOGI(TAG,
                 "Received Wi-Fi credentials"
                 "\n\tSSID     : %s\n\tPassword : %s",
                 (const char *) wifi_sta_cfg->ssid,
                 (const char *) wifi_sta_cfg->password);

        // Automatically save provisioned credentials as primary WiFi configuration
        strncpy(g_persistent.wifiPrimarySsid, (char *) wifi_sta_cfg->ssid, sizeof(g_persistent.wifiPrimarySsid) - 1);
        g_persistent.wifiPrimarySsid[sizeof(g_persistent.wifiPrimarySsid) - 1] = '\0';

        strncpy(g_persistent.wifiPrimaryPassword,
                (char *) wifi_sta_cfg->password,
                sizeof(g_persistent.wifiPrimaryPassword) - 1);
        g_persistent.wifiPrimaryPassword[sizeof(g_persistent.wifiPrimaryPassword) - 1] = '\0';

        // Save to NVS
        nvs_set_str(g_nvsHandle, "wifiPriSsid", g_persistent.wifiPrimarySsid);
        nvs_set_str(g_nvsHandle, "wifiPriPass", g_persistent.wifiPrimaryPassword);
        nvs_commit(g_nvsHandle);

        ESP_LOGI(TAG, "Saved provisioned credentials as primary WiFi");
        break;
      }
      case NETWORK_PROV_WIFI_CRED_FAIL: {
        network_prov_wifi_sta_fail_reason_t *reason = (network_prov_wifi_sta_fail_reason_t *) event_data;
        ESP_LOGE(TAG,
                 "Provisioning failed!\n\tReason : %s"
                 "\n\tPlease reset to factory and retry provisioning",
                 (*reason == NETWORK_PROV_WIFI_STA_AUTH_ERROR) ? "Wi-Fi station authentication failed"
                                                               : "Wi-Fi access-point not found");
#ifdef CONFIG_WCANG_RESET_PROV_MGR_ON_FAILURE
        retries++;
        if (retries >= CONFIG_WCANG_PROV_MGR_MAX_RETRY_CNT) {
          ESP_LOGI(TAG, "Failed to connect with provisioned AP, reseting provisioned credentials");
          network_prov_mgr_reset_wifi_sm_state_on_failure();
          retries = 0;
        }
#endif
        break;
      }
      case NETWORK_PROV_WIFI_CRED_SUCCESS:
        ESP_LOGI(TAG, "Provisioning successful");
#ifdef CONFIG_WCANG_RESET_PROV_MGR_ON_FAILURE
        retries = 0;
#endif
        break;
      case NETWORK_PROV_END:
        /* De-initialize manager once provisioning is finished */
        network_prov_mgr_deinit();
        break;
      default:
        break;
    }
  }
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    // WiFi station started, initiate connection
    esp_wifi_connect();
  }
  else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    ESP_LOGI(TAG, "Connected with IP Address:" IPSTR, IP2STR(&event->ip_info.ip));
    s_wifi_retry_counter = 0;
    // Signal main application that WiFi connection is established
    xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_EVENT);
  }
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    const wifi_event_sta_disconnected_t *disc = (const wifi_event_sta_disconnected_t *) event_data;
    s_wifi_retry_counter++;

    if ((NULL != disc) && (disc->reason != WIFI_REASON_NO_AP_FOUND)) {
      ESP_LOGW(TAG,
               "WiFi disconnected (reason=%d), retry=%u, profile=%s",
               disc->reason,
               s_wifi_retry_counter,
               s_wifi_use_secondary ? "secondary" : "primary");
    }

    if (!s_wifi_use_secondary && (s_wifi_retry_counter >= WIFI_PRIMARY_RETRY_LIMIT) &&
        ('\0' != g_persistent.wifiSecondarySsid[0])) {
      wifi_config_t wifi_cfg = { 0 };
      strncpy((char *) wifi_cfg.sta.ssid, g_persistent.wifiSecondarySsid, sizeof(wifi_cfg.sta.ssid) - 1);
      strncpy((char *) wifi_cfg.sta.password, g_persistent.wifiSecondaryPassword, sizeof(wifi_cfg.sta.password) - 1);

      if (ESP_OK == esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg)) {
        s_wifi_use_secondary = true;
        s_wifi_retry_counter = 0;
        ESP_LOGW(TAG,
                 "Primary WiFi failed after %d retries. Switching to secondary SSID: %s",
                 WIFI_PRIMARY_RETRY_LIMIT,
                 g_persistent.wifiSecondarySsid);
      }
      else {
        ESP_LOGE(TAG, "Failed to switch to secondary WiFi profile");
      }
    }

    ESP_LOGI(TAG, "Disconnected. Connecting to the AP again...");
    esp_wifi_connect();
  }
}

// ============================================================================
//                      WiFi Station Initialization
// ============================================================================

/**
 * @brief Initialize WiFi in station mode and start connection
 *
 * Configures ESP32 WiFi to operate in station (client) mode and initiates
 * connection to the configured access point. Used after successful provisioning
 * when credentials are already stored.
 */
static void
wifi_init_sta(void)
{
  wifi_config_t wifi_cfg = { 0 };

  s_wifi_use_secondary = false;
  s_wifi_retry_counter = 0;

  if ('\0' != g_persistent.wifiPrimarySsid[0]) {
    strncpy((char *) wifi_cfg.sta.ssid, g_persistent.wifiPrimarySsid, sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char *) wifi_cfg.sta.password, g_persistent.wifiPrimaryPassword, sizeof(wifi_cfg.sta.password) - 1);
    ESP_LOGI(TAG, "Using primary WiFi profile: %s", g_persistent.wifiPrimarySsid);
  }
  else if ('\0' != g_persistent.wifiSecondarySsid[0]) {
    strncpy((char *) wifi_cfg.sta.ssid, g_persistent.wifiSecondarySsid, sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char *) wifi_cfg.sta.password, g_persistent.wifiSecondaryPassword, sizeof(wifi_cfg.sta.password) - 1);
    s_wifi_use_secondary = true;
    ESP_LOGW(TAG, "Primary SSID not set. Starting with secondary WiFi profile: %s", g_persistent.wifiSecondarySsid);
  }

  /* Start Wi-Fi in station mode */
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
  ESP_ERROR_CHECK(esp_wifi_start());
}

static bool
parse_ipv4_addr(const char *str, esp_ip4_addr_t *addr)
{
  if ((NULL == str) || (NULL == addr) || ('\0' == *str)) {
    return false;
  }

  ip4_addr_t parsed = { 0 };
  if (!ip4addr_aton(str, &parsed)) {
    return false;
  }

  addr->addr = parsed.addr;
  return true;
}

static esp_err_t
apply_sta_ip_config(void)
{
  if (NULL == s_sta_netif) {
    ESP_LOGW(TAG, "STA netif is not available");
    return ESP_ERR_INVALID_STATE;
  }

  if (!g_persistent.wifiStaticEnable) {
    ESP_LOGI(TAG, "WiFi static IPv4 disabled, using DHCP");
    esp_err_t rv = esp_netif_dhcpc_start(s_sta_netif);
    if ((ESP_OK != rv) && (ESP_ERR_ESP_NETIF_DHCP_ALREADY_STARTED != rv)) {
      ESP_LOGW(TAG, "Failed to start DHCP client: %s", esp_err_to_name(rv));
      return rv;
    }
    return ESP_OK;
  }

  esp_netif_ip_info_t ip_info = { 0 };
  if (!parse_ipv4_addr(g_persistent.wifiStaticIp, &ip_info.ip) ||
      !parse_ipv4_addr(g_persistent.wifiStaticNetmask, &ip_info.netmask) ||
      !parse_ipv4_addr(g_persistent.wifiStaticGateway, &ip_info.gw)) {
    ESP_LOGW(TAG,
             "Invalid static IPv4 settings (ip=%s, mask=%s, gw=%s). Falling back to DHCP.",
             g_persistent.wifiStaticIp,
             g_persistent.wifiStaticNetmask,
             g_persistent.wifiStaticGateway);
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t rv = esp_netif_dhcpc_stop(s_sta_netif);
  if ((ESP_OK != rv) && (ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED != rv)) {
    ESP_LOGW(TAG, "Failed to stop DHCP client: %s", esp_err_to_name(rv));
    return rv;
  }

  ESP_ERROR_CHECK(esp_netif_set_ip_info(s_sta_netif, &ip_info));

  if ('\0' != g_persistent.wifiStaticDns[0]) {
    esp_netif_dns_info_t dns_info = { 0 };
    if (parse_ipv4_addr(g_persistent.wifiStaticDns, &dns_info.ip.u_addr.ip4)) {
      dns_info.ip.type = ESP_IPADDR_TYPE_V4;
      ESP_ERROR_CHECK(esp_netif_set_dns_info(s_sta_netif, ESP_NETIF_DNS_MAIN, &dns_info));
    }
    else {
      ESP_LOGW(TAG, "Invalid static DNS address: %s", g_persistent.wifiStaticDns);
    }
  }

  ESP_LOGI(TAG,
           "Applied static IPv4 settings: ip=" IPSTR " mask=" IPSTR " gw=" IPSTR,
           IP2STR(&ip_info.ip),
           IP2STR(&ip_info.netmask),
           IP2STR(&ip_info.gw));

  return ESP_OK;
}

esp_err_t
wcang_reconfigure_wifi_sta(void)
{
  if (NULL == s_sta_netif) {
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t rv = apply_sta_ip_config();
  if (ESP_OK != rv) {
    return rv;
  }

  rv = esp_wifi_disconnect();
  if ((ESP_OK != rv) && (ESP_ERR_WIFI_NOT_CONNECT != rv) && (ESP_ERR_WIFI_NOT_STARTED != rv)) {
    ESP_LOGW(TAG, "esp_wifi_disconnect failed: %s", esp_err_to_name(rv));
  }

  rv = esp_wifi_connect();
  if (ESP_OK != rv) {
    ESP_LOGW(TAG, "esp_wifi_connect failed: %s", esp_err_to_name(rv));
    return rv;
  }

  ESP_LOGI(TAG, "WiFi station reconfiguration requested");
  return ESP_OK;
}

/**
 * @brief Generate unique device service name for provisioning
 *
 * Creates a unique service name using the device's MAC address. This name is
 * used as:
 * - WiFi SSID when using SoftAP provisioning
 * - BLE device name when using BLE provisioning
 *
 * Format: "PROV_XXYYZZ" where XX, YY, ZZ are last 3 bytes of MAC address
 *
 * @param[out] service_name  Buffer to store generated service name
 * @param max                Maximum buffer size
 */
static void
get_device_service_name(char *service_name, size_t max)
{
  uint8_t eth_mac[6];
  const char *ssid_prefix = "PROV_";
  esp_wifi_get_mac(WIFI_IF_STA, eth_mac);
  snprintf(service_name, max, "%s%02X%02X%02X", ssid_prefix, eth_mac[3], eth_mac[4], eth_mac[5]);
}

// ============================================================================
//                   Custom Provisioning Data Handler
// ============================================================================

/**
 * @brief Handle custom data during WiFi provisioning
 *
 * This handler is called when custom data is sent to the "VSCP-WCANG" provisioning
 * endpoint. It allows configuring secondary WiFi credentials during the initial
 * provisioning process.
 *
 * Expected data format: "SSID:PASSWORD"
 * Example: "MyBackupWiFi:SecretPass123"
 *
 * The secondary WiFi credentials are saved to NVS and can be used as a fallback
 * if the primary WiFi becomes unavailable.
 *
 * @param session_id  Provisioning session identifier
 * @param inbuf       Input data buffer containing custom configuration
 * @param inlen       Length of input data
 * @param outbuf      Output buffer for response (unused, set to NULL)
 * @param outlen      Output length (unused, set to 0)
 * @param priv_data   Private user data (unused)
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t
custom_prov_data_handler(uint32_t session_id,
                         const uint8_t *inbuf,
                         ssize_t inlen,
                         uint8_t **outbuf,
                         ssize_t *outlen,
                         void *priv_data)
{
  if (inbuf && inlen > 0) {
    ESP_LOGI(TAG, "Received custom data: %.*s", inlen, (char *) inbuf);

    // Parse secondary WiFi credentials from format "SSID:PASSWORD"
    char *data_copy = strndup((char *) inbuf, inlen);
    if (data_copy) {
      char *colon = strchr(data_copy, ':');
      if (colon) {
        // Split string at colon to separate SSID and password
        *colon         = '\0';
        char *ssid     = data_copy;
        char *password = colon + 1;

        // Save as secondary WiFi credentials
        strncpy(g_persistent.wifiSecondarySsid, ssid, sizeof(g_persistent.wifiSecondarySsid) - 1);
        g_persistent.wifiSecondarySsid[sizeof(g_persistent.wifiSecondarySsid) - 1] = '\0';

        strncpy(g_persistent.wifiSecondaryPassword, password, sizeof(g_persistent.wifiSecondaryPassword) - 1);
        g_persistent.wifiSecondaryPassword[sizeof(g_persistent.wifiSecondaryPassword) - 1] = '\0';

        // Save to NVS
        nvs_set_str(g_nvsHandle, "wifiSecSsid", g_persistent.wifiSecondarySsid);
        nvs_set_str(g_nvsHandle, "wifiSecPass", g_persistent.wifiSecondaryPassword);
        nvs_commit(g_nvsHandle);

        ESP_LOGI(TAG, "Saved secondary WiFi: SSID=%s", g_persistent.wifiSecondarySsid);
      }
      free(data_copy);
    }
  }

  char response[] = "SUCCESS";
  *outbuf         = (uint8_t *) strdup(response);

  if (*outbuf == NULL) {
    ESP_LOGE(TAG, "System out of memory");
    return ESP_ERR_NO_MEM;
  }

  *outlen = strlen(response) + 1; // Include NULL terminator

  return ESP_OK;
}

/**
 * @brief Generate and display QR code for WiFi provisioning
 *
 * Creates a QR code containing provisioning information that can be scanned
 * by the ESP provisioning mobile app. The QR code includes:
 * - Service name (device identifier)
 * - Security credentials (username/password or POP)
 * - Transport method (BLE or SoftAP)
 *
 * The QR code is displayed on the terminal (if configured) and a URL is
 * provided for browser-based provisioning.
 *
 * @param name       Service/device name
 * @param username   Username for SEC2 authentication (SEC2 only)
 * @param pop        Proof of possession string
 * @param transport  Transport method ("ble" or "softap")
 */
static void
wifi_prov_print_qr(const char *name, const char *username, const char *pop, const char *transport)
{
  if (!name || !transport) {
    ESP_LOGW(TAG, "Cannot generate QR code payload. Data missing.");
    return;
  }

  // Build JSON payload with provisioning information
  char payload[150] = { 0 };
  if (pop) {
#if CONFIG_WCANG_PROV_SECURITY_VERSION_1
    snprintf(payload,
             sizeof(payload),
             "{\"ver\":\"%s\",\"name\":\"%s\""
             ",\"pop\":\"%s\",\"transport\":\"%s\"}",
             PROV_QR_VERSION,
             name,
             pop,
             transport);
#elif CONFIG_WCANG_PROV_SECURITY_VERSION_2
    snprintf(payload,
             sizeof(payload),
             "{\"ver\":\"%s\",\"name\":\"%s\""
             ",\"username\":\"%s\",\"pop\":\"%s\",\"transport\":\"%s\"}",
             PROV_QR_VERSION,
             name,
             username,
             pop,
             transport);
#endif
  }
  else {
    snprintf(payload,
             sizeof(payload),
             "{\"ver\":\"%s\",\"name\":\"%s\""
             ",\"transport\":\"%s\"}",
             PROV_QR_VERSION,
             name,
             transport);
  }
#ifdef CONFIG_WCANG_PROV_SHOW_QR
  ESP_LOGI(TAG, "Scan this QR code from the provisioning application for Provisioning.");
  esp_qrcode_config_t cfg = ESP_QRCODE_CONFIG_DEFAULT();
  esp_qrcode_generate(&cfg, payload);
#endif
  // Provide URL for browser-based provisioning as alternative to QR code
  ESP_LOGI(TAG,
           "If QR code is not visible, copy paste the below URL in a browser.\n%s?data=%s",
           QRCODE_BASE_URL,
           payload);
}

/**
 * @brief Init indicator LED device on specified GPIO
 *
 * @param LED indicator handle to initialize
 * @param gpio_num GPIO number for the LED
 */
void
led_indicator_init(led_indicator_handle_t *handle, int gpio_num)
{
  led_indicator_gpio_config_t led_indicator_gpio_config = {
    .is_active_level_high = 1,
    .gpio_num             = gpio_num, /**< num of GPIO */
  };

  led_indicator_config_t config = {
    .blink_lists    = (void *) NULL,
    .blink_list_num = 0,
  };

  esp_err_t ret = led_indicator_new_gpio_device(&config, &led_indicator_gpio_config, handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize LED indicator: %s", esp_err_to_name(ret));
  }
}

// ============================================================================
//                      Application Entry Point
// ============================================================================

/**
 * @brief Main application entry point
 *
 * Initializes all system components and enters the main event loop:
 *
 * 1. NVS Flash: Initialize non-volatile storage for configuration
 * 2. Networking: Initialize TCP/IP stack and event loop
 * 3. GPIO: Configure status LEDs (connected, active)
 * 4. Message Queues: Create FreeRTOS queues for CAN and network transports
 * 5. WiFi: Register event handlers and initialize WiFi subsystem
 * 6. Provisioning: Handle initial WiFi setup (BLE or SoftAP based)
 * 7. Configuration: Load persistent settings from NVS
 * 8. Web Server: Start HTTP configuration interface
 * 9. CAN/TWAI: Initialize CAN bus driver with configured parameters
 * 10. Network Services: Start TCP server for VSCP link protocol
 * 11. Main Loop: Distribute CAN messages to active network transports
 *
 * The main loop waits for CAN messages on tr_twai_rx queue and distributes
 * them to all enabled transport mechanisms (TCP, MQTT, UDP, etc.).
 */
void
app_main(void)
{
  esp_err_t ret;

  // Initialize the Task Watchdog Timer
  init_watchdog_timer();

  crcInit(); // For calculating CRC of VSCP events

  // ============================================================================
  //                GPIO Configuration (Status LEDs) ande buttons
  // ============================================================================

  gpio_config_t io_conf = {};

  // Configure LED GPIO pins as outputs without interrupts
  io_conf.intr_type    = GPIO_INTR_DISABLE;   // No interrupts
  io_conf.mode         = GPIO_MODE_OUTPUT;    // Output mode
  io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL; // LED pin mask
  io_conf.pull_down_en = 0;                   // No pull-down
  io_conf.pull_up_en   = 0;                   // No pull-up

  // Apply GPIO configuration
  gpio_config(&io_conf);

  // Initialize status LEDs (active high)
  gpio_set_level(LED_WIFI_GPIO, 1);   // Connection status LED
  gpio_set_level(LED_STATUS_GPIO, 1); // Activity indicator LED

  led_indicator_init(&led_handle_wifi, LED_WIFI_GPIO);
  ret = led_indicator_start(led_handle_wifi, BLINK_CONNECTING);

  led_indicator_init(&led_handle_status, LED_STATUS_GPIO);
  ret = led_indicator_start(led_handle_status, BLINK_FACTORY_RESET);

  // --------------------------------------------------------------------------

  // create gpio button
  const button_config_t btn_cfg           = { 0 };
  const button_gpio_config_t btn_gpio_cfg = {
    .gpio_num     = 0,
    .active_level = 0,
  };
  button_handle_t gpio_btn = NULL;
  ret                      = iot_button_new_gpio_device(&btn_cfg, &btn_gpio_cfg, &gpio_btn);
  if (NULL == gpio_btn) {
    ESP_LOGE(TAG, "Button create failed");
  }

  // vscp_event_t ev = { 0 };
  // ;
  // char *jsonobj = "{\"vscpHead\":321,\"vscpObid\":12345,\"vscpTimeStamp\":67890,\"vscpClass\":10,\"vscpType\":6,"
  //                 "\"vscpGuid\":\"FF:FF:FF:FF:FF:FF:FF:FF:FF:FF:FF:FF:FF:FF:FF\",\"vscpData\":[1,2,3,4,5,6,7,8]}";

  // int result = vscp_fwhlp_parse_json(&ev, jsonobj);
  // ESP_LOGI(TAG,
  //          "Parsed event: status=%d class=%d, type=%d, data_len=%d",
  //          result,
  //          ev.vscp_class,
  //          ev.vscp_type,
  //          ev.sizeData);

  // vscp_event_ex_t pex = { 0 };
  // vscp_fwhlp_parse_json_ex(&pex, jsonobj);
  // ESP_LOGE(TAG,
  //          "Parsed event: status=%d class=%d, type=%d, data_len=%d",
  //          result,
  //          pex.vscp_class,
  //          pex.vscp_type,
  //          pex.sizeData);

  // ============================================================================
  //                      NVS (Non-Volatile Storage) Init
  // ============================================================================

  // Initialize NVS partition for configuration storage
  esp_err_t rv = nvs_flash_init();
  if (rv == ESP_ERR_NVS_NO_FREE_PAGES || rv == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    // NVS partition was truncated or version updated - erase and retry
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());
  }

  // Load all persistent configuration from NVS and increment boot counter
  initPersistentStorage();

  // ============================================================================
  //                      Network Stack Initialization
  // ============================================================================

  // Initialize TCP/IP stack (LwIP)
  ESP_ERROR_CHECK(esp_netif_init());

  // Create default event loop for system events
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  // Create event group for WiFi connection synchronization
  wifi_event_group = xEventGroupCreate();

  // ============================================================================
  //                      FreeRTOS Message Queue Creation
  // ============================================================================

  /**
   * Message queue architecture:
   * - tr_twai_rx: Receives CAN messages from bus (distributed to all transports)
   * - tr_twai_tx: Transmits CAN messages to bus (from all transports)
   * - tr_tcpsrv[]: One queue per TCP connection for VSCP link protocol
   * - tr_mqtt: MQTT transport message queue
   * - tr_multicast: UDP multicast transport queue
   * - tr_udp: UDP unicast/broadcast transport queue
   * - tr_websockets: WebSocket transport queue
   */
  tr_twai_rx = xQueueCreate(10, sizeof(can4vscp_frame_t)); // Incoming CAN messages  <-- from bus
  tr_twai_tx = xQueueCreate(40, sizeof(can4vscp_frame_t)); // Outgoing CAN messages  --> to bus

  // Create message queues for each TCP connection
  for (int i = 0; i < MAX_TCP_CONNECTIONS; i++) {
    tr_tcpsrv[i].tocan_queue   = xQueueCreate(10, sizeof(can4vscp_frame_t));
    tr_tcpsrv[i].fromcan_queue = xQueueCreate(10, sizeof(can4vscp_frame_t));
  }

  // Create message queues for network transports
  tr_mqtt.tocan_queue         = xQueueCreate(10, sizeof(can4vscp_frame_t));
  tr_mqtt.fromcan_queue       = xQueueCreate(10, sizeof(can4vscp_frame_t));
  tr_multicast.tocan_queue    = xQueueCreate(10, sizeof(can4vscp_frame_t));
  tr_multicast.fromcan_queue  = xQueueCreate(10, sizeof(can4vscp_frame_t));
  tr_udp.tocan_queue          = xQueueCreate(10, sizeof(can4vscp_frame_t));
  tr_udp.fromcan_queue        = xQueueCreate(10, sizeof(can4vscp_frame_t));
  tr_websockets.tocan_queue   = xQueueCreate(10, sizeof(can4vscp_frame_t));
  tr_websockets.fromcan_queue = xQueueCreate(10, sizeof(can4vscp_frame_t));

  // HTTP CAN API command/reply queues
  tr_canapi.tocan_queue   = xQueueCreate(10, sizeof(can4vscp_frame_t));
  tr_canapi.fromcan_queue = xQueueCreate(50, sizeof(can4vscp_frame_t));

  // Monitor transport queues (depth 50 buffers bursts between poll intervals)
  tr_monitor.tocan_queue   = xQueueCreate(10, sizeof(can4vscp_frame_t));
  tr_monitor.fromcan_queue = xQueueCreate(50, sizeof(can4vscp_frame_t));

  // Create control semaphore for main task synchronization
  ctrl_task_sem = xSemaphoreCreateBinary();

  // ============================================================================
  //                      WiFi Event Handler Registration
  // ============================================================================

  // Register event handlers for WiFi provisioning, WiFi state, and IP events
  ESP_ERROR_CHECK(esp_event_handler_register(NETWORK_PROV_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

  // ============================================================================
  //                      WiFi Stack Initialization
  // ============================================================================

  // Create WiFi station network interface
  s_sta_netif = esp_netif_create_default_wifi_sta();

#ifdef CONFIG_WCANG_PROV_TRANSPORT_SOFTAP
  // Create WiFi AP interface for SoftAP provisioning
  esp_netif_create_default_wifi_ap();
#endif

  // Initialize WiFi driver with default configuration
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  // Apply static IP/DNS configuration before WiFi STA starts
  apply_sta_ip_config();

  // ============================================================================
  //                      WiFi Provisioning Setup
  // ============================================================================

  /**
   * WiFi provisioning allows initial configuration via:
   * - BLE (CONFIG_WCANG_PROV_TRANSPORT_BLE): Bluetooth Low Energy
   * - SoftAP (CONFIG_WCANG_PROV_TRANSPORT_SOFTAP): Temporary WiFi AP
   *
   * Security options:
   * - SEC1: Proof of Possession (POP) based authentication
   * - SEC2: Username/Password based authentication (SRP6a)
   */

  // Configure provisioning manager
  network_prov_mgr_config_t config = {
  // Select provisioning transport scheme (BLE or SoftAP)
#ifdef CONFIG_WCANG_PROV_TRANSPORT_BLE
    .scheme = network_prov_scheme_ble,
#endif
#ifdef CONFIG_WCANG_PROV_TRANSPORT_SOFTAP
    .scheme = network_prov_scheme_softap,
#endif

  /**
   * Event handler configuration:
   * - BLE: NETWORK_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM
   *   Automatically releases BT/BLE memory after provisioning
   * - SoftAP: NETWORK_PROV_EVENT_HANDLER_NONE
   *   No special memory management needed
   */
#ifdef CONFIG_WCANG_PROV_TRANSPORT_BLE
    .scheme_event_handler = NETWORK_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM
#endif
#ifdef CONFIG_WCANG_PROV_TRANSPORT_SOFTAP
                              .scheme_event_handler = NETWORK_PROV_EVENT_HANDLER_NONE
#endif
  };

  /*
   * Initialize provisioning manager with the
   * configuration parameters set above
   */
  ESP_ERROR_CHECK(network_prov_mgr_init(config));

  bool provisioned = false;
#ifdef CONFIG_WCANG_RESET_PROVISIONED
  network_prov_mgr_reset_wifi_provisioning();
#else
  /* Let's find out if the device is provisioned */
  ESP_ERROR_CHECK(network_prov_mgr_is_wifi_provisioned(&provisioned));

#endif

  /* If device is not yet provisioned start provisioning service */
  if (!provisioned) {

    ESP_LOGI(TAG, "Starting provisioning");

    /*
     * What is the Device Service Name that we want
     *
     * This translates to :
     *     - Wi-Fi SSID when scheme is network_prov_scheme_softap
     *     - device name when scheme is network_prov_scheme_ble
     */
    char service_name[12];
    get_device_service_name(service_name, sizeof(service_name));

#ifdef CONFIG_WCANG_PROV_SECURITY_VERSION_1
    /*
     * What is the security level that we want (0, 1, 2):
     *
     *   - NETWORK_PROV_SECURITY_0 is simply plain text communication.
     *   - NETWORK_PROV_SECURITY_1 is secure communication which consists of secure handshake
     *      using X25519 key exchange and proof of possession (pop) and AES-CTR
     *      for encryption/decryption of messages.
     *   - NETWORK_PROV_SECURITY_2 SRP6a based authentication and key exchange
     *      + AES-GCM encryption/decryption of messages
     */
    network_prov_security_t security = NETWORK_PROV_SECURITY_1;

    /*
     * Do we want a proof-of-possession (ignored if Security 0 is selected):
     *   - this should be a string with length > 0
     *   - NULL if not used
     */
    const char *pop = "VSCP-CAN4VSCP-GATEWAY-WiFi";
    /*
     * If the pop is allocated dynamically, then it should be valid till
     * the provisioning process is running.
     * it can be only freed when the NETWORK_PROV_END event is triggered
     */

    /*
     * This is the structure for passing security parameters
     * for the protocomm security 1.
     * This does not need not be static i.e. could be dynamically allocated
     */
    network_prov_security1_params_t *sec_params = pop;

    const char *username = NULL;

#elif CONFIG_WCANG_PROV_SECURITY_VERSION_2
    network_prov_security_t security = NETWORK_PROV_SECURITY_2;
    // The username must be the same one, which has been used in the generation of salt and verifier

#if CONFIG_WCANG_PROV_SEC2_DEV_MODE
    /*
     * This pop field represents the password that will be used to generate salt and verifier.
     * The field is present here in order to generate the QR code containing password.
     * In production this password field shall not be stored on the device
     */
    const char *username = WCANG_PROV_SEC2_USERNAME;
    const char *pop      = WCANG_PROV_SEC2_PWD;
#elif CONFIG_WCANG_PROV_SEC2_PROD_MODE
    /*
     * The username and password shall not be embedded in the firmware,
     * they should be provided to the user by other means.
     * e.g. QR code sticker
     */
    const char *username = NULL;
    const char *pop      = NULL;
#endif
    /*
     * This is the structure for passing security parameters
     * for the protocomm security 2.
     * This does not need not be static i.e. could be dynamically allocated
     */
    network_prov_security2_params_t sec2_params = {};

    ESP_ERROR_CHECK(wcang_get_sec2_salt(&sec2_params.salt, &sec2_params.salt_len));
    ESP_ERROR_CHECK(wcang_get_sec2_verifier(&sec2_params.verifier, &sec2_params.verifier_len));

    network_prov_security2_params_t *sec_params = &sec2_params;
#endif

    // Service key for provisioning (password for SoftAP, ignored for BLE)
    const char *service_key = NULL;

#ifdef CONFIG_WCANG_PROV_TRANSPORT_BLE
    /**
     * Set custom 128-bit UUID for BLE provisioning service
     *
     * This UUID is advertised in BLE advertisements and identifies the primary
     * GATT service for provisioning. Each characteristic is formed using this
     * UUID as a base with auto-assigned 12th and 13th bytes.
     *
     * Clients identify endpoints by reading the User Characteristic Description
     * descriptor (0x2901) for each characteristic.
     */
    uint8_t custom_service_uuid[] = {
      // LSB <-----------------------------------------------------------------> MSB
      0xb4, 0xdf, 0x5a, 0x1c, 0x3f, 0x6b, 0xf4, 0xbf, 0xea, 0x4a, 0x82, 0x03, 0x04, 0x90, 0x1a, 0x02,
    };

    network_prov_scheme_ble_set_service_uuid(custom_service_uuid);
#endif

    /**
     * Create custom provisioning endpoint for secondary WiFi configuration
     *
     * The "VSCP-WCANG" endpoint allows sending additional configuration data
     * during provisioning (e.g., secondary WiFi credentials). Must be created
     * before starting provisioning.
     */
    network_prov_mgr_endpoint_create("VSCP-WCANG");

    // Start provisioning service with configured security and service name
    ESP_ERROR_CHECK(
      network_prov_mgr_start_provisioning(security, (const void *) sec_params, service_name, service_key));

    /**
     * Register handler for custom endpoint
     *
     * The custom_prov_data_handler receives data sent to the "VSCP-WCANG" endpoint,
     * allowing configuration of secondary WiFi and other custom parameters.
     * Must be registered after starting provisioning.
     */
    network_prov_mgr_endpoint_register("VSCP-WCANG", custom_prov_data_handler, NULL);

    // Display QR code for easy provisioning via mobile app
#ifdef CONFIG_WCANG_PROV_TRANSPORT_BLE
    wifi_prov_print_qr(service_name, username, pop, PROV_TRANSPORT_BLE);
#else  /* CONFIG_WCANG_PROV_TRANSPORT_SOFTAP */
    wifi_prov_print_qr(service_name, username, pop, PROV_TRANSPORT_SOFTAP);
#endif /* CONFIG_WCANG_PROV_TRANSPORT_BLE */
  }
  else {
    // Device already provisioned - skip provisioning and connect directly
    ESP_LOGI(TAG, "Already provisioned, starting Wi-Fi STA");

    // Release provisioning manager resources
    network_prov_mgr_deinit();

    // Start WiFi in station mode with saved credentials
    wifi_init_sta();
  }

  // ============================================================================
  //                    Wait for WiFi Connection
  // ============================================================================

  // Block until WiFi connection established and IP address obtained
  xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_EVENT, false, true, portMAX_DELAY);

  // ============================================================================
  //                    Load Configuration from NVS
  // ============================================================================

  // ============================================================================
  //                    Start Web Configuration Server
  // ============================================================================

  // Start HTTP server for web-based configuration interface
  server_httpd = start_webserver();

  // Start websocket instance for real-time event monitoring and control (if enabled)
  // if (g_persistent.enableWebsock) {
  //   server_websock = setup_websocket_server();
  // }

  // ============================================================================
  //                    CAN/TWAI Initialization
  // ============================================================================

  /**
   * Initialize CAN (TWAI) bus driver:
   * - Configure timing based on bitrate (125K default)
   * - Set up message filtering
   * - Enable/disable listen-only mode based on configuration
   * - Create receive task for incoming CAN messages
   */

  // Initialize CAN interface with 125 kbps bitrate
  can4vscp_init(CAN4VSCP_125K);

  // Apply configured bitrate (loaded from NVS)
  can4vscp_setBitrate(CAN4VSCP_125K);

  // Enable CAN controller
  can4vscp_enable();

  // ============================================================================
  //                    Task Creation
  // ============================================================================

  // Create CAN receive task that fills tr_twai_rx queue
  // Create CAN receive and transmit tasks
  xTaskCreate(twai_receive_task, "can4vscp_rx", 4096, &tr_twai_rx, 10, NULL);
  xTaskCreate(twai_transmit_task, "can4vscp_tx", 2048, NULL, 10, NULL);

  // Release control semaphore to allow task execution
  xSemaphoreGive(ctrl_task_sem);

  // Create TCP server task for VSCP link protocol (IPv4)
  xTaskCreate(tcpsrv_task, "tcpsrv", 4096, (void *) AF_INET, 5, NULL);

#ifdef CONFIG_EXAMPLE_IPV6
  // Create TCP server task for IPv6 if enabled
  xTaskCreate(tcpsrv_task, "tcpsrv", 4096, (void *) AF_INET6, 5, NULL);
#endif

  // Start MQTT client task if enabled
  if (g_persistent.enableMqtt) {
    mqtt_start();
  }

  if (g_persistent.enableUdpRx || g_persistent.enableUdpTx) {
    udp_start(); // Start UDP unicast/broadcast task if enabled
  }

  if (g_persistent.enableMulticast) {
    multicast_start(); // Start UDP multicast task if enabled
  }

  // ws_start();          // Start WebSocket server task if enabled

  // ============================================================================
  //                    Main Event Distribution Loop
  // ============================================================================

  /**
   * Main application loop:
   *
   * 1. Wait for CAN messages on tr_twai_rx queue (from CAN receive task)
   * 2. Distribute received messages to all active transport queues:
   *    - TCP connections (VSCP link protocol clients)
   *    - MQTT broker (if enabled)
   *    - UDP multicast/unicast (if enabled)
   *    - WebSocket clients (if enabled)
   * 3. Transport-specific tasks handle queue-to-network transmission
   *
   * This implements the gateway's core functionality: bridging CAN bus
   * to various network protocols for remote VSCP event access.
   */

  while (1) {

    // esp_task_wdt_reset(); // Reset the watchdog timer

    can4vscp_frame_t msg = {};

    // Block waiting for CAN message from receive task
    if (xQueueReceive(tr_twai_rx, &msg, portMAX_DELAY) == pdPASS) {

      ESP_LOGI(TAG, "--> CAN event received, ID: 0x%X", (unsigned int) msg.identifier);

      // Log queue depth for monitoring
      UBaseType_t cnt = uxQueueMessagesWaiting(tr_twai_rx);
      ESP_LOGD(TAG, "Queue depth: %u", cnt);

      /**
       * TODO: Distribute message to all active transport queues
       *
       * For each enabled transport:
       * - Check if transport is enabled (from g_persistent config)
       * - Check if queue has space (xQueueSendToBack with timeout)
       * - Send message to queue for transmission
       *
       * Example:
       * if (g_persistent.enableMqtt) {
       *   xQueueSendToBack(tr_mqtt.msg_queue, &msg, 0);
       * }
       */
    }

    // Synchronize with control task
    xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
    xSemaphoreGive(ctrl_task_sem);

    // Yield to other tasks
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  // ============================================================================
  //                           Cleanup (unreachable)
  // ============================================================================

  // Close NVS handle (never reached due to infinite loop)
  nvs_close(g_nvsHandle);
}
