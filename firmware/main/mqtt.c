/*
  VSCP Wireless CAN4VSCP Gateway (VSCP-WCANG)

  MQTT SSL/TLS Client Module

  This module implements MQTT client functionality for the VSCP gateway,
  supporting both standard MQTT and secure MQTT over TLS/SSL. It handles:
  - MQTT broker connection with configurable credentials
  - Publishing VSCP events as JSON payloads
  - Topic name template substitution (mustache-style tags)
  - TLS/SSL encryption with CA and client certificate authentication
  - Connection statistics and error tracking
  - Event logging via MQTT

  The MIT License (MIT)
  Copyright (C) 2022-2026 Ake Hedman, the VSCP project <info@vscp.org>

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

#include "vscp-compiler.h"
#include "vscp-projdefs.h"

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include <esp_system.h>
#include <esp_partition.h>
#include <spi_flash_mmap.h>
#include <nvs_flash.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <esp_log.h>
#include <mqtt_client.h>
#include <esp_task_wdt.h>
#include <esp_tls.h>
#include <esp_ota_ops.h>
#include <esp_mac.h> // esp_base_mac_addr_get
#include <sys/param.h>

#ifndef VSCP_FWHLP_BINARY_FRAME_SUPPORT
#define VSCP_FWHLP_BINARY_FRAME_SUPPORT
#endif

#include <vscp.h>
#include <vscp-firmware-helper.h>

#include "can4vscp.h"
#include <main.h>
#include "mqtt.h"

// ============================================================================
//                              Global Variables
// ============================================================================

// Persistent configuration from main module
extern node_persistent_config_t g_persistent;

extern transport_t tr_mqtt;

// Logging tag for ESP-IDF logger
static const char *TAG = "MQTT";

// MQTT client handle for ESP-IDF MQTT library
esp_mqtt_client_handle_t g_mqtt_client;

// Connection state flag - true when connected to MQTT broker
static bool s_mqtt_connected = false;

// MQTT statistics: tracks publish counts, failures, and connection events
static mqtt_stats_t s_mqtt_statistics = { 0 };

// MQTT callback handoff queue (keep callback work minimal)
#define MQTT_RX_QUEUE_LEN       10
#define MQTT_RX_TOPIC_MAX_LEN   128
#define MQTT_RX_PAYLOAD_MAX_LEN 2048

typedef struct {
  size_t topic_len;
  size_t payload_len;
  char topic[MQTT_RX_TOPIC_MAX_LEN];
  char payload[MQTT_RX_PAYLOAD_MAX_LEN];
} mqtt_rx_msg_t;

static QueueHandle_t s_mqtt_rx_queue = NULL;

// ============================================================================
//                          Certificate Data (Legacy)
// ============================================================================

// Legacy: Embedded Eclipse IoT certificate (not used when TLS is configured via web UI)
// These are kept for backward compatibility but dynamic certificates from
// g_persistent.mqttCaCert/mqttClientCert/mqttClientKey are preferred
extern const uint8_t mqtt_eclipse_io_pem_start[] asm("_binary_mqtt_eclipse_io_pem_start");
extern const uint8_t mqtt_eclipse_io_pem_end[] asm("_binary_mqtt_eclipse_io_pem_end");

///////////////////////////////////////////////////////////////////////////////
// send_binary
//
//
// Note: this function is for testing purposes only publishing part of the active partition
//       (to be checked against the original binary)
//

// static void
// send_binary(esp_mqtt_client_handle_t client)
// {
//   spi_flash_mmap_handle_t out_handle;
//   const void *binary_address;
//   const esp_partition_t *partition = esp_ota_get_running_partition();
//   esp_partition_mmap(partition, 0, partition->size, SPI_FLASH_MMAP_DATA, &binary_address, &out_handle);
//   // sending only the configured portion of the partition (if it's less than the partition size)
//   int binary_size = MIN(4096, partition->size);
//   int msg_id      = esp_mqtt_client_publish(client, "/topic/binary", binary_address, binary_size, 0, 0);
//   ESP_LOGI(TAG, "binary sent with msg_id=%d", msg_id);
// }

// ============================================================================
//                          Topic Substitution
// ============================================================================

/**
 * @brief Perform mustache-style template substitution in MQTT topic strings
 *
 * This function replaces template tags (e.g., {{guid}}, {{class}}) with actual
 * values from the node configuration and/or VSCP event. This enables dynamic
 * topic generation based on event content and node identity.
 *
 * Supported substitution tags:
 * - {{node}}       : Node name from configuration
 * - {{guid}}       : Node GUID (16-byte hex string with colons)
 * - {{evguid}}     : Event GUID (requires pev)
 * - {{class}}      : VSCP event class number (requires pev)
 * - {{type}}       : VSCP event type number (requires pev)
 * - {{nickname}}   : Node nickname (LSB 2 bytes of GUID as decimal)
 * - {{evnickname}} : Event source nickname (requires pev)
 * - {{sindex}}     : Sensor index for measurement events (requires pev)
 *
 * Example:
 *   Input:  "vscp/{{guid}}/{{class}}/{{type}}"
 *   Output: "vscp/FF:FF:FF:FF:FF:FF:FF:F5:01:00:00:00:00:00:00:02/20/9"
 *
 * @param newTopic  Buffer to store the substituted topic string
 * @param len       Size of newTopic buffer
 * @param pTopic    Template topic string with {{tags}} to substitute
 * @param pev       Pointer to VSCP event (can be NULL if event tags not needed)
 *
 * @return VSCP_ERROR_SUCCESS on success, VSCP_ERROR_MEMORY if allocation fails
 */
static int
mqtt_topic_subst(char *newTopic, size_t len, const char *pTopic, const vscp_event_t *pev)
{
  char workbuf[48];

  // Supported mustache template tags (currently implemented subset):
  // {{node}}      - Node name
  // {{guid}}      - Node GUID
  // {{evguid}}    - Event GUID
  // {{class}}     - Event class
  // {{type}}      - Event type
  // {{nickname}}  - Node nickname (16-bit)
  // {{evnickname}}- Event source nickname (16-bit)
  // {{sindex}}    - Sensor index (measurements only)
  //
  // Note: Additional tags like {{timestamp}}, {{zone}}, {{subzone}}, {{d[n]}},
  //       date/time fields are planned but not yet implemented
  //
  // Example: "vscp/{{guid}}/{{class}}/{{type}}" becomes
  //          "vscp/FF:FF:FF:FF:FF:FF:FF:F5:01:00:00:00:00:00:00:02/20/9"

  // Copy template topic to output buffer
  strncpy(newTopic, pTopic, MIN(len, strlen(pTopic)));

  // Allocate temporary buffer for iterative substitution
  char *saveTopic = (char *) calloc(1, len);
  if (NULL == saveTopic) {
    return VSCP_ERROR_MEMORY;
  }

  // Substitute {{node}} with configured node name
  vscp_fwhlp_strsubst(newTopic, len, pTopic, "{{node}}", g_persistent.nodeName);
  ESP_LOGV(TAG, "Substituted {{node}}: %s", newTopic);
  strncpy(saveTopic, newTopic, MIN(strlen(newTopic), len));

  // Substitute {{guid}} with node GUID (format: XX:XX:XX:...)
  vscp_fwhlp_writeGuidToString(workbuf, g_persistent.guid);
  vscp_fwhlp_strsubst(newTopic, len, saveTopic, "{{guid}}", workbuf);
  strcpy(saveTopic, newTopic);

  // Process event-specific tags only if event pointer provided
  if (NULL != pev) {

    // Event GUID
    vscp_fwhlp_writeGuidToString(workbuf, pev->GUID);
    vscp_fwhlp_strsubst(newTopic, len, saveTopic, "{{evguid}}", workbuf);
    strcpy(saveTopic, newTopic);

    // Class
    sprintf(workbuf, "%d", pev->vscp_class);
    vscp_fwhlp_strsubst(newTopic, len, saveTopic, "{{class}}", workbuf);
    strcpy(saveTopic, newTopic);

    // Type
    sprintf(workbuf, "%d", pev->vscp_type);
    vscp_fwhlp_strsubst(newTopic, len, saveTopic, "{{type}}", workbuf);
    strcpy(saveTopic, newTopic);

    // nickname
    sprintf(workbuf, "%d", ((g_persistent.guid[14] << 8) + (g_persistent.guid[15])));
    vscp_fwhlp_strsubst(newTopic, len, saveTopic, "{{nickname}}", workbuf);
    strcpy(saveTopic, newTopic);

    // event nickname
    sprintf(workbuf, "%d", ((pev->GUID[14] << 8) + (pev->GUID[15])));
    vscp_fwhlp_strsubst(newTopic, len, saveTopic, "{{evnickname}}", workbuf);
    strcpy(saveTopic, newTopic);

    // sensor index
    if (VSCP_ERROR_SUCCESS == vscp_fwhlp_isMeasurement(pev)) {
      sprintf(workbuf, "%d", vscp_fwhlp_getMeasurementSensorIndex(pev));
    }
    else {
      memset(workbuf, 0, sizeof(workbuf));
    }
    vscp_fwhlp_strsubst(newTopic, len, saveTopic, "{{sindex}}", workbuf);
    strcpy(saveTopic, newTopic);
  }

  free(saveTopic);

  return VSCP_ERROR_SUCCESS;
}

// Buffer size for topic substitution and JSON conversion
#define MQTT_SUBST_BUF_LEN 3000

// ============================================================================
//                          VSCP Event Publishing
// ============================================================================

/**
 * @brief Publish a VSCP event to the MQTT broker as JSON
 *
 * Converts the VSCP event to JSON format and publishes it to the specified
 * MQTT topic. If no topic is provided, uses the default publish topic from
 * configuration. Topic templates are expanded using mqtt_topic_subst().
 *
 * The event is serialized to JSON using vscp_fwhlp_create_json() which creates
 * a standard VSCP JSON representation containing timestamp, class, type, GUID,
 * and data fields.
 *
 * @param topic  MQTT topic string (may contain {{tags}}), or NULL for default
 * @param pev    Pointer to VSCP event to publish (must not be NULL)
 *
 * @return VSCP_ERROR_SUCCESS on success
 *         VSCP_ERROR_INVALID_POINTER if pev is NULL
 *         VSCP_ERROR_MEMORY if buffer allocation fails
 *         Error code from vscp_fwhlp_create_json() on JSON conversion failure
 *
 * @note If MQTT is not connected, increments failure counter and returns success
 * @note Updates s_mqtt_statistics.nPub on success or .nPubFailures on failure
 */
// int
// mqtt_send_vscp_event(const char *topic, const vscp_event_t *pev)
// {
//   int rv             = VSCP_ERROR_SUCCESS;
//   const char *pTopic = topic;

//   // Check event pointer
//   if (NULL == pev) {
//     return VSCP_ERROR_INVALID_POINTER;
//   }

//   // Use default publish topic from config if none specified
//   if (NULL == topic) {
//     pTopic = g_persistent.mqttPubTopic;
//   }

//   // Silently fail if not connected (count as failure but return success)
//   if (!s_mqtt_connected) {
//     s_mqtt_statistics.nPubFailures++;
//     return VSCP_ERROR_SUCCESS;
//   }

//   // Allocate buffer for JSON conversion of VSCP event
//   char *pbuf = malloc(MQTT_SUBST_BUF_LEN);
//   if (NULL == pbuf) {
//     ESP_LOGE(TAG, "Unable to allocate JSON buffer for conversion");
//     return VSCP_ERROR_MEMORY;
//   }

//   // Determine the format to use for the MQTT message
//   switch (g_persistent.mqttFormat) {
//     case MQTT_FORMAT_JSON:
//       rv = vscp_fwhlp_create_json(pbuf, MQTT_SUBST_BUF_LEN, pev);
//       break;

//     case MQTT_FORMAT_XML:
//       //rv = vscp_fwhlp_create_xml(pbuf, MQTT_SUBST_BUF_LEN, pev);
//       break;

//     case MQTT_FORMAT_STRING:
//       rv = vscp_fwhlp_eventToString(pbuf, MQTT_SUBST_BUF_LEN, pev);
//       break;

//     case MQTT_FORMAT_BINARY:
//       //rv = vscp_fwhlp_create_binary(pbuf, MQTT_SUBST_BUF_LEN, pev);
//       break;

//     default:
//       ESP_LOGE(TAG, "Unsupported MQTT format");
//       free(pbuf);
//       return VSCP_ERROR_INVALID_SYNTAX;
//   }

//   if (VSCP_ERROR_SUCCESS != rv) {
//     free(pbuf);
//     ESP_LOGE(TAG, "Failed to convert event to selected format rv = %d", rv);
//     return rv;
//   }

//   ESP_LOGV(TAG, "Event converted to JSON format");

//   // Allocate buffer for topic template expansion
//   char *newTopic = calloc(MQTT_SUBST_BUF_LEN, 1);
//   if (NULL == newTopic) {
//     ESP_LOGE(TAG, "Unable to allocate memory.");
//     free(pbuf);
//     return VSCP_ERROR_MEMORY;
//   }

//   // Expand topic template with event-specific values
//   mqtt_topic_subst(newTopic, MQTT_SUBST_BUF_LEN, pTopic, pev);
//   ESP_LOGV(TAG, "Expanded MQTT topic: %s", newTopic);

//   // Publish JSON payload to MQTT broker with configured QoS and retain flag
//   int msgid =
//     esp_mqtt_client_publish(g_mqtt_client, newTopic, pbuf, strlen(pbuf), g_persistent.mqttQos, g_persistent.mqttRetain);

//   // Update statistics based on publish result
//   if (-1 != msgid) {
//     s_mqtt_statistics.nPub++;
//   }
//   else {
//     s_mqtt_statistics.nPubFailures++;
//     ESP_LOGE(TAG,
//              "Failed to publish MQTT message. id=%d Topic=%s outbox-size = %d",
//              msgid,
//              newTopic,
//              esp_mqtt_client_get_outbox_size(g_mqtt_client));
//   }

//   free(newTopic);
//   free(pbuf);

//   return VSCP_ERROR_SUCCESS;
// }

// ============================================================================
//                              Logging
// ============================================================================

/**
 * @brief Publish a log message to the MQTT broker
 *
 * Publishes plain text log messages to the configured log topic. This allows
 * remote monitoring of device logs via MQTT. The log topic is separate from
 * the event publish topic and can include template tags.
 *
 * @param msg  Log message string to publish (plain text)
 *
 * @return VSCP_ERROR_SUCCESS on success
 *         VSCP_ERROR_MEMORY if buffer allocation fails
 *
 * @note If log topic is not configured (empty string), function returns immediately
 * @note Updates s_mqtt_statistics.nPubLog on success or .nPubLogFailures on failure
 */
int
mqtt_log(char *msg)
{
  // Nothing to do if message is empty
  if ((NULL == msg) || ('\0' == *msg)) {
    return VSCP_ERROR_SUCCESS;
  }

  // Only publish if MQTT is selected as logging type and topic is configured
  if (!g_persistent.enableMqtt || (LOG_TYPE_MQTT != g_persistent.logType) || !strlen(g_persistent.logMqttTopic)) {
    return VSCP_ERROR_SUCCESS;
  }

  // MQTT log publishing can be called very early during boot via log hooks.
  // Silently skip until the client is initialized and connected.
  if ((NULL == g_mqtt_client) || !s_mqtt_connected) {
    return VSCP_ERROR_SUCCESS;
  }

  char *newTopic = calloc(MQTT_SUBST_BUF_LEN, 1);
  if (NULL == newTopic) {
    ESP_LOGE(TAG, "Unable to allocate memory.");
    return VSCP_ERROR_MEMORY;
  }

  // Expand log topic template (no event context)
  const char *pTopic = g_persistent.logMqttTopic;
  mqtt_topic_subst(newTopic, MQTT_SUBST_BUF_LEN, pTopic, NULL);

  // Publish log message with configured QoS and retain flag
  int msgid = esp_mqtt_client_publish(g_mqtt_client,
                                      newTopic,
                                      msg,
                                      strlen(msg),
                                      g_persistent.mqttQos,
                                      g_persistent.mqttRetain);
  if (-1 != msgid) {
    s_mqtt_statistics.nPubLog++;
  }
  else {
    s_mqtt_statistics.nPubLogFailures++;
    ESP_LOGE(TAG,
             "Failed to publish MQTT log message. id=%d Topic=%s outbox-size = %d",
             msgid,
             g_persistent.logMqttTopic,
             esp_mqtt_client_get_outbox_size(g_mqtt_client));
  }

  free(newTopic);

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// mqtt_task_tx
//
// From CAN
//
// This task receives VSCP events from the CAN queue, converts them to JSON,
// performs topic substitution, and publishes them to the MQTT broker. It runs
// continuously and handles all outgoing MQTT messages based on CAN events.

static void
mqtt_task_tx(void *pvParameters)
{
  int rv;
  can4vscp_frame_t rxmsg = {};

  ESP_LOGI(TAG, "MQTT rx (from CAN) task started");

  // Add the task to the watchdog timer
  // esp_task_wdt_add(NULL);

  while (1) {

    // esp_task_wdt_reset(); // Reset the watchdog timer

    long status = xQueueReceive(tr_mqtt.fromcan_queue, (void *) &rxmsg, 500);

    if (status == pdPASS) {
      ESP_LOGI(TAG, "Received message from CAN queue: ID=0x%X DLC=%d", rxmsg.identifier, rxmsg.data_length_code);
    }

    // Create a VSCP event from the received CAN message
    vscp_event_t *pev;
    if (VSCP_ERROR_SUCCESS != (rv = can4vscp_msg_to_event(&pev, &rxmsg))) {
      ESP_LOGE(TAG, "Failed to convert CAN message to VSCP event rv=%d", rv);
      vscp_fwhlp_deleteEvent(&pev);
      continue;
    }

    char buf_msg[MQTT_SUBST_BUF_LEN]   = { 0 };
    char buf_topic[MQTT_SUBST_BUF_LEN] = { 0 };
    int payload_len                    = 0;

    switch (g_persistent.mqttFormat) {

      case MQTT_FORMAT_JSON:
        // Create JSON representaion of the VSCP event for MQTT payload
        if (VSCP_ERROR_SUCCESS != (rv = vscp_fwhlp_create_json(buf_msg, sizeof(buf_msg), pev))) {
          ESP_LOGE(TAG, "Failed to convert VSCP event to JSON rv=%d", rv);
          vscp_fwhlp_deleteEvent(&pev);
          continue;
        }
        payload_len = strnlen(buf_msg, sizeof(buf_msg));
        break;

      case MQTT_FORMAT_XML:
        ESP_LOGV(TAG, "XML format %s", buf_msg);
        if (VSCP_ERROR_SUCCESS != (rv = vscp_fwhlp_event_to_xml(buf_msg, sizeof(buf_msg), pev))) {
          ESP_LOGE(TAG, "Failed to convert VSCP event to XML rv=%d", rv);
          vscp_fwhlp_deleteEvent(&pev);
          continue;
        }

        payload_len = strnlen(buf_msg, sizeof(buf_msg));

        break;

      case MQTT_FORMAT_STRING:
        // Create string representaion of the VSCP event for MQTT payload
        if (VSCP_ERROR_SUCCESS != (rv = vscp_fwhlp_eventToString(buf_msg, sizeof(buf_msg), pev))) {
          ESP_LOGE(TAG, "Failed to convert VSCP event to string rv=%d", rv);
          vscp_fwhlp_deleteEvent(&pev);
          continue;
        }
        payload_len = strnlen(buf_msg, sizeof(buf_msg));
        break;

      case MQTT_FORMAT_BINARY:
        payload_len = (int) vscp_fwhlp_getFrameSizeFromEvent(pev);
        if ((payload_len <= 0) || (payload_len > sizeof(buf_msg))) {
          ESP_LOGE(TAG, "Invalid VSCP binary frame size %d", payload_len);
          vscp_fwhlp_deleteEvent(&pev);
          continue;
        }

        if (VSCP_ERROR_SUCCESS != (rv = vscp_fwhlp_writeEventToFrame((uint8_t *) buf_msg, payload_len, 0, pev))) {
          ESP_LOGE(TAG, "Failed to convert VSCP event to binary format rv=%d", rv);
          vscp_fwhlp_deleteEvent(&pev);
          continue;
        }
        break;

      default:
        ESP_LOGE(TAG, "Unsupported MQTT format");
        vscp_fwhlp_deleteEvent(&pev);
        continue;
    }

    // Create the dynamic topic name by substituting template tags with event values
    if (VSCP_ERROR_SUCCESS != mqtt_topic_subst(buf_topic, sizeof(buf_topic), g_persistent.mqttPubTopic, pev)) {
      ESP_LOGE(TAG, "Failed to substitute MQTT topic");
      continue;
    }

    // Event not needed anymore, free it before topic substitution
    vscp_fwhlp_deleteEvent(&pev);

    int msgid = esp_mqtt_client_publish(g_mqtt_client,
                                        buf_topic,
                                        buf_msg,
                                        payload_len,
                                        g_persistent.mqttQos,
                                        g_persistent.mqttRetain);
    if (-1 != msgid) {
      s_mqtt_statistics.nPub++;
    }
    else {
      s_mqtt_statistics.nPubFailures++;
      ESP_LOGE(TAG,
               "Failed to publish MQTT message. Topic=%s outbox-size=%d",
               buf_topic,
               esp_mqtt_client_get_outbox_size(g_mqtt_client));
    }

    // vTaskDelay(pdMS_TO_TICKS(1000));
  }

  vTaskDelete(NULL);
}

///////////////////////////////////////////////////////////////////////////////
// mqtt_task_rx
//
// To CAN
//
// This task receives MQTT messages from the mqtt_event_handler via a queue,
// converts them to VSCP events, and forwards them to the CAN interface. It runs
// continuously and handles all incoming MQTT messages that are relevant to the gateway.

static void
mqtt_task_rx(void *pvParameters)
{
  int rv;
  ESP_LOGI(TAG, "MQTT rx (to CAN) task started");

  // Add the task to the watchdog timer
  // esp_task_wdt_add(NULL);

  while (1) {

    // esp_task_wdt_reset(); // Reset the watchdog timer

    mqtt_rx_msg_t rx = { 0 };
    long status      = xQueueReceive(s_mqtt_rx_queue, (void *) &rx, 500);
    if (status == pdPASS) {
      vscp_event_t ev = { 0 };
      ESP_LOGV(TAG, "Received MQTT message: topic='%s' \npayload='%s'", rx.topic, rx.payload);

      switch (rx.payload[0]) {

        case '{': // Assume JSON payload
          if (VSCP_ERROR_SUCCESS != (rv = vscp_fwhlp_parse_json(&ev, rx.payload))) {
            ESP_LOGE(TAG, "Failed to parse MQTT JSON payload as VSCP event rv=%d", rv);
            continue;
          }
          break;

        case '<': // Assume XML payload (not implemented)
          if (VSCP_ERROR_SUCCESS != (rv = vscp_fwhlp_parse_xml_event(&ev, rx.payload))) {
            ESP_LOGE(TAG, "Failed to parse MQTT XML payload as VSCP event rv=%d", rv);
            continue;
          }
          break;

        case 0x00: // Assume binary payload (not implemented)
          ESP_LOGI(TAG, "Binary payload format not supported yet");
          // if (VSCP_ERROR_SUCCESS != vscp_fwhlp_parse_binary_event(&ev, rx.payload)) {
          //   ESP_LOGE(TAG, "Failed to parse MQTT binary payload as VSCP event");
          //   continue;
          // }
          continue;

        default: // Assume string payload
          if (VSCP_ERROR_SUCCESS != (rv = vscp_fwhlp_parseStringToEvent(&ev, rx.payload))) {
            ESP_LOGE(TAG, "Failed to parse MQTT string payload as VSCP event rv=%d", rv);
            continue;
          }
          break;
      }

      can4vscp_frame_t txmsg = { 0 };
      txmsg.extd             = 1;
      txmsg.identifier       = ev.GUID[0] + (ev.vscp_type << 8) + (ev.vscp_class << 16) + (((ev.head >> 5) & 7) << 26);

      if (ev.sizeData > sizeof(txmsg.data)) {
        ESP_LOGW(TAG,
                 "VSCP event payload too large for CAN frame (%d), truncating to %d",
                 (int) ev.sizeData,
                 (int) sizeof(txmsg.data));
      }

      txmsg.data_length_code = MIN((size_t) ev.sizeData, sizeof(txmsg.data));
      if (txmsg.data_length_code > 0 && NULL != ev.pdata) {
        memcpy(txmsg.data, ev.pdata, txmsg.data_length_code);
      }

      if (ESP_OK != can4vscp_send(&txmsg, pdMS_TO_TICKS(100))) {
        ESP_LOGW(TAG, "Failed to transmit CAN frame from MQTT payload");
      }
      else {
        s_mqtt_statistics.nSub++;
        ESP_LOGI(TAG,
                 "MQTT->CAN forwarded topic=%s id=0x%X dlc=%d",
                 rx.topic,
                 txmsg.identifier,
                 txmsg.data_length_code);
      }

      if (NULL != ev.pdata) {
        free(ev.pdata);
        ev.pdata = NULL;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  vTaskDelete(NULL);
}

// ============================================================================
//                          MQTT Event Handler
// ============================================================================

/**
 * @brief Event handler callback for MQTT client events
 *
 * This function is called by the ESP-IDF MQTT event loop for all MQTT-related
 * events including connection, disconnection, publish confirmation, incoming
 * data, subscriptions, and errors. It maintains connection state and statistics.
 *
 * Key functionality:
 * - Sets s_mqtt_connected flag on CONNECT/DISCONNECT
 * - Subscribes to configured topics on connection
 * - Updates statistics counters for all events
 * - Logs detailed error information for troubleshooting
 *
 * @param handler_args User data registered with the event (unused)
 * @param base         Event base identifier (ESP_EVENT_MQTT_BASE)
 * @param event_id     Specific MQTT event type (MQTT_EVENT_*)
 * @param event_data   Event data structure (esp_mqtt_event_handle_t)
 */
static void
mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
  ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, (int) event_id);
  esp_mqtt_event_handle_t event   = event_data;
  esp_mqtt_client_handle_t client = event->client;
  int msg_id;
  switch ((esp_mqtt_event_id_t) event_id) {

    case MQTT_EVENT_BEFORE_CONNECT:
      ESP_LOGI(TAG, "Preparing to connect");
      break;

    case MQTT_EVENT_CONNECTED:
      s_mqtt_connected = true;
      s_mqtt_statistics.nConnect++;
      ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");

      // Subscribe to configured topics (TODO: make this configurable)
      msg_id = esp_mqtt_client_subscribe(client, g_persistent.mqttSubTopic,
                                         0); // QoS 0
      ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
      break;

    case MQTT_EVENT_DISCONNECTED:
      s_mqtt_connected = false;
      s_mqtt_statistics.nDisconnect++;
      ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
      // esp_mqtt_client_reconnect(client);
      break;

    case MQTT_EVENT_SUBSCRIBED:
      ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
      // Acknowledge successful subscription with status message
      // msg_id = esp_mqtt_client_publish(client, "esp-now/status", "Successful subscribe", 0, 0, 0);
      // ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
      break;

    case MQTT_EVENT_UNSUBSCRIBED:
      ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
      break;

    case MQTT_EVENT_PUBLISHED:
      // Increment confirmation counter (for QoS > 0)
      s_mqtt_statistics.nPubConfirm++;
      ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
      break;

    case MQTT_EVENT_DATA:
      // Keep callback work minimal: copy payload/topic and hand off to worker task
      ESP_LOGI(TAG, "MQTT_EVENT_DATA");

      if ((event->total_data_len > event->data_len) || (event->current_data_offset > 0)) {
        ESP_LOGW(TAG,
                 "Fragmented MQTT payload not supported in callback handoff (%d/%d)",
                 event->data_len,
                 event->total_data_len);
        break;
      }

      if (NULL == s_mqtt_rx_queue) {
        ESP_LOGW(TAG, "MQTT RX queue not initialized");
        break;
      }

      mqtt_rx_msg_t rx = { 0 };
      rx.topic_len     = MIN((size_t) event->topic_len, (size_t) (MQTT_RX_TOPIC_MAX_LEN - 1));
      rx.payload_len   = MIN((size_t) event->data_len, (size_t) (MQTT_RX_PAYLOAD_MAX_LEN - 1));

      if ((size_t) event->data_len >= MQTT_RX_PAYLOAD_MAX_LEN) {
        ESP_LOGW(TAG,
                 "Incoming MQTT payload truncated from %d to %d bytes",
                 event->data_len,
                 MQTT_RX_PAYLOAD_MAX_LEN - 1);
      }

      memcpy(rx.topic, event->topic, rx.topic_len);
      rx.topic[rx.topic_len] = '\0';

      memcpy(rx.payload, event->data, rx.payload_len);
      rx.payload[rx.payload_len] = '\0';

      if (pdPASS != xQueueSendToBack(s_mqtt_rx_queue, (void *) &rx, (TickType_t) 0)) {
        tr_mqtt.overruns++;
        ESP_LOGW(TAG, "MQTT RX queue full, dropping message");
      }
      break;

    case MQTT_EVENT_ERROR:
      s_mqtt_statistics.nErrors++;
      ESP_LOGI(TAG, "MQTT_EVENT_ERROR");

      // Detailed error logging for troubleshooting
      if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
        // TLS/TCP transport errors (including certificate validation failures)
        ESP_LOGI(TAG, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
        ESP_LOGI(TAG, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
        ESP_LOGI(TAG,
                 "Last captured errno : %d (%s)",
                 event->error_handle->esp_transport_sock_errno,
                 strerror(event->error_handle->esp_transport_sock_errno));
      }
      else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
        // MQTT protocol-level connection refusal (bad credentials, etc.)
        ESP_LOGI(TAG, "Connection refused error: 0x%x", event->error_handle->connect_return_code);
      }
      else {
        ESP_LOGW(TAG, "Unknown error type: 0x%x", event->error_handle->error_type);
      }
      break;

    default:
      ESP_LOGI(TAG, "Other event id:%d", event->event_id);
      break;
  }
}

// ============================================================================
//                          MQTT Client Control
// ============================================================================

/**
 * @brief Initialize and start the MQTT client
 *
 * This function creates and starts the MQTT client using configuration from
 * g_persistent. It supports both plain MQTT and MQTT over TLS/SSL with
 * optional client certificate authentication.
 *
 * Configuration sources:
 * - Broker: g_persistent.mqttUrl and mqttPort
 * - Credentials: mqttUser, mqttPw
 * - Client ID: mqttClientId (supports {{node}} and {{guid}} templates)
 * - TLS: enableMqttTls, mqttCaCert, mqttClientCert, mqttClientKey
 *
 * TLS Features:
 * - Automatic scheme selection (mqtt:// vs mqtts://)
 * - Server certificate verification via CA certificate
 * - Optional mutual TLS with client certificate and private key
 * - Certificates stored in PEM format (up to 1024 bytes each)
 *
 * @note Client ID templates are expanded before connection
 * @note Session is persistent (clean_session = false)
 * @note Keepalive interval is 60 seconds
 */
void
mqtt_start(void)
{
  ESP_LOGI(TAG, "Starting MQTT client");

  if (NULL == s_mqtt_rx_queue) {
    s_mqtt_rx_queue = xQueueCreate(MQTT_RX_QUEUE_LEN, sizeof(mqtt_rx_msg_t));
    if (NULL == s_mqtt_rx_queue) {
      ESP_LOGE(TAG, "Failed to create MQTT RX handoff queue");
      return;
    }
  }

  // Get base MAC address (used for unique identification if needed)
  uint8_t mac[8];
  ESP_ERROR_CHECK(esp_base_mac_addr_get(mac));

  char clientid[128], save[128], workbuf[48];

  memset(clientid, 0, sizeof(clientid));
  strncpy(clientid, g_persistent.mqttClientId, sizeof(clientid) - 1);

  // Expand client ID template: {{node}} → node name
  vscp_fwhlp_strsubst(clientid, sizeof(clientid), g_persistent.mqttClientId, "{{node}}", g_persistent.nodeName);
  strcpy(save, clientid);

  // Expand client ID template: {{guid}} → GUID string
  vscp_fwhlp_writeGuidToString(workbuf, g_persistent.guid);
  vscp_fwhlp_strsubst(clientid, sizeof(clientid), save, "{{guid}}", workbuf);

  char uri[256];
  // Build broker URI with appropriate scheme based on TLS setting
  // mqtt://  = plain TCP connection (port 1883 typical)
  // mqtts:// = TLS/SSL encrypted connection (port 8883 typical)
  if (g_persistent.enableMqttTls) {
    sprintf(uri, "mqtts://%s:%d", g_persistent.mqttUrl, g_persistent.mqttPort);
  }
  else {
    sprintf(uri, "mqtt://%s", g_persistent.mqttUrl);
    // sprintf(uri, "mqtt://%s:%d", g_persistent.mqttUrl, g_persistent.mqttPort);
  }

  ESP_LOGI(TAG, "MQTT client id: %s", clientid);

  // Configure MQTT client based on ESP-IDF version
  // clang-format off
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  // ESP-IDF v5.0+ configuration structure
  esp_mqtt_client_config_t mqtt_cfg = {
    .broker = { 
                .address.uri = uri,               // Complete URI with scheme
                .address.port = g_persistent.mqttPort,
              },    
    .session.disable_clean_session = true,          // Persistent session
    .session.keepalive = 60,                        // 60 second keepalive
    .credentials.username                = g_persistent.mqttUser,
    .credentials.authentication.password = g_persistent.mqttPw,
    .task.priority = 5,
  };

  // Only set client_id if it's not empty (ESP-IDF v5 allows empty client_id for auto-generated ID)
  if (strlen(clientid) > 0) {
    mqtt_cfg.credentials.client_id               = clientid;
  }

  // Configure TLS/SSL certificates if TLS is enabled
  // Certificates are in PEM format and stored in persistent config
  if (g_persistent.enableMqttTls) {
    // CA certificate for server verification (required for TLS)
    if (strlen(g_persistent.mqttCaCert) > 0) {
      mqtt_cfg.broker.verification.certificate = g_persistent.mqttCaCert;
      ESP_LOGI(TAG, "MQTT TLS: CA certificate configured");
    }
    // Client certificate for mutual TLS authentication (optional)
    if (strlen(g_persistent.mqttClientCert) > 0) {
      mqtt_cfg.credentials.authentication.certificate = g_persistent.mqttClientCert;
      ESP_LOGI(TAG, "MQTT TLS: Client certificate configured");
    }
    // Client private key for mutual TLS authentication (optional, pairs with client cert)
    if (strlen(g_persistent.mqttClientKey) > 0) {
      mqtt_cfg.credentials.authentication.key = g_persistent.mqttClientKey;
      ESP_LOGI(TAG, "MQTT TLS: Client private key configured");
    }
  }
#else
  // ESP-IDF v4.x configuration structure (legacy)
  esp_mqtt_client_config_t mqtt_cfg = {
    .uri          = uri,
    .event_handle = mqtt_event_handler,
    .client_id    = clientid
  };
#endif
  // clang-format on

  // Initialize MQTT client with configuration
  g_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
  // Register event handler for all MQTT events
  if (ESP_OK != esp_mqtt_client_register_event(g_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL)) {
    ESP_LOGE(TAG, "Failed to register MQTT event handler");
  }

  // Start the MQTT client (initiates connection to broker)
  if (ESP_OK != esp_mqtt_client_start(g_mqtt_client)) {
    ESP_LOGE(TAG, "Failed to start MQTT client");
  }

  // Create separate tasks for MQTT TX and RX processing
  // mqtt_task_tx handles outgoing messages based on CAN events
  // mqtt_task_rx handles incoming MQTT messages and converts them to CAN events
  xTaskCreate(mqtt_task_tx, "mqtt_tx", 1024 * 12, (void *) &tr_mqtt, 5, NULL);
  xTaskCreate(mqtt_task_rx, "mqtt_rx", 8192, (void *) &tr_mqtt, 5, NULL);

  ESP_LOGI(TAG, "Outbox-size = %d", esp_mqtt_client_get_outbox_size(g_mqtt_client));
}

/**
 * @brief Stop the MQTT client and disconnect from broker
 *
 * Cleanly shuts down the MQTT client connection. Should be called before
 * restarting WiFi or during device shutdown.
 */
void
mqtt_stop(void)
{
  esp_mqtt_client_stop(g_mqtt_client);
}
