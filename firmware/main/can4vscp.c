/*
 * This file is part of the VSCP CAN4VSCP Gateway project.
 *
 * Copyright (C) 2022  Meatpi Electronics.
 * Written by Ali Slim <ali@meatpi.com>
 * Changes and additions Copyright (C) 2022-2026 Ake Hedman, the VSCP Project <ake@vscp.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*!
  Modifications done for the VSCP can gw project by
  Ake Hedman, Grodans Paradis AB 2022-2025
  Part of the VSCP project (https://www.vcsp.org)
*/

#include "vscp-compiler.h"
#include "vscp-projdefs.h"

#include "driver/gpio.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"

#include <esp_timer.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_wifi.h>

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include <nvs_flash.h>
#include <lwip/sockets.h>

#include <vscp-firmware-helper.h>

#include "main.h"
#include "can4vscp.h"

#include <string.h>

static EventGroupHandle_t s_can4vscp_event_group;
#define CAN4VSCP_ENABLE_BIT BIT0

extern SemaphoreHandle_t ctrl_task_sem;
// extern QueueHandle_t xmsg_Rx_Queue;

// Global stuff
extern node_persistent_config_t g_persistent;
extern transport_t tr_twai_rx;
extern transport_t tr_twai_tx;
extern transport_t tr_tcpsrv[MAX_TCP_CONNECTIONS];
extern transport_t tr_mqtt;
extern transport_t tr_multicast;
extern transport_t tr_udp;
extern transport_t tr_websockets;
extern transport_t tr_canapi;
extern transport_t tr_monitor;

#define TAG __func__
enum bus_state { OFF_BUS, ON_BUS };

static twai_node_handle_t s_twai_node  = NULL;
static QueueHandle_t s_twai_rx_queue   = NULL;
static volatile uint32_t s_rx_overruns = 0;

// temperature_sensor_handle_t temp_handle = NULL;

// static EventGroupHandle_t s_twai_event_group;
//
static TimerHandle_t xCAN4VSCP_EN_Timer;
static uint8_t datarate = CAN4VSCP_125K;
static can4vscp_cfg_t can4vscp_cfg;

#define CAN4VSCP_TWAI_TX_QUEUE_DEPTH 100
#define CAN4VSCP_TWAI_RX_QUEUE_DEPTH 100

static uint32_t
can4vscp_bitrate_from_code(uint8_t rate)
{
  switch (rate) {
    case CAN4VSCP_5K:
      return 5000;
    case CAN4VSCP_10K:
      return 10000;
    case CAN4VSCP_20K:
      return 20000;
    case CAN4VSCP_25K:
      return 25000;
    case CAN4VSCP_50K:
      return 50000;
    case CAN4VSCP_100K:
      return 100000;
    case CAN4VSCP_125K:
      return 125000;
    case CAN4VSCP_250K:
      return 250000;
    case CAN4VSCP_500K:
      return 500000;
    case CAN4VSCP_800K:
      return 800000;
    case CAN4VSCP_1000K:
      return 1000000;
    default:
      return 125000;
  }
}

static bool IRAM_ATTR
can4vscp_rx_done_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
  (void) edata;
  (void) user_ctx;

  uint8_t data[TWAI_FRAME_MAX_LEN] = { 0 };
  twai_frame_t rx_frame            = {
               .buffer     = data,
               .buffer_len = sizeof(data),
  };

  if (ESP_OK != twai_node_receive_from_isr(handle, &rx_frame)) {
    return false;
  }

  can4vscp_frame_t frame = { 0 };
  frame.identifier       = rx_frame.header.id;
  frame.extd             = rx_frame.header.ide;
  frame.rtr              = rx_frame.header.rtr;

  size_t len             = MIN((size_t) twaifd_dlc2len(rx_frame.header.dlc), sizeof(frame.data));
  frame.data_length_code = len;
  if (len) {
    memcpy(frame.data, data, len);
  }

  BaseType_t task_woken = pdFALSE;
  if ((NULL == s_twai_rx_queue) || (pdPASS != xQueueSendToBackFromISR(s_twai_rx_queue, &frame, &task_woken))) {
    s_rx_overruns++;
  }

  return (task_woken == pdTRUE);
}

///////////////////////////////////////////////////////////////////////////////
// can4vscp_msg_to_ev
//
// time data and obid is zeroed by calloc
//

int
can4vscp_msg_to_event(vscp_event_t **pev, const can4vscp_frame_t *msg)
{
  if ((NULL == pev) || (NULL == msg)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Allocate a new event
  *pev = (vscp_event_t *) calloc(1, sizeof(vscpEvent));
  if (NULL == *pev) {
    return VSCP_ERROR_MEMORY;
  }

  // Allocate data if the message has data
  if (msg->data_length_code) {
    (*pev)->pdata = (uint8_t *) calloc(1, msg->data_length_code);
    if (NULL == (*pev)->pdata) {
      vscp_fwhlp_deleteEvent(pev);
      return VSCP_ERROR_MEMORY;
    }
    // Copy in data
    (*pev)->sizeData = msg->data_length_code;
    memcpy((*pev)->pdata, msg->data, msg->data_length_code);
  }

  (*pev)->head       = (msg->identifier >> (26 - 5)) & 0x00e0;
  (*pev)->timestamp  = esp_timer_get_time(); // Microseconds since boot
  (*pev)->vscp_class = (msg->identifier >> 16) & 0x1ff;
  (*pev)->vscp_type  = (msg->identifier >> 8) & 0xff;

  // GUID
  memcpy((*pev)->GUID, g_persistent.guid, 16);

  // Set nickname
  (*pev)->GUID[14] = 0x00; // MSB of Node ID is never used for CAN4VSCP
  (*pev)->GUID[15] = msg->identifier & 0xff;
  (*pev)->sizeData = msg->data_length_code;
  // Copy in data if any
  if (msg->data_length_code) {
    memcpy((*pev)->pdata, msg->data, msg->data_length_code);
  }

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// can4vscp_event_to_msg
//

int
can4vscp_event_to_msg(can4vscp_frame_t *msg, const vscp_event_t *pev)
{
  if ((NULL == pev) || (NULL == msg)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  msg->identifier = pev->GUID[0] + (pev->vscp_type << 8) + (pev->vscp_class << 16) + (((pev->head >> 5) & 7) << 26);
  msg->extd       = 1;
  msg->rtr        = 0;

  if (pev->sizeData > sizeof(msg->data)) {
    return VSCP_ERROR_INVALID_FORMAT;
  }

  if (pev->sizeData) {
    memcpy(msg->data, pev->pdata, pev->sizeData);
    msg->data_length_code = pev->sizeData;
  }
  else {
    msg->data_length_code = 0;
  }

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// can4vscp_block
//
// block tx/rx
//

void
can4vscp_block(void)
{
  // Not enabled
  xEventGroupClearBits(s_can4vscp_event_group, CAN4VSCP_ENABLE_BIT);

  // Turn of timer
  if (xTimerIsTimerActive(xCAN4VSCP_EN_Timer) != pdFALSE) {
    xTimerReset(xCAN4VSCP_EN_Timer, 0);
    xTimerStop(xCAN4VSCP_EN_Timer, 0);
  }

  vTaskDelay(pdMS_TO_TICKS(1)); // wait for rx to finish
}

///////////////////////////////////////////////////////////////////////////////
// can4vscp_unblock
//
// unblock tx/rx
//

void
can4vscp_unblock(void)
{
  if (ON_BUS == can4vscp_cfg.bus_state) {
    return;
  }

  if (xTimerIsTimerActive(xCAN4VSCP_EN_Timer) == pdFALSE) {
    xTimerStart(xCAN4VSCP_EN_Timer, 0);
  }
  else {
    xTimerReset(xCAN4VSCP_EN_Timer, 0);
  }
}

///////////////////////////////////////////////////////////////////////////////
// can4vscp_enable
//

void
can4vscp_enable(void)
{
  if (ON_BUS == can4vscp_cfg.bus_state) {
    return;
  }

  twai_onchip_node_config_t node_config = {
    .io_cfg = {
               .tx                = TWAI_TX_GPIO_NUM,
               .rx                = TWAI_RX_GPIO_NUM,
               .quanta_clk_out    = GPIO_NUM_NC,
               .bus_off_indicator = GPIO_NUM_NC,
    },
    .bit_timing = {
                   .bitrate = can4vscp_bitrate_from_code(datarate),
    },
    .tx_queue_depth = CAN4VSCP_TWAI_TX_QUEUE_DEPTH,
    .fail_retry_cnt = -1,
  };

  node_config.flags.enable_listen_only = can4vscp_cfg.silent ? 1 : 0;
  node_config.flags.enable_loopback    = can4vscp_cfg.loopback ? 1 : 0;

  if (ESP_OK != twai_new_node_onchip(&node_config, &s_twai_node)) {
    ESP_LOGE(TAG, "Failed to create TWAI node");
    return;
  }

  twai_event_callbacks_t cbs = {
    .on_rx_done = can4vscp_rx_done_cb,
  };

  if (ESP_OK != twai_node_register_event_callbacks(s_twai_node, &cbs, NULL)) {
    ESP_LOGE(TAG, "Failed to register TWAI callbacks");
    twai_node_delete(s_twai_node);
    s_twai_node = NULL;
    return;
  }

  // Accept all classic extended frames by default
  twai_mask_filter_config_t filter_cfg = {
    .id          = 0,
    .mask        = 0,
    .is_ext      = true,
    .no_classic  = false,
    .no_fd       = true,
    .dual_filter = false,
  };
  if (ESP_OK != twai_node_config_mask_filter(s_twai_node, 0, &filter_cfg)) {
    ESP_LOGW(TAG, "Failed to apply TWAI filter config, continuing with defaults");
  }

  if (ESP_OK != twai_node_enable(s_twai_node)) {
    ESP_LOGE(TAG, "Failed to enable TWAI node");
    twai_node_delete(s_twai_node);
    s_twai_node = NULL;
    return;
  }

  if (NULL != s_twai_rx_queue) {
    xQueueReset(s_twai_rx_queue);
  }

  can4vscp_unblock();
  can4vscp_cfg.bus_state = ON_BUS;
  ESP_LOGI(TAG, "TWAI node enabled");
}

///////////////////////////////////////////////////////////////////////////////
// can4vscp_disable
//

void
can4vscp_disable(void)
{
  // Do nothing if already off
  if (OFF_BUS == can4vscp_cfg.bus_state) {
    return;
  }

  can4vscp_block();
  if (NULL != s_twai_node) {
    twai_node_disable(s_twai_node);
    twai_node_delete(s_twai_node);
    s_twai_node = NULL;
  }
  if (NULL != s_twai_rx_queue) {
    xQueueReset(s_twai_rx_queue);
  }
  can4vscp_cfg.bus_state = OFF_BUS;
}

///////////////////////////////////////////////////////////////////////////////
// can4vscp_setSilent
//

void
can4vscp_setSilent(uint8_t flag)
{
  // Do nothing if active
  if (ON_BUS == can4vscp_cfg.bus_state) {
    return;
  }

  can4vscp_cfg.silent = flag;
}

///////////////////////////////////////////////////////////////////////////////
// can4vscp_isSilent
//

uint8_t
can4vscp_isSilent(void)
{
  return can4vscp_cfg.silent;
}

///////////////////////////////////////////////////////////////////////////////
// can4vscp_setLoopback
//

void
can4vscp_setLoopback(uint8_t flag)
{
  // Do nothing if active
  if (ON_BUS == can4vscp_cfg.bus_state) {
    return;
  }

  can4vscp_cfg.loopback = flag;
}

///////////////////////////////////////////////////////////////////////////////
// can4vscp_isLoopback
//

uint8_t
can4vscp_isLoopback(void)
{
  return can4vscp_cfg.loopback;
}

///////////////////////////////////////////////////////////////////////////////
// can4vscp_setFilter

void
can4vscp_setFilter(uint32_t filter)
{
  // Do nothing if active
  if (ON_BUS == can4vscp_cfg.bus_state) {
    return;
  }

  can4vscp_cfg.filter = filter;
}

///////////////////////////////////////////////////////////////////////////////
// can4vscp_setMask

void
can4vscp_setMask(uint32_t mask)
{
  // Do nothing if active
  if (ON_BUS == can4vscp_cfg.bus_state) {
    return;
  }

  can4vscp_cfg.mask = mask;
}

///////////////////////////////////////////////////////////////////////////////
// can4vscp_setBitrate
//

void
can4vscp_setBitrate(uint8_t rate)
{
  // Should be off bus
  if (ON_BUS == can4vscp_cfg.bus_state) {
    return;
  }

  datarate = rate;
}

///////////////////////////////////////////////////////////////////////////////
// can4vscp_getBitrate
//

uint8_t
can4vscp_getBitrate(void)
{
  return datarate;
}

///////////////////////////////////////////////////////////////////////////////
// can4vscp_Timer_Callback
//

static void
can4vscp_Timer_Callback(TimerHandle_t xTimer)
{
  xEventGroupSetBits(s_can4vscp_event_group, CAN4VSCP_ENABLE_BIT);
  // ESP_LOGI(TAG, "CAN4VSCP Timer Callback");

  // Get converted sensor data
  // float tsens_out;
  // ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_handle, &tsens_out));
  // printf("Temperature is %0.2f °C\n", tsens_out);
}

///////////////////////////////////////////////////////////////////////////////
// can4vscp_init
//

void
can4vscp_init(uint8_t bitrate)
{
  // Tempsensor
  // temperature_sensor_config_t temp_sensor = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 50);
  // ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor, &temp_handle));

  // ESP_ERROR_CHECK(temperature_sensor_enable(temp_handle));

  datarate = bitrate;

  if (NULL == s_twai_rx_queue) {
    s_twai_rx_queue = xQueueCreate(CAN4VSCP_TWAI_RX_QUEUE_DEPTH, sizeof(can4vscp_frame_t));
  }

  s_can4vscp_event_group = xEventGroupCreate();
  xCAN4VSCP_EN_Timer     = xTimerCreate(
    // Just a text name, not used by the RTOS kernel.
    "TwaiTimer",
    // The timer period in ticks, must be greater than 0.
    pdMS_TO_TICKS(15000),
    // The timer will auto-reload when it expire.
    pdTRUE,
    // The ID is used to store a count of the number of times the timer
    // has expired, which is initialised to 0.
    (void *) 0,
    // Each timer calls the same callback when it expires.
    can4vscp_Timer_Callback);

  if (xTimerIsTimerActive(xCAN4VSCP_EN_Timer) != pdFALSE) {
    xTimerStop(xCAN4VSCP_EN_Timer, 0);
  }
}

///////////////////////////////////////////////////////////////////////////////
// can4vscp_isEnabled
//

bool
can4vscp_isEnabled(void)
{
  if (ON_BUS == can4vscp_cfg.bus_state) {
    return true;
  }
  else {
    return false;
  }
}

///////////////////////////////////////////////////////////////////////////////
// can4vscp_getRxMsgCount
//

uint32_t
can4vscp_getRxMsgCount(void)
{
  if (NULL == s_twai_rx_queue) {
    return 0;
  }

  return (uint32_t) uxQueueMessagesWaiting(s_twai_rx_queue);
}

///////////////////////////////////////////////////////////////////////////////
// can4vscp_receive
//

esp_err_t
can4vscp_receive(can4vscp_frame_t *message, TickType_t ticks_to_wait)
{
  if ((NULL == message) || (NULL == s_twai_rx_queue)) {
    return ESP_ERR_INVALID_ARG;
  }

  if (pdPASS == xQueueReceive(s_twai_rx_queue, message, ticks_to_wait)) {
    return ESP_OK;
  }

  return ESP_ERR_TIMEOUT;
}

///////////////////////////////////////////////////////////////////////////////
// can4vscp_send
//

esp_err_t
can4vscp_send(can4vscp_frame_t *message, TickType_t ticks_to_wait)
{
  if ((NULL == message) || (NULL == s_twai_node)) {
    return ESP_ERR_INVALID_ARG;
  }

  uint8_t tx_data[TWAI_FRAME_MAX_LEN] = { 0 };
  size_t len                          = MIN((size_t) message->data_length_code, sizeof(tx_data));
  if (len > 0) {
    memcpy(tx_data, message->data, len);
  }

  twai_frame_t tx_frame = {
    .header = {
               .id  = message->identifier,
               .ide = message->extd,
               .rtr = message->rtr,
               .dlc = len,
    },
    .buffer = tx_data,
    .buffer_len = len,
  };

  int timeout_ms = (ticks_to_wait == portMAX_DELAY) ? -1 : (int) (ticks_to_wait * portTICK_PERIOD_MS);
  esp_err_t rv = twai_node_transmit(s_twai_node, &tx_frame, timeout_ms);

  // Update statistics
  if (ESP_OK == rv) {
    g_persistent.nSent++;
  } else {
    g_persistent.nErr++;
    g_persistent.lastError = (uint32_t) rv;
  }

  return rv;
}

///////////////////////////////////////////////////////////////////////////////
// twai_receive_task
//

void
twai_receive_task(void *arg)
{
  esp_err_t rv;
  // transport_t *ptr = (transport_t *)arg;
  // QueueHandle_t h = ptr->msg_queue;

  ESP_LOGI(TAG, "TWAI receive driver started");

  // QueueHandle_t h = xQueueCreate(10, sizeof( twai_message_t) );

  while (1) {

    can4vscp_frame_t rxmsg = {};

    if (ESP_OK == (rv = can4vscp_receive(&rxmsg, portMAX_DELAY))) {

      ESP_LOGI(TAG, "TWAI msg received id= %X", (unsigned int) rxmsg.identifier);

      // Update statistics
      g_persistent.nRecv++;

      // Must be extended msg to be VSCP event
      if (rxmsg.extd) {

        ESP_LOGI(TAG, "VSCP Event received");

        // VSCP link
        if (g_persistent.enableVscpLink) {
          // Send to all open tcp/ip link clients
          for (int i = 0; i < MAX_TCP_CONNECTIONS; i++) {
            // If not open take next
            if (!tr_tcpsrv[i].open) {
              continue;
            }
            ESP_LOGV(TAG, "Add incoming CAN message to VSCP link out queue %d", i);
            // Put message in queue for task to handle
            if (pdPASS != (rv = xQueueSendToBack(tr_tcpsrv[i].fromcan_queue, (void *) &rxmsg, (TickType_t) 10))) {
              tr_tcpsrv[i].overruns++;
              ESP_LOGD(TAG, "VSCP link protocol buffer full: Failed to save message to queue");
            }
          }
        }

        // MQTT
        if (g_persistent.enableMqtt) {
          ESP_LOGV(TAG, "Add incoming CAN message to MQTT out queue");
          if (pdPASS != (rv = xQueueSendToBack(tr_mqtt.fromcan_queue, (void *) &rxmsg, (TickType_t) 10))) {
            tr_mqtt.overruns++;
            ESP_LOGD(TAG, "MQTT buffer full: Failed to save message to queue");
          }
        }

        // Multicast
        if (g_persistent.enableMulticast) {
          ESP_LOGV(TAG, "Add incoming CAN message to Multicast out queue");
          if (pdPASS != (rv = xQueueSendToBack(tr_multicast.fromcan_queue, (void *) &rxmsg, (TickType_t) 10))) {
            tr_multicast.overruns++;
            ESP_LOGD(TAG, "Multicast buffer full: Failed to save message to queue");
          }
        }

        // UDP
        if (g_persistent.enableUdpTx) {
          ESP_LOGV(TAG, "Add incoming CAN message to UDP out queue");
          if (pdPASS != (rv = xQueueSendToBack(tr_udp.fromcan_queue, (void *) &rxmsg, (TickType_t) 10))) {
            tr_udp.overruns++;
            ESP_LOGD(TAG, "UDP buffer full: Failed to save message to queue");
          }
        }

        // Websockets
        if (g_persistent.enableWebsock) {
          ESP_LOGV(TAG, "Add incoming CAN message to Websocket out queue");
          if (pdPASS != (rv = xQueueSendToBack(tr_websockets.fromcan_queue, (void *) &rxmsg, (TickType_t) 10))) {
            tr_websockets.overruns++;
            ESP_LOGD(TAG, "Websocket buffer full: Failed to save message to queue");
          }
        }

        // HTTP CAN API reply transport — used by SCAN/WHIS and other command replies.
        if (NULL != tr_canapi.fromcan_queue) {
          xQueueSendToBack(tr_canapi.fromcan_queue, (void *) &rxmsg, 0);
        }

        // Passive monitor transport — always filled for the HTTP monitor endpoint.
        if (NULL != tr_monitor.fromcan_queue) {
          // Non-blocking: if full just discard; old frames are stale anyway
          xQueueSendToBack(tr_monitor.fromcan_queue, (void *) &rxmsg, 0);
        }

        // UBaseType_t cnt = uxQueueMessagesWaiting(ptr->msg_queue);
        // ESP_LOGI(TAG,"count=%u %d",cnt,rv);
        xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
        // Tell the controller that there is a received event
        xSemaphoreGive(ctrl_task_sem);
      }
    }
  }

} // while

///////////////////////////////////////////////////////////////////////////////
// twai_transmit_task
//
// Drains tr_canapi.tocan_queue and sends each frame to the CAN bus.
// Running as a dedicated task means the HTTP handler (cmd=send) returns
// immediately after queuing the frame, not blocking on CAN TX.
//

void
twai_transmit_task(void *arg)
{
  ESP_LOGI(TAG, "TWAI transmit task started");

  while (1) {
    can4vscp_frame_t msg = { 0 };
    if (NULL == tr_canapi.tocan_queue) {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    if (pdPASS == xQueueReceive(tr_canapi.tocan_queue, &msg, portMAX_DELAY)) {
      esp_err_t rv = can4vscp_send(&msg, pdMS_TO_TICKS(100));
      if (ESP_OK != rv) {
        ESP_LOGW(TAG, "TWAI transmit failed: %s (id=0x%lX)", esp_err_to_name(rv), msg.identifier);
      }
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
// twai_recover_stopped_check
//

void
twai_recover_stopped_check(void)
{
  while (true) {
    if (NULL == s_twai_node) {
      vTaskDelay(200);
      continue;
    }

    twai_node_status_t info = { 0 };
    if (ESP_OK != twai_node_get_info(s_twai_node, &info, NULL)) {
      vTaskDelay(200);
      continue;
    }

    if (info.state == TWAI_ERROR_BUS_OFF) {
      ESP_LOGW(TAG, "TWAI in bus-off state, attempting recovery...");
      twai_node_recover(s_twai_node);
    }
    vTaskDelay(200);
  }
}