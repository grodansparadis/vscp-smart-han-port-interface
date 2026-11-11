/* TWAI Network Master Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
 * The following example demonstrates a master node in a TWAI network. The master
 * node is responsible for initiating and stopping the transfer of data messages.
 * The example will execute multiple iterations, with each iteration the master
 * node will do the following:
 * 1) Start the TWAI driver
 * 2) Repeatedly send ping messages until a ping response from slave is received
 * 3) Send start command to slave and receive data messages from slave
 * 4) Send stop command to slave and wait for stop response from slave
 * 5) Stop the TWAI driver
 */
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"

/* --------------------- Definitions and static variables ------------------ */
//Example Configuration
#define PING_PERIOD_MS          250
#define NO_OF_DATA_MSGS         50
#define NO_OF_ITERS             3
#define ITER_DELAY_MS           1000
#define RX_TASK_PRIO            8
#define TX_TASK_PRIO            9
#define CTRL_TSK_PRIO           10
#define TX_GPIO_NUM             GPIO_NUM_9  // CONFIG_EXAMPLE_TX_GPIO_NUM
#define RX_GPIO_NUM             GPIO_NUM_10  // GPIO_NUM_3 CONFIG_EXAMPLE_RX_GPIO_NUM
#define EXAMPLE_TAG             "TWAI Master"

#define ID_MASTER_STOP_CMD      0x0A0
#define ID_MASTER_START_CMD     0x0A1
#define ID_MASTER_PING          0x0A2
#define ID_SLAVE_STOP_RESP      0x0B0
#define ID_SLAVE_DATA           0x0B1
#define ID_SLAVE_PING_RESP      0x0B2

typedef enum {
    TX_SEND_PINGS,
    TX_SEND_START_CMD,
    TX_SEND_STOP_CMD,
    TX_TASK_EXIT,
} tx_task_action_t;

typedef enum {
    RX_RECEIVE_PING_RESP,
    RX_RECEIVE_DATA,
    RX_RECEIVE_STOP_RESP,
    RX_TASK_EXIT,
} rx_task_action_t;

typedef struct {
  uint32_t id;
  uint8_t len;
  uint8_t data[8];
} can_msg_t;

static twai_node_handle_t s_node;
static QueueHandle_t s_rx_frame_queue;

static QueueHandle_t tx_task_queue;
static QueueHandle_t rx_task_queue;
static SemaphoreHandle_t stop_ping_sem;
static SemaphoreHandle_t ctrl_task_sem;
static SemaphoreHandle_t done_sem;

static bool IRAM_ATTR twai_master_rx_done_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
  (void) edata;
  (void) user_ctx;

  uint8_t data[8] = {0};
  twai_frame_t rx_frame = {
    .buffer = data,
    .buffer_len = sizeof(data),
  };

  if (ESP_OK != twai_node_receive_from_isr(handle, &rx_frame)) {
    return false;
  }

  can_msg_t msg = {
    .id = rx_frame.header.id,
    .len = (uint8_t) MIN((size_t) twaifd_dlc2len(rx_frame.header.dlc), sizeof(data)),
  };
  if (msg.len) {
    memcpy(msg.data, data, msg.len);
  }

  BaseType_t task_woken = pdFALSE;
  xQueueSendToBackFromISR(s_rx_frame_queue, &msg, &task_woken);
  return (task_woken == pdTRUE);
}

static esp_err_t twai_master_tx(uint32_t id, const uint8_t *data, size_t len)
{
  uint8_t tx_data[8] = {0};
  len = MIN(len, sizeof(tx_data));
  if (len && data) {
    memcpy(tx_data, data, len);
  }

  twai_frame_t tx = {
    .header = {
      .id = id,
      .ide = false,
      .rtr = false,
      .dlc = len,
    },
    .buffer = tx_data,
    .buffer_len = len,
  };

  return twai_node_transmit(s_node, &tx, -1);
}

/* --------------------------- Tasks and Functions -------------------------- */

///////////////////////////////////////////////////////////////////////////////
// twai_receive_task
//

static void twai_receive_task(void *arg)
{
    while (1) {
        rx_task_action_t action;
        xQueueReceive(rx_task_queue, &action, portMAX_DELAY);

        if (action == RX_RECEIVE_PING_RESP) {
            //Listen for ping response from slave
            while (1) {
            can_msg_t rx_msg;
            xQueueReceive(s_rx_frame_queue, &rx_msg, portMAX_DELAY);
            if (rx_msg.id == ID_SLAVE_PING_RESP) {
                    xSemaphoreGive(stop_ping_sem);
                    xSemaphoreGive(ctrl_task_sem);
                    break;
                }
            }
        } 
        else if (action == RX_RECEIVE_DATA) {
            //Receive data messages from slave
            uint32_t data_msgs_rec = 0;
            while (data_msgs_rec < NO_OF_DATA_MSGS) {
              can_msg_t rx_msg;
              xQueueReceive(s_rx_frame_queue, &rx_msg, portMAX_DELAY);
              if (rx_msg.id == ID_SLAVE_DATA) {
                    uint32_t data = 0;
                for (int i = 0; i < rx_msg.len; i++) {
                        data |= (rx_msg.data[i] << (i * 8));
                    }
                    ESP_LOGI(EXAMPLE_TAG, "Received data value %"PRIu32, data);
                    data_msgs_rec ++;
                }
            }
            xSemaphoreGive(ctrl_task_sem);
        } 
        else if (action == RX_RECEIVE_STOP_RESP) {
            // Listen for stop response from slave
            while (1) {
            can_msg_t rx_msg;
            xQueueReceive(s_rx_frame_queue, &rx_msg, portMAX_DELAY);
            if (rx_msg.id == ID_SLAVE_STOP_RESP) {
                    xSemaphoreGive(ctrl_task_sem);
                    break;
                }
            }
        } 
        else if (action == RX_TASK_EXIT) {
            break;
        }
    }
    vTaskDelete(NULL);
}

///////////////////////////////////////////////////////////////////////////////
// twai_transmit_task
//

static void twai_transmit_task(void *arg)
{
  while (1) {

    tx_task_action_t action;
    xQueueReceive(tx_task_queue, &action, portMAX_DELAY);

    if (action == TX_SEND_PINGS) {
      //Repeatedly transmit pings
      ESP_LOGI(EXAMPLE_TAG, "Transmitting ping");
      while (xSemaphoreTake(stop_ping_sem, 0) != pdTRUE) {
          twai_master_tx(ID_MASTER_PING, NULL, 0);
          vTaskDelay(pdMS_TO_TICKS(PING_PERIOD_MS));
      }
    } 
    else if (action == TX_SEND_START_CMD) {
      // Transmit start command to slave
      twai_master_tx(ID_MASTER_START_CMD, NULL, 0);
      ESP_LOGI(EXAMPLE_TAG, "Transmitted start command");
    } 
    else if (action == TX_SEND_STOP_CMD) {
      // Transmit stop command to slave
      twai_master_tx(ID_MASTER_STOP_CMD, NULL, 0);
      ESP_LOGI(EXAMPLE_TAG, "Transmitted stop command");
    } 
    else if (action == TX_TASK_EXIT) {
      break;
    }
  }
  vTaskDelete(NULL);
}

///////////////////////////////////////////////////////////////////////////////
// twai_control_task
//

static void twai_control_task(void *arg)
{
  xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
  tx_task_action_t tx_action;
  rx_task_action_t rx_action;

  for (int iter = 0; iter < NO_OF_ITERS; iter++) {

    ESP_ERROR_CHECK(twai_node_enable(s_node));
    xQueueReset(s_rx_frame_queue);
    ESP_LOGI(EXAMPLE_TAG, "Driver started");

    // Start transmitting pings, and listen for ping response
    tx_action = TX_SEND_PINGS;
    rx_action = RX_RECEIVE_PING_RESP;
    xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
    xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);

    // Send Start command to slave, and receive data messages
    xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
    tx_action = TX_SEND_START_CMD;
    rx_action = RX_RECEIVE_DATA;
    xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
    xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);

    // Send Stop command to slave when enough data messages have been received. Wait for stop response
    xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
    tx_action = TX_SEND_STOP_CMD;
    rx_action = RX_RECEIVE_STOP_RESP;
    xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
    xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);

    xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
    ESP_ERROR_CHECK(twai_node_disable(s_node));
    ESP_LOGI(EXAMPLE_TAG, "Driver stopped");
    vTaskDelay(pdMS_TO_TICKS(ITER_DELAY_MS));
  }

  // Stop TX and RX tasks
  tx_action = TX_TASK_EXIT;
  rx_action = RX_TASK_EXIT;
  xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
  xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);

  // Delete Control task
  xSemaphoreGive(done_sem);
  vTaskDelete(NULL);
}

///////////////////////////////////////////////////////////////////////////////
// app_main
//

//static const gpio_config_t cfggpio10 = {GPIO_NUM_9, };

void app_main(void)
{
  // Create tasks, queues, and semaphores
  rx_task_queue = xQueueCreate(1, sizeof(rx_task_action_t));
  tx_task_queue = xQueueCreate(1, sizeof(tx_task_action_t));
  s_rx_frame_queue = xQueueCreate(32, sizeof(can_msg_t));
  
  ctrl_task_sem = xSemaphoreCreateBinary();
  stop_ping_sem = xSemaphoreCreateBinary();
  done_sem = xSemaphoreCreateBinary();

  //gpio_reset()
  //REG_SET_BIT(GPIO_ENABLE_REG, BIT10);
  //gpio_config(&cfggpio10)
  //gpio_reset_pin(GPIO_NUM_9);
  //gpio_reset_pin(GPIO_NUM_10);

  xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, RX_TASK_PRIO, NULL, tskNO_AFFINITY);
  xTaskCreatePinnedToCore(twai_transmit_task, "TWAI_tx", 4096, NULL, TX_TASK_PRIO, NULL, tskNO_AFFINITY);
  xTaskCreatePinnedToCore(twai_control_task, "TWAI_ctrl", 4096, NULL, CTRL_TSK_PRIO, NULL, tskNO_AFFINITY);

    twai_onchip_node_config_t node_config = {
      .io_cfg = {
        .tx = TX_GPIO_NUM,
        .rx = RX_GPIO_NUM,
        .quanta_clk_out = GPIO_NUM_NC,
        .bus_off_indicator = GPIO_NUM_NC,
      },
      .bit_timing.bitrate = 125000,
      .bit_timing.sp_permill = 800,
      .tx_queue_depth = 16,
      .fail_retry_cnt = -1,
    };
    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &s_node));

    twai_range_filter_config_t filter_cfg = {
      .range_low = ID_SLAVE_STOP_RESP,
      .range_high = ID_SLAVE_PING_RESP,
      .is_ext = false,
      .no_classic = false,
      .no_fd = true,
    };
    ESP_ERROR_CHECK(twai_node_config_range_filter(s_node, 0, &filter_cfg));

    twai_event_callbacks_t cbs = {
      .on_rx_done = twai_master_rx_done_cb,
    };
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(s_node, &cbs, NULL));
  ESP_LOGI(EXAMPLE_TAG, "Driver installed");

  xSemaphoreGive(ctrl_task_sem);              // Start control task
  xSemaphoreTake(done_sem, portMAX_DELAY);    // Wait for completion

  // Uninstall TWAI driver
  ESP_ERROR_CHECK(twai_node_delete(s_node));
  ESP_LOGI(EXAMPLE_TAG, "Driver uninstalled");

  // Cleanup
  vQueueDelete(rx_task_queue);
  vQueueDelete(tx_task_queue);
  vQueueDelete(s_rx_frame_queue);
  
  vSemaphoreDelete(ctrl_task_sem);
  vSemaphoreDelete(stop_ping_sem);
  vSemaphoreDelete(done_sem);
}
