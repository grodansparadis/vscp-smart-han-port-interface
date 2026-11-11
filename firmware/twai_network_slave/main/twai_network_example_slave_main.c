/* TWAI Network Slave Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
 * The following example demonstrates a slave node in a TWAI network. The slave
 * node is responsible for sending data messages to the master. The example will
 * execute multiple iterations, with each iteration the slave node will do the
 * following:
 * 1) Start the TWAI driver
 * 2) Listen for ping messages from master, and send ping response
 * 3) Listen for start command from master
 * 4) Send data messages to master and listen for stop command
 * 5) Send stop response to master
 * 6) Stop the TWAI driver
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
#define DATA_PERIOD_MS                  50
#define NO_OF_ITERS                     3
#define ITER_DELAY_MS                   1000
#define RX_TASK_PRIO                    8       //Receiving task priority
#define TX_TASK_PRIO                    9       //Sending task priority
#define CTRL_TSK_PRIO                   10      //Control task priority
#define TX_GPIO_NUM                     CONFIG_EXAMPLE_TX_GPIO_NUM
#define RX_GPIO_NUM                     CONFIG_EXAMPLE_RX_GPIO_NUM
#define EXAMPLE_TAG                     "TWAI Slave"

#define ID_MASTER_STOP_CMD              0x0A0
#define ID_MASTER_START_CMD             0x0A1
#define ID_MASTER_PING                  0x0A2
#define ID_SLAVE_STOP_RESP              0x0B0
#define ID_SLAVE_DATA                   0x0B1
#define ID_SLAVE_PING_RESP              0x0B2

typedef enum {
    TX_SEND_PING_RESP,
    TX_SEND_DATA,
    TX_SEND_STOP_RESP,
    TX_TASK_EXIT,
} tx_task_action_t;

typedef enum {
    RX_RECEIVE_PING,
    RX_RECEIVE_START_CMD,
    RX_RECEIVE_STOP_CMD,
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
static SemaphoreHandle_t ctrl_task_sem;
static SemaphoreHandle_t stop_data_sem;
static SemaphoreHandle_t done_sem;

static bool IRAM_ATTR twai_slave_rx_done_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
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

static esp_err_t twai_slave_tx(uint32_t id, const uint8_t *data, size_t len)
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

static void twai_receive_task(void *arg)
{
    while (1) {
        rx_task_action_t action;
        xQueueReceive(rx_task_queue, &action, portMAX_DELAY);
        if (action == RX_RECEIVE_PING) {
            //Listen for pings from master
            can_msg_t rx_msg;
            while (1) {
                xQueueReceive(s_rx_frame_queue, &rx_msg, portMAX_DELAY);
                if (rx_msg.id == ID_MASTER_PING) {
                    xSemaphoreGive(ctrl_task_sem);
                    break;
                }
            }
        } else if (action == RX_RECEIVE_START_CMD) {
            //Listen for start command from master
            can_msg_t rx_msg;
            while (1) {
                xQueueReceive(s_rx_frame_queue, &rx_msg, portMAX_DELAY);
                if (rx_msg.id == ID_MASTER_START_CMD) {
                    xSemaphoreGive(ctrl_task_sem);
                    break;
                }
            }
        } else if (action == RX_RECEIVE_STOP_CMD) {
            //Listen for stop command from master
            can_msg_t rx_msg;
            while (1) {
                xQueueReceive(s_rx_frame_queue, &rx_msg, portMAX_DELAY);
                if (rx_msg.id == ID_MASTER_STOP_CMD) {
                    xSemaphoreGive(stop_data_sem);
                    xSemaphoreGive(ctrl_task_sem);
                    break;
                }
            }
        } else if (action == RX_TASK_EXIT) {
            break;
        }
    }
    vTaskDelete(NULL);
}

static void twai_transmit_task(void *arg)
{
    while (1) {
        tx_task_action_t action;
        xQueueReceive(tx_task_queue, &action, portMAX_DELAY);

        if (action == TX_SEND_PING_RESP) {
            //Transmit ping response to master
            twai_slave_tx(ID_SLAVE_PING_RESP, NULL, 0);
            ESP_LOGI(EXAMPLE_TAG, "Transmitted ping response");
            xSemaphoreGive(ctrl_task_sem);
        } else if (action == TX_SEND_DATA) {
            //Transmit data messages until stop command is received
            ESP_LOGI(EXAMPLE_TAG, "Start transmitting data");
            while (1) {
                //FreeRTOS tick count used to simulate sensor data
                uint32_t sensor_data = xTaskGetTickCount();
                uint8_t data[4] = {0};
                for (int i = 0; i < 4; i++) {
                    data[i] = (sensor_data >> (i * 8)) & 0xFF;
                }
                twai_slave_tx(ID_SLAVE_DATA, data, sizeof(data));
                ESP_LOGI(EXAMPLE_TAG, "Transmitted data value %"PRIu32, sensor_data);
                vTaskDelay(pdMS_TO_TICKS(DATA_PERIOD_MS));
                if (xSemaphoreTake(stop_data_sem, 0) == pdTRUE) {
                    break;
                }
            }
        } else if (action == TX_SEND_STOP_RESP) {
            //Transmit stop response to master
            twai_slave_tx(ID_SLAVE_STOP_RESP, NULL, 0);
            ESP_LOGI(EXAMPLE_TAG, "Transmitted stop response");
            xSemaphoreGive(ctrl_task_sem);
        } else if (action == TX_TASK_EXIT) {
            break;
        }
    }
    vTaskDelete(NULL);
}

static void twai_control_task(void *arg)
{
    xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);
    tx_task_action_t tx_action;
    rx_task_action_t rx_action;

    for (int iter = 0; iter < NO_OF_ITERS; iter++) {
        ESP_ERROR_CHECK(twai_node_enable(s_node));
        xQueueReset(s_rx_frame_queue);
        ESP_LOGI(EXAMPLE_TAG, "Driver started");

        //Listen of pings from master
        rx_action = RX_RECEIVE_PING;
        xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);
        xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);

        //Send ping response
        tx_action = TX_SEND_PING_RESP;
        xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
        xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);

        //Listen for start command
        rx_action = RX_RECEIVE_START_CMD;
        xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);
        xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);

        //Start sending data messages and listen for stop command
        tx_action = TX_SEND_DATA;
        rx_action = RX_RECEIVE_STOP_CMD;
        xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
        xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);
        xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);

        //Send stop response
        tx_action = TX_SEND_STOP_RESP;
        xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
        xSemaphoreTake(ctrl_task_sem, portMAX_DELAY);

        //Wait for bus to become free
        ESP_ERROR_CHECK(twai_node_transmit_wait_all_done(s_node, -1));

        ESP_ERROR_CHECK(twai_node_disable(s_node));
        ESP_LOGI(EXAMPLE_TAG, "Driver stopped");
        vTaskDelay(pdMS_TO_TICKS(ITER_DELAY_MS));
    }

    //Stop TX and RX tasks
    tx_action = TX_TASK_EXIT;
    rx_action = RX_TASK_EXIT;
    xQueueSend(tx_task_queue, &tx_action, portMAX_DELAY);
    xQueueSend(rx_task_queue, &rx_action, portMAX_DELAY);

    //Delete Control task
    xSemaphoreGive(done_sem);
    vTaskDelete(NULL);
}

void app_main(void)
{
    //Add short delay to allow master it to initialize first
    for (int i = 3; i > 0; i--) {
        printf("Slave starting in %d\n", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }


    //Create semaphores and tasks
    tx_task_queue = xQueueCreate(1, sizeof(tx_task_action_t));
    rx_task_queue = xQueueCreate(1, sizeof(rx_task_action_t));
    s_rx_frame_queue = xQueueCreate(32, sizeof(can_msg_t));
    ctrl_task_sem = xSemaphoreCreateBinary();
    stop_data_sem  = xSemaphoreCreateBinary();;
    done_sem  = xSemaphoreCreateBinary();;
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
        .bit_timing.bitrate = 25000,
        .bit_timing.sp_permill = 800,
        .tx_queue_depth = 16,
        .fail_retry_cnt = -1,
    };
    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &s_node));

    twai_range_filter_config_t filter_cfg = {
        .range_low = ID_MASTER_STOP_CMD,
        .range_high = ID_MASTER_PING,
        .is_ext = false,
        .no_classic = false,
        .no_fd = true,
    };
    ESP_ERROR_CHECK(twai_node_config_range_filter(s_node, 0, &filter_cfg));

    twai_event_callbacks_t cbs = {
        .on_rx_done = twai_slave_rx_done_cb,
    };
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(s_node, &cbs, NULL));
    ESP_LOGI(EXAMPLE_TAG, "Driver installed");

    xSemaphoreGive(ctrl_task_sem);              //Start Control task
    xSemaphoreTake(done_sem, portMAX_DELAY);    //Wait for tasks to complete

    //Uninstall TWAI driver
    ESP_ERROR_CHECK(twai_node_delete(s_node));
    ESP_LOGI(EXAMPLE_TAG, "Driver uninstalled");

    //Cleanup
    vSemaphoreDelete(ctrl_task_sem);
    vSemaphoreDelete(stop_data_sem);
    vSemaphoreDelete(done_sem);
    vQueueDelete(tx_task_queue);
    vQueueDelete(rx_task_queue);
    vQueueDelete(s_rx_frame_queue);
}
