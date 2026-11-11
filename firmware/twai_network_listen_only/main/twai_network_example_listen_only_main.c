/* TWAI Network Listen Only Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
 * The following example demonstrates a Listen Only node in a TWAI network. The
 * Listen Only node will not take part in any TWAI bus activity (no acknowledgments
 * and no error frames). This example will execute multiple iterations, with each
 * iteration the Listen Only node will do the following:
 * 1) Listen for ping and ping response
 * 2) Listen for start command
 * 3) Listen for data messages
 * 4) Listen for stop and stop response
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
#define NO_OF_ITERS                     3
#define RX_TASK_PRIO                    9
#define TX_GPIO_NUM                     CONFIG_EXAMPLE_TX_GPIO_NUM
#define RX_GPIO_NUM                     CONFIG_EXAMPLE_RX_GPIO_NUM
#define EXAMPLE_TAG                     "TWAI Listen Only"

#define ID_MASTER_STOP_CMD              0x0A0
#define ID_MASTER_START_CMD             0x0A1
#define ID_MASTER_PING                  0x0A2
#define ID_SLAVE_STOP_RESP              0x0B0
#define ID_SLAVE_DATA                   0x0B1
#define ID_SLAVE_PING_RESP              0x0B2

typedef struct {
    uint32_t id;
    uint8_t len;
    uint8_t data[8];
} can_msg_t;

static twai_node_handle_t s_node;
static QueueHandle_t s_rx_frame_queue;

static SemaphoreHandle_t rx_sem;

static bool IRAM_ATTR twai_listen_rx_done_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
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

/* --------------------------- Tasks and Functions -------------------------- */

static void twai_receive_task(void *arg)
{
    xSemaphoreTake(rx_sem, portMAX_DELAY);
    bool start_cmd = false;
    bool stop_cmd = false;
    uint32_t iterations = 0;

    while (iterations < NO_OF_ITERS) {
        can_msg_t rx_msg;
        xQueueReceive(s_rx_frame_queue, &rx_msg, portMAX_DELAY);
        if (rx_msg.id == ID_MASTER_PING) {
            ESP_LOGI(EXAMPLE_TAG, "Received master ping");
        } else if (rx_msg.id == ID_MASTER_START_CMD) {
            ESP_LOGI(EXAMPLE_TAG, "Received master start command");
            start_cmd = true;
        } else if (rx_msg.id == ID_MASTER_STOP_CMD) {
            ESP_LOGI(EXAMPLE_TAG, "Received master stop command");
            stop_cmd = true;
        }
        else {
            ESP_LOGI(EXAMPLE_TAG, "---------------------> CAN");
        }
        if (start_cmd && stop_cmd) {
            // Each iteration is complete after start and stop command are observed
            iterations++;
            start_cmd = 0;
            stop_cmd = 0;
        }
    }

    xSemaphoreGive(rx_sem);
    vTaskDelete(NULL);
}

void app_main(void)
{
    rx_sem = xSemaphoreCreateBinary();
    s_rx_frame_queue = xQueueCreate(32, sizeof(can_msg_t));
    xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, RX_TASK_PRIO, NULL, tskNO_AFFINITY);

    //Install and start TWAI driver
    twai_onchip_node_config_t node_config = {
        .io_cfg = {
            .tx = TX_GPIO_NUM,
            .rx = RX_GPIO_NUM,
            .quanta_clk_out = GPIO_NUM_NC,
            .bus_off_indicator = GPIO_NUM_NC,
        },
        .bit_timing.bitrate = 25000,
        .bit_timing.sp_permill = 800,
        .tx_queue_depth = 1,
    };
    node_config.flags.enable_listen_only = 1;
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
        .on_rx_done = twai_listen_rx_done_cb,
    };
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(s_node, &cbs, NULL));
    ESP_LOGI(EXAMPLE_TAG, "Driver installed");
    ESP_ERROR_CHECK(twai_node_enable(s_node));
    ESP_LOGI(EXAMPLE_TAG, "Driver started");

    xSemaphoreGive(rx_sem);                     //Start RX task
    vTaskDelay(pdMS_TO_TICKS(100));
    xSemaphoreTake(rx_sem, portMAX_DELAY);      //Wait for RX task to complete

    //Stop and uninstall TWAI driver
    ESP_ERROR_CHECK(twai_node_disable(s_node));
    ESP_LOGI(EXAMPLE_TAG, "Driver stopped");
    ESP_ERROR_CHECK(twai_node_delete(s_node));
    ESP_LOGI(EXAMPLE_TAG, "Driver uninstalled");

    //Cleanup
    vSemaphoreDelete(rx_sem);
    vQueueDelete(s_rx_frame_queue);
}
