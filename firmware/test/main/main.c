#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "Sagemcom_P1";

// Pin Configurations
#define P1_UART_NUM      UART_NUM_2       // Use Hardware UART2 to avoid flashing conflicts
#define P1_RX_PIN        GPIO_NUM_16      // Connect to Sagemcom RJ12 Pin 5 (Data)
#define P1_TX_PIN        GPIO_NUM_17      // Unused for reading, but driver requires assignment
#define P1_RTS_PIN       GPIO_NUM_18      // Connect to Sagemcom RJ12 Pin 2 (Request line)

// DSMR 5.0 Buffer Configurations
#define BUF_SIZE         2048             // Max telegram payload allocation
#define RD_BUF_SIZE      1024

/**
 * Parses out specific lines from the raw text buffer using basic C string manipulation.
 * Sagemcom uses standard OBIS codes.
 */
void parse_dsmr_line(const char* line) {
    // OBIS 1-0:1.8.1 = Electricity consumed (Tariff 1)
    if (strstr(line, "1-0:1.8.1")) {
        float val;
        if (sscanf(line, "1-0:1.8.1(%f*kWh)", &val) == 1) {
            ESP_LOGI(TAG, "Energy Consumed (T1): %.3f kWh", val);
        }
    }
    // OBIS 1-0:1.8.2 = Electricity consumed (Tariff 2)
    else if (strstr(line, "1-0:1.8.2")) {
        float val;
        if (sscanf(line, "1-0:1.8.2(%f*kWh)", &val) == 1) {
            ESP_LOGI(TAG, "Energy Consumed (T2): %.3f kWh", val);
        }
    }
    // OBIS 1-0:1.7.0 = Current Active Power Import (+)
    else if (strstr(line, "1-0:1.7.0")) {
        float val;
        if (sscanf(line, "1-0:1.7.0(%f*kW)", &val) == 1) {
            ESP_LOGI(TAG, "Current Power Draw: %.3f kW", val);
        }
    }
}

/**
 * FreeRTOS Background task handling incoming serial bytes.
 */
static void p1_reader_task(void *pvParameters) {
    uint8_t *data = (uint8_t *) malloc(RD_BUF_SIZE);
    char telegram_accumulator[BUF_SIZE] = {0};
    int accum_len = 0;

    // Sagemcom meters require RTS pulled HIGH to constantly request/unlock data
    gpio_set_level(P1_RTS_PIN, 1);
    ESP_LOGI(TAG, "RTS Pin pulled HIGH. Listening for Sagemcom data stream...");

    while (1) {
        // Read raw data stream from Sagemcom P1 port
        int len = uart_read_bytes(P1_UART_NUM, data, RD_BUF_SIZE, pdMS_TO_TICKS(100)); // 100ms timeout
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                char c = data[i];

                // DSMR Telegrams begin with an exclamation slash mark '/'
                if (c == '/') {
                    accum_len = 0;
                    memset(telegram_accumulator, 0, BUF_SIZE);
                }

                // Add to internal working string buffer safely
                if (accum_len < (BUF_SIZE - 1)) {
                    telegram_accumulator[accum_len++] = c;
                }

                // DSMR Telegrams wrap up with an exclamation mark '!' followed by a CRC hex string
                if (c == '!') {
                    telegram_accumulator[accum_len] = '\0';
                    
                    // Tokenize block into explicit lines to isolate OBIS parameters
                    char *line = strtok(telegram_accumulator, "\r\n");
                    while (line != NULL) {
                        parse_dsmr_line(line);
                        line = strtok(NULL, "\r\n");
                    }
                    
                    ESP_LOGD(TAG, "--- Entire Telegram Parsed Successfully ---");
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    free(data);
}

void app_main(void) {
    // Initialize RTS (Request Line) GPIO to toggle Sagemcom output on
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << P1_RTS_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);

    // Set Up DSMR 5.0 UART Hardware Configuration: 115200 Baud, 8N1
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Allocate resources and initialize standard hardware driver
    ESP_ERROR_CHECK(uart_driver_install(P1_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(P1_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(P1_UART_NUM, P1_TX_PIN, P1_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Spin up standard execution thread
    xTaskCreate(p1_reader_task, "p1_reader_task", 4096, NULL, 5, NULL);
}
