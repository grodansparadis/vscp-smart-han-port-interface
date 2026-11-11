/*
  File: tcpsrv_tls.c
  Minimal TLS-enabled TCP server for ESP32 (using esp-tls)
  Adapted for VSCP project, 2026
*/

#include <string.h>
#include <sys/param.h>
#include <esp_log.h>
#include <esp_tls.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <lwip/sockets.h>
#include "tcpsrv.h"

#define TAG "tcpsrv_tls"
#define SERVER_PORT 9598

// Example server certificate and key (PEM format, replace with your own)
extern const char server_cert_pem_start[];
extern const char server_cert_pem_end[];
extern const char server_key_pem_start[];
extern const char server_key_pem_end[];

static void tcpsrv_tls_task(void *pvParameters) {
    struct sockaddr_in listen_addr = {0};
    listen_addr.sin_family = AF_INET;
    listen_addr.sin_port = htons(SERVER_PORT);
    listen_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    if (bind(listen_sock, (struct sockaddr *)&listen_addr, sizeof(listen_addr)) != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }
    if (listen(listen_sock, 1) != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "TLS server listening on port %d", SERVER_PORT);

    while (1) {
        struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
        uint addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            continue;
        }
        ESP_LOGI(TAG, "Client connected, starting TLS handshake");

        esp_tls_cfg_server_t cfg = {
            .cacert_pem_buf = NULL,
            .cacert_pem_bytes = 0,
            .servercert_pem_buf = (const unsigned char *)server_cert_pem_start,
            .servercert_pem_bytes = server_cert_pem_end - server_cert_pem_start,
            .serverkey_pem_buf = (const unsigned char *)server_key_pem_start,
            .serverkey_pem_bytes = server_key_pem_end - server_key_pem_start,
        };
        esp_tls_t *tls = esp_tls_init();
        if (!tls) {
            ESP_LOGE(TAG, "Failed to allocate esp_tls");
            close(sock);
            continue;
        }
        if (esp_tls_server_session_create(&cfg, sock, tls) != ESP_OK) {
            ESP_LOGE(TAG, "TLS handshake failed");
            esp_tls_conn_destroy(tls);
            close(sock);
            continue;
        }
        ESP_LOGI(TAG, "TLS handshake successful");

        char rxbuf[128];
        int len = esp_tls_conn_read(tls, rxbuf, sizeof(rxbuf) - 1);
        if (len > 0) {
            rxbuf[len] = 0;
            ESP_LOGI(TAG, "Received %d bytes: %s", len, rxbuf);
            // Echo back
            esp_tls_conn_write(tls, rxbuf, len);
        }
        esp_tls_conn_destroy(tls);
        close(sock);
        ESP_LOGI(TAG, "Connection closed");
    }
    close(listen_sock);
    vTaskDelete(NULL);
}

void start_tcpsrv_tls(void) {
    xTaskCreate(tcpsrv_tls_task, "tcpsrv_tls", 8192, NULL, 5, NULL);
}
