

idf_component_register(SRCS "main.c"
                            "mqtt.c" 
                            "websrv.c"
                            "websocksrv.c"
                            "keep-alive.c"
                            "can4vscp.c"
                            "urldecode.c"
                            "tcpsrv.c" 
                            "multicastsrv.c"
                            "udpsrv.c"
                            "callbacks-link.c"
                            "callbacks-vscp-protocol.c"
                            "vscp-ws1.c"
                            "callbacks-ws1.c"
                            "vscp-binary.c"
                            "callbacks-binary.c"
                            "${VSCP_FIRMWARE_DIR}/common/vscp-link-protocol.c"
                            "${VSCP_FIRMWARE_DIR}/common/vscp-firmware-helper.c"
                            "${VSCP_FIRMWARE_DIR}/common/vscp-crc.c"
                            "${VSCP_FIRMWARE_DIR}/common/vscp-aes.c"

                    INCLUDE_DIRS "." 
                                  "${VSCP_FIRMWARE_DIR}/common/"
                    REQUIRES mbedtls
                             esp_driver_twai
                             spi_flash
                             esp_partition
                             app_update
                             nvs_flash
                             esp_http_server
                             esp_wifi
                             esp_event
                             esp_netif
                             esp_timer
                             esp-tls
                             vfs
                             spiffs
                             mqtt
)
