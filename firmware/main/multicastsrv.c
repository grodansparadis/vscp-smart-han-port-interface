/*
  VSCP Multicast support

  This file is part of the VSCP (https://www.vscp.org)

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

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/sockets.h"
#include "lwip/err.h"
#include "lwip/sys.h"

// Enable functionality from  firmware helper
#define VSCP_FWHLP_CRYPTO_SUPPORT    // For encryption support
#define VSCP_FWHLP_UDP_FRAME_SUPPORT // For UDP frame support

#include "main.h"

#include <vscp-crc.h>

#include <vscp.h>
#include <vscp-class.h>
#include <vscp-type.h>
#include <vscp-firmware-helper.h>

#include "multicastsrv.h"

static const char *TAG = "multicastsrv";

// External from main
// extern nvs_handle_t g_nvsHandle;
extern node_persistent_config_t g_persistent;
extern vprintf_like_t g_stdLogFunc;
extern transport_t tr_multicast;

#define VSCP_MULTICAST_BUFFER_SIZE 1024

///////////////////////////////////////////////////////////////////////////////
// multicast_sendEvent
//
// socket - Socket to send event on
// pstrev - VSCP event on string form to send
// bEncrypt - true if the event should be encrypted with the set key
// nAlgorithm - Encryption algorithm to use (vscp.h)
//

// int32_t
// multicast_sendEvent(int sock, const char *pstrev, bool bEncrypt, uint8_t nAlgorithm)
// {
//   int32_t rv;
//   uint16_t len                            = 0;
//   uint8_t buf[VSCP_MULTICAST_BUFFER_SIZE] = { 0 };
//   struct sockaddr_in multicast_addr;

//   vscp_event_ex_t ex;
//   if (VSCP_ERROR_SUCCESS != (rv = vscp_fwhlp_parseStringToEventEx(&ex, pstrev))) {
//     ESP_LOGE(TAG, "Error parsing event string");
//     close(sock);
//     return VSCP_ERROR_SUCCESS;
//   }

// #ifdef _MULTICAST_DEBUG_
//   printf("Parsed event:\n");
//   printf("=============\n");
//   printf("Class: %d\n", ex.vscp_class);
//   printf("Type: %d\n", ex.vscp_type);
//   printf("Priority: %d\n", ex.head & 0xE0);
//   printf("GUID: ");
//   for (int i = 0; i < 16; i++) {
//     printf("%02x:", ex.GUID[i]);
//   }
//   printf("\n");
//   printf("Data: ");
//   for (int i = 0; i < ex.sizeData; i++) {
//     printf("%02x:", ex.data[i]);
//   }
//   printf("\n");
//   printf("----------------------------------------------------\n");
//   printf("Timestamp: 0x%08lX\n", (long unsigned int) ex.timestamp);
//   printf("Obid: 0x%08lX\n", (long unsigned int) ex.obid);
//   printf("Head: %d\n", ex.head);
//   printf("Year: %d\n", ex.year);
//   printf("Month: %d\n", ex.month);
//   printf("Day: %d\n", ex.day);
//   printf("Hour: %d\n", ex.hour);
//   printf("Minute: %d\n", ex.minute);
//   printf("Second: %d\n", ex.second);
//   printf("----------------------------------------------------\n");
// #endif

//   // Calculate needed buffer size
//   len = vscp_fwhlp_getFrameSizeFromEventEx(&ex);
//   if (VSCP_ERROR_SUCCESS != (rv = vscp_fwhlp_writeEventExToFrame(buf, sizeof(buf), 0, &ex))) {
//     fprintf(stderr, "Error writing event to frame. rv=%u\n", (unsigned) rv);
//     return VSCP_ERROR_SUCCESS;
//   }

// #ifdef _MULTICAST_DEBUG_
//   printf("Frame size: %d\n", len);
//   printf("Frame:\n");
//   for (int i = 0; i < len; i++) {
//     printf("%02x ", buf[i]);
//   }
//   printf("\n");
// #endif

//   // Encrypt frame as needed
//   if (bEncrypt) {

//     uint8_t newlen       = 0;
//     uint8_t encbuf[1024] = { 0 };

//     if (0 == (newlen = vscp_fwhlp_encryptFrame(encbuf, buf, len, g_persistent.pmk, NULL, nAlgorithm))) {
//       fprintf(stderr, "Error encrypting frame. newlen = %d\n", newlen);
//       return VSCP_ERROR_SUCCESS;
//     }

//     memcpy(buf, encbuf, newlen);
//     buf[0] = (buf[0] & 0xF0) | (VSCP_HLO_ENCRYPTION_AES128 & 0x0F); // Set encryption type
//     // Set the new length (may be padded to be modulo 16 + 1)
//     len = newlen;

//     if (1) {
//       printf("Encrypted frame:\n");
//       for (int i = 0; i < len; i++) {
//         printf("%02x ", buf[i]);
//       }
//       printf("\nNew length: %d\n", len);
//     }
//   } // encrypted frame

//   ssize_t nsent;
//   if ((nsent = sendto(sock, buf, len, 0, (struct sockaddr *) &multicast_addr, sizeof(multicast_addr))) != len) {
//     fprintf(stderr, "send event: Failed %d\n", errno);
//     return VSCP_ERROR_ERROR;
//   }
//   // if ((nsent = sendto(sock, buf, len, s_multicast_ip, s_multicast_port)) < len) {
//   //   fprintf(stderr, "send event: Failed %d\n", errno);
//   //   return VSCP_ERROR_ERROR;
//   // }

//   return VSCP_ERROR_SUCCESS;
// }

///////////////////////////////////////////////////////////////////////////////
// multicast_send_heartbeat
//
// socket - Socket to send event on
//
// This function is not used directly but is called from the multicast_tx_task which sends out
// heartbeat events on a regular basis. The heartbeat is always sent as an unencrypted frame.
// is is always sent out on port 9598
//

static int
multicast_send_heartbeat(int sock)
{
  int32_t rv;
  uint8_t *frame;
  uint16_t len = 0;
  struct sockaddr_in multicast_addr;

  // if ((sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
  //   ESP_LOGE(TAG, "socket() failed");
  //   exit(1);
  // }

  // if ((setsockopt(sock,
  //                 IPPROTO_IP,
  //                 IP_MULTICAST_TTL,
  //                 (void *) &g_persistent.multicastTtl,
  //                 sizeof(g_persistent.multicastTtl))) < 0) {
  //   ESP_LOGE(TAG, "setsockopt() failed");
  //   exit(1);
  // }

  memset(&multicast_addr, 0, sizeof(multicast_addr));
  multicast_addr.sin_family      = AF_INET;
  multicast_addr.sin_addr.s_addr = inet_addr(g_persistent.multicastUrl);
  multicast_addr.sin_port        = htons(9598);

  vscp_event_t *pev = calloc(1, sizeof(vscpEvent));
  if (NULL == pev) {
    ESP_LOGE(TAG, "Failed to allocate memory for VSCP event");
    return VSCP_ERROR_MEMORY;
  }

  // Allocate data for heartbeat event
  pev->pdata = (uint8_t *) calloc(1, 3);
  if (NULL == pev->pdata) {
    ESP_LOGE(TAG, "Failed to allocate memory for VSCP event data");
    free(pev);
    return VSCP_ERROR_MEMORY;
  }

  pev->vscp_class = VSCP_CLASS1_INFORMATION;
  pev->vscp_type  = VSCP_TYPE_INFORMATION_NODE_HEARTBEAT;
  pev->sizeData   = 3;
  pev->pdata[0]   = 0;
  pev->pdata[1]   = g_persistent.nodeZone;    // Zone
  pev->pdata[2]   = g_persistent.nodeSubzone; // Subzone

  len   = vscp_fwhlp_getFrameSizeFromEvent(pev);
  frame = calloc(1, len);
  if (NULL == frame) {
    ESP_LOGE(TAG, "Failed to allocate memory for VSCP frame");
    vscp_fwhlp_deleteEvent(&pev);
    free(frame);
    return VSCP_ERROR_MEMORY;
  }

  if (VSCP_ERROR_SUCCESS != (rv = vscp_fwhlp_writeEventToFrame(frame, len, 0, pev))) {
    ESP_LOGE(TAG, "Failed to serialize VSCP event to frame rv=%d", rv);
    vscp_fwhlp_deleteEvent(&pev);
    free(frame);
    return VSCP_ERROR_ERROR;
  }

  ssize_t sent = sendto(sock, frame, len, 0, (struct sockaddr *) &multicast_addr, sizeof(multicast_addr));
  if (sent != (ssize_t) len) {
    ESP_LOGE(TAG, "sendto() failed: errno=%d", errno);
    vscp_fwhlp_deleteEvent(&pev);
    free(frame);
    return VSCP_ERROR_COMMUNICATION;
  }

  vscp_fwhlp_deleteEvent(&pev);
  free(frame);

  ESP_LOGI(TAG, "Sent Multicast heartbeat frame (%d bytes)", len);

  return VSCP_ERROR_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////
// multicast_heartbeat_task
//

void
multicast_heartbeat_task(void *pvParameters)
{
  (void) pvParameters;
  vscp_event_ex_t ex;
  memset(&ex, 0, sizeof(ex));

  // Define heartbeat event
  ex.head       = 0;  // Priority and other flags can be set here
  ex.vscp_class = 20; // VSCP_CLASS1_INFORMATION
  ex.vscp_type  = 3;  // VSCP_TYPE_INFORMATION_HEARTBEAT
  ex.sizeData   = 16; // Example data size
  for (int i = 0; i < ex.sizeData; i++) {
    ex.data[i] = i; // Example data content
  }

  ESP_LOGI(TAG, "Multicast heartbeat task started");

  while (1) {
    // if (g_persistent.bHeartbeat) {
    //   ESP_LOGI(TAG, "Sending heartbeat event");
    //   if (VSCP_ERROR_SUCCESS != multicast_sendEvent(-1,
    //                                                 "0,20,3,,,,0:1:2:3:4:5:6:7:8:9:10:11:12:13:14:15,0,1,35",
    //                                                 g_persistent.bMcastEncrypt,
    //                                                 VSCP_HLO_ENCRYPTION_AES128)) {
    //     ESP_LOGE(TAG, "Failed to send heartbeat event");
    //   }
    // }
    vTaskDelay(pdMS_TO_TICKS(10000)); // Send heartbeat every 10 seconds
  }

  ESP_LOGI(TAG, "Multicast heartbeat task ended, restarting...");

  vTaskDelete(NULL);
}

////////////////////////////////////////////////////////////////////////////
// multicast_tx_task
//

void
multicast_tx_task(void *pvParameters)
{
  (void) pvParameters;

  int rv;
  TickType_t lastHeartbeatSent = 0;
  can4vscp_frame_t rxmsg       = {};
  // struct ip_mreq mc_req;
  struct sockaddr_in multicast_addr;
  int sock;

  ESP_LOGI(TAG, "Multicast transmit task started");

  if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    ESP_LOGE(TAG, "socket() failed");
    goto terminate;
  }

  // Allow broadcast (for UDP)
  int broadcastPermission = 1;
  if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcastPermission, sizeof(broadcastPermission)) < 0) {
    ESP_LOGE(TAG, "setsockopt failed");
    close(sock);
    goto terminate;
  }

  // Set up multcast group
  memset(&multicast_addr, 0, sizeof(multicast_addr));
  multicast_addr.sin_family      = AF_INET;
  multicast_addr.sin_addr.s_addr = inet_addr(g_persistent.multicastUrl);
  multicast_addr.sin_port        = htons(g_persistent.multicastPort);

  int ttl = g_persistent.multicastTtl;
  if ((setsockopt(sock, IPPROTO_IP, IP_MULTICAST_TTL, (void *) &ttl, sizeof(ttl))) < 0) {
    ESP_LOGE(TAG, "setsockopt() TTL failed");
    close(sock);
    goto terminate;
  }

  while (1) {
    // if (g_persistent.bHeartbeat) {
    //   ESP_LOGI(TAG, "Sending heartbeat event");
    //   if (VSCP_ERROR_SUCCESS != multicast_sendEvent(-1,
    //                                                 "0,10,6,0,0,0,0,0,0,0,0,0,0,0,0,0:1",
    //                                                 g_persistent.bMcastEncrypt,
    //                                                 VSCP_HLO_ENCRYPTION_AES128)) {
    //     ESP_LOGE(TAG, "Failed to send heartbeat event");
    //   }
    // }

    if (g_persistent.bHeartbeat && (xTaskGetTickCount() - lastHeartbeatSent) > pdMS_TO_TICKS(60000)) {
      if (VSCP_ERROR_SUCCESS != multicast_send_heartbeat(sock)) {
        ESP_LOGE(TAG, "Failed to send heartbeat event");
      }
      lastHeartbeatSent = xTaskGetTickCount();
    }

    if (pdPASS != xQueueReceive(tr_multicast.fromcan_queue, (void *) &rxmsg, 500)) {
      continue;
    }

    vscp_event_t *pev = NULL;
    if (VSCP_ERROR_SUCCESS != (rv = can4vscp_msg_to_event(&pev, &rxmsg))) {
      ESP_LOGE(TAG, "Failed to convert CAN message to VSCP event rv=%d", rv);
      vscp_fwhlp_deleteEvent(&pev);
      continue;
    }

    ESP_LOGI(TAG,
             "Received message from CAN queue: class=%u type=%u size=%u",
             pev->vscp_class,
             pev->vscp_type,
             pev->sizeData);

    uint8_t frame[VSCP_MULTICAST_BUFFER_SIZE] = { 0 };
    uint16_t len                              = vscp_fwhlp_getFrameSizeFromEvent(pev);
    if (len > sizeof(frame)) {
      ESP_LOGE(TAG, "VSCP frame too large: len=%u", len);
      vscp_fwhlp_deleteEvent(&pev);
      continue;
    }

    if (VSCP_ERROR_SUCCESS != (rv = vscp_fwhlp_writeEventToFrame(frame, sizeof(frame), 0, pev))) {
      ESP_LOGE(TAG, "Failed to serialize VSCP event to frame rv=%d", rv);
      vscp_fwhlp_deleteEvent(&pev);
      continue;
    }

    vscp_fwhlp_deleteEvent(&pev);

    uint8_t newlen                             = 0;
    uint8_t encbuf[VSCP_MULTICAST_BUFFER_SIZE] = { 0 };

    if (0 == (newlen = vscp_fwhlp_encryptFrame(encbuf, frame, len, g_persistent.pmk, NULL, frame[0] & 0x0F))) {
      ESP_LOGI(TAG, "Error encrypting frame. newlen = %d", newlen);
      continue;
    }

    ESP_LOGI(TAG,
             "Prepared Multicast frame (%d bytes) from CAN message. %02X %02X",
             len,
             frame[len - 2],
             frame[len - 1]);

    ssize_t sent = sendto(sock, encbuf, newlen, 0, (struct sockaddr *) &multicast_addr, sizeof(multicast_addr));
    if (sent != (ssize_t) newlen) {
      ESP_LOGE(TAG, "sendto() failed: errno=%d", errno);
      continue;
    }

    ESP_LOGI(TAG, "Sent Multicast frame (%d bytes)", len);

    // vTaskDelay(pdMS_TO_TICKS(10000)); // Send heartbeat every 10 seconds
  }

  // close(sock);

terminate:

  ESP_LOGI(TAG, "Multicast transmit task ended, restarting...");

  vTaskDelete(NULL);
}

////////////////////////////////////////////////////////////////////////////
// multicast_rx_task
//

void
multicast_rx_task(void *pvParameters)
{
  int rv;
  int sock;
  int flag_on = 1;
  struct sockaddr_in multicast_addr;
  struct sockaddr_in bind_addr;
  uint8_t buf[VSCP_MULTICAST_BUFFER_SIZE];
  int len;
  struct ip_mreq mc_req;
  struct sockaddr_in from_addr;
  socklen_t from_len;

  if ((sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
    ESP_LOGE(TAG, "socket() failed");
    goto terminate;
  }

  // Allow broadcast (for UDP)
  int broadcastPermission = 1;
  if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcastPermission, sizeof(broadcastPermission)) < 0) {
    ESP_LOGE(TAG, "setsockopt failed");
    close(sock);
    goto terminate;
  }

  if ((setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &flag_on, sizeof(flag_on))) < 0) {
    ESP_LOGE(TAG, "setsockopt() failed");
    goto terminate;
  }

  // Set up the multicast group
  memset(&multicast_addr, 0, sizeof(multicast_addr));
  multicast_addr.sin_family      = AF_INET;
  multicast_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  multicast_addr.sin_port        = htons(g_persistent.multicastPort);

  memset(&bind_addr, 0, sizeof(bind_addr));
  bind_addr.sin_family      = AF_INET;
  bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  bind_addr.sin_port        = htons(g_persistent.multicastPort);

  ESP_LOGD(TAG, "Binding to %s:%d\n", inet_ntoa(bind_addr.sin_addr), ntohs(bind_addr.sin_port));

  if ((rv = bind(sock, (struct sockaddr *) &bind_addr, sizeof(bind_addr))) < 0) {
    ESP_LOGE(TAG, "bind() failed error=%d", errno);
    goto terminate;
  }

  mc_req.imr_multiaddr.s_addr = inet_addr(g_persistent.multicastUrl);
  mc_req.imr_interface.s_addr = htonl(INADDR_ANY);

  if ((setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, (void *) &mc_req, sizeof(mc_req))) < 0) {
    ESP_LOGE(TAG, "setsockopt() failed error=%d", errno);
    goto terminate;
  }

  ESP_LOGI(TAG, "Starting multicast receive task");

  while (1) {

    memset(buf, 0, sizeof(buf));
    from_len = sizeof(from_addr);
    memset(&from_addr, 0, from_len);

    ESP_LOGI(TAG, "Wait for message");

    if ((len = recvfrom(sock, buf, VSCP_MULTICAST_BUFFER_SIZE, 0, (struct sockaddr *) &from_addr, &from_len)) < 0) {
      ESP_LOGE(TAG, "recvfrom() failed");
      break;
    }

    ESP_LOGI(TAG,
             "Message received %d bytes from %s:%d",
             len,
             inet_ntoa(from_addr.sin_addr),
             ntohs(from_addr.sin_port));
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, buf, len, ESP_LOG_VERBOSE);

    // If encrypted frame decrypt it
    if (buf[0] & 0x0F) {

      if (1) {
        ESP_LOGI(TAG, "Encrypted frame detected. Type: %d", buf[0] & 0x0F);
      }

      uint8_t encbuf[VSCP_MULTICAST_BUFFER_SIZE] = { 0 };
      if (VSCP_ERROR_SUCCESS != vscp_fwhlp_decryptFrame(encbuf,
                                                        buf,
                                                        len - 16,
                                                        g_persistent.pmk,
                                                        buf + len - 16,
                                                        VSCP_ENCRYPTION_FROM_TYPE_BYTE)) {
        ESP_LOGE(TAG, "Error decrypting frame.");
        continue;
      }
      if (1) {
        ESP_LOGV(TAG, "Decrypted frame: Length: %d", len);
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, encbuf, len, ESP_LOG_VERBOSE);
      }

      // Copy decrypted frame back to buffer
      memcpy(buf, encbuf, len);

    } // encrypted

    vscp_event_t ev;
    memset(&ev, 0, sizeof(ev));
    if (VSCP_ERROR_SUCCESS != (rv = vscp_fwhlp_getEventFromFrame(&ev, buf, len))) {
      ESP_LOGE(TAG, "Error reading event from frame. rv=%d", rv);
      continue;
    }

    // Add event to CAN queue
    can4vscp_frame_t txmsg = {};
    if (VSCP_ERROR_SUCCESS != (rv = can4vscp_event_to_msg(&txmsg, &ev))) {
      ESP_LOGE(TAG, "Failed to convert VSCP event to CAN message rv=%d", rv);
      if (ev.pdata) {
        free(ev.pdata);
      }
      continue;
    }

    can4vscp_frame_t tx_msg;
    tx_msg.data_length_code = ev.sizeData;
    tx_msg.extd             = 1;
    tx_msg.identifier       = ev.GUID[0] + (ev.vscp_type << 8) + (ev.vscp_class << 16) + (((ev.head >> 5) & 7) << 26);
    memcpy(tx_msg.data, ev.pdata, ev.sizeData);

    if (0) {
      printf("Event:\n");
      printf("Head: %d\n", ev.head);
      printf("Class: %d\n", ev.vscp_class);
      printf("Type: %d\n", ev.vscp_type);
      printf("Size: %d\n", ev.sizeData);
      for (int i = 0; i < ev.sizeData; i++) {
        printf("%02x ", ev.pdata[i]);
      }
      printf("\n");
      printf("----------------------------------------------------\n");
      printf("Timestamp: 0x%08lX\n", (long unsigned int) ev.timestamp);
      printf("Obid: 0x%08lX\n", (long unsigned int) ev.obid);
      printf("Year: %d\n", ev.year);
      printf("Month: %d\n", ev.month);
      printf("Day: %d\n", ev.day);
      printf("Hour: %d\n", ev.hour);
      printf("Minute: %d\n", ev.minute);
      printf("Second: %d\n", ev.second);
      printf("----------------------------------------------------\n");
    }

    can4vscp_frame_t msg = { 0 };
    if (VSCP_ERROR_SUCCESS != can4vscp_event_to_msg(&msg, &ev)) {
      ESP_LOGE(TAG, "Failed to convert VSCP event to CAN message rv=%d", rv);
      if (ev.pdata) {
        free(ev.pdata);
      }
      continue;
    }

    ESP_LOGI(TAG,
             "Send multicast event as CAN message to queue: class=%u type=%u size=%u",
             ev.vscp_class,
             ev.vscp_type,
             ev.sizeData);

    esp_err_t err;         
    if (ESP_OK != (err = can4vscp_send(&msg, pdMS_TO_TICKS(10)))) {
      ESP_LOGE(TAG, "Failed to send CAN message to queue: %d", err);
    }

    if (ev.pdata) {
      free(ev.pdata);
    }

  } // while

  /* send a DROP MEMBERSHIP message via setsockopt */
  if ((setsockopt(sock, IPPROTO_IP, IP_DROP_MEMBERSHIP, (void *) &mc_req, sizeof(mc_req))) < 0) {
    ESP_LOGE(TAG, "setsockopt() failed");
    exit(1);
  }
  close(sock);

terminate:

  ESP_LOGI(TAG, "Multicast receive task ended");

  vTaskDelete(NULL);
}

////////////////////////////////////////////////////////////////////////////
// multicast_start
//

void
multicast_start(void)
{
  crcInit();

  if (g_persistent.bHeartbeat) {
    xTaskCreate(multicast_tx_task, "multicast_tx", 5 * 1024, NULL, 5, NULL);
#ifdef CONFIG_EXAMPLE_IPV6
    xTaskCreate(udpsrv_tx_task, "multicast_tx", 5 * 1024, (void *) AF_INET6, 5, NULL);
#endif
  }

  xTaskCreate(multicast_rx_task, "multicast_rx", 4 * 1024, NULL, 5, NULL);
#ifdef CONFIG_EXAMPLE_IPV6
  xTaskCreate(udpsrv_rx_task, "multicast_rx", 4 * 1024, (void *) AF_INET6, 5, NULL);
#endif
}