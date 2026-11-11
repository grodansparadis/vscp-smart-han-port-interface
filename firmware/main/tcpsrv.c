/*
  File: tcpsrv.c

  VSCP Smart meter i/f (VSCP-SMP1)

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

  Config
  ------
  Enable (yes)
  ip-address for server
  port (9598)
  Valid client ip's (all)
  Username (admin)
  Password (secret)
*/

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include <esp_timer.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <nvs_flash.h>

#include <lwip/err.h>
#include <lwip/netdb.h>
#include <lwip/sockets.h>
#include <lwip/sys.h>

#include <string.h>
#include <sys/param.h>

#include "main.h"
#include <vscp.h>

#include "tcpsrv.h"

/*
  Keep-alive idle time. In idle time without receiving any data from peer,
  will send keep-alive probe packet
 */
#define KEEPALIVE_IDLE     5
#define KEEPALIVE_INTERVAL 5 // Keep-alive probe packet interval time.
#define KEEPALIVE_COUNT    3 // Keep-alive probe packet retry count.

// Global stuff
extern transport_t tr_tcpsrv[MAX_TCP_CONNECTIONS];
extern node_persistent_config_t g_persistent;

static const char *TAG = "tcpsrv";


/**
  Received event are written to this fifo
  from all channels and events is consumed by the
  VSCP protocol handler.
*/
// vscp_fifo_t fifoEventsIn;

static ctx_t ctx[MAX_TCP_CONNECTIONS]; // Socket context

///////////////////////////////////////////////////////////////////////////////
// setContextDefaults
//

void
setContextDefaults(ctx_t *pctx)
{
  int id = pctx->link.id;

  memset(pctx, 0, sizeof(ctx_t));
  pctx->link.id                = id;
  pctx->link.ops               = &g_link_ops;
  pctx->link.user_data         = pctx;
  pctx->link.sock              = 0;
  pctx->link.last_rcvloop_time = (uint32_t) (esp_timer_get_time() / 1000); // Milliseconds
  memcpy(pctx->link.guid, g_persistent.guid, 16);
  // Filter: All events received (all zero)
}

///////////////////////////////////////////////////////////////////////////////
// do_retransmit
//

// static void
//  do_retransmit(const int sock)
//  {
//    int len;
//    char rx_buffer[128];

//   do {
//     len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
//     if (len < 0) {
//       ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
//     }
//     else if (len == 0) {
//       ESP_LOGW(TAG, "Connection closed");
//     }
//     else {
//       rx_buffer[len] = 0; // Null-terminate whatever is received and treat it like a string
//       ESP_LOGI(TAG, "Received %d bytes: %s", len, rx_buffer);

//       // send() can return less bytes than supplied length.
//       // Walk-around for robust implementation.
//       int to_write = len;
//       while (to_write > 0) {
//         int written = send(sock, rx_buffer + (len - to_write), to_write, 0);
//         if (written < 0) {
//           ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
//         }
//         to_write -= written;
//       }
//     }
//   } while (len > 0);
// }

///////////////////////////////////////////////////////////////////////////////
// client_task
//

static void
client_task(void *pvParameters)
{
  int rv;
  char rxbuf[128];

  // The accept loop hands us a claimed slot with the socket filled in
  ctx_t *pctx = (ctx_t *) pvParameters;
  int id      = pctx->link.id;

  // Mark transport channel as open
  tr_tcpsrv[id].open = true;

  ESP_LOGI(TAG, "Client worker socket=%d id=%d", pctx->link.sock, id);

  // Wait no more than 500 milliseconds for input so the idle worker can run
  struct timeval tv;
  tv.tv_sec  = 0;
  tv.tv_usec = 500000;
  setsockopt(pctx->link.sock, SOL_SOCKET, SO_RCVTIMEO, (const char *) &tv, sizeof tv);

  // Greet client
  vscp_link_connect(&pctx->link);

  while (pctx->link.sock) {

    // Read command from client
    rv = recv(pctx->link.sock, rxbuf, sizeof(rxbuf), 0);
    if (rv < 0) {
      if ((EAGAIN == errno) || (EWOULDBLOCK == errno)) {
        vscp_link_idle_worker(&pctx->link); // Handle rcvloop etc
        continue;
      }
      ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
      break;
    }
    else if (0 == rv) {
      ESP_LOGI(TAG, "Connection closed");
      break;
    }

    // Append to the command buffer and handle every complete line in it
    for (int i = 0; (i < rv) && pctx->link.sock; i++) {
      char c = rxbuf[i];

      if (pctx->bDiscard) {
        // Skip until the end of the over-long line
        if ('\n' == c) {
          pctx->bDiscard = false;
        }
        continue;
      }

      if ('\n' == c) {
        // Strip a trailing CR and pass the line (null terminated) to the parser
        if ((pctx->size > 0) && ('\r' == pctx->buf[pctx->size - 1])) {
          pctx->size--;
        }
        pctx->buf[pctx->size] = '\0';

        int parser_rv = vscp_link_parser(&pctx->link, pctx->buf);
        if (VSCP_ERROR_SUCCESS != parser_rv) {
          ESP_LOGD(TAG, "VSCP parser returned %d", parser_rv);
        }
        pctx->size = 0;
      }
      else if (pctx->size < (sizeof(pctx->buf) - 1)) {
        pctx->buf[pctx->size++] = c;
      }
      else {
        // Line longer than the buffer: drop it and tell the client
        ESP_LOGW(TAG, "Command line too long, discarded");
        pctx->size     = 0;
        pctx->bDiscard = true;
        g_link_ops.write_client(&pctx->link, VSCP_LINK_MSG_ERROR);
      }
    }

    // Handle rcvloop etc
    if (pctx->link.sock) {
      vscp_link_idle_worker(&pctx->link);
    }
  }

  ESP_LOGI(TAG, "Closing down tcp/ip client id=%d", id);

  // Mark transport channel as closed and empty its queue
  tr_tcpsrv[id].open = false;
  transport_flush_queue(tr_tcpsrv[id].event_queue);

  // Close the socket unless "quit" already did
  if (pctx->link.sock) {
    shutdown(pctx->link.sock, 0);
    close(pctx->link.sock);
    pctx->link.sock = 0;
  }

  // Wipe the context and release the slot
  setContextDefaults(pctx);
  pctx->bInUse = false;

  vTaskDelete(NULL);
}

///////////////////////////////////////////////////////////////////////////////
// tcpsrv_task
//

void
tcpsrv_task(void *pvParameters)
{
  char addr_str[128];
  int addr_family  = (int) pvParameters;
  int ip_protocol  = 0;
  int keepAlive    = 1;
  int keepIdle     = KEEPALIVE_IDLE;
  int keepInterval = KEEPALIVE_INTERVAL;
  int keepCount    = KEEPALIVE_COUNT;
  struct sockaddr_storage dest_addr;

  for (int i = 0; i < MAX_TCP_CONNECTIONS; i++) {
    ctx[i].link.id = i;
    setContextDefaults(&ctx[i]);
  }

  // Initialize the input fifo
  // vscp_fifo_init(&fifoEventsIn, RECEIVE_FIFO_SIZE);

  if (addr_family == AF_INET) {
    struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *) &dest_addr;
    dest_addr_ip4->sin_addr.s_addr    = htonl(INADDR_ANY);
    dest_addr_ip4->sin_family         = AF_INET;
    dest_addr_ip4->sin_port           = htons(VSCP_DEFAULT_TCP_PORT);
    ip_protocol                       = IPPROTO_IP;
  }
  else if (addr_family == AF_INET6) {
    struct sockaddr_in6 *dest_addr_ip6 = (struct sockaddr_in6 *) &dest_addr;
    bzero(&dest_addr_ip6->sin6_addr.un, sizeof(dest_addr_ip6->sin6_addr.un));
    dest_addr_ip6->sin6_family = AF_INET6;
    dest_addr_ip6->sin6_port   = htons(VSCP_DEFAULT_TCP_PORT);
    ip_protocol                = IPPROTO_IPV6;
  }

  int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
  if (listen_sock < 0) {
    ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
    vTaskDelete(NULL);
    return;
  }
  int opt = 1;
  setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
  // Note that by default IPV6 binds to both protocols, it must be disabled
  // if both protocols used at the same time (used in CI)
  setsockopt(listen_sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
#endif

  ESP_LOGI(TAG, "Socket created");

  int err = bind(listen_sock, (struct sockaddr *) &dest_addr, sizeof(dest_addr));
  if (err != 0) {
    ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
    ESP_LOGE(TAG, "IPPROTO: %d", addr_family);
    goto CLEAN_UP;
  }
  ESP_LOGI(TAG, "Socket bound, port %d", VSCP_DEFAULT_TCP_PORT);

  err = listen(listen_sock, 1);
  if (err != 0) {
    ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
    goto CLEAN_UP;
  }

  int sock;
  while (1) {

    ESP_LOGI(TAG, "Socket listening");

    struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
    socklen_t addr_len = sizeof(source_addr);
    sock               = accept(listen_sock, (struct sockaddr *) &source_addr, &addr_len);
    if (sock < 0) {
      ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
      break;
    }

    // Set tcp keepalive option
    setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));

    // Convert ip address to string
    if (source_addr.ss_family == PF_INET) {
      inet_ntoa_r(((struct sockaddr_in *) &source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
    }
    else if (source_addr.ss_family == PF_INET6) {
      inet6_ntoa_r(((struct sockaddr_in6 *) &source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
    }

    ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

    // Claim a free connection slot. Only this task claims slots; a client
    // task releases its slot when it ends.
    ctx_t *pctx = NULL;
    for (int i = 0; i < MAX_TCP_CONNECTIONS; i++) {
      if (!ctx[i].bInUse) {
        pctx = &ctx[i];
        break;
      }
    }

    if (NULL == pctx) {
      ESP_LOGI(TAG, "Max number of clients. Closing connection");
      send(sock, MSG_MAX_CLIENTS, strlen(MSG_MAX_CLIENTS), 0);
      shutdown(sock, 0);
      close(sock);
      continue;
    }

    setContextDefaults(pctx);
    pctx->link.sock = sock;
    pctx->bInUse    = true;

    if (pdPASS != xTaskCreate(client_task, "link-client", 0x2000, (void *) pctx, 5, NULL)) {
      ESP_LOGE(TAG, "Failed to create client task");
      shutdown(sock, 0);
      close(sock);
      pctx->link.sock = 0;
      pctx->bInUse    = false;
    }
  }

CLEAN_UP:
  close(listen_sock);
  vTaskDelete(NULL);
}