/*
  File: tcpsrv.c

  VSCP Wireless CAN4VSCP Gateway (VSCP-WCANG)

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
extern transport_t tr_twai_rx;
extern transport_t tr_tcpsrv[MAX_TCP_CONNECTIONS];

static const char *TAG = "tcpsrv";

static uint8_t cntClients = 0; // Holds current number of clients

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
  pctx->bValidated        = 0;
  pctx->privLevel         = 0;
  pctx->bRcvLoop          = 0;
  pctx->size              = 0;
  pctx->last_rcvloop_time = esp_timer_get_time();
  // vscp_fifo_clear(&pctx->fifoEventsOut);
  memset(pctx->buf, 0, TCPIP_BUF_MAX_SIZE);
  memset(pctx->user, 0, VSCP_LINK_MAX_USER_NAME_LENGTH);
  // Filter: All events received
  memset(&pctx->filter, 0, sizeof(vscpEventFilter));
  memset(&pctx->statistics, 0, sizeof(vscp_statistics_t));
  memset(&pctx->status, 0, sizeof(vscp_status_t));
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
  size_t len;
  ctx_t ctx;
  char rxbuf[128];

  // Set default
  // setContextDefaults(&ctx);
  ctx.bValidated        = 0;
  ctx.privLevel         = 0;
  ctx.bRcvLoop          = 0;
  ctx.size              = 0;
  ctx.last_rcvloop_time = esp_timer_get_time();
  memset(ctx.buf, 0, TCPIP_BUF_MAX_SIZE);
  memset(ctx.user, 0, VSCP_LINK_MAX_USER_NAME_LENGTH);
  // Filter: All events received
  memset(&ctx.filter, 0, sizeof(vscpEventFilter));
  memset(&ctx.statistics, 0, sizeof(vscp_statistics_t));
  memset(&ctx.status, 0, sizeof(vscp_status_t));

  // int sock = (int)*((int *)pvParameters);
  ctx.sock = *((int *) pvParameters);
  ctx.id   = cntClients++;

  // Mark transport channel as open
  tr_tcpsrv[ctx.id].open = true;

  ESP_LOGI(TAG, "Client worker socket=%d id=%d", ctx.sock, ctx.id);

  // Greet client
  send(ctx.sock, TCPSRV_WELCOME_MSG, sizeof(TCPSRV_WELCOME_MSG), 0);

  // twai_message_t rxmsg = {};
  // rxmsg.identifier = 0x12345;
  // if( pdPASS != (rv = xQueueSendToBack( tr_twai_rx.msg_queue,
  //                                       (void *)&rxmsg,
  //                                       (TickType_t)10)) ) {
  //   ESP_LOGI(TAG, "Buffer full: Failed to save message toRX queue");
  // }

  while (1) {

    // input data from socket but wait no more than 500 milliseconds
    struct timeval tv;
    tv.tv_sec  = 0;
    tv.tv_usec = 500000; // 500 milliseconds
    setsockopt(ctx.sock, SOL_SOCKET, SO_RCVTIMEO, (const char *) &tv, sizeof tv);

    // Read command from client
    memset(rxbuf, 0, sizeof(rxbuf));
    len = (rv = recv(ctx.sock, rxbuf, sizeof(rxbuf) - 1, 0));
    if (rv < 0) {

      if (errno == EAGAIN) {
        vscp_link_idle_worker(&ctx); // Handle rcvloop etc
        continue;
      }

      // Timeout
      ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
      break;
    }
    else if (len == 0) {
      ESP_LOGI(TAG, "Connection closed");
      break;
    }
    else {

      // Append to command buffer with strict bounds
      size_t used      = strnlen(ctx.buf, sizeof(ctx.buf));
      size_t remaining = sizeof(ctx.buf) - used - 1; // keep room for terminator
      size_t copy_len  = MIN((size_t) len, remaining);

      if (copy_len < (size_t) len) {
        ESP_LOGW(TAG,
                 "Input command buffer full, truncating incoming data (%u -> %u)",
                 (unsigned) len,
                 (unsigned) copy_len);
      }

      if (copy_len > 0) {
        memcpy(ctx.buf + used, rxbuf, copy_len);
        ctx.buf[used + copy_len] = '\0';
      }
      ctx.size = used + copy_len;

      ESP_LOGI(TAG, "Received %d bytes: %s", len, rxbuf);

      // Parse all complete VSCP commands currently available in the context buffer
      while (ctx.size > 0) {

        char *p = NULL;
        int parser_rv = vscp_link_parser(&ctx, ctx.buf, &p);

        if (VSCP_ERROR_MISSING == parser_rv) {
          break; // incomplete command, wait for more data
        }

        if (VSCP_ERROR_SUCCESS != parser_rv) {
          ESP_LOGW(TAG, "VSCP parser returned error=%d", parser_rv);
        }

        if (!ctx.sock) {
          break;
        }

        if ((NULL == p) || (p < ctx.buf) || (p > (ctx.buf + ctx.size))) {
          ESP_LOGW(TAG, "Invalid parser next pointer, resetting command buffer");
          ctx.buf[0] = '\0';
          ctx.size   = 0;
          break;
        }

        size_t consumed = (size_t) (p - ctx.buf);
        size_t left     = ctx.size - consumed;
        memmove(ctx.buf, p, left);
        ctx.buf[left] = '\0';
        ctx.size      = left;
      }

      // If socket gets closed ("quit" command)
      // pctx->sock is zero
      if (!ctx.sock) {
        break;
      }

      // Get event from input fifo
      // vscp_event_t *pev = NULL;
      // vscp_fifo_read(&fifoEventsIn, &pev);

      // pev is NULL if no event is available here
      // The worker is still called.
      // if pev != NULL the worker is responsible for
      // freeing the event

      // Do VSCP protocol work here
      // vscp2_do_work(pev);

      // Handle rcvloop etc
      // for (int i = 0; i < MAX_TCP_CONNECTIONS; i++) {
      vscp_link_idle_worker(&ctx);
      //}

      ESP_LOGV(TAG, "Command handled");
    }
  };

  // Mark transport channel as closed

  tr_tcpsrv[ctx.id].open = false;

  // Empty the queue
  xQueueReset(tr_tcpsrv[ctx.id].tocan_queue);
  xQueueReset(tr_tcpsrv[ctx.id].fromcan_queue);

  ESP_LOGI(TAG, "Closing down tcp/ip client");

  // If not shutdown all ready do it here
  if (!ctx.sock) {
    shutdown(ctx.sock, 0);
    close(ctx.sock);
  }

  cntClients--;
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
    ctx[i].id   = i;
    ctx[i].sock = 0;
    // vscp_fifo_init(&gctx[i].fifoEventsOut, TRANSMIT_FIFO_SIZE);
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

    if (cntClients > MAX_TCP_CONNECTIONS) {
      ESP_LOGI(TAG, "Max number of clients. Closing connection");
      send(sock, MSG_MAX_CLIENTS, sizeof(MSG_MAX_CLIENTS), 0);
      shutdown(sock, 0);
      close(sock);
      cntClients--;
      continue;
    }

    xTaskCreate(client_task, "link-client", 0x2000, (void *) &sock, 5, NULL);
  }

CLEAN_UP:
  close(listen_sock);
  vTaskDelete(NULL);
}