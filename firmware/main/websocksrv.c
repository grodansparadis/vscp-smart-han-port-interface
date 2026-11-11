/*
  File: websocksrv.h

  VSCP Smart meter i/f (VSCP-SMP1)

  This file is part of the VSCP (https://www.vscp.org)

  The MIT License (MIT)
  Copyright (C) 2025-2026 Ake Hedman, the VSCP project <info@vscp.org>

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
#include <ctype.h>
#include <sys/param.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <dirent.h>

#include <esp_system.h>
#include <esp_chip_info.h>
#include <esp_flash_spi_init.h>
#include <esp_flash.h>
#include <esp_wifi.h>
#include <esp_mac.h>
#include <nvs.h>
#include <esp_ota_ops.h>
#include <esp_timer.h>
#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <esp_event_base.h>
#include <esp_tls_crypto.h>
#include <esp_vfs.h>
#include <esp_spiffs.h>
#include <esp_http_server.h>
// #include <wifi_provisioning/manager.h>
#include "keep-alive.h"
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include <vscp.h>
#include <vscp-firmware-helper.h>

#include "urldecode.h"
#include "main.h"

#include "vscp-ws-common.h"
#include "vscp-ws1.h"
#include "vscp-ws2.h"
#include "websocksrv.h"

#if !CONFIG_HTTPD_WS_SUPPORT
#error This code cannot be used unless HTTPD_WS_SUPPORT is enabled in esp-http-server component configuration
#endif

// External from main
extern nvs_handle_t g_nvsHandle;
extern node_persistent_config_t g_persistent;
extern vprintf_like_t g_stdLogFunc;

extern transport_t tr_websockets;

#define TAG __func__

static const size_t max_clients = 4;

// Global server handle for WebSocket server instance
httpd_handle_t g_websocket_srv = NULL;
// httpd_handle_t g_server_websocket = NULL;

esp_err_t
wss_close_client_fd(int fd)
{
  if (NULL == g_websocket_srv) {
    ESP_LOGW(TAG, "wss_close_client_fd: websocket server not running");
    return ESP_ERR_INVALID_STATE;
  }

  if (fd < 0) {
    ESP_LOGW(TAG, "wss_close_client_fd: invalid socket fd=%d", fd);
    return ESP_ERR_INVALID_ARG;
  }

  int client_info = httpd_ws_get_fd_info(g_websocket_srv, fd);
  if (HTTPD_WS_CLIENT_WEBSOCKET != client_info) {
    ESP_LOGW(TAG, "wss_close_client_fd: fd=%d is not an active websocket client (info=%d)", fd, client_info);
    return ESP_ERR_NOT_FOUND;
  }

  ESP_LOGI(TAG, "wss_close_client_fd: closing websocket fd=%d", fd);
  return httpd_sess_trigger_close(g_websocket_srv, fd);
}

/*
 * Structure holding server handle
 * and internal socket fd in order
 * to use out of request send
 */
struct async_resp_arg {
  httpd_handle_t hd;
  int fd;
  vscp_event_ex_t ex; // Canonical event payload. Serialization is done in send_event.
};

///////////////////////////////////////////////////////////////////////////////
// ws_async_send
//

static void
ws_async_send(void *arg)
{
  httpd_ws_frame_t ws_pkt;
  struct async_resp_arg *resp_arg = arg;
  httpd_handle_t hd               = resp_arg->hd;

  static int led_state = 0;
  char buf[4];
  memset(buf, 0, sizeof(buf));
  sprintf(buf, "%d", led_state);

  memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
  ws_pkt.payload = (uint8_t *) buf;
  ws_pkt.len     = strlen(buf);
  ws_pkt.type    = HTTPD_WS_TYPE_TEXT;

  static size_t max_clients = CONFIG_LWIP_MAX_LISTENING_TCP;
  size_t fds                = max_clients;
  int client_fds[max_clients];

  esp_err_t ret = httpd_get_client_list(g_websocket_srv, &fds, client_fds);

  if (ret != ESP_OK) {
    free(resp_arg);
    return;
  }

  for (int i = 0; i < fds; i++) {
    int client_info = httpd_ws_get_fd_info(g_websocket_srv, client_fds[i]);
    if (client_info == HTTPD_WS_CLIENT_WEBSOCKET) {
      httpd_ws_send_frame_async(hd, client_fds[i], &ws_pkt);
    }
  }
  free(resp_arg);
}

///////////////////////////////////////////////////////////////////////////////
// trigger_async_send
//

static esp_err_t __attribute__((unused))
trigger_async_send(httpd_handle_t handle, httpd_req_t *req)
{
  struct async_resp_arg *resp_arg = malloc(sizeof(struct async_resp_arg));
  if (resp_arg == NULL) {
    return ESP_ERR_NO_MEM;
  }
  resp_arg->hd  = req->handle;
  resp_arg->fd  = httpd_req_to_sockfd(req);
  esp_err_t ret = httpd_queue_work(handle, ws_async_send, resp_arg);
  if (ret != ESP_OK) {
    free(resp_arg);
  }
  return ret;
}

// ----------------------------------------------------------------------------

///////////////////////////////////////////////////////////////////////////////
// wss_open_fd
//

// static esp_err_t
// ws_handler(httpd_req_t *req)
// {
//   if (req->method == HTTP_GET) {
//     ESP_LOGI(TAG, "Handshake done, the new connection was opened");
//     return ESP_OK;
//   }
//   httpd_ws_frame_t ws_pkt;
//   uint8_t *buf = NULL;
//   memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));

//   // First receive the full ws message
//   /* Set max_len = 0 to get the frame len */
//   esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
//   if (ret != ESP_OK) {
//     ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with %d", ret);
//     return ret;
//   }
//   ESP_LOGI(TAG, "frame len is %d", ws_pkt.len);
//   if (ws_pkt.len) {
//     /* ws_pkt.len + 1 is for NULL termination as we are expecting a string */
//     buf = calloc(1, ws_pkt.len + 1);
//     if (buf == NULL) {
//       ESP_LOGE(TAG, "Failed to calloc memory for buf");
//       return ESP_ERR_NO_MEM;
//     }
//     ws_pkt.payload = buf;
//     /* Set max_len = ws_pkt.len to get the frame payload */
//     ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
//     if (ret != ESP_OK) {
//       ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
//       free(buf);
//       return ret;
//     }
//   }
//   // If it was a PONG, update the keep-alive
//   if (ws_pkt.type == HTTPD_WS_TYPE_PONG) {
//     ESP_LOGD(TAG, "Received PONG message");
//     free(buf);
//     return wss_keep_alive_client_is_active(httpd_get_global_user_ctx(req->handle), httpd_req_to_sockfd(req));

//     // If it was a TEXT message, just echo it back
//   }
//   else if (ws_pkt.type == HTTPD_WS_TYPE_TEXT || ws_pkt.type == HTTPD_WS_TYPE_PING ||
//            ws_pkt.type == HTTPD_WS_TYPE_CLOSE) {
//     if (ws_pkt.type == HTTPD_WS_TYPE_TEXT) {
//       ESP_LOGI(TAG, "Received packet with message: %s", ws_pkt.payload);
//     }
//     else if (ws_pkt.type == HTTPD_WS_TYPE_PING) {
//       // Response PONG packet to peer
//       ESP_LOGI(TAG, "Got a WS PING frame, Replying PONG");
//       ws_pkt.type = HTTPD_WS_TYPE_PONG;
//     }
//     else if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE) {
//       // Response CLOSE packet with no payload to peer
//       ws_pkt.len     = 0;
//       ws_pkt.payload = NULL;
//     }
//     ret = httpd_ws_send_frame(req, &ws_pkt);
//     if (ret != ESP_OK) {
//       ESP_LOGE(TAG, "httpd_ws_send_frame failed with %d", ret);
//     }
//     ESP_LOGI(TAG,
//              "ws_handler: httpd_handle_t=%p, sockfd=%d, client_info:%d",
//              req->handle,
//              httpd_req_to_sockfd(req),
//              httpd_ws_get_fd_info(req->handle, httpd_req_to_sockfd(req)));
//     free(buf);
//     return ret;
//   }
//   free(buf);
//   return ESP_OK;
// }

///////////////////////////////////////////////////////////////////////////////
// wss_open_fd
//

esp_err_t
wss_open_fd(httpd_handle_t hd, int sockfd)
{
  ESP_LOGI(TAG, "New client connected %d", sockfd);
  wss_keep_alive_t h = httpd_get_global_user_ctx(hd);
  return wss_keep_alive_add_client(h, sockfd);
}

///////////////////////////////////////////////////////////////////////////////
// wss_close_fd
//

void
wss_close_fd(httpd_handle_t hd, int sockfd)
{
  ESP_LOGI(TAG, "Client disconnected %d", sockfd);
  wss_keep_alive_t h = httpd_get_global_user_ctx(hd);
  wss_keep_alive_remove_client(h, sockfd);
  close(sockfd);
}

///////////////////////////////////////////////////////////////////////////////
// send_event
//

static void
send_event(void *arg)
{
  struct async_resp_arg *resp_arg = arg;
  httpd_handle_t hd               = resp_arg->hd;
  int fd                          = resp_arg->fd;

  // We need to check which protocol the client expects
  vscp_ws_connection_context_t *pctx = (vscp_ws_connection_context_t *) httpd_sess_get_ctx(hd, fd);

  if (pctx == NULL) {
    ESP_LOGW(TAG, "send_event: client context is NULL, skipping send for fd=%d", fd);
    free(resp_arg);
    return;
  }

  // ws2: only authenticated clients with an open channel get events that pass their filter
  if (VSCP_WS2_PROTOCOL == pctx->protocol) {
    if (!vscp_ws2_should_send_event(pctx, &resp_arg->ex)) {
      free(resp_arg);
      return;
    }

    char *json = vscp_ws2_format_event(&resp_arg->ex);
    if (NULL == json) {
      ESP_LOGE(TAG, "send_event: failed to format ws2 event for fd=%d", fd);
      free(resp_arg);
      return;
    }

    httpd_ws_frame_t ws2_pkt = { 0 };
    ws2_pkt.type             = HTTPD_WS_TYPE_TEXT;
    ws2_pkt.payload          = (uint8_t *) json;
    ws2_pkt.len              = strlen(json);

    esp_err_t ret = httpd_ws_send_frame_async(hd, fd, &ws2_pkt);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "send_event: ws2 async send failed fd=%d err=%d", fd, ret);
    }
    else {
      pctx->stats.cntTransmitFrames++;
      pctx->stats.cntTransmitData += resp_arg->ex.sizeData;
    }

    free(json);
    free(resp_arg);
    return;
  }

  ESP_LOGI(TAG,
           "send_event: Sending to fd=%d protocol=%d binary=%d enc=%d",
           fd,
           pctx->protocol,
           pctx->bBinary,
           pctx->encryption & 0x0F);

  // Create a new frame with the event payload
  httpd_ws_frame_t ws_pkt;
  memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
  if (pctx->bBinary) {
    uint8_t event_bin[1 + VSCP_BINARY_PACKET_FRAME0_HEADER_LENGTH + 2 + 512] = { 0 };
    uint16_t event_bin_len                                                     = vscp_fwhlp_getFrameSizeFromEventEx(&resp_arg->ex);

    if (event_bin_len > sizeof(event_bin)) {
      ESP_LOGE(TAG, "send_event: binary event frame too large len=%u", event_bin_len);
      free(resp_arg);
      return;
    }

    if (VSCP_ERROR_SUCCESS != vscp_fwhlp_writeEventExToFrame(event_bin, sizeof(event_bin), 0, &resp_arg->ex)) {
      ESP_LOGE(TAG, "send_event: failed to format binary event for fd=%d", fd);
      free(resp_arg);
      return;
    }

    const uint8_t encryption = pctx->encryption & 0x0F;

    if (VSCP_HLO_ENCRYPTION_NONE != encryption) {
      uint8_t plain[1 + VSCP_BINARY_PACKET_FRAME0_HEADER_LENGTH + 2 + 512] = { 0 };
      uint8_t enc[sizeof(plain) + 32]                                      = { 0 };

      if (event_bin_len > sizeof(plain)) {
        ESP_LOGE(TAG, "send_event: binary event frame too large for encryption len=%u", event_bin_len);
        free(resp_arg);
        return;
      }

      memcpy(plain, event_bin, event_bin_len);
      plain[0] = (plain[0] & 0xF0) | encryption;

      size_t enclen = vscp_fwhlp_encryptFrame(enc,
                                              plain,
                                              event_bin_len,
                                              (uint8_t *) vscp_ws1_callback_get_primary_key(pctx),
                                              NULL,
                                              VSCP_ENCRYPTION_FROM_TYPE_BYTE);
      if (0 == enclen) {
        ESP_LOGE(TAG, "send_event: failed to encrypt binary event for fd=%d", fd);
        free(resp_arg);
        return;
      }

      ws_pkt.type    = HTTPD_WS_TYPE_BINARY;
      ws_pkt.payload = enc;
      ws_pkt.len     = enclen;

      esp_err_t ret = httpd_ws_send_frame_async(hd, fd, &ws_pkt);
      if (ret != ESP_OK) {
        ESP_LOGE(TAG, "send_event: encrypted async send failed fd=%d err=%d", fd, ret);
      }

      free(resp_arg);
      return;
    }

    ws_pkt.type    = HTTPD_WS_TYPE_BINARY;
    ws_pkt.payload = event_bin;
    ws_pkt.len     = event_bin_len;
  }
  else {
    char event_str[512] = { 0 };
    if (VSCP_ERROR_SUCCESS != vscp_fwhlp_eventToStringEx(event_str, sizeof(event_str), &resp_arg->ex)) {
      ESP_LOGE(TAG, "send_event: failed to format text event for fd=%d", fd);
      free(resp_arg);
      return;
    }

    ws_pkt.type    = HTTPD_WS_TYPE_TEXT;
    ws_pkt.payload = (uint8_t *) event_str;
    ws_pkt.len     = strlen(event_str);
  }

  // Send the frame - httpd_ws_send_frame_async copies the payload internally
  esp_err_t ret = httpd_ws_send_frame_async(hd, fd, &ws_pkt);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "send_event: httpd_ws_send_frame_async failed for fd=%d, error=%d", fd, ret);
  }

  free(resp_arg);
}

///////////////////////////////////////////////////////////////////////////////
// send_ping
//

static void
send_ping(void *arg)
{
  struct async_resp_arg *resp_arg = arg;
  httpd_handle_t hd               = resp_arg->hd;
  int fd                          = resp_arg->fd;
  httpd_ws_frame_t ws_pkt;
  memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
  ws_pkt.payload = NULL;
  ws_pkt.len     = 0;
  ws_pkt.type    = HTTPD_WS_TYPE_PING;

  httpd_ws_send_frame_async(hd, fd, &ws_pkt);
  free(resp_arg);
}

///////////////////////////////////////////////////////////////////////////////
// client_not_alive_cb
//

bool
client_not_alive_cb(wss_keep_alive_t h, int fd)
{
  ESP_LOGD(TAG, "Client not alive, closing fd %d", fd);
  httpd_sess_trigger_close(wss_keep_alive_get_user_ctx(h), fd);
  return true;
}

///////////////////////////////////////////////////////////////////////////////
// check_client_alive_cb
//

bool
check_client_alive_cb(wss_keep_alive_t h, int fd)
{
  ESP_LOGD(TAG, "Checking if client (fd=%d) is alive", fd);
  struct async_resp_arg *resp_arg = malloc(sizeof(struct async_resp_arg));
  assert(resp_arg != NULL);
  resp_arg->hd = wss_keep_alive_get_user_ctx(h);
  resp_arg->fd = fd;

  if (httpd_queue_work(resp_arg->hd, send_ping, resp_arg) == ESP_OK) {
    return true;
  }
  free(resp_arg);
  return false;
}

///////////////////////////////////////////////////////////////////////////////
// start_wss_echo_server
//

// static httpd_handle_t
// start_wss_echo_server(void)
// {
//   // Prepare keep-alive engine
//   wss_keep_alive_config_t keep_alive_config = KEEP_ALIVE_CONFIG_DEFAULT();
//   keep_alive_config.max_clients             = max_clients;
//   keep_alive_config.client_not_alive_cb     = client_not_alive_cb;
//   keep_alive_config.check_client_alive_cb   = check_client_alive_cb;
//   wss_keep_alive_t keep_alive               = wss_keep_alive_start(&keep_alive_config);

//   // Start the httpd server
//   httpd_handle_t server = NULL;
//   ESP_LOGI(TAG, "Starting server");

//   httpd_config_t conf   = HTTPD_DEFAULT_CONFIG();
//   conf.max_open_sockets = max_clients;
//   conf.global_user_ctx  = keep_alive;
//   conf.open_fn          = wss_open_fd;
//   conf.close_fn         = wss_close_fd;

//   // extern const unsigned char servercert_start[] asm("_binary_servercert_pem_start");
//   // extern const unsigned char servercert_end[] asm("_binary_servercert_pem_end");
//   // conf.servercert     = servercert_start;
//   // conf.servercert_len = servercert_end - servercert_start;

//   // extern const unsigned char prvtkey_pem_start[] asm("_binary_prvtkey_pem_start");
//   // extern const unsigned char prvtkey_pem_end[] asm("_binary_prvtkey_pem_end");
//   // conf.prvtkey_pem = prvtkey_pem_start;
//   // conf.prvtkey_len = prvtkey_pem_end - prvtkey_pem_start;

//   esp_err_t ret = httpd_start(&server, &conf);
//   if (ESP_OK != ret) {
//     ESP_LOGI(TAG, "Error starting server!");
//     return NULL;
//   }

//   // Set URI handlers
//   ESP_LOGI(TAG, "Registering URI handlers");
//   // httpd_register_uri_handler(server, &ws);
//   wss_keep_alive_set_user_ctx(keep_alive, server);

//   return server;
// }

///////////////////////////////////////////////////////////////////////////////
// stop_wss_echo_server
//

// static esp_err_t
// stop_wss_echo_server(httpd_handle_t server)
// {
//   // Stop keep alive thread
//   wss_keep_alive_stop(httpd_get_global_user_ctx(server));
//   // Stop the httpd server
//   return httpd_stop(server);
// }

///////////////////////////////////////////////////////////////////////////////
// disconnect_handler
//

// static void
// disconnect_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
// {
//   httpd_handle_t *server = (httpd_handle_t *) arg;
//   if (*server) {
//     if (stop_wss_echo_server(*server) == ESP_OK) {
//       *server = NULL;
//     }
//     else {
//       ESP_LOGE(TAG, "Failed to stop https server");
//     }
//   }
// }

///////////////////////////////////////////////////////////////////////////////
// connect_handler
//

// static void
// connect_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
// {
//   httpd_handle_t *server = (httpd_handle_t *) arg;
//   if (*server == NULL) {
//     *server = start_wss_echo_server();
//   }
// }

///////////////////////////////////////////////////////////////////////////////
// handle_ws1_req
//

// static esp_err_t
// handle_ws1_req(httpd_req_t *req)
// {
//   if (req->method == HTTP_GET) {
//     ESP_LOGI(TAG, "Handshake done, the new connection was opened");
//     return ESP_OK;
//   }

//   httpd_ws_frame_t ws_pkt;
//   uint8_t *buf = NULL;
//   memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
//   ws_pkt.type   = HTTPD_WS_TYPE_TEXT;
//   esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
//   if (ret != ESP_OK) {
//     ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with %d", ret);
//     return ret;
//   }

//   if (ws_pkt.len) {
//     buf = calloc(1, ws_pkt.len + 1);
//     if (buf == NULL) {
//       ESP_LOGE(TAG, "Failed to calloc memory for buf");
//       return ESP_ERR_NO_MEM;
//     }
//     ws_pkt.payload = buf;
//     ret            = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
//     if (ret != ESP_OK) {
//       ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
//       free(buf);
//       return ret;
//     }
//     ESP_LOGI(TAG, "Got packet with message: %s", ws_pkt.payload);
//   }

//   ESP_LOGI(TAG, "frame len is %d", ws_pkt.len);

//   if (ws_pkt.type == HTTPD_WS_TYPE_TEXT && strcmp((char *) ws_pkt.payload, "toggle") == 0) {
//     free(buf);
//     return trigger_async_send(req->handle, req);
//   }
//   return ESP_OK;
// }

///////////////////////////////////////////////////////////////////////////////
// handle_ws2_req
//

// static esp_err_t
// handle_ws2_req(httpd_req_t *req)
// {
//   if (req->method == HTTP_GET) {
//     ESP_LOGI(TAG, "Handshake done, the new connection was opened");
//     return ESP_OK;
//   }

//   httpd_ws_frame_t ws_pkt;
//   uint8_t *buf = NULL;
//   memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
//   ws_pkt.type   = HTTPD_WS_TYPE_TEXT;
//   esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
//   if (ret != ESP_OK) {
//     ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with %d", ret);
//     return ret;
//   }

//   if (ws_pkt.len) {
//     buf = calloc(1, ws_pkt.len + 1);
//     if (buf == NULL) {
//       ESP_LOGE(TAG, "Failed to calloc memory for buf");
//       return ESP_ERR_NO_MEM;
//     }
//     ws_pkt.payload = buf;
//     ret            = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
//     if (ret != ESP_OK) {
//       ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
//       free(buf);
//       return ret;
//     }
//     ESP_LOGI(TAG, "Got packet with message: %s", ws_pkt.payload);
//   }

//   ESP_LOGI(TAG, "frame len is %d", ws_pkt.len);

//   if (ws_pkt.type == HTTPD_WS_TYPE_TEXT && strcmp((char *) ws_pkt.payload, "toggle") == 0) {
//     free(buf);
//     return trigger_async_send(req->handle, req);
//   }
//   return ESP_OK;
// }

///////////////////////////////////////////////////////////////////////////////
// setup_websocket_server
//

// httpd_handle_t
// setup_websocket_server(void)
// {
//   httpd_config_t config = HTTPD_DEFAULT_CONFIG();

//   httpd_uri_t ws1 = { .uri          = "/ws1",
//                       .method       = HTTP_GET,
//                       .handler      = handle_ws1_req,
//                       .user_ctx     = NULL,
//                       .is_websocket = true,
//                       .handle_ws_control_frames = true };

//   httpd_uri_t ws2 = { .uri          = "/ws2",
//                       .method       = HTTP_GET,
//                       .handler      = handle_ws2_req,
//                       .user_ctx     = NULL,
//                       .is_websocket = true,
//                       .handle_ws_control_frames = true };

//   // Start the httpd server
//   ESP_LOGI(TAG, "Starting httpd server on port: '%d'", config.server_port);

//   if (httpd_start(&g_server_websocket, &config) == ESP_OK) {
//     httpd_register_uri_handler(g_server_websocket, &ws1);
//     httpd_register_uri_handler(g_server_websocket, &ws2);
//   }

//   return g_server_websocket;
// }

///////////////////////////////////////////////////////////////////////////////
// wss_send_event_task
//
// Get all clients and send async events
//

void
wss_send_event_task(void *pvParameters)
{
  int rv;
  bool send_messages   = true;
  uint32_t event_count = 0;

  httpd_handle_t *server = (httpd_handle_t *) pvParameters;

  ESP_LOGI(TAG, "wss_send_event_task: Started, listening for events on tr_websockets.event_queue");

  // Continuously monitor outgoing events and broadcast to all WebSocket clients
  while (send_messages) {

    // Wait for event with 500ms timeout. We own the event from now on.
    vscp_event_t *pev = NULL;
    if (pdPASS != xQueueReceive(tr_websockets.event_queue, (void *) &pev, 500)) {
      // Every 10 seconds, log that we're waiting for events
      if ((event_count % 20) == 0) {
        ESP_LOGD(TAG, "wss_send_event_task: Waiting for events (processed %lu so far)", event_count);
      }
      event_count++;
      continue;
    }

    ESP_LOGI(TAG,
             "wss_send_event_task: Received event (event_count=%lu, class=%u, type=%u)",
             event_count,
             pev->vscp_class,
             pev->vscp_type);

    event_count++;

    if (!*server) { // httpd might not have been created by now
      ESP_LOGW(TAG, "wss_send_event_task: httpd server not ready yet");
      vscp_fwhlp_deleteEvent(&pev);
      continue;
    }

    vscp_event_ex_t ex;
    memset(&ex, 0, sizeof(ex));
    if (VSCP_ERROR_SUCCESS != (rv = vscp_fwhlp_convertEventToEventEx(&ex, pev))) {
      ESP_LOGE(TAG, "wss_send_event_task: Failed to convert event to eventEx rv=%d", rv);
      vscp_fwhlp_deleteEvent(&pev);
      continue;
    }

    ESP_LOGI(TAG, "wss_send_event_task: Broadcasting event class=%u type=%u size=%u", ex.vscp_class, ex.vscp_type, ex.sizeData);

    // Get list of connected WebSocket clients
    size_t clients = max_clients;
    int client_fds[max_clients];
    if (httpd_get_client_list(g_websocket_srv, &clients, client_fds) == ESP_OK) {
      ESP_LOGI(TAG, "wss_send_event_task: Found %u connected WebSocket clients", clients);

      if (clients == 0) {
        ESP_LOGW(TAG, "wss_send_event_task: No connected clients to send event to");
      }

      for (size_t i = 0; i < clients; ++i) {
        int sock = client_fds[i];
        if (httpd_ws_get_fd_info(g_websocket_srv, sock) == HTTPD_WS_CLIENT_WEBSOCKET) {
          ESP_LOGI(TAG, "wss_send_event_task: Queuing event for WebSocket client fd=%d", sock);
          struct async_resp_arg *resp_arg = malloc(sizeof(struct async_resp_arg));
          assert(resp_arg != NULL);
          resp_arg->hd = g_websocket_srv;
          resp_arg->fd = sock;
          memcpy(&resp_arg->ex, &ex, sizeof(vscp_event_ex_t));

          if (httpd_queue_work(resp_arg->hd, send_event, resp_arg) != ESP_OK) {
            ESP_LOGE(TAG, "wss_send_event_task: httpd_queue_work failed for fd=%d!", sock);
            free(resp_arg);
            send_messages = false;
            break;
          }
        }
        else {
          ESP_LOGD(TAG, "wss_send_event_task: Client fd=%d is not a WebSocket client", sock);
        }
      }
    }
    else {
      ESP_LOGE(TAG, "wss_send_event_task: httpd_get_client_list failed!");
    }

    // We have no use for the event anymore
    vscp_fwhlp_deleteEvent(&pev);
  }

  ESP_LOGI(TAG, "websocket async send event task ended (processed %lu events total)", event_count);

  vTaskDelete(NULL);
}

///////////////////////////////////////////////////////////////////////////////
// ws1_get_handler
//
// WebSocket protocol endpoint (/ws1)
// Expects text frames on format: C;COMMAND;optional-data
//

static esp_err_t
ws1_get_handler(httpd_req_t *req)
{
  esp_err_t rv = ESP_OK;

  ESP_LOGI(TAG, "WS1 handler called with method=%d", req->method);

  if (req->method == HTTP_GET) {
    ESP_LOGI(TAG, "WS1 handshake complete");

    // // Get remote IP address
    // int sockfd = httpd_req_to_sockfd(req);
    // struct sockaddr_storage addr;
    // socklen_t addr_len               = sizeof(addr);
    // char remote_ip[INET6_ADDRSTRLEN] = { 0 };

    // if (getpeername(sockfd, (struct sockaddr *) &addr, &addr_len) == 0) {
    //   if (addr.ss_family == AF_INET) {
    //     inet_ntoa_r(((struct sockaddr_in *) &addr)->sin_addr, remote_ip, sizeof(remote_ip) - 1);
    //   }
    //   else if (addr.ss_family == AF_INET6) {
    //     inet6_ntoa_r(((struct sockaddr_in6 *) &addr)->sin6_addr, remote_ip, sizeof(remote_ip) - 1);
    //   }
    //   ESP_LOGI(TAG, "WS1 remote IP: %s", remote_ip);
    // }

    // If no session data allocated we allocate a session context for this connection and save the
    // request pointer in it so we can use it in the callbacks
    if (req->sess_ctx == NULL) {

      ESP_LOGI(TAG, "WS1 session context is NULL");
      req->sess_ctx = calloc(1, sizeof(vscp_ws_connection_context_t));
      if (NULL == req->sess_ctx) {
        ESP_LOGE(TAG, "WS1 failed to allocate session context");
        return ESP_ERR_NO_MEM;
      }

      // Init the context structure for this connection. Save the request pointer in
      // the session context so we can use it in the callbacks
      if (VSCP_ERROR_SUCCESS != vscp_ws1_init((vscp_ws_connection_context_t *) req->sess_ctx, (void *) req)) {
        ESP_LOGE(TAG, "WS1 failed to initialize session context");
        free(req->sess_ctx);
        req->sess_ctx = NULL;
        return ESP_FAIL;
      }

      // Save protocol type for async handlers
      ((vscp_ws_connection_context_t *) req->sess_ctx)->protocol = VSCP_WS1_PROTOCOL;

    } // no context

    return ESP_OK;
  }

  httpd_ws_frame_t ws_pkt = { 0 };
  // rx.type             = HTTPD_WS_TYPE_TEXT;

  /*
    First receive the full ws payload
    Set max_len = 0 to get the frame len
  */
  rv = httpd_ws_recv_frame(req, &ws_pkt, 0);
  if (ESP_OK != rv) {
    ESP_LOGE(TAG, "WS1 failed to get frame len rv=%d", rv);
    return rv;
  }

  ESP_LOGI(TAG, "WS1 frame len is %d type=%d", ws_pkt.len, ws_pkt.type);

  if (ws_pkt.len && (ws_pkt.type == HTTPD_WS_TYPE_TEXT)) {

    // Client is currently in text mode.
    ((vscp_ws_connection_context_t *) req->sess_ctx)->bBinary = false;

    // Allocate spece for the payload (+1 for NULL termination since we are expecting a string)
    uint8_t *payload = calloc(1, ws_pkt.len + 1);
    if (NULL == payload) {
      return ESP_ERR_NO_MEM;
    }

    // Get full payload
    ws_pkt.payload = payload;
    rv             = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
    if (ESP_OK != rv) {
      ESP_LOGE(TAG, "WS1 failed to receive frame rv=%d", rv);
      free(payload);
      return rv;
    }

    vscp_ws1_handle_text_protocol_request((vscp_ws_connection_context_t *) req->sess_ctx, (char *) payload, ws_pkt.len);

    free(payload);
  }
  if (ws_pkt.len && (ws_pkt.type == HTTPD_WS_TYPE_BINARY)) {

    // Client is currently in binary mode.
    ((vscp_ws_connection_context_t *) req->sess_ctx)->bBinary = true;

    // Allocate spece for the payload (+1 for NULL termination since we are expecting a string)
    uint8_t *payload = calloc(1, ws_pkt.len + 1);
    if (NULL == payload) {
      return ESP_ERR_NO_MEM;
    }

    // Get full payload
    ws_pkt.payload = payload;
    rv             = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
    if (ESP_OK != rv) {
      ESP_LOGE(TAG, "WS1 failed to receive frame rv=%d", rv);
      free(payload);
      return rv;
    }

    vscp_ws1_handle_binary_protocol_request((vscp_ws_connection_context_t *) req->sess_ctx, payload, ws_pkt.len);

    free(payload);
  }
  else if (ws_pkt.type == HTTPD_WS_TYPE_PING) {
    ESP_LOGI(TAG, "WS1 got a WS PING frame, Replying PONG");
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_PONG;
    rv          = httpd_ws_send_frame(req, &ws_pkt);
    if (ESP_OK != rv) {
      ESP_LOGE(TAG, "WS1 failed to send PONG frame with %d", rv);
      return rv;
    }
  }
  // If it was a PONG, update the keep-alive
  else if (ws_pkt.type == HTTPD_WS_TYPE_PONG) {
    ESP_LOGD(TAG, "Received PONG message");
    return wss_keep_alive_client_is_active(httpd_get_global_user_ctx(req->handle), httpd_req_to_sockfd(req));
  }
  else if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE) {
    ESP_LOGI(TAG, "WS1 got a WS CLOSE frame, Replying CLOSE");
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_CLOSE;
    rv          = httpd_ws_send_frame(req, &ws_pkt);
    if (ESP_OK != rv) {
      ESP_LOGE(TAG, "WS1 failed to send CLOSE frame with %d", rv);
      return rv;
    }
  }
  else if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE) {
    ESP_LOGI(TAG, "WS1 got a WS CLOSE frame, Replying CLOSE");
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_CLOSE;
    rv          = httpd_ws_send_frame(req, &ws_pkt);
    if (ESP_OK != rv) {
      ESP_LOGE(TAG, "WS1 failed to send CLOSE frame with %d", rv);
      return rv;
    }
  }

  return rv;
}

///////////////////////////////////////////////////////////////////////////////
// ws2_get_handler
//
// WebSocket protocol endpoint (/ws2)
// JSON based protocol, see vscp-ws2.h
//

static esp_err_t
ws2_get_handler(httpd_req_t *req)
{
  esp_err_t rv = ESP_OK;

  if (req->method == HTTP_GET) {
    ESP_LOGI(TAG, "WS2 handshake complete");

    // Allocate a session context for this connection and send the AUTH0 challenge
    if (NULL == req->sess_ctx) {
      req->sess_ctx = calloc(1, sizeof(vscp_ws_connection_context_t));
      if (NULL == req->sess_ctx) {
        ESP_LOGE(TAG, "WS2 failed to allocate session context");
        return ESP_ERR_NO_MEM;
      }

      if (VSCP_ERROR_SUCCESS != vscp_ws2_init((vscp_ws_connection_context_t *) req->sess_ctx, (void *) req)) {
        ESP_LOGE(TAG, "WS2 failed to initialize session context");
        free(req->sess_ctx);
        req->sess_ctx = NULL;
        return ESP_FAIL;
      }
    }

    return ESP_OK;
  }

  vscp_ws_connection_context_t *pctx = (vscp_ws_connection_context_t *) req->sess_ctx;
  if ((NULL == pctx) || (VSCP_WS2_PROTOCOL != pctx->protocol)) {
    ESP_LOGE(TAG, "WS2 frame without a ws2 session context");
    return ESP_FAIL;
  }

  // The request object is only valid while this frame is handled
  pctx->pdata = (void *) req;

  httpd_ws_frame_t ws_pkt = { 0 };

  // Get the frame length first
  rv = httpd_ws_recv_frame(req, &ws_pkt, 0);
  if (ESP_OK != rv) {
    ESP_LOGE(TAG, "WS2 failed to get frame len rv=%d", rv);
    return rv;
  }

  if (HTTPD_WS_TYPE_TEXT == ws_pkt.type) {

    if (0 == ws_pkt.len) {
      return ESP_OK;
    }

    // Refuse over-long frames before allocating memory for them. Returning an
    // error closes the connection.
    if (ws_pkt.len > VSCP_WS2_MAX_PACKET_SIZE) {
      ESP_LOGE(TAG, "WS2 frame too large len=%u, closing connection", (unsigned) ws_pkt.len);
      return ESP_FAIL;
    }

    uint8_t *payload = calloc(1, ws_pkt.len + 1);
    if (NULL == payload) {
      return ESP_ERR_NO_MEM;
    }

    ws_pkt.payload = payload;
    rv             = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
    if (ESP_OK != rv) {
      ESP_LOGE(TAG, "WS2 failed to receive frame rv=%d", rv);
      free(payload);
      return rv;
    }

    int vrv = vscp_ws2_handle_request(pctx, (const char *) payload, ws_pkt.len);
    secure_zero(payload, ws_pkt.len); // May hold AUTH credentials
    free(payload);

    if (VSCP_ERROR_INVALID_PERMISSION == vrv) {
      ESP_LOGW(TAG, "WS2 too many failed logins, closing connection");
      return ESP_FAIL;
    }
  }
  else if (HTTPD_WS_TYPE_BINARY == ws_pkt.type) {
    // ws2 is text only. Refuse binary frames without reading them.
    ESP_LOGE(TAG, "WS2 binary frames are not supported, closing connection");
    return ESP_FAIL;
  }
  else if (HTTPD_WS_TYPE_PING == ws_pkt.type) {
    httpd_ws_frame_t pong = { 0 };
    pong.type             = HTTPD_WS_TYPE_PONG;
    rv                    = httpd_ws_send_frame(req, &pong);
    if (ESP_OK != rv) {
      ESP_LOGE(TAG, "WS2 failed to send PONG frame rv=%d", rv);
    }
  }
  else if (HTTPD_WS_TYPE_PONG == ws_pkt.type) {
    return wss_keep_alive_client_is_active(httpd_get_global_user_ctx(req->handle), httpd_req_to_sockfd(req));
  }
  else if (HTTPD_WS_TYPE_CLOSE == ws_pkt.type) {
    httpd_ws_frame_t close_frame = { 0 };
    close_frame.type             = HTTPD_WS_TYPE_CLOSE;
    rv                           = httpd_ws_send_frame(req, &close_frame);
    if (ESP_OK != rv) {
      ESP_LOGE(TAG, "WS2 failed to send CLOSE frame rv=%d", rv);
    }
  }

  return rv;
}

static const httpd_uri_t ws1 = { .uri                      = "/ws1",
                                 .method                   = HTTP_GET,
                                 .handler                  = ws1_get_handler,
                                 .user_ctx                 = NULL,
                                 .is_websocket             = true,
                                 .handle_ws_control_frames = true };

static const httpd_uri_t ws2 = { .uri                      = "/ws2",
                                 .method                   = HTTP_GET,
                                 .handler                  = ws2_get_handler,
                                 .user_ctx                 = NULL,
                                 .is_websocket             = true,
                                 .handle_ws_control_frames = true };

///////////////////////////////////////////////////////////////////////////////
// wss_register_handlers
//

int
wss_register_handlers(httpd_handle_t server)
{
  esp_err_t err;

  if (ESP_OK != (err = httpd_register_uri_handler(server, &ws1))) {
    ESP_LOGE(TAG, "Failed to register WS1 URI handler err=%d", err);
    return err;
  }

  if (ESP_OK != (err = httpd_register_uri_handler(server, &ws2))) {
    ESP_LOGE(TAG, "Failed to register WS2 URI handler err=%d", err);
    return err;
  }

  return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////
// wss_start_websocket_server
//

httpd_handle_t
wss_start_websocket_server(void)
{
  // Prepare keep-alive engine
  wss_keep_alive_config_t keep_alive_config = KEEP_ALIVE_CONFIG_DEFAULT();
  keep_alive_config.max_clients             = max_clients;
  keep_alive_config.client_not_alive_cb     = client_not_alive_cb;
  keep_alive_config.check_client_alive_cb   = check_client_alive_cb;
  wss_keep_alive_t keep_alive               = wss_keep_alive_start(&keep_alive_config);

  httpd_handle_t ws_srv      = NULL;
  httpd_config_t ws_config   = HTTPD_DEFAULT_CONFIG();
  ws_config.server_port      = g_persistent.websockPort;
  ws_config.ctrl_port        = ESP_HTTPD_DEF_CTRL_PORT + 1;
  ws_config.stack_size       = 1024 * 5;
  ws_config.lru_purge_enable = false; // Reject connection when max clients is reached instead of purging LRU connection
  ws_config.global_user_ctx  = keep_alive;
  ws_config.open_fn          = wss_open_fd;
  ws_config.close_fn         = wss_close_fd;
  ws_config.max_uri_handlers = 2;
  ws_config.max_open_sockets = max_clients; // Limit to 4 WebSocket connections

  ESP_LOGI(TAG, "Starting websocket server on port: '%d' (ctrl: '%d')", ws_config.server_port, ws_config.ctrl_port);
  if (httpd_start(&ws_srv, &ws_config) == ESP_OK) {

    if (ESP_OK != wss_register_handlers(ws_srv)) {
      httpd_stop(ws_srv);
      return NULL;
    }

    g_websocket_srv = ws_srv;

    // Start workerthread to send async messages to clients
    xTaskCreate(wss_send_event_task, "wss_send_event_task", 4096, &ws_srv, 5, NULL);

    return ws_srv;
  }

  ESP_LOGE(TAG, "Error starting websocket server!");
  return NULL;
}
