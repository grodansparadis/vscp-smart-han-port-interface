/*
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
#include <esp_ota_ops.h>
#include <esp_timer.h>
#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_http_server.h>

#include <esp_event_base.h>
#include <esp_tls_crypto.h>
#include <esp_vfs.h>
#include <esp_spiffs.h>
#include <esp_http_server.h>
//#include <wifi_provisioning/manager.h>

#include <netinet/in.h>
#include <lwip/sockets.h>

// Fallback if not defined by headers
#ifndef INET6_ADDRSTRLEN
#define INET6_ADDRSTRLEN 46
#endif

#include <vscp.h>
#include <vscp-firmware-helper.h>

#include "urldecode.h"

#include "main.h"
#include "websrv.h"

#include "vscp-ws1.h"

extern node_persistent_config_t g_persistent;

#define TAG __func__

///////////////////////////////////////////////////////////////////////////////
// vscp_ws1_callback_init
//

int
vscp_ws1_callback_init(vscp_ws_connection_context_t *pctx)
{
  memcpy(pctx->guid, g_persistent.guid, 16); // Initialise GUID from persistent storage
  pctx->encryption = 0; // No encryption
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_ws1_callback_cleanup
//

int
vscp_ws1_callback_cleanup(vscp_ws_connection_context_t *pctx)
{
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_ws1_callback_generate_sid
//

int
vscp_ws1_callback_generate_sid(vscp_ws_connection_context_t *pctx, uint8_t *sid, size_t size)
{
  return vscp_ws1_generate_sid(pctx, pctx->sid, sizeof(pctx->sid));
}

///////////////////////////////////////////////////////////////////////////////
// vscp_ws1_callback_get_primary_key
//

const uint8_t *
vscp_ws1_callback_get_primary_key(vscp_ws_connection_context_t *pctx)
{
  // Return a pointer to the encryption key. If the key is larger on ths system
  // only the first 16 bytes will be used as the 128 bit key for encryption.
  return g_persistent.pmk;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_ws1_callback_is_allowed_event
//

int
vscp_ws1_callback_is_allowed_event(vscp_ws_connection_context_t *pctx, vscp_event_t *pEvent)
{
  // All events are allowed
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_ws1_callback_is_allowed_connection
//

int
vscp_ws1_callback_is_allowed_connection(vscp_ws_connection_context_t *pctx, const char *pip)
{
  int sockfd = httpd_req_to_sockfd((httpd_req_t *) pctx->pdata);
  struct sockaddr_storage addr;
  socklen_t addr_len               = sizeof(addr);
  char remote_ip[INET6_ADDRSTRLEN] = { 0 };

  // Get remote IP address
  if (getpeername(sockfd, (struct sockaddr *) &addr, &addr_len) == 0) {
    if (addr.ss_family == AF_INET) {
      inet_ntoa_r(((struct sockaddr_in *) &addr)->sin_addr, remote_ip, sizeof(remote_ip) - 1);
    }
    else if (addr.ss_family == AF_INET6) {
      inet6_ntoa_r(((struct sockaddr_in6 *) &addr)->sin6_addr, remote_ip, sizeof(remote_ip) - 1);
    }
    ESP_LOGI(TAG, "WS1 remote IP: %s", remote_ip);
  }
  else {
    ESP_LOGE(TAG, "Failed to get remote IP address");
  }

  // All connections allowed
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_ws1_callback_validate_user
//

int
vscp_ws1_callback_validate_user(vscp_ws_connection_context_t *pctx,
                                const uint8_t *pcrypt,
                                uint8_t crypto_len,
                                const uint8_t *psid)
{
  int rv;
  //size_t len;
  uint8_t encbuf[128] = { 0 };

  ESP_LOGI(TAG, "Validating user with encrypted credentials crypto_len=%d", crypto_len);

  // Decrypt the credentials using the session ID as the IV and the pre-shared key
  if (VSCP_ERROR_SUCCESS !=
      (rv = vscp_fwhlp_decryptFrame(encbuf, pcrypt, crypto_len, g_persistent.pmk, psid, VSCP_HLO_ENCRYPTION_AES128))) {
    ESP_LOGE(TAG, "Failed to decrypt credentials with error %d", rv);
    return rv;
  }

  ESP_LOGD(TAG, "Decrypted credentials: %s", encbuf + 1);

  // The decrypted credentials should be in the format "username:password"
  char *p = strchr((char *) encbuf + 1, ':');
  if (NULL == p) {
    ESP_LOGE(TAG, "Invalid decrypted credentials format");
    return VSCP_ERROR_INVALID_PERMISSION;
  }

  char *username = (char *) encbuf + 1;
  *p++           = '\0'; // Null-terminate username
  char *password = p;    // Null terminated password starts after the colon

  ESP_LOGV(TAG, "Decrypted credentials - Username: %s, Password: %s", username, password);

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_ws1_callback_reply
//

int
vscp_ws1_callback_reply(vscp_ws_connection_context_t *pctx, const char *response)
{
  if (NULL == response) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  httpd_req_t *req = (httpd_req_t *) pctx->pdata;
  if (NULL == req) {
    return VSCP_ERROR_INVALID_CONTEXT;
  }

  // size_t reply_len = 40; // "+;AUTH0;55BCA4DC7C1FD9C3E6967F37C8747698" is 40 chars long
  // char *reply = calloc(1, strlen(response) + 1);
  // if (NULL == reply) {
  //   return VSCP_ERROR_MEMORY;
  // }

  httpd_ws_frame_t tx = { 0 };
  tx.type             = HTTPD_WS_TYPE_TEXT;
  ESP_LOGI(TAG, "WS1 replying with: %s", response);
  //snprintf(reply, strlen(response) + 1, "%s", response);
  tx.payload    = (uint8_t *) response;
  tx.len        = strlen(response);
  esp_err_t err = httpd_ws_send_frame(req, &tx);
  //free(reply);

  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to send WS1 reply with error %d", err);
    return VSCP_ERROR_INTERFACE;
  }

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_ws1_callback_event
//

int
vscp_ws1_callback_event(vscp_ws_connection_context_t *pctx, vscp_event_t *pEvent)
{
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_ws1_callback_copyright
//

int
vscp_ws1_callback_copyright(vscp_ws_connection_context_t *pctx)
{
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_ws1_callback_open
//

int
vscp_ws1_callback_open(vscp_ws_connection_context_t *pctx)
{
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_ws1_callback_close
//

int
vscp_ws1_callback_close(vscp_ws_connection_context_t *pctx)
{
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_ws1_callback_setfilter
//

int
vscp_ws1_callback_setfilter(vscp_ws_connection_context_t *pctx, const vscpEventFilter *pfilter)
{
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_ws1_callback_clrqueue
//

int
vscp_ws1_callback_clrqueue(vscp_ws_connection_context_t *pctx)
{
  return VSCP_ERROR_SUCCESS;
}