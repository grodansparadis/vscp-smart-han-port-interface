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

// ESP-IDF specific callbacks for the ws2 websocket protocol (vscp-ws2.c)

#include <stdio.h>
#include <string.h>

#include <esp_log.h>
#include <esp_random.h>
#include <esp_http_server.h>

#include <vscp.h>
#include <vscp-firmware-helper.h>

#include "main.h"
#include "vscp-ws2.h"

#define TAG "ws2-cb"

// Upper bound for a reply frame sent with vscp_ws2_callback_reply
#define VSCP_WS2_MAX_REPLY_LENGTH 1024

#define VSCP_WS2_COPYRIGHT "Copyright (C) 2000-2026 Grodans Paradis AB, https://www.grodansparadis.com"

extern node_persistent_config_t g_persistent;

///////////////////////////////////////////////////////////////////////////////
// vscp_ws2_callback_reply
//

int
vscp_ws2_callback_reply(vscp_ws_connection_context_t *pctx, const char *msg)
{
  if ((NULL == pctx) || (NULL == msg)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  httpd_req_t *req = (httpd_req_t *) pctx->pdata;
  if (NULL == req) {
    return VSCP_ERROR_INVALID_CONTEXT;
  }

  size_t len = strnlen(msg, VSCP_WS2_MAX_REPLY_LENGTH);
  if (len >= VSCP_WS2_MAX_REPLY_LENGTH) {
    ESP_LOGE(TAG, "reply: message not null terminated within %d bytes, not sent", VSCP_WS2_MAX_REPLY_LENGTH);
    return VSCP_ERROR_PARAMETER;
  }

  httpd_ws_frame_t tx = { 0 };
  tx.type             = HTTPD_WS_TYPE_TEXT;
  tx.payload          = (uint8_t *) msg;
  tx.len              = len;

  ESP_LOGD(TAG, "WS2 TX: %s", msg);

  if (ESP_OK != httpd_ws_send_frame(req, &tx)) {
    ESP_LOGE(TAG, "reply: failed to send frame");
    return VSCP_ERROR_INTERFACE;
  }

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_ws2_callback_generate_sid
//

int
vscp_ws2_callback_generate_sid(vscp_ws_connection_context_t *pctx, uint8_t *sid, size_t size)
{
  if (NULL == sid) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Hardware RNG. Random when WiFi or Bluetooth is running, which is always
  // the case while the websocket server is reachable.
  esp_fill_random(sid, size);
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_ws2_callback_get_key
//

const uint8_t *
vscp_ws2_callback_get_key(vscp_ws_connection_context_t *pctx)
{
  return g_persistent.pmk;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_ws2_callback_get_guid
//

const uint8_t *
vscp_ws2_callback_get_guid(vscp_ws_connection_context_t *pctx)
{
  return g_persistent.guid;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_ws2_callback_validate_user
//

int
vscp_ws2_callback_validate_user(vscp_ws_connection_context_t *pctx, const char *username, const char *password)
{
  if ((NULL == pctx) || (NULL == username) || (NULL == password)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  if (!validate_ws_user(username, password)) {
    ESP_LOGE(TAG, "Credentials: Invalid");
    return VSCP_ERROR_INVALID_PERMISSION;
  }

  strncpy(pctx->user.username, username, sizeof(pctx->user.username) - 1);
  pctx->user.username[sizeof(pctx->user.username) - 1] = '\0';
  pctx->user.privlevel                                  = 15;

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_ws2_callback_event
//

int
vscp_ws2_callback_event(vscp_ws_connection_context_t *pctx, const vscp_event_ex_t *pex)
{
  int rv;

  if ((NULL == pctx) || (NULL == pex)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  vscp_event_t *pev = vscp_fwhlp_newEvent();
  if (NULL == pev) {
    return VSCP_ERROR_MEMORY;
  }

  if (VSCP_ERROR_SUCCESS != (rv = vscp_fwhlp_convertEventExToEvent(pev, pex))) {
    vscp_fwhlp_deleteEvent(&pev);
    return rv;
  }

  // Mark the event as coming from this connection
  httpd_req_t *req = (httpd_req_t *) pctx->pdata;
  if (NULL != req) {
    pev->obid = (uint32_t) httpd_req_to_sockfd(req);
  }

  if (0 == pev->timestamp) {
    pev->timestamp = (uint32_t) esp_log_timestamp() * 1000; // Microseconds
  }

  pctx->stats.cntReceiveFrames++;
  pctx->stats.cntReceiveData += pev->sizeData;

  rv = handle_incoming_event(pev, CH_WS);
  vscp_fwhlp_deleteEvent(&pev);
  return rv;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_ws2_callback_get_copyright
//

const char *
vscp_ws2_callback_get_copyright(vscp_ws_connection_context_t *pctx)
{
  return VSCP_WS2_COPYRIGHT;
}
