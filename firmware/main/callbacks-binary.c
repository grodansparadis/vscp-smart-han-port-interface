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

  This file contains callback implementations for the VSCP binary protocol.
*/

/*!
  The abstraction of the binary interface is defined in vscp-binary.h and vscp-binary.c
  This file moves the abstraction into the real world on a real device.
  The callbacks defined in vscp-binary.h are implemented here.
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
#include "esp_log.h"
#include "esp_log_buffer.h"
#include <nvs_flash.h>
#include <esp_http_server.h>

#include <esp_event_base.h>
#include <esp_tls_crypto.h>
#include <esp_vfs.h>
#include <esp_spiffs.h>
#include <esp_http_server.h>

#include <netinet/in.h>
#include <lwip/sockets.h>
// #include <freertos/FreeRTOS.h>
// #include <freertos/task.h>

#include "vscp-compiler.h"
#include "vscp-projdefs.h"

#include <vscp.h>
#include <vscp-class.h>
#include <vscp-crc.h>
#include <vscp-firmware-helper.h>

#include "main.h"

#include "vscp-binary.h"
#include "can4vscp.h"
#include "vscp-ws1.h"
#include "vscp-ws-common.h"
#include "websocksrv.h"

extern node_persistent_config_t g_persistent;

#define TAG __func__

static int
vscp_binary_build_reply_plain(uint16_t command,
                              uint16_t error,
                              const uint8_t *parg,
                              size_t len,
                              uint8_t encryption,
                              uint8_t **pplain,
                              size_t *pplain_len)
{
  if ((NULL == pplain) || (NULL == pplain_len)) {
    return VSCP_ERROR_PARAMETER;
  }

  const size_t plain_len = 1 + 4 + len + 2; // type + command/error + argument + crc
  uint8_t *plain         = calloc(1, plain_len);
  if (NULL == plain) {
    ESP_LOGE(TAG, "Reply: failed to allocate memory for reply");
    return VSCP_ERROR_MEMORY;
  }

  // Frame type 0xF0 is reply, lower nibble is encryption algorithm.
  plain[0] = 0xF0 | (encryption & 0x0F);
  plain[1] = (command >> 8) & 0xFF;
  plain[2] = command & 0xFF;
  plain[3] = (error >> 8) & 0xFF;
  plain[4] = error & 0xFF;

  if ((NULL != parg) && (len > 0)) {
    memcpy(plain + 5, parg, len);
  }

  // CRC is over command + error + argument (skip type byte and crc bytes).
  uint16_t crc   = crcFast(plain + 1, 4 + len);
  plain[5 + len] = (crc >> 8) & 0xFF;
  plain[6 + len] = crc & 0xFF;

  *pplain     = plain;
  *pplain_len = plain_len;

  return VSCP_ERROR_SUCCESS;
}

static int
vscp_binary_prepare_reply_payload(uint8_t *plain,
                                  size_t plain_len,
                                  const vscp_ws_connection_context_t *pctx,
                                  const uint8_t **ptx_payload,
                                  size_t *ptx_len,
                                  uint8_t **penc)
{
  const uint8_t encryption = pctx->encryption & 0x0F;

  *penc = NULL;
  if (encryption == VSCP_HLO_ENCRYPTION_NONE) {
    *ptx_payload = plain;
    *ptx_len     = plain_len;
    return VSCP_ERROR_SUCCESS;
  }

  uint8_t *enc = calloc(1, plain_len + 32); // Room for padding and trailing IV.
  if (NULL == enc) {
    ESP_LOGE(TAG, "Reply: failed to allocate memory for encryption");
    return VSCP_ERROR_MEMORY;
  }

  size_t tx_len = vscp_fwhlp_encryptFrame(enc,
                                          plain,
                                          plain_len,
                                          vscp_ws1_callback_get_primary_key((void *) pctx),
                                          NULL,
                                          VSCP_ENCRYPTION_FROM_TYPE_BYTE);
  if (0 == tx_len) {
    ESP_LOGE(TAG, "Reply: failed to encrypt frame");
    free(enc);
    return VSCP_ERROR_ERROR;
  }

  *penc        = enc;
  *ptx_payload = enc;
  *ptx_len     = tx_len;

  return VSCP_ERROR_SUCCESS;
}

static int
vscp_binary_send_ws_binary_frame(httpd_req_t *req, const uint8_t *payload, size_t len)
{
  if ((NULL == req) || (NULL == payload)) {
    return VSCP_ERROR_PARAMETER;
  }

  httpd_ws_frame_t tx = { 0 };
  tx.type             = HTTPD_WS_TYPE_BINARY;
  tx.payload          = (uint8_t *) payload;
  tx.len              = len;

  esp_err_t err = httpd_ws_send_frame(req, &tx);
  if (ESP_OK != err) {
    ESP_LOGE(TAG, "Reply: failed to send frame, error=%d", err);
    return VSCP_ERROR_INTERFACE;
  }

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_reply
//

int
vscp_binary_callback_reply(const void *pdata, uint16_t command, uint16_t error, const uint8_t *parg, size_t len)
{
  if (NULL == pdata) {
    return VSCP_ERROR_PARAMETER;
  }

  vscp_ws_connection_context_t *pctx = (vscp_ws_connection_context_t *) pdata;
  httpd_req_t *req                   = (httpd_req_t *) pctx->pdata;
  if (NULL == req) {
    ESP_LOGE(TAG, "Reply: no request context, cannot send reply");
    return VSCP_ERROR_INVALID_CONTEXT;
  }

  uint8_t *plain           = NULL;
  size_t plain_len         = 0;
  const uint8_t *tx_payload = NULL;
  size_t tx_len            = 0;
  uint8_t *enc             = NULL;

  int rv = vscp_binary_build_reply_plain(command, error, parg, len, pctx->encryption, &plain, &plain_len);
  if (VSCP_ERROR_SUCCESS != rv) {
    return rv;
  }

  rv = vscp_binary_prepare_reply_payload(plain, plain_len, pctx, &tx_payload, &tx_len, &enc);
  if (VSCP_ERROR_SUCCESS != rv) {
    free(plain);
    return rv;
  }

  rv = vscp_binary_send_ws_binary_frame(req, tx_payload, tx_len);

  free(enc);
  free(plain);

  if (VSCP_ERROR_SUCCESS != rv) {
    return rv;
  }

  ESP_LOGI(TAG, "Reply: sent command=0x%04X error=0x%04X len=%d", command, error, len);
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_challenge
//

int
vscp_binary_callback_challenge(const void *pdata, uint8_t *challenge, size_t challenge_len)
{
  if (NULL == pdata) {
    return VSCP_ERROR_PARAMETER;
  }

  vscp_ws_connection_context_t *pctx = (vscp_ws_connection_context_t *) pdata;

  // Generate a new random 16-byte sid (session ID) for authentication and encryption
  esp_fill_random(pctx->sid, sizeof(pctx->sid));
  if (challenge) {
    if (challenge_len < sizeof(pctx->sid)) {
      return VSCP_ERROR_PARAMETER;
    }
    memcpy(challenge, pctx->sid, sizeof(pctx->sid));
  }

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_get_chid
//

int
vscp_binary_callback_get_chid(const void *pdata, uint32_t *pchid)
{
  if (NULL == pdata || NULL == pchid) {
    return VSCP_ERROR_PARAMETER;
  }

  // Generate a unique channel ID from session ID bytes
  vscp_ws_connection_context_t *pctx = (vscp_ws_connection_context_t *) pdata;
  *pchid                             = pctx->chid;

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_set_guid
//

int
vscp_binary_callback_set_guid(const void *pdata, uint8_t *pguid)
{
  if (NULL == pdata || NULL == pguid) {
    return VSCP_ERROR_PARAMETER;
  }

  // Set the GUID in the connection context
  vscp_ws_connection_context_t *pctx = (vscp_ws_connection_context_t *) pdata;
  memcpy(pctx->guid, pguid, 16);

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_get_guid
//

int
vscp_binary_callback_get_guid(const void *pdata, uint8_t *pguid)
{
  if (NULL == pdata || NULL == pguid) {
    return VSCP_ERROR_PARAMETER;
  }

  // Return GUID from connection context
  vscp_ws_connection_context_t *pctx = (vscp_ws_connection_context_t *) pdata;
  pguid                              = pctx->guid;

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_setfilter
//

int
vscp_binary_callback_setfilter(const void *pdata, const vscpEventFilter *pfilter)
{
  if (NULL == pdata || NULL == pfilter) {
    return VSCP_ERROR_PARAMETER;
  }

  vscp_ws_connection_context_t *pctx = (vscp_ws_connection_context_t *) pdata;

  pctx->filter.filter_priority = pfilter->filter_priority;
  pctx->filter.filter_class    = pfilter->filter_class;
  pctx->filter.filter_type     = pfilter->filter_type;
  memcpy(pctx->filter.filter_GUID, pfilter->filter_GUID, 16);

  // Send reply confirming filter set
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_setmask
//

int
vscp_binary_callback_setmask(const void *pdata, const vscpEventFilter *pfilter)
{
  if (NULL == pdata || NULL == pfilter) {
    return VSCP_ERROR_PARAMETER;
  }

  vscp_ws_connection_context_t *pctx = (vscp_ws_connection_context_t *) pdata;

  pctx->filter.mask_priority = pfilter->mask_priority;
  pctx->filter.mask_class    = pfilter->mask_class;
  pctx->filter.mask_type     = pfilter->mask_type;
  memcpy(pctx->filter.mask_GUID, pfilter->mask_GUID, 16);

  // Send reply confirming mask set
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_get_version
//

int
vscp_binary_callback_get_version(const void *pdata, uint8_t *pversion)
{
  uint16_t major = 0;
  uint16_t minor = 0;
  uint16_t patch = 0;
  uint32_t build = 0;

  if (NULL == pdata || NULL == pversion) {
    return VSCP_ERROR_PARAMETER;
  }

  const esp_app_desc_t *appDescr = esp_app_get_description();
  char strversion[32];

  // Version string from app description (e.g. "1.0.0" or "1.0.0-rc1")
  strncpy(strversion, appDescr->version, sizeof(strversion) - 1);
  strversion[sizeof(strversion) - 1] = '\0';
  char *pstrversion                  = strversion;

  major = strtoul((const char *) pstrversion, &pstrversion, 10); // Major version
  pstrversion++;                                                 // Skip dot
  if (*pstrversion == '\0') {
    goto DONE;
  }

  minor = strtoul(pstrversion, &pstrversion, 10); // Minor version
  pstrversion++;                                  // Skip dot
  if (*pversion == '\0') {
    goto DONE;
  }

  patch = strtoul((const char *) pstrversion, &pstrversion, 10); // Patch version
  pversion++;                                                    // Skip dot
  if (*pstrversion == '\0') {
    goto DONE;
  }

  build = strtoul((const char *) pversion,
                  &pstrversion,
                  10); // Build number (not used in this example, but could be extracted from version string if needed)

DONE:

  memcpy(pversion, &major, 2);
  memcpy(pversion + 2, &minor, 2);
  memcpy(pversion + 4, &patch, 2);
  memcpy(pversion + 6, &build, 4);

  /*
    Return version information (6 bytes: major, minor, patch, build, reserved, reserved)
    Note: ESP32C3 is little-endian and we want to return big-endian in the version bytes for consistency with other
    platforms.
  */

  // if (vscp_fwhlp_isLittleEndian()) {
  major = VSCP_UINT16_SWAP_ON_LE(major);
  minor = VSCP_UINT16_SWAP_ON_LE(minor);
  patch = VSCP_UINT16_SWAP_ON_LE(patch);
  build = VSCP_UINT32_SWAP_ON_LE(build);
  //}

  // Send reply with version byte
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_is_open
//

bool
vscp_binary_callback_is_open(const void *pdata)
{
  if (NULL == pdata) {
    return false;
  }

  vscp_ws_connection_context_t *pctx = (vscp_ws_connection_context_t *) pdata;
  return pctx->bOpen;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_open
//

int
vscp_binary_callback_open(const void *pdata)
{
  if (NULL == pdata) {
    return VSCP_ERROR_PARAMETER;
  }

  vscp_ws_connection_context_t *pctx = (vscp_ws_connection_context_t *) pdata;
  pctx->bOpen                        = true;

  // Send reply confirming opened
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_close
//

int
vscp_binary_callback_close(const void *pdata)
{
  if (NULL == pdata) {
    return VSCP_ERROR_PARAMETER;
  }

  vscp_ws_connection_context_t *pctx = (vscp_ws_connection_context_t *) pdata;
  pctx->bOpen                        = false;

  // Send reply confirming closed
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_user
//

int
vscp_binary_callback_user(const void *pdata, const char *user)
{
  if (NULL == pdata || NULL == user) {
    ESP_LOGE(TAG, "vscp_binary_callback_user: Invalid parameters");
    return VSCP_ERROR_PARAMETER;
  }

  ESP_LOGI(TAG, "Received username: '%s'", user);

  // We just save the username in this stage (even if username is invalid)
  vscp_ws_connection_context_t *pctx = (vscp_ws_connection_context_t *) pdata;
  pctx->bAuthenticated               = false; // Reset authentication status when username changes

  // trim password
  const char *p = user;
  while (*p && isspace((unsigned char) *p)) {
    p++;
  }

  strncpy(pctx->user.username, p, sizeof(pctx->user.username) - 1);
  pctx->user.username[sizeof(pctx->user.username) - 1] = '\0';

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_password
//

int
vscp_binary_callback_password(const void *pdata, const char *password)
{
  // char buf[VSCP_BINARY_MAX_USERNAME_LENGTH + VSCP_BINARY_MAX_PASSWORD_LENGTH + 1 + 1] = { 0 }; // "user:password\0"
  if (NULL == pdata || NULL == password) {
    return VSCP_ERROR_PARAMETER;
  }

  vscp_ws_connection_context_t *pctx = (vscp_ws_connection_context_t *) pdata;

  // Must have a username before a password
  if (!strlen(pctx->user.username)) {
    ESP_LOGE(TAG, "Password: No username yet\n");
    return VSCP_ERROR_ERROR;
  }

  // trim password
  const char *p = password;
  while (*p && isspace((unsigned char) *p)) {
    p++;
  }

  ESP_LOGI(TAG, "Username:'%s'\n", pctx->user.username);
  ESP_LOGI(TAG, "Password '%s'\n", p);

  if (validate_user(pctx->user.username, p)) {
    pctx->bAuthenticated = true;
    pctx->user.privlevel = 15;
  }
  else {
    memset(pctx->user.username, 0, sizeof(pctx->user.username)); // Clear username on failed authentication
    pctx->bAuthenticated = false;
    pctx->user.privlevel = 0;
    ESP_LOGE(TAG, "Credentials: Invalid\n");
    return VSCP_ERROR_ERROR;
  }

  // Send reply confirming password set (don't store password in context)
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_check_authenticated
//

int
vscp_binary_callback_check_authenticated(const void *pdata)
{
  if (NULL == pdata) {
    return VSCP_ERROR_PARAMETER;
  }

  vscp_ws_connection_context_t *pctx = (vscp_ws_connection_context_t *) pdata;

  if (!pctx->bAuthenticated) {
    return VSCP_ERROR_ERROR;
  }

  // Send reply with authentication status
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_check_privilege
//

// int
// vscp_binary_callback_check_privilege(const void *pdata, uint8_t priv)
// {
//   if (NULL == pdata) {
//     return VSCP_ERROR_PARAMETER;
//   }

//   // Check if authenticated (simple privilege check)
//   vscp_ws_connection_context_t *pctx = (vscp_ws_connection_context_t *) pdata;
//   uint8_t has_priv                   = pctx->bAuthenticated ? 1 : 0;

//   // Send reply with privilege status
//   return vscp_binary_callback_reply(pdata, 0, VSCP_ERROR_SUCCESS, &has_priv, 1);
// }

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_test
//

int
vscp_binary_callback_test(const void *pdata, const uint8_t *arg, size_t len)
{
  if (NULL == pdata) {
    return VSCP_ERROR_PARAMETER;
  }

  if (len > 0 && arg != NULL) {
    ESP_LOG_BUFFER_HEX(TAG, arg, len);
  }

  // Echo back the test data as reply
  return vscp_binary_callback_reply(pdata, VSCP_BINARY_COMMAND_CODE_TEST, VSCP_ERROR_SUCCESS, arg, len);
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_wcyd
//

int
vscp_binary_callback_wcyd(const void *pdata, uint64_t *pwcyd)
{
  if (NULL == pdata || NULL == pwcyd) {
    return VSCP_ERROR_PARAMETER;
  }

  // Send reply with WCYD (What Can You Do) info - 8 bytes, big-endian
  pwcyd[0] = 0x00; // Reserved for future use, set to 0 for now
  pwcyd[1] = 0x00; // Reserved for future use, set to 0 for now
  pwcyd[2] = 0x00; // Reserved for future use, set to 0 for now
  pwcyd[3] = 0x00; // Reserved for future use, set to 0 for now
  pwcyd[4] = 0x00; // Reserved for future use, set to 0 for now
  pwcyd[5] = 0x00; // Reserved for future use, set to 0 for now
  pwcyd[6] = 0x00; // Reserved for future use, set to 0 for now
  pwcyd[7] = 0x00; // Reserved for future use, set to 0 for now

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_check_data
//

int
vscp_binary_callback_check_data(const void *pdata, uint32_t *pcount)
{
  if (NULL == pdata || NULL == pcount) {
    return VSCP_ERROR_PARAMETER;
  }

  *pcount = 0; // For this example, we have no events ready to be received
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_clrall
//

int
vscp_binary_callback_clrall(const void *pdata)
{
  if (NULL == pdata) {
    return VSCP_ERROR_PARAMETER;
  }

  // No input queue to clear

  // Send reply confirming all cleared
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_send_event
//

int
vscp_binary_callback_send_event(const void *pdata, const vscp_event_t *pev)
{
  if (NULL == pdata || NULL == pev) {
    return VSCP_ERROR_PARAMETER;
  }

  vscp_ws_connection_context_t *pctx = (vscp_ws_connection_context_t *) pdata;

  // Filter
  // if (!vscp_fwhlp_doLevel2FilterEx(pex, &pctx->filter)) {
  //   return VSCP_ERROR_SUCCESS; // Filter out == OK
  // }

  // Update send statistics
  pctx->stats.cntTransmitFrames++;
  pctx->stats.cntTransmitData += pev->sizeData;

  // Mark this event as coming from this interface
  // TODO pev->obid = pctx->sock;

  // Check for Level II event
  if (pev->vscp_class > 1024) {
    // can not send level II events on TWAI, so we just drop them in this example
  }
  // Check for proxy event
  else if (pev->vscp_class > 512) {
    ;
  }
  /*
    Level I event. If addressed to nodeid = 0
    and VSCP_CLASS1_PROTOCOL it is addressed to us
    and we should handle it. If not send event
    on the TWAI interface.
  */
  else {
    if ((VSCP_CLASS1_PROTOCOL == pev->vscp_class) && (0 == pev->GUID[15])) {
      ;
    }
    else {
      can4vscp_frame_t tx_msg;
      tx_msg.data_length_code = pev->sizeData;
      tx_msg.extd             = 1;
      tx_msg.identifier =
        pev->GUID[0] + (pev->vscp_type << 8) + (pev->vscp_class << 16) + (((pev->head >> 5) & 7) << 26);
      can4vscp_send(&tx_msg, portMAX_DELAY);
      ESP_LOGI(TAG, "Transmitted start command");
    }
  }

  // Write to send buffer of other interfaces
  for (int i = 0; i < MAX_TCP_CONNECTIONS; i++) {
    // if (pctx->sock != i) {
    //   vscp_event_ex_t *pnew = vscp_fwhlp_mkEventCopy(pex);
    //   if (NULL == pnew) {
    //     vscp_fwhlp_deleteEvent(&pnew);
    //     vscp_fwhlp_deleteEvent(&pex);
    //     return VSCP_ERROR_MEMORY;
    //   }
    //   else {
    //     if (!vscp_fifo_write(&gctx[i].fifoEventsOut, pnew)) {
    //       vscp_fwhlp_deleteEvent(&pnew);
    //       vscp_fwhlp_deleteEvent(&pex);
    //       gctx[i].statistics.cntOverruns++;
    //       return VSCP_ERROR_TRM_FULL;
    //     }
    //   }
    // }
  }

  // Send reply confirming event sent
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_get_event
//

int
vscp_binary_callback_get_event(const void *pdata, vscp_event_t *pev)
{
  if (NULL == pdata) {
    return VSCP_ERROR_PARAMETER;
  }

  // No event queue implemented; indicate empty
  return VSCP_ERROR_FIFO_EMPTY;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_get_eventex
//

int
vscp_binary_callback_get_eventex(const void *pdata, vscp_event_ex_t *pex)
{
  if (NULL == pdata) {
    return VSCP_ERROR_PARAMETER;
  }

  // No event queue implemented; indicate empty
  return VSCP_ERROR_FIFO_EMPTY;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_send_asyncevent
//

int
vscp_binary_callback_send_asyncevent(const void *pdata, vscp_event_t *pev)
{
  if (NULL == pdata || NULL == pev) {
    return VSCP_ERROR_PARAMETER;
  }

  // Send reply confirming async event sent
  return vscp_binary_callback_reply(pdata, 0, VSCP_ERROR_SUCCESS, NULL, 0);
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_quit
//

int
vscp_binary_callback_quit(const void *pdata)
{
  if (NULL == pdata) {
    return VSCP_ERROR_PARAMETER;
  }

  // Reply first, then close the websocket session.
  int rv = vscp_binary_callback_reply(pdata, VSCP_BINARY_COMMAND_CODE_QUIT, VSCP_ERROR_SUCCESS, NULL, 0);
  if (VSCP_ERROR_SUCCESS != rv) {
    return rv;
  }

  vscp_ws_connection_context_t *pctx = (vscp_ws_connection_context_t *) pdata;
  httpd_req_t *req                   = (httpd_req_t *) pctx->pdata;
  if (NULL == req) {
    return VSCP_ERROR_SUCCESS;
  }

  int fd = httpd_req_to_sockfd(req);
  if (ESP_OK != wss_close_client_fd(fd)) {
    ESP_LOGW(TAG, "QUIT: failed to close websocket fd=%d", fd);
    return VSCP_ERROR_INTERFACE;
  }

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_write_client
//

int
vscp_binary_callback_write_client(const void *pdata, const char *msg)
{
  if (NULL == pdata || NULL == msg) {
    return VSCP_ERROR_PARAMETER;
  }

  vscp_ws_connection_context_t *pctx = (vscp_ws_connection_context_t *) pdata;

  vscp_event_t ev = { 0 };
  ev.vscp_class   = VSCP_CLASS1_PROTOCOL;
  ev.vscp_type    = 0;
  memcpy(ev.GUID, pctx->guid, sizeof(ev.GUID));

  size_t msg_len = strlen(msg);
  if (msg_len > 8) {
    ESP_LOGW(TAG, "write_client: truncating payload from %u to 8 bytes for TWAI", (unsigned) msg_len);
    msg_len = 8;
  }

  ev.sizeData = msg_len;
  ev.pdata    = (uint8_t *) msg;

  can4vscp_frame_t tx_msg = { 0 };
  int rv                  = can4vscp_event_to_msg(&tx_msg, &ev);
  if (VSCP_ERROR_SUCCESS != rv) {
    return rv;
  }

  if (ESP_OK != can4vscp_send(&tx_msg, portMAX_DELAY)) {
    return VSCP_ERROR_INTERFACE;
  }

  pctx->stats.cntTransmitFrames++;
  pctx->stats.cntTransmitData += ev.sizeData;

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_get_interface_count
//

int
vscp_binary_callback_get_interface_count(const void *pdata, uint16_t *pcount)
{
  if (NULL == pdata || NULL == pcount) {
    return VSCP_ERROR_PARAMETER;
  }

  // There is only one interface
  pcount[0] = 0;
  pcount[1] = 1;

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_get_interface
//

int
vscp_binary_callback_get_interface(const void *pdata, uint16_t idx, vscp_interface_info_t *pifinfo)
{
  if (NULL == pdata || NULL == pifinfo) {
    return VSCP_ERROR_PARAMETER;
  }

  if (idx != 0) {
    return VSCP_ERROR_INDEX_OOB;
  }

  vscp_ws_connection_context_t *pctx = (vscp_ws_connection_context_t *) pdata;

  pifinfo->idx  = 0;
  pifinfo->type = VSCP_INTERFACE_TYPE_INTERNAL;
  memcpy(pifinfo->guid, pctx->guid, sizeof(pifinfo->guid));
  strncpy(pifinfo->description, "Web socket binary interface", sizeof(pifinfo->description) - 1);
  pifinfo->description[sizeof(pifinfo->description) - 1] = '\0';

  // Send reply with interface info (just confirm for now)
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_interface_open
//

int
vscp_binary_callback_interface_open(const void *pdata, uint16_t idx)
{
  if (NULL == pdata) {
    return VSCP_ERROR_PARAMETER;
  }

  if (idx != 0) {
    return VSCP_ERROR_INDEX_OOB;
  }

  // Send reply confirming interface opened
  return vscp_binary_callback_open(pdata);
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_interface_close
//

int
vscp_binary_callback_interface_close(const void *pdata, uint16_t idx)
{
  // Here we need pdata
  if (NULL == pdata) {
    return VSCP_ERROR_PARAMETER;
  }

  if (idx != 0) {
    return VSCP_ERROR_INDEX_OOB;
  }

  // Simulate the
  return vscp_binary_callback_close(pdata);
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_event_received
//

int
vscp_binary_callback_event_received(const void *pdata, const vscp_event_t *pev)
{
  // Check pointers
  if ((NULL == pdata) || (NULL == pev)) {
    return VSCP_ERROR_PARAMETER;
  }

  vscp_ws_connection_context_t *pctx = (vscp_ws_connection_context_t *) pdata;

  // Make twai message from event
  can4vscp_frame_t tx_msg = { 0 };
  int rv                  = can4vscp_event_to_msg(&tx_msg, pev);
  if (VSCP_ERROR_SUCCESS != rv) {
    return rv;
  }

  // Send on TWAI interface
  if (ESP_OK != can4vscp_send(&tx_msg, portMAX_DELAY)) {
    return VSCP_ERROR_INTERFACE;
  }

  // Update statistics
  pctx->stats.cntTransmitFrames++;
  pctx->stats.cntTransmitData += pev->sizeData;

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_disconnect_client
//

int
vscp_binary_callback_disconnect_client(const void *pdata)
{
  if (NULL == pdata) {
    return VSCP_ERROR_PARAMETER;
  }

  // Send reply confirming disconnect
  return vscp_binary_callback_reply(pdata, 0, VSCP_ERROR_SUCCESS, NULL, 0);
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_statistics
//

int
vscp_binary_callback_statistics(const void *pdata, vscp_statistics_t *pStatistics)
{
  if (NULL == pdata || NULL == pStatistics) {
    return VSCP_ERROR_PARAMETER;
  }

  vscp_ws_connection_context_t *pctx = (vscp_ws_connection_context_t *) pdata;
  memcpy(pStatistics, &pctx->stats, sizeof(vscp_statistics_t));

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_info
//

int
vscp_binary_callback_info(const void *pdata, vscp_status_t *pstatus)
{
  if (NULL == pdata || NULL == pstatus) {
    return VSCP_ERROR_PARAMETER;
  }

  // Get pointer to context
  vscp_ws_connection_context_t *pctx = (vscp_ws_connection_context_t *) pdata;
  memcpy(pstatus, &pctx->status, sizeof(vscp_status_t));

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_user_command
//

int
vscp_binary_callback_user_command(const void *pdata, uint16_t command, const uint8_t *parg, size_t len)
{
  if (NULL == pdata) {
    return VSCP_ERROR_PARAMETER;
  }

  // There is no user commnds
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_restart
//

int
vscp_binary_callback_restart(const void *pdata)
{
  if (NULL == pdata) {
    return VSCP_ERROR_PARAMETER;
  }

  // Send reply with correct command code before restarting.
  // Give the websocket stack a brief window to flush the frame.
  int rv = vscp_binary_callback_reply(pdata, VSCP_BINARY_COMMAND_CODE_RESTART, VSCP_ERROR_SUCCESS, NULL, 0);
  if (VSCP_ERROR_SUCCESS != rv) {
    return rv;
  }

  vTaskDelay(pdMS_TO_TICKS(100));

  // Schedule device restart
  esp_restart();
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_shutdown
//

int
vscp_binary_callback_shutdown(const void *pdata)
{
  if (NULL == pdata) {
    return VSCP_ERROR_PARAMETER;
  }

  // Send reply with correct command code
  vscp_binary_callback_reply(pdata, VSCP_BINARY_COMMAND_CODE_SHUTDOWN, VSCP_ERROR_SUCCESS, NULL, 0);

  // Note: Actual system shutdown would require additional implementation
  // For now, just return success after notifying the client
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_binary_callback_text
//

int
vscp_binary_callback_text(const void *pdata)
{
  // We do noting except resetting the binary flag in the context.

  // Check pointer
  if (NULL == pdata) {
    return VSCP_ERROR_PARAMETER;
  }

  // Set the binary flag in the context
  vscp_ws_connection_context_t *pctx = (vscp_ws_connection_context_t *) pdata;
  pctx->bBinary                      = false;

  return VSCP_ERROR_SUCCESS;
}
