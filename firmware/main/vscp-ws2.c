/* ******************************************************************************
 * VSCP (Very Simple Control Protocol)
 * http://www.vscp.org
 *
 * The MIT License (MIT)
 *
 * Copyright (C) 2000-2026 Ake Hedman,
 * The VSCP Project <info@grodansparadis.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 *  This file is part of VSCP - Very Simple Control Protocol
 *  http://www.vscp.org
 *
 * ******************************************************************************
 */

#include "vscp-compiler.h"
#include "vscp-projdefs.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <esp_log.h>
#include <cJSON.h>

#include "vscp.h"
#include "vscp-firmware-helper.h"

#include "main.h"
#include "vscp-ws2.h"

#define TAG "ws2"

// Longest command name accepted ("CLRQUEUE", "COPYRIGHT", ...)
#define VSCP_WS2_MAX_COMMAND_LEN 16

///////////////////////////////////////////////////////////////////////////////
// Helpers
///////////////////////////////////////////////////////////////////////////////

static void
ws2_sid_to_hex(char *out, size_t outlen, const uint8_t *sid)
{
  vscp_fwhlp_bin2hex(out, outlen, sid, VSCP_WS2_SID_SIZE);
}

static bool
ws2_is_hex(const char *str, size_t len)
{
  for (size_t i = 0; i < len; i++) {
    if (!isxdigit((unsigned char) str[i])) {
      return false;
    }
  }
  return true;
}

//
// Serialise and send a JSON object. Takes ownership of root.
//

static int
ws2_send_json(vscp_ws_connection_context_t *pctx, cJSON *root)
{
  if (NULL == root) {
    return VSCP_ERROR_MEMORY;
  }

  char *str = cJSON_PrintUnformatted(root);
  cJSON_Delete(root);
  if (NULL == str) {
    return VSCP_ERROR_MEMORY;
  }

  int rv = vscp_ws2_callback_reply(pctx, str);
  cJSON_free(str);
  return rv;
}

//
// Positive reply. Takes ownership of args (NULL gives "args": null).
//

static int
ws2_reply_positive(vscp_ws_connection_context_t *pctx, const char *command, cJSON *args)
{
  cJSON *root = cJSON_CreateObject();
  if (NULL == root) {
    cJSON_Delete(args);
    return VSCP_ERROR_MEMORY;
  }

  cJSON_AddStringToObject(root, "type", "+");
  cJSON_AddStringToObject(root, "command", command);
  cJSON_AddItemToObject(root, "args", (NULL != args) ? args : cJSON_CreateNull());

  return ws2_send_json(pctx, root);
}

static int
ws2_reply_negative(vscp_ws_connection_context_t *pctx, const char *command, int errcode, const char *errstr)
{
  cJSON *root = cJSON_CreateObject();
  if (NULL == root) {
    return VSCP_ERROR_MEMORY;
  }

  cJSON_AddStringToObject(root, "type", "-");
  cJSON_AddStringToObject(root, "command", command);
  cJSON_AddNumberToObject(root, "errcode", errcode);
  cJSON_AddStringToObject(root, "errstr", errstr);

  return ws2_send_json(pctx, root);
}

//
// Read an optional non-negative integer member. Returns false if the member
// exists but is not an integer in [0, max]. *pval is left unchanged if the
// member is missing.
//

static bool
ws2_get_uint(const cJSON *obj, const char *name, uint32_t max, uint32_t *pval, bool *pfound)
{
  const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, name);
  if (NULL != pfound) {
    *pfound = (NULL != item);
  }
  if (NULL == item) {
    return true;
  }
  if (!cJSON_IsNumber(item)) {
    return false;
  }

  double d = item->valuedouble;
  if ((d < 0) || (d > (double) max) || (d != (double) (uint32_t) d)) {
    return false;
  }

  *pval = (uint32_t) d;
  return true;
}

//
// Read an optional GUID string member. Returns false if the member exists
// but is not a valid GUID string.
//

static bool
ws2_get_guid(const cJSON *obj, const char *name, uint8_t *guid)
{
  const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, name);
  if (NULL == item) {
    return true;
  }
  if (!cJSON_IsString(item) || (NULL == item->valuestring) || (strlen(item->valuestring) > 64)) {
    return false;
  }

  return (VSCP_ERROR_SUCCESS == vscp_fwhlp_parseGuid(guid, item->valuestring, NULL));
}

//
// Parse "YYYY-MM-DDTHH:MM:SS" with an optional trailing "Z"
//

static bool
ws2_parse_datetime(const char *str, vscp_event_ex_t *pex)
{
  unsigned int year, month, day, hour, minute, second;
  int consumed = 0;

  if (6 != sscanf(str, "%4u-%2u-%2uT%2u:%2u:%2u%n", &year, &month, &day, &hour, &minute, &second, &consumed)) {
    return false;
  }

  const char *rest = str + consumed;
  if (('\0' != *rest) && (0 != strcmp(rest, "Z"))) {
    return false;
  }

  if ((month < 1) || (month > 12) || (day < 1) || (day > 31) || (hour > 23) || (minute > 59) || (second > 59)) {
    return false;
  }

  pex->year   = (uint16_t) year;
  pex->month  = (uint8_t) month;
  pex->day    = (uint8_t) day;
  pex->hour   = (uint8_t) hour;
  pex->minute = (uint8_t) minute;
  pex->second = (uint8_t) second;
  return true;
}

//
// Parse a ws2 event object into pex. Every field is type and range checked.
//

static bool
ws2_parse_event(const cJSON *jev, vscp_event_ex_t *pex)
{
  uint32_t val;
  bool found;

  if (!cJSON_IsObject(jev)) {
    return false;
  }

  memset(pex, 0, sizeof(vscp_event_ex_t));

  val = 0;
  if (!ws2_get_uint(jev, "vscpHead", 0xffff, &val, NULL)) {
    return false;
  }
  // Date/time is stored as separate fields below, so use the original frame format
  pex->head = (uint16_t) (val & ~VSCP_HEADER16_FRAME_VERSION_MASK);

  if (!ws2_get_uint(jev, "vscpClass", 0xffff, &val, &found) || !found) {
    return false;
  }
  pex->vscp_class = (uint16_t) val;

  if (!ws2_get_uint(jev, "vscpType", 0xffff, &val, &found) || !found) {
    return false;
  }
  pex->vscp_type = (uint16_t) val;

  val = 0;
  if (!ws2_get_uint(jev, "vscpTimeStamp", 0xffffffff, &val, NULL)) {
    return false;
  }
  pex->timestamp = val;

  const cJSON *jdt = cJSON_GetObjectItemCaseSensitive(jev, "vscpDateTime");
  if (NULL != jdt) {
    if (!cJSON_IsString(jdt) || (NULL == jdt->valuestring)) {
      return false;
    }
    if (('\0' != jdt->valuestring[0]) && !ws2_parse_datetime(jdt->valuestring, pex)) {
      return false;
    }
  }

  if (!ws2_get_guid(jev, "vscpGuid", pex->GUID)) {
    return false;
  }

  const cJSON *jdata = cJSON_GetObjectItemCaseSensitive(jev, "vscpData");
  if ((NULL != jdata) && !cJSON_IsNull(jdata)) {
    if (!cJSON_IsArray(jdata)) {
      return false;
    }

    int size = cJSON_GetArraySize(jdata);
    // Level I events carry at most 8 data bytes, Level II at most VSCP_MAX_DATA
    int max = (pex->vscp_class < 512) ? 8 : VSCP_MAX_DATA;
    if ((size < 0) || (size > max)) {
      return false;
    }

    int i = 0;
    const cJSON *jbyte;
    cJSON_ArrayForEach(jbyte, jdata)
    {
      if (!cJSON_IsNumber(jbyte)) {
        return false;
      }
      double d = jbyte->valuedouble;
      if ((d < 0) || (d > 255) || (d != (double) (uint8_t) d)) {
        return false;
      }
      pex->data[i++] = (uint8_t) d;
    }
    pex->sizeData = (uint16_t) i;
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////
// Command handlers
///////////////////////////////////////////////////////////////////////////////

static int
ws2_cmd_auth(vscp_ws_connection_context_t *pctx, const cJSON *args)
{
  uint8_t iv[VSCP_WS2_SID_SIZE];
  uint8_t crypto[1 + VSCP_WS2_MAX_CRYPTO_BIN_LEN]    = { 0 };
  uint8_t plain[1 + VSCP_WS2_MAX_CRYPTO_BIN_LEN + 1] = { 0 };
  bool bValid                                        = false;

  // A new AUTH always drops a previous login
  pctx->bAuthenticated = false;
  pctx->bOpen          = false;
  memset(&pctx->user, 0, sizeof(pctx->user));

  const cJSON *jiv     = cJSON_GetObjectItemCaseSensitive(args, "iv");
  const cJSON *jcrypto = cJSON_GetObjectItemCaseSensitive(args, "crypto");

  if (!cJSON_IsString(jiv) || !cJSON_IsString(jcrypto) || (NULL == jiv->valuestring) ||
      (NULL == jcrypto->valuestring)) {
    ESP_LOGE(TAG, "AUTH: missing iv or crypto");
    goto done;
  }

  const char *iv_hex     = jiv->valuestring;
  const char *crypto_hex = jcrypto->valuestring;
  size_t iv_hex_len      = strnlen(iv_hex, 2 * VSCP_WS2_SID_SIZE + 1);
  size_t crypto_hex_len  = strnlen(crypto_hex, 2 * VSCP_WS2_MAX_CRYPTO_BIN_LEN + 1);

  // The iv must be the challenge sent to this connection, so a login captured
  // on another connection can not be replayed
  if ((2 * VSCP_WS2_SID_SIZE != iv_hex_len) || !ws2_is_hex(iv_hex, iv_hex_len) ||
      (VSCP_WS2_SID_SIZE != vscp_fwhlp_hex2bin(iv, sizeof(iv), iv_hex)) ||
      (0 != memcmp(iv, pctx->sid, VSCP_WS2_SID_SIZE))) {
    ESP_LOGE(TAG, "AUTH: iv does not match the session challenge");
    goto done;
  }

  // Ciphertext must be whole AES blocks
  if ((0 == crypto_hex_len) || (crypto_hex_len > 2 * VSCP_WS2_MAX_CRYPTO_BIN_LEN) || (0 != (crypto_hex_len % 32)) ||
      !ws2_is_hex(crypto_hex, crypto_hex_len)) {
    ESP_LOGE(TAG, "AUTH: invalid crypto length or format");
    goto done;
  }

  size_t crypto_len = crypto_hex_len / 2;
  if ((int) crypto_len != vscp_fwhlp_hex2bin(crypto + 1, sizeof(crypto) - 1, crypto_hex)) {
    ESP_LOGE(TAG, "AUTH: failed to convert crypto from hex");
    goto done;
  }

  // decryptFrame keeps byte 0 as an unencrypted frame type byte
  if (VSCP_ERROR_SUCCESS != vscp_fwhlp_decryptFrame(plain,
                                                    crypto,
                                                    crypto_len + 1,
                                                    vscp_ws2_callback_get_key(pctx),
                                                    pctx->sid,
                                                    VSCP_HLO_ENCRYPTION_AES128)) {
    ESP_LOGE(TAG, "AUTH: failed to decrypt credentials");
    goto done;
  }

  // Remove PKCS#7 padding if present, otherwise the text ends at the first zero byte
  uint8_t *ptext  = plain + 1;
  size_t text_len = crypto_len;
  uint8_t pad     = ptext[text_len - 1];
  if ((pad >= 1) && (pad <= 16)) {
    bool bPkcs7 = true;
    for (size_t i = text_len - pad; i < text_len; i++) {
      if (ptext[i] != pad) {
        bPkcs7 = false;
        break;
      }
    }
    if (bPkcs7) {
      text_len -= pad;
    }
  }
  ptext[text_len] = '\0';

  // Credentials are "username:password"
  char *username = (char *) ptext;
  char *password = strchr(username, ':');
  if (NULL == password) {
    ESP_LOGE(TAG, "AUTH: malformed credentials, expected 'username:password'");
    goto done;
  }
  *password++ = '\0';

  if (('\0' == *username) || ('\0' == *password)) {
    ESP_LOGE(TAG, "AUTH: empty username or password");
    goto done;
  }

  bValid = (VSCP_ERROR_SUCCESS == vscp_ws2_callback_validate_user(pctx, username, password));

done:
  secure_zero(plain, sizeof(plain));
  secure_zero(crypto, sizeof(crypto));

  if (bValid) {
    pctx->bAuthenticated = true;
    pctx->authFailures   = 0;
    ESP_LOGI(TAG, "AUTH: user '%s' authenticated", pctx->user.username);
    return ws2_reply_positive(pctx, "AUTH", NULL);
  }

  memset(&pctx->user, 0, sizeof(pctx->user));
  if (pctx->authFailures < UINT8_MAX) {
    pctx->authFailures++;
  }
  ESP_LOGE(TAG, "AUTH: failed (%u of %u attempts)", pctx->authFailures, VSCP_WS2_MAX_AUTH_FAILURES);
  ws2_reply_negative(pctx, "AUTH", VSCP_WS2_ERROR_NOT_AUTHORIZED, VSCP_WS2_STR_ERROR_NOT_AUTHORISED);

  if (pctx->authFailures >= VSCP_WS2_MAX_AUTH_FAILURES) {
    return VSCP_ERROR_INVALID_PERMISSION; // Caller closes the connection
  }
  return VSCP_ERROR_SUCCESS;
}

static int
ws2_cmd_setfilter(vscp_ws_connection_context_t *pctx, const char *command, const cJSON *args)
{
  vscpEventFilter filter;
  uint32_t val;

  memset(&filter, 0, sizeof(filter));

  if (!cJSON_IsObject(args)) {
    return ws2_reply_negative(pctx, command, VSCP_WS2_ERROR_SYNTAX, VSCP_WS2_STR_ERROR_SYNTAX);
  }

  // clang-format off
  val = 0; if (!ws2_get_uint(args, "mask_priority", 7, &val, NULL)) goto syntax;     filter.mask_priority   = (uint8_t) val;
  val = 0; if (!ws2_get_uint(args, "filter_priority", 7, &val, NULL)) goto syntax;   filter.filter_priority = (uint8_t) val;
  val = 0; if (!ws2_get_uint(args, "mask_class", 0xffff, &val, NULL)) goto syntax;   filter.mask_class      = (uint16_t) val;
  val = 0; if (!ws2_get_uint(args, "filter_class", 0xffff, &val, NULL)) goto syntax; filter.filter_class    = (uint16_t) val;
  val = 0; if (!ws2_get_uint(args, "mask_type", 0xffff, &val, NULL)) goto syntax;    filter.mask_type       = (uint16_t) val;
  val = 0; if (!ws2_get_uint(args, "filter_type", 0xffff, &val, NULL)) goto syntax;  filter.filter_type     = (uint16_t) val;
  // clang-format on
  if (!ws2_get_guid(args, "mask_guid", filter.mask_GUID) || !ws2_get_guid(args, "filter_guid", filter.filter_GUID)) {
    goto syntax;
  }

  memcpy(&pctx->filter, &filter, sizeof(filter));
  return ws2_reply_positive(pctx, command, NULL);

syntax:
  return ws2_reply_negative(pctx, command, VSCP_WS2_ERROR_SYNTAX, VSCP_WS2_STR_ERROR_SYNTAX);
}

static int
ws2_handle_command(vscp_ws_connection_context_t *pctx, const char *command, const cJSON *args)
{
  char sid_hex[2 * VSCP_WS2_SID_SIZE + 1];

  ESP_LOGD(TAG, "Command: %s", command);

  // Commands allowed without authentication
  if (0 == strcmp(command, "NOOP")) {
    return ws2_reply_positive(pctx, command, NULL);
  }

  if (0 == strcmp(command, "CHALLENGE")) {
    ws2_sid_to_hex(sid_hex, sizeof(sid_hex), pctx->sid);
    cJSON *jargs = cJSON_CreateObject();
    if (NULL != jargs) {
      cJSON_AddStringToObject(jargs, "sid", sid_hex);
    }
    return ws2_reply_positive(pctx, command, jargs);
  }

  if (0 == strcmp(command, "AUTH")) {
    if (!cJSON_IsObject(args)) {
      return ws2_reply_negative(pctx, command, VSCP_WS2_ERROR_SYNTAX, VSCP_WS2_STR_ERROR_SYNTAX);
    }
    return ws2_cmd_auth(pctx, args);
  }

  if ((0 == strcmp(command, "VERSION")) || (0 == strcmp(command, "VER"))) {
    char ver[32];
    snprintf(ver,
             sizeof(ver),
             "%d.%d.%d-%d",
             VSCP_WS2_PROTOCOL_VERSION,
             VSCP_WS2_PROTOCOL_MINOR_VERSION,
             VSCP_WS2_PROTOCOL_RELEASE_VERSION,
             VSCP_WS2_PROTOCOL_BUILD_VERSION);
    cJSON *jargs = cJSON_CreateObject();
    if (NULL != jargs) {
      cJSON_AddStringToObject(jargs, "version", ver);
    }
    return ws2_reply_positive(pctx, command, jargs);
  }

  if (0 == strcmp(command, "COPYRIGHT")) {
    cJSON *jargs = cJSON_CreateObject();
    if (NULL != jargs) {
      cJSON_AddStringToObject(jargs, "copyright", vscp_ws2_callback_get_copyright(pctx));
    }
    return ws2_reply_positive(pctx, command, jargs);
  }

  if (0 == strcmp(command, "CLOSE")) {
    pctx->bOpen = false;
    return ws2_reply_positive(pctx, command, NULL);
  }

  // Everything below needs an authenticated client
  bool bKnown = (0 == strcmp(command, "OPEN")) || (0 == strcmp(command, "SETFILTER")) ||
                (0 == strcmp(command, "SF")) || (0 == strcmp(command, "CLRQUEUE")) || (0 == strcmp(command, "CLRQ"));
  if (!bKnown) {
    return ws2_reply_negative(pctx, command, VSCP_WS2_ERROR_UNKNOWN_COMMAND, VSCP_WS2_STR_ERROR_UNKNOWN_COMMAND);
  }

  if (!pctx->bAuthenticated) {
    return ws2_reply_negative(pctx, command, VSCP_WS2_ERROR_NOT_AUTHORIZED, VSCP_WS2_STR_ERROR_NOT_AUTHORISED);
  }

  if (0 == strcmp(command, "OPEN")) {
    pctx->bOpen = true;
    return ws2_reply_positive(pctx, command, NULL);
  }

  if ((0 == strcmp(command, "SETFILTER")) || (0 == strcmp(command, "SF"))) {
    if (pctx->user.privlevel < VSCP_WS2_PRIV_SETFILTER) {
      return ws2_reply_negative(pctx, command, VSCP_WS2_ERROR_NOT_ALLOWED, VSCP_WS2_STR_ERROR_NOT_ALLOWED_TO_DO_THAT);
    }
    return ws2_cmd_setfilter(pctx, command, args);
  }

  // CLRQUEUE / CLRQ: events are sent to the client as they arrive, so there is
  // no per-client queue to clear
  if (pctx->user.privlevel < VSCP_WS2_PRIV_CLRQUEUE) {
    return ws2_reply_negative(pctx, command, VSCP_WS2_ERROR_NOT_ALLOWED, VSCP_WS2_STR_ERROR_NOT_ALLOWED_TO_DO_THAT);
  }
  return ws2_reply_positive(pctx, command, NULL);
}

static int
ws2_handle_event(vscp_ws_connection_context_t *pctx, const cJSON *jev)
{
  vscp_event_ex_t ex;

  if (!pctx->bAuthenticated) {
    return ws2_reply_negative(pctx, "EVENT", VSCP_WS2_ERROR_NOT_AUTHORIZED, VSCP_WS2_STR_ERROR_NOT_AUTHORISED);
  }

  if (!pctx->bOpen || (pctx->user.privlevel < VSCP_WS2_PRIV_SEND_EVENT)) {
    return ws2_reply_negative(pctx,
                              "EVENT",
                              VSCP_WS2_ERROR_NOT_AUTHORIZED_SEND,
                              VSCP_WS2_STR_ERROR_NOT_ALLOWED_TO_SEND_EVENT);
  }

  if (!ws2_parse_event(jev, &ex)) {
    return ws2_reply_negative(pctx, "EVENT", VSCP_WS2_ERROR_PARSE, VSCP_WS2_STR_ERROR_PARSE_FORMAT);
  }

  // Events without a GUID get the GUID of the interface
  static const uint8_t zero_guid[16] = { 0 };
  if (0 == memcmp(ex.GUID, zero_guid, 16)) {
    memcpy(ex.GUID, vscp_ws2_callback_get_guid(pctx), 16);
  }

  if (VSCP_ERROR_SUCCESS != vscp_ws2_callback_event(pctx, &ex)) {
    return ws2_reply_negative(pctx, "EVENT", VSCP_WS2_ERROR_TX_BUFFER_FULL, VSCP_WS2_STR_ERROR_TX_BUFFER_FULL);
  }

  return ws2_reply_positive(pctx, "EVENT", NULL);
}

///////////////////////////////////////////////////////////////////////////////
// vscp_ws2_init
//

int
vscp_ws2_init(vscp_ws_connection_context_t *pctx, void *pdata)
{
  int rv;
  char sid_hex[2 * VSCP_WS2_SID_SIZE + 1];

  if (NULL == pctx) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  memset(pctx, 0, sizeof(vscp_ws_connection_context_t));
  pctx->pdata    = pdata;
  pctx->protocol = VSCP_WS2_PROTOCOL;
  memcpy(pctx->guid, vscp_ws2_callback_get_guid(pctx), 16);

  if (VSCP_ERROR_SUCCESS != (rv = vscp_ws2_callback_generate_sid(pctx, pctx->sid, sizeof(pctx->sid)))) {
    ESP_LOGE(TAG, "Failed to generate SID rv=%d", rv);
    return rv;
  }

  // Send the AUTH0 challenge: { "type": "+", "args": ["AUTH0", "<sid>"] }
  ws2_sid_to_hex(sid_hex, sizeof(sid_hex), pctx->sid);

  cJSON *root = cJSON_CreateObject();
  if (NULL == root) {
    return VSCP_ERROR_MEMORY;
  }
  cJSON_AddStringToObject(root, "type", "+");
  cJSON *jargs = cJSON_AddArrayToObject(root, "args");
  if (NULL != jargs) {
    cJSON_AddItemToArray(jargs, cJSON_CreateString("AUTH0"));
    cJSON_AddItemToArray(jargs, cJSON_CreateString(sid_hex));
  }

  return ws2_send_json(pctx, root);
}

///////////////////////////////////////////////////////////////////////////////
// vscp_ws2_handle_request
//

int
vscp_ws2_handle_request(vscp_ws_connection_context_t *pctx, const char *pframe, size_t len)
{
  int rv = VSCP_ERROR_SUCCESS;
  char command[VSCP_WS2_MAX_COMMAND_LEN + 1];

  if ((NULL == pctx) || (NULL == pframe)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  if ((0 == len) || (len > VSCP_WS2_MAX_PACKET_SIZE)) {
    return ws2_reply_negative(pctx, "?", VSCP_WS2_ERROR_PARSE, VSCP_WS2_STR_ERROR_PARSE_FORMAT);
  }

  cJSON *root = cJSON_ParseWithLength(pframe, len);
  if (!cJSON_IsObject(root)) {
    cJSON_Delete(root);
    return ws2_reply_negative(pctx, "?", VSCP_WS2_ERROR_PARSE, VSCP_WS2_STR_ERROR_PARSE_FORMAT);
  }

  const cJSON *jtype = cJSON_GetObjectItemCaseSensitive(root, "type");
  if (!cJSON_IsString(jtype) || (NULL == jtype->valuestring)) {
    rv = ws2_reply_negative(pctx, "?", VSCP_WS2_ERROR_PARSE, VSCP_WS2_STR_ERROR_PARSE_FORMAT);
    goto done;
  }

  const char *type = jtype->valuestring;

  if ((0 == strcasecmp(type, "C")) || (0 == strcasecmp(type, "CMD")) || (0 == strcasecmp(type, "COMMAND"))) {

    const cJSON *jcmd = cJSON_GetObjectItemCaseSensitive(root, "command");
    if (!cJSON_IsString(jcmd) || (NULL == jcmd->valuestring)) {
      rv = ws2_reply_negative(pctx, "?", VSCP_WS2_ERROR_SYNTAX, VSCP_WS2_STR_ERROR_SYNTAX);
      goto done;
    }

    // Upper case copy of the command name. Over-long names are unknown commands.
    size_t cmdlen = strnlen(jcmd->valuestring, VSCP_WS2_MAX_COMMAND_LEN + 1);
    if ((0 == cmdlen) || (cmdlen > VSCP_WS2_MAX_COMMAND_LEN)) {
      rv = ws2_reply_negative(pctx, "?", VSCP_WS2_ERROR_UNKNOWN_COMMAND, VSCP_WS2_STR_ERROR_UNKNOWN_COMMAND);
      goto done;
    }
    for (size_t i = 0; i < cmdlen; i++) {
      command[i] = (char) toupper((unsigned char) jcmd->valuestring[i]);
    }
    command[cmdlen] = '\0';

    rv = ws2_handle_command(pctx, command, cJSON_GetObjectItemCaseSensitive(root, "args"));
  }
  else if ((0 == strcasecmp(type, "E")) || (0 == strcasecmp(type, "EVENT"))) {
    rv = ws2_handle_event(pctx, cJSON_GetObjectItemCaseSensitive(root, "event"));
  }
  else {
    rv = ws2_reply_negative(pctx, "?", VSCP_WS2_ERROR_UNKNOWN_TYPE, VSCP_WS2_STR_ERROR_UNKNOWN_TYPE);
  }

done:
  cJSON_Delete(root);
  return rv;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_ws2_should_send_event
//

bool
vscp_ws2_should_send_event(const vscp_ws_connection_context_t *pctx, const vscp_event_ex_t *pex)
{
  if ((NULL == pctx) || (NULL == pex)) {
    return false;
  }

  if ((VSCP_WS2_PROTOCOL != pctx->protocol) || !pctx->bAuthenticated || !pctx->bOpen) {
    return false;
  }

  return vscp_fwhlp_doLevel2FilterEx(pex, &pctx->filter) ? true : false;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_ws2_format_event
//

char *
vscp_ws2_format_event(const vscp_event_ex_t *pex)
{
  char str[64];
  unsigned int year, month, day, hour, minute, second;
  uint32_t timestamp;

  if (NULL == pex) {
    return NULL;
  }

  if ((pex->head & VSCP_HEADER16_FRAME_VERSION_MASK) == VSCP_HEADER16_FRAME_VERSION_UNIX_NS) {
    time_t secs = (time_t) (pex->timestamp_ns / 1000000000ULL);
    struct tm tm;
    gmtime_r(&secs, &tm);
    year      = (unsigned int) tm.tm_year + 1900;
    month     = (unsigned int) tm.tm_mon + 1;
    day       = (unsigned int) tm.tm_mday;
    hour      = (unsigned int) tm.tm_hour;
    minute    = (unsigned int) tm.tm_min;
    second    = (unsigned int) tm.tm_sec;
    timestamp = (uint32_t) (pex->timestamp_ns / 1000ULL); // Microseconds, wraps like the original field
  }
  else {
    year      = pex->year;
    month     = pex->month;
    day       = pex->day;
    hour      = pex->hour;
    minute    = pex->minute;
    second    = pex->second;
    timestamp = pex->timestamp;
  }

  cJSON *root = cJSON_CreateObject();
  if (NULL == root) {
    return NULL;
  }
  cJSON_AddStringToObject(root, "type", "EVENT");

  cJSON *jev = cJSON_AddObjectToObject(root, "event");
  if (NULL == jev) {
    cJSON_Delete(root);
    return NULL;
  }

  cJSON_AddNumberToObject(jev, "vscpHead", pex->head);
  cJSON_AddNumberToObject(jev, "vscpObId", pex->obid);
  snprintf(str, sizeof(str), "%04u-%02u-%02uT%02u:%02u:%02uZ", year, month, day, hour, minute, second);
  cJSON_AddStringToObject(jev, "vscpDateTime", str);
  cJSON_AddNumberToObject(jev, "vscpTimeStamp", timestamp);
  cJSON_AddNumberToObject(jev, "vscpClass", pex->vscp_class);
  cJSON_AddNumberToObject(jev, "vscpType", pex->vscp_type);
  vscp_fwhlp_writeGuidToString(str, pex->GUID);
  cJSON_AddStringToObject(jev, "vscpGuid", str);

  cJSON *jdata = cJSON_AddArrayToObject(jev, "vscpData");
  if (NULL != jdata) {
    uint16_t size = (pex->sizeData <= VSCP_MAX_DATA) ? pex->sizeData : VSCP_MAX_DATA;
    for (uint16_t i = 0; i < size; i++) {
      cJSON_AddItemToArray(jdata, cJSON_CreateNumber(pex->data[i]));
    }
  }

  char *json = cJSON_PrintUnformatted(root);
  cJSON_Delete(root);
  if (NULL == json) {
    return NULL;
  }

  // Hand back a malloc'ed copy so the caller can use free()
  char *out = strdup(json);
  cJSON_free(json);
  return out;
}