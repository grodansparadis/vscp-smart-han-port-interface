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

/*!
  The VSCP Websocket Server (WS2) protocol is a JSON based protocol designed
  for communication between a VSCP device and a client over a WebSocket connection.
  The protocol allows clients to send commands to the device and receive responses,
  as well as receive events from the device in real-time.

  It is documented here
  https://grodansparadis.github.io/vscp-doc-spec/#/./vscp_websocket?id=ws2-description

  Packets (all JSON objects)
    Command:          { "type": "C|CMD|COMMAND", "command": "...", "args": {...} | null }
    Positive reply:   { "type": "+", "command": "...", "args": {...} | null }
    Negative reply:   { "type": "-", "command": "...", "errcode": n, "errstr": "..." }
    Event:            { "type": "E|EVENT", "event": { "vscpClass": n, ... } }

  On connect the server sends { "type": "+", "args": ["AUTH0", "<sid>"] }. The client
  logs in with AUTH, args { "iv": "<sid>", "crypto": "<hex>" } where crypto is
  AES-128-CBC("username:password") with the sid as IV and the device key. The iv
  must equal the sid of the connection.

  Only NOOP, CHALLENGE, AUTH, VERSION, COPYRIGHT and CLOSE are accepted before
  the client has authenticated.
 */

#ifndef __VSCP_WS2_H__
#define __VSCP_WS2_H__

#include "vscp-compiler.h"
#include "vscp-projdefs.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "vscp.h"
#include "vscp-firmware-helper.h"

#include "vscp-ws-common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VSCP_WS2_PROTOCOL_VERSION         1
#define VSCP_WS2_PROTOCOL_MINOR_VERSION   0
#define VSCP_WS2_PROTOCOL_RELEASE_VERSION 0
#define VSCP_WS2_PROTOCOL_BUILD_VERSION   0

#define VSCP_WS2_SID_SIZE 16

// Largest text frame accepted from a client. An event with 512 data bytes
// written as a JSON array needs a bit more than 2 KB.
#ifndef VSCP_WS2_MAX_PACKET_SIZE
#define VSCP_WS2_MAX_PACKET_SIZE 4096
#endif

// Largest AUTH ciphertext in bytes (whole AES blocks). Enough for
// a 32 character username, ':' and a 32 character password.
#define VSCP_WS2_MAX_CRYPTO_BIN_LEN 128

// The connection is closed after this many failed AUTH attempts
#ifndef VSCP_WS2_MAX_AUTH_FAILURES
#define VSCP_WS2_MAX_AUTH_FAILURES 3
#endif

// Privilege levels needed for commands (see the ws2 spec command table)
#define VSCP_WS2_PRIV_CLRQUEUE   1
#define VSCP_WS2_PRIV_SETFILTER  6
#define VSCP_WS2_PRIV_SEND_EVENT 6

// Error codes, same as for ws1
#define VSCP_WS2_ERROR_NONE                0
#define VSCP_WS2_ERROR_SYNTAX              1
#define VSCP_WS2_ERROR_UNKNOWN_COMMAND     2
#define VSCP_WS2_ERROR_TX_BUFFER_FULL      3
#define VSCP_WS2_ERROR_MEMORY              4
#define VSCP_WS2_ERROR_NOT_AUTHORIZED      5
#define VSCP_WS2_ERROR_NOT_AUTHORIZED_SEND 6
#define VSCP_WS2_ERROR_NOT_ALLOWED         7
#define VSCP_WS2_ERROR_PARSE               8
#define VSCP_WS2_ERROR_UNKNOWN_TYPE        9
#define VSCP_WS2_ERROR_GENERAL             10

#define VSCP_WS2_STR_ERROR_SYNTAX                    "Syntax error."
#define VSCP_WS2_STR_ERROR_UNKNOWN_COMMAND           "Unknown command."
#define VSCP_WS2_STR_ERROR_TX_BUFFER_FULL            "Transmit buffer full."
#define VSCP_WS2_STR_ERROR_MEMORY_ALLOCATION         "Having problems to allocate memory."
#define VSCP_WS2_STR_ERROR_NOT_AUTHORISED            "Not authorised."
#define VSCP_WS2_STR_ERROR_NOT_ALLOWED_TO_SEND_EVENT "Not allowed to send event."
#define VSCP_WS2_STR_ERROR_NOT_ALLOWED_TO_DO_THAT    "Not allowed to do that (check privileges)"
#define VSCP_WS2_STR_ERROR_PARSE_FORMAT              "Parse error, invalid format."
#define VSCP_WS2_STR_ERROR_UNKNOWN_TYPE              "Unknown type, only know 'COMMAND' and 'EVENT'."
#define VSCP_WS2_STR_ERROR_GENERAL                   "Exception or other general error."

/*!
  @brief Initialise a ws2 connection context and send the AUTH0 challenge.

  @param pctx Connection context to initialise.
  @param pdata Platform data (the httpd request) saved in the context.
  @return VSCP_ERROR_SUCCESS if all is OK, errorcode otherwise.
*/
int
vscp_ws2_init(vscp_ws_connection_context_t *pctx, void *pdata);

/*!
  @brief Handle one text frame received from a ws2 client.

  @param pctx Connection context.
  @param pframe Frame data (does not need to be null terminated).
  @param len Length of frame data.
  @return VSCP_ERROR_SUCCESS if the frame was handled (also when a negative
          reply was sent), VSCP_ERROR_INVALID_PERMISSION if the connection
          should be closed, other errorcode on failure.
*/
int
vscp_ws2_handle_request(vscp_ws_connection_context_t *pctx, const char *pframe, size_t len);

/*!
  @brief Check if an event should be delivered to a ws2 client.

  The client must be authenticated, have an open channel and the event must
  pass the client's filter.

  @param pctx Connection context.
  @param pex Event to check.
  @return true if the event should be sent.
*/
bool
vscp_ws2_should_send_event(const vscp_ws_connection_context_t *pctx, const vscp_event_ex_t *pex);

/*!
  @brief Format an event as a ws2 event object.

  @param pex Event to format.
  @return Null terminated JSON string allocated on the heap that the caller must
          release with free(), or NULL on failure.
*/
char *
vscp_ws2_format_event(const vscp_event_ex_t *pex);

// ----------------------------------------------------------------------------
//                     Callbacks (implemented in callbacks-ws2.c)
// ----------------------------------------------------------------------------

/*!
  @brief Send a null terminated text frame to the client.
*/
int
vscp_ws2_callback_reply(vscp_ws_connection_context_t *pctx, const char *msg);

/*!
  @brief Fill sid with random bytes from a cryptographically secure source.
*/
int
vscp_ws2_callback_generate_sid(vscp_ws_connection_context_t *pctx, uint8_t *sid, size_t size);

/*!
  @brief Get the 16 byte AES-128 key used to decrypt AUTH credentials.
*/
const uint8_t *
vscp_ws2_callback_get_key(vscp_ws_connection_context_t *pctx);

/*!
  @brief Get the 16 byte GUID used for events from this connection that have no GUID.
*/
const uint8_t *
vscp_ws2_callback_get_guid(vscp_ws_connection_context_t *pctx);

/*!
  @brief Validate decrypted credentials.

  On success the callback fills in pctx->user (username, privilege level).

  @return VSCP_ERROR_SUCCESS if valid, VSCP_ERROR_INVALID_PERMISSION if not.
*/
int
vscp_ws2_callback_validate_user(vscp_ws_connection_context_t *pctx, const char *username, const char *password);

/*!
  @brief Deliver an event sent by the client to the rest of the system.
*/
int
vscp_ws2_callback_event(vscp_ws_connection_context_t *pctx, const vscp_event_ex_t *pex);

/*!
  @brief Get the copyright text reported by the COPYRIGHT command.
*/
const char *
vscp_ws2_callback_get_copyright(vscp_ws_connection_context_t *pctx);

#ifdef __cplusplus
}
#endif

#endif /* __VSCP_WS2_H__ */
