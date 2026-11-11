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
 *  This file contains common definitions and functions for the VSCP Websocket Server (WS1 and WS2)
 *  protocol implementations.
 *
 * ******************************************************************************
 */

#ifndef __VSCP_WS_COMMON_H__
#define __VSCP_WS_COMMON_H__

// ws1 is 0 and ws2 is 1, this is used in the session context to determine which protocol
// is being used for a given connection
#define VSCP_WS1_PROTOCOL 0
#define VSCP_WS2_PROTOCOL 1

/*
  This structure contains information about an authenticated user.
  It is used in the authentication process and may be populated with information
  from a user database or other source of user information. Some system may only
  allow for one user. In that case this structure can be populated with the information
  for that single user and the authentication process can simply check if the provided
  credentials match the information in this structure.

  Only the callbacks see this structure so the implementer may change it as he/she pleases.

  The fields in the structure are as follows:
  - username: The username of the user.
  - password: The password of the user (this should be encrypted or hashed in a secure manner).
  - fullname: The full name of the user.
  - filtermask: The filter mask for the user.
  - rights:  The rights or permissions of the user. This can hold bits for rights this user has.
  - remotes: The remote access information for the user. This can be a comma-separated list of IP addresses or hostnames
             that the user is allowed to connect from.
  - events:  The events associated with the user. This can be a comma-separated list of event types or IDs that the user
  is allowed to receive.
  - note: Any additional notes about the user.
*/
typedef struct _ws_user {

  // Username of user
  char username[32];

  // Full name of user
#ifndef VSCP_WS1_DISABLE_USER_FULL_NAME
  char fullname[64];
#endif

  // Privileges 0-15 (0 = no privileges, 15 = all privileges)
  uint8_t privlevel;

  /*
    Default filter mask for the user. Set after authentication. The user is allowed to change the
    filter mask after authentication if he/she has the right to do so.
    The filter mask is used to determine which events the user receives from the server.

    Serial format is
    C;SF;filter-priority, filter-class, filter-type, filter-GUID;mask-priority,
    mask-class, mask-type, mask-GUID”
  */
  vscpEventFilter rxfilter;

  /*
    Bitfield for user rights/permissions, see vscp.h for defined rights.
    This field is used to determine what the user is allowed to do and not to do.
    Alternative to the privlevel field, or can be used in combination with it for more fine-grained control.
  */
  uint64_t rights;

  /*
    Remote IP addresses the user is allowed to connect from.
    The list is binary and the IPs are stored in network byte order.
    The list is terminated by an all-zero entry. For IPv4 addresses,
    the first 12 bytes should be set to 0 and the last 4 bytes should
    contain the IPv4 address in network byte order.
    All zeros = end of list.
    An empty list (first entry is all zeros) means all remotes are allowed.
    For IPv4: store in last 4 bytes, first 12 = 0x00...00FFFF prefix
  */
  uint8_t allowed_remotes[16][16];

  /*
    The list can be used to specify which events a user is allowed to send if the
    user has the right to send events.  Events are stored binary as 16-bit value (MSB first)
    pairs (class,type). Two zeros (class=0, type=0) indicate the end of the list. An empty
    list (zeros on the first position) means all events are allowed.
  */
  uint16_t events[16][2]; // Events the user is allowed to send

  /*
     Additional notes about the user.
     If disabled by defining VSCP_WS1_DISABLE_USER_NOTES this field is not included in the
     structure to save memory. Notes will in this case be left blank on reports.
  */
#ifndef VSCP_WS1_DISABLE_USER_NOTES
  char note[256];
#endif

} ws_user_t;

typedef struct _vscp_ws_connection_context {
  uint8_t protocol; // 0 == stringbased, 1 == JSON
  uint8_t guid[16]; // GUID for the connection (defaults to module GUID)
  uint8_t sid[16];  // Session ID for authentication and encryption
  uint32_t chid;    // Channel ID for the connection, used for multiplexing multiple connections over the same websocket
  bool bOpen;       // Flag for open/closed channel- True if open
  bool bBinary;     // True if this is a binary protocol connection, false if text-based protocol
  uint8_t encryption;     // Encryption type used for the connection. 0 = no encryption, 1 = AES-128, etc.
  vscpEventFilter filter; // Filter/mask for received VSCP events for this connection
  bool bAuthenticated;    // Whether the client is authenticated. True if authenticated
  ws_user_t user;         // User information for athenticated user
  vscp_statistics_t stats; // Statistics for the connection
  vscp_status_t status;    // Status information for the connection
  void *pdata;            // Pointer to user data that can be used to store connection-specific information
} vscp_ws_connection_context_t;

#endif /* __VSCP_WS_COMMON_H__ */