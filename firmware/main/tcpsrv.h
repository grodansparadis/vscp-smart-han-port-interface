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
*/

#ifndef __TCPSRV__
#define __TCPSRV__

#include <stdbool.h>
#include <vscp.h>
#include <vscp-fifo.h>
#include <vscp-firmware-helper.h>
#include <vscp-link-protocol.h>
#include <vscp-firmware-level2.h>



// Buffer 
#define TCPIP_BUF_MAX_SIZE (1024)

// Upper bound for a null terminated message sent with vscp_link_callback_write_client.
// Messages not terminated within this many bytes are rejected.
#ifndef VSCP_LINK_MAX_CLIENT_MSG_LENGTH
#define VSCP_LINK_MAX_CLIENT_MSG_LENGTH 4096
#endif

/** 
 * VSCP TCP link protocol character buffer size
 */
#ifndef DATA_BUF_SIZE
#define DATA_BUF_SIZE 512
#endif

/** 
 * Max number of events in the receive fifo 
 */
#define RECEIVE_FIFO_SIZE 16

/** 
 * Max number of events in each of the transmit fifos  
 */
#define TRANSMIT_FIFO_SIZE 16

/*
  Socket context
  This is the context for each open socket/channel. The VSCP link protocol
  state (socket, user, filter, statistics...) lives in the library context
  'link', which is what the link protocol callbacks get.
*/
typedef struct _ctx {
  vscp_link_ctx_t link;         // VSCP link protocol context (link.user_data points back here)
  volatile bool bInUse;         // Slot is taken by a client task
  bool bDiscard;                // Dropping the rest of an over-long line
  size_t size;                  // Number of characters in buf
  char buf[TCPIP_BUF_MAX_SIZE]; // Command buffer (lines are passed to the parser one by one)
} ctx_t;

/*!
  Operations table for the VSCP link protocol (callbacks-link.c)
*/
extern const vscp_link_ops_t g_link_ops;

#define MSG_MAX_CLIENTS "Max number of clients reached. Disconnecting.\r\n"

// This text is displayed when the user types "HELP" or "?" in the command line interface.
#define TCPIPSRV_HELP "See https://grodansparadis.github.io/vscp-doc-spec/#/./vscp_tcpiplink for help\r\n"


/**
 * @brief Set defaults for a connection context
 *
 * @param pctx Pointer to context
 */
void
setContextDefaults(ctx_t *pctx);

/*!
  VSCP tcp/ip link protocol task
  Start a client task for each accepted connection. The client task handles 
  the VSCP link protocol for the connection.
  @param pvParameters Task parameters
*/
void
tcpsrv_task(void *pvParameters);

#endif