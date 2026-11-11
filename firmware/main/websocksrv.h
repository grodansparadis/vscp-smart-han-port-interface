/*
  File: websocksrv.h

  VSCP Wireless CAN4VSCP Gateway (VSCP-WCANG)

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

#ifndef __VSCP_WEBSOCK_SERVER_H__
#define __VSCP_WEBSOCK_SERVER_H__

#include <stddef.h>
#include <stdint.h>
#include <esp_err.h>

#include "esp_http_server.h"

/*!
  Websocket server task
  @param pvParameters Server parameters
*/
void
websocksrv_task(void *pvParameters);


/*!
  Register URI handlers for the WebSocket server instance
  @param server HTTP server handle for the WebSocket server instance
  @return ESP_OK on success, error code otherwise

  This is mainfly for the case where port is the same for the webserver and
  the WebSocket server. If the WebSocket server is running on a different port
  than the webserver, then the handlers should be registered in the function
  that starts the WebSocket server (e.g. wss_start_websocket_server) after 
  the server is started and the server handle is available.
*/
int wss_register_handlers(httpd_handle_t server);

/*!
  Setup and start the WebSocket server
  @return HTTP server handle for the WebSocket server instance
*/
httpd_handle_t
wss_start_websocket_server(void);

/*!
  Close a connected WebSocket client session by socket file descriptor.
  @param fd Client socket file descriptor
  @return ESP_OK on success, error code otherwise
*/
esp_err_t
wss_close_client_fd(int fd);

#endif /* __VSCP_WEBSOCK_SERVER_H__ */