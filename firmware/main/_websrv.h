/*
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

  curl 192.168.43.130:80/hello
  curl -X POST --data-binary @registers.txt 192.168.1.112:80/echo > tmpfile
  curl -X PUT -d "0" 192.168.1.112:80/ctrl
  curl -X PUT -d "1" 192.168.1.112:80/ctrl

  1. "curl 192.168.43.130:80/hello"  - tests the GET "\hello" handler
  2. "curl -X POST --data-binary @anyfile 192.168.43.130:80/echo > tmpfile"
      * "anyfile" is the file being sent as request body and "tmpfile" is where the body of the response is saved
      * since the server echoes back the request body, the two files should be same, as can be confirmed using : "cmp anyfile tmpfile"
  3. "curl -X PUT -d "0" 192.168.43.130:80/ctrl" - disable /hello and /echo handlers
  4. "curl -X PUT -d "1" 192.168.43.130:80/ctrl" -  enable /hello and /echo handlers

*/

#ifndef __VSCP_WCANG_WEBSRV_H__
#define __VSCP_WCANG_WEBSRV_H__

#define CONFIG_EXAMPLE_BASIC_AUTH_USERNAME "admin"
#define CONFIG_EXAMPLE_BASIC_AUTH_PASSWORD "secret"

typedef struct {
  char *username;
  char *password;
} basic_auth_info_t;

#define HTTPD_401      "401 UNAUTHORIZED"           /*!< HTTP Response 401 */

/*!
  Start the webserver
  @return esp error code
*/

httpd_handle_t
start_webserver(void);

/*!
  Stop the webserver
  @param server Server handle
  @return esp error code
*/

esp_err_t
stop_webserver(httpd_handle_t server);




#endif