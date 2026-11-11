/*
  File: udpsrv.h
  
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

#ifndef __VSCP_UDP_SERVER_H__
#define __VSCP_UDP_SERVER_H__

#include <stddef.h>
#include <stdint.h>
#include <esp_err.h>

/*!
  UDP server task
  @param pvParameters Server parameters
*/
void
udpsrv_task(void *pvParameters);

/*!
  Send a raw payload over UDP as unicast or broadcast.

  @param payload Pointer to payload buffer
  @param payload_len Number of payload bytes to send
  @param destination_ip Destination string (example "192.168.1.10",
                        "255.255.255.255", "myhost.local" or
                        "udp://myhost.local"). If NULL or invalid, current
                        configuration value is used and fallback is broadcast.
  @param destination_port Destination UDP port. If zero, configured UDP port
                          is used (or default VSCP UDP port if configured
                          value is also zero).
  @return ESP_OK on success, otherwise ESP_FAIL or ESP_ERR_INVALID_ARG
*/
esp_err_t
udpsrv_broadcast_message(const uint8_t *payload,
                         size_t payload_len,
                         const char *destination_ip,
                         uint16_t destination_port);

/*!
  Start UDP server task
*/
void udp_start(void);

#endif /* __VSCP_UDP_SERVER_H__ */