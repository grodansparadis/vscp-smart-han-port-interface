/*
  VSCP Multicast support

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

#ifndef __VSCP_MULTICAST_ESP32_H__
#define __VSCP_MULTICAST_ESP32_H__

/*!
  @fn multicast_sendEvent
  @brief Send a VSCP event as multicast UDP packet

  @param sock Socket to send event on
  @param pstrev VSCP event on string form to send
  @param bEncrypt true if the event should be encrypted with the set key
  @param nAlgorithm Encryption algorithm to use (vscp.h)
  @return VSCP_ERROR_SUCCESS on success else error code
*/
// int32_t
// multicast_sendEvent(int sock, const char *pstrev, bool bEncrypt, uint8_t nAlgorithm);


/*!
  Multicast heartbeat task
  @param pvParameters Task parameters
*/
void
multicast_heartbeat_task(void *pvParameters);

/*!
  Multicast transmit task
  @param pvParameters Task parameters
*/
void
multicast_tx_task(void *pvParameters);




/*!
  Multicast receive task
  @param pvParameters Task parameters
*/
void
multicast_rx_task(void *pvParameters);

/*!
  Start Multicast server task
*/
void multicast_start(void);

#endif