// FILE: callbacks-link.c

// This file holds callbacks for the VSCP tcp/ip link protocol

/* ******************************************************************************
 * 	VSCP (Very Simple Control Protocol)
 * 	https://www.vscp.org
 *
 * The MIT License (MIT)
 *
 * Copyright (C) 2000-2026 Ake Hedman, Grodans Paradis AB <info@grodansparadis.com>
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
 *	This file is part of VSCP - Very Simple Control Protocol
 *	https://www.vscp.org
 *
 * ******************************************************************************
 */

#include "vscp-compiler.h"
#include "vscp-projdefs.h"

#include <vscp-class.h>
#include <vscp-type.h>

#include <esp_log.h>
#include <esp_timer.h>
#include <esp_random.h>
#include <nvs_flash.h>
#include <lwip/sockets.h>

#include "main.h"
#include "tcpsrv.h"

#include <ctype.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "tcpsrv-cb";

// Global stuff
extern transport_t tr_tcpsrv[MAX_TCP_CONNECTIONS];
// from main
extern nvs_handle_t g_nvsHandle;
extern node_persistent_config_t g_persistent;

extern uint32_t
time_us_32(void);

// ****************************************************************************
//                       VSCP Link protocol callbacks
// ****************************************************************************

// Forward declarations
static int vscp_link_callback_write_client(vscp_link_ctx_t *pctx, const char *msg);
static int vscp_link_callback_disconnect(vscp_link_ctx_t *pctx);

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_write_client
//

static int
vscp_link_callback_welcome(vscp_link_ctx_t *pctx)
{
  if (NULL == pctx) {
    return VSCP_ERROR_INVALID_POINTER;
  }


  return vscp_link_callback_write_client(pctx, TCPSRV_WELCOME_MSG);
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_write_client
//

static int
vscp_link_callback_write_client(vscp_link_ctx_t *pctx, const char *msg)
{
  if ((NULL == pctx) || (NULL == msg)) {
    return VSCP_ERROR_INVALID_POINTER;
  }


  // Bounded length scan: never read past VSCP_LINK_MAX_CLIENT_MSG_LENGTH bytes
  // even if the caller passes a buffer that is not null terminated.
  size_t len = strnlen(msg, VSCP_LINK_MAX_CLIENT_MSG_LENGTH);
  if (len >= VSCP_LINK_MAX_CLIENT_MSG_LENGTH) {
    ESP_LOGE(TAG,
             "write_client: message not null terminated within %d bytes, not sent",
             VSCP_LINK_MAX_CLIENT_MSG_LENGTH);
    return VSCP_ERROR_PARAMETER;
  }

  // send() may write fewer bytes than requested; loop until all is sent.
  const uint8_t *p = (const uint8_t *) msg;
  while (len > 0) {
    ssize_t n = send(pctx->sock, p, len, 0);
    if (n < 0) {
      if (EINTR == errno) {
        continue;
      }
      ESP_LOGE(TAG, "write_client: send failed, errno=%d", errno);
      return VSCP_ERROR_ERROR;
    }
    p += n;
    len -= (size_t) n;
  }

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_quit
//

static int
vscp_link_callback_quit(vscp_link_ctx_t *pctx)
{
  if (NULL == pctx) {
    return VSCP_ERROR_INVALID_POINTER;
  }


  // Confirm quit
  vscp_link_callback_write_client(pctx, VSCP_LINK_MSG_GOODBYE);

  return vscp_link_callback_disconnect(pctx);
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_disconnect
//

static int
vscp_link_callback_disconnect(vscp_link_ctx_t *pctx)
{
  if (NULL == pctx) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  // The client task sees sock == 0, ends and releases the connection slot
  if (pctx->sock) {
    shutdown(pctx->sock, 0);
    close(pctx->sock);
    pctx->sock = 0;
  }

  pctx->bValidated = 0;
  pctx->privLevel  = 0;
  pctx->bRcvLoop   = 0;

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_get_interface_count
//

static uint16_t
vscp_link_callback_get_interface_count(vscp_link_ctx_t *pctx)
{
  /* Return number of interfaces we support */
  return 1; // see vscp_link_callback_get_interface
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_get_interface
//

static int
vscp_link_callback_get_interface(vscp_link_ctx_t *pctx, uint16_t index, struct vscp_interface_info *pif)
{
  if ((NULL == pctx) || (NULL == pif)) {
    return VSCP_ERROR_UNKNOWN_ITEM;
  }

  /*
    We have one interface on this device
    00:00 is the internal interface for the device itself.

    Each interfaces is returned as a comma separated string with the following format:

    'interface-id-n, type, interface-GUID-n, interface_real-name-n'

    interface types is in vscp.h
   */

  switch (index) {

    case 0:
      pif->idx  = index;
      pif->type = VSCP_INTERFACE_TYPE_LEVEL2DRV;
      memcpy(pif->guid, g_persistent.guid, 16);
      strncpy(pif->description, "Interface for the device itself", sizeof(pif->description));
      break;

    default:
      return VSCP_ERROR_UNKNOWN_ITEM;
      break;
  }

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_close_interface
//

static int
vscp_link_callback_close_interface(vscp_link_ctx_t *pctx, uint8_t *pguid)
{
  return VSCP_ERROR_NOT_SUPPORTED;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_check_user
//

static int
vscp_link_callback_check_user(vscp_link_ctx_t *pctx, const char *arg)
{
  if ((NULL == pctx) || (NULL == arg)) {
    ESP_LOGE(TAG, "Invalid context pointer or user arg\n");
    return VSCP_ERROR_INVALID_POINTER;
  }

  // trim
  const char *p = arg;
  while (*p && isspace((unsigned char) *p)) {
    p++;
  }

  ESP_LOGI(TAG, "Username: %s\n", p);


  strncpy(pctx->user, p, sizeof(pctx->user) - 1);
  pctx->user[sizeof(pctx->user) - 1] = '\0';
  ESP_LOGI(TAG, "Username: %s\n", pctx->user);

  vscp_link_callback_write_client(pctx, VSCP_LINK_MSG_USENAME_OK);
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_check_password
//

static int
vscp_link_callback_check_password(vscp_link_ctx_t *pctx, const char *arg)
{
  if ((NULL == pctx) || (NULL == arg)) {
    ESP_LOGE(TAG, "Invalid context pointer or password arg\n");
    return VSCP_ERROR_INVALID_POINTER;
  }


  // Must have a username before a password
  if (!strlen(pctx->user)) {
    vscp_link_callback_write_client(pctx, VSCP_LINK_MSG_NEED_USERNAME);
    ESP_LOGE(TAG, "Password: No username yet\n");
    return VSCP_ERROR_SUCCESS;
  }

  // trim password
  const char *p = arg;
  while (*p && isspace((unsigned char) *p)) {
    p++;
  }

  ESP_LOGI(TAG, "Username:'%s'\n", pctx->user);

  // if (0 == strcmp(pctx->user, "admin") && 0 == strcmp(p, "secret")) {

  if (validate_user(pctx->user, p)) {
    pctx->bValidated = true;
    pctx->privLevel  = 15;
  }
  else {
    pctx->user[0]    = '\0';
    pctx->bValidated = false;
    pctx->privLevel  = 0;
    vscp_link_callback_write_client(pctx, VSCP_LINK_MSG_PASSWORD_ERROR);
    ESP_LOGE(TAG, "Credentials: Invalid\n");
    return VSCP_ERROR_INVALID_PERMISSION; // The library then disconnects the client
  }

  vscp_link_callback_write_client(pctx, VSCP_LINK_MSG_PASSWORD_OK);
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_challenge
//

static int
vscp_link_callback_challenge(vscp_link_ctx_t *pctx, const char *arg)
{
  char buf[80];
  char random_data[32];
  if ((NULL == pctx) || (NULL == arg)) {
    return VSCP_ERROR_INVALID_POINTER;
  }


  const char *p = arg;
  while (*p && isspace((unsigned char) *p)) {
    p++;
  }

  static const char challenge_prefix[] = "+OK - ";
  size_t pos                           = (size_t) snprintf(buf, sizeof(buf), "%s", challenge_prefix);
  if (pos >= sizeof(buf)) {
    return VSCP_ERROR_MEMORY;
  }

  for (int i = 0; i < 32; i++) {
    random_data[i] = (char) (esp_random() & 0xff);
    if (i < (int) (sizeof(challenge_prefix) - 1)) {
      random_data[i] += (uint8_t) challenge_prefix[i];
    }

    if ((pos + 2) >= sizeof(buf)) {
      return VSCP_ERROR_MEMORY;
    }

    vscp_fwhlp_dec2hex(random_data[i], &buf[pos], 2);
    pos += 2;
  }

  if ((pos + 2) >= sizeof(buf)) {
    return VSCP_ERROR_MEMORY;
  }

  buf[pos++] = '\r';
  buf[pos++] = '\n';
  buf[pos]   = '\0';

  return vscp_link_callback_write_client(pctx, buf);
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_check_authenticated
//

static int
vscp_link_callback_check_authenticated(vscp_link_ctx_t *pctx)
{
  if (NULL == pctx) {
    return VSCP_ERROR_INVALID_POINTER;
  }


  if (pctx->bValidated) {
    return VSCP_ERROR_SUCCESS;
  }

  return VSCP_ERROR_INVALID_PERMISSION;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_check_privilege
//

static int
vscp_link_callback_check_privilege(vscp_link_ctx_t *pctx, uint8_t priv)
{
  if (NULL == pctx) {
    return VSCP_ERROR_INVALID_POINTER;
  }


  if (pctx->privLevel >= priv) {
    return VSCP_ERROR_SUCCESS;
  }

  return VSCP_ERROR_INVALID_PERMISSION;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_challenge
//

static int
vscp_link_callback_test(vscp_link_ctx_t *pctx, const char *arg)
{
  if ((NULL == pctx) || (NULL == arg)) {
    return VSCP_ERROR_INVALID_POINTER;
  }


  vscp_link_callback_write_client(pctx, VSCP_LINK_MSG_OK);
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_send
//

static int
vscp_link_callback_send(vscp_link_ctx_t *pctx, vscp_event_t *pev)
{
  if ((NULL == pctx) || (NULL == pev)) {
    return VSCP_ERROR_INVALID_POINTER;
  }


  // Filter
  // if (!vscp_fwhlp_doLevel2FilterEx(pex, &pctx->filter)) {
  //   return VSCP_ERROR_SUCCESS; // Filter out == OK
  // }

  // Update send statistics
  pctx->statistics.cntTransmitFrames++;
  pctx->statistics.cntTransmitData += pev->sizeData;

  // Mark this event as coming from this interface
  pev->obid = pctx->sock;

  // Let the node handle the event
  handle_incoming_event(pev, CH_LINK);

  // On success the callback owns the event (the library frees it only on error)
  vscp_fwhlp_deleteEvent(&pev);


  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_retr
//

static int
vscp_link_callback_retr(vscp_link_ctx_t *pctx, vscp_event_t **pev)
{
  if ((NULL == pctx) || (NULL == pev)) {
    return VSCP_ERROR_INVALID_POINTER;
  }


  // Get the next event that passes the filter. We own it from now on.
  // Events that do not pass are dropped (returning success with no event
  // would make the library report an error).
  for (;;) {
    *pev = NULL;
    if (pdPASS != xQueueReceive(tr_tcpsrv[pctx->id].event_queue, (void *) pev, 0)) {
      *pev = NULL;
      return VSCP_ERROR_RCV_EMPTY;
    }
    if (vscp_fwhlp_doLevel2Filter(*pev, &pctx->filter)) {
      break;
    }
    vscp_fwhlp_deleteEvent(pev);
  }

  ESP_LOGV(TAG,
           "--> Event fetched class=%u type=%u count=%u",
           (*pev)->vscp_class,
           (*pev)->vscp_type,
           (unsigned) uxQueueMessagesWaiting(tr_tcpsrv[pctx->id].event_queue));

  // We use socket id as obid
  (*pev)->obid = pctx->sock;

  // Update receive statistics
  pctx->statistics.cntReceiveFrames++;
  pctx->statistics.cntReceiveData += (*pev)->sizeData;

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_enable_rcvloop
//

static int
vscp_link_callback_enable_rcvloop(vscp_link_ctx_t *pctx, int bEnable)
{
  if (NULL == pctx) {
    return VSCP_ERROR_INVALID_POINTER;
  }


  pctx->bRcvLoop          = bEnable;
  pctx->last_rcvloop_time = (uint32_t) (esp_timer_get_time() / 1000); // Milliseconds

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_get_rcvloop_status
//

static int
vscp_link_callback_get_rcvloop_status(vscp_link_ctx_t *pctx, int *pRcvLoop)
{
  if (NULL == pctx) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  if (NULL == pRcvLoop) {
    return VSCP_ERROR_INVALID_POINTER;
  }


  *pRcvLoop = pctx->bRcvLoop;

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_chkData
//

static int
vscp_link_callback_chkdata(vscp_link_ctx_t *pctx, uint16_t *pcount)
{
  if (NULL == pctx) {
    return VSCP_ERROR_INVALID_POINTER;
  }


  //*pcount     = TRANSMIT_FIFO_SIZE - vscp_fifo_getFree(&pctx->fifoEventsOut);
  *pcount = uxQueueMessagesWaiting(tr_tcpsrv[pctx->id].event_queue);

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_clrAll
//

static int
vscp_link_callback_clrall(vscp_link_ctx_t *pctx)
{
  if (NULL == pctx) {
    return VSCP_ERROR_INVALID_POINTER;
  }


  transport_flush_queue(tr_tcpsrv[pctx->id].event_queue);

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_get_channel_id
//

static int
vscp_link_callback_get_channel_id(vscp_link_ctx_t *pctx, uint16_t *pchid)
{
  if ((NULL == pctx) || (NULL == pchid)) {
    return VSCP_ERROR_INVALID_POINTER;
  }


  *pchid = pctx->sock;

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_get_guid
//

static int
vscp_link_callback_get_guid(vscp_link_ctx_t *pctx, uint8_t *pguid)
{
  if ((NULL == pctx) || (NULL == pguid)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  memcpy(pguid, g_persistent.guid, 16);
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_set_guid
//

static int
vscp_link_callback_set_guid(vscp_link_ctx_t *pctx, uint8_t *pguid)
{
  if ((NULL == pctx) || (NULL == pguid)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  return VSCP_ERROR_NOT_SUPPORTED;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_get_version
//

static int
vscp_link_callback_get_version(vscp_link_ctx_t *pctx, uint8_t *pversion)
{
  if ((NULL == pctx) || (NULL == pversion)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  pversion[0] = THIS_FIRMWARE_MAJOR_VERSION;
  pversion[1] = THIS_FIRMWARE_MINOR_VERSION;
  pversion[2] = THIS_FIRMWARE_RELEASE_VERSION;
  pversion[3] = THIS_FIRMWARE_BUILD_VERSION;

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_setFilter
//

static int
vscp_link_callback_set_filter(vscp_link_ctx_t *pctx, vscpEventFilter *pfilter)
{
  if ((NULL == pctx) || (NULL == pfilter)) {
    return VSCP_ERROR_INVALID_POINTER;
  }


  pctx->filter.filter_class    = pfilter->filter_class;
  pctx->filter.filter_type     = pfilter->filter_type;
  pctx->filter.filter_priority = pfilter->filter_priority;
  memcpy(pctx->filter.filter_GUID, pfilter->filter_GUID, 16);

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_setMask
//

static int
vscp_link_callback_set_mask(vscp_link_ctx_t *pctx, vscpEventFilter *pfilter)
{
  if ((NULL == pctx) || (NULL == pfilter)) {
    return VSCP_ERROR_INVALID_POINTER;
  }


  pctx->filter.mask_class    = pfilter->mask_class;
  pctx->filter.mask_type     = pfilter->mask_type;
  pctx->filter.mask_priority = pfilter->mask_priority;
  memcpy(pctx->filter.mask_GUID, pfilter->mask_GUID, 16);

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_statistics
//

static int
vscp_link_callback_statistics(vscp_link_ctx_t *pctx, vscp_statistics_t *pStatistics)
{
  if ((NULL == pctx) || (NULL == pStatistics)) {
    return VSCP_ERROR_INVALID_POINTER;
  }


  memcpy(pStatistics, &pctx->statistics, sizeof(vscp_statistics_t));

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_info
//

static int
vscp_link_callback_info(vscp_link_ctx_t *pctx, vscp_status_t *pstatus)
{
  if ((NULL == pctx) || (NULL == pstatus)) {
    return VSCP_ERROR_INVALID_POINTER;
  }


  memcpy(pstatus, &pctx->status, sizeof(vscp_status_t));

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_rcvloop
//

static int
vscp_link_callback_rcvloop(vscp_link_ctx_t *pctx, vscp_event_t **pev)
{
  // Check pointer
  if ((NULL == pctx) || (NULL == pev)) {
    return VSCP_ERROR_INVALID_POINTER;
  }


  // Every second output '+OK\r\n' in rcvloop mode (unsigned arithmetic handles wrap)
  uint32_t now_ms = (uint32_t) (esp_timer_get_time() / 1000);
  if ((uint32_t) (now_ms - pctx->last_rcvloop_time) > 1000) {
    pctx->last_rcvloop_time = now_ms;
    return VSCP_ERROR_TIMEOUT;
  }

  return vscp_link_callback_retr(pctx, pev);
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_wcyd
//

static int
vscp_link_callback_wcyd(vscp_link_ctx_t *pctx, uint64_t *pwcyd)
{
  // Check pointers
  if ((NULL == pctx) || (NULL == pwcyd)) {
    return VSCP_ERROR_INVALID_POINTER;
  }


  // TODO
  *pwcyd = VSCP_SERVER_CAPABILITY_TCPIP | VSCP_SERVER_CAPABILITY_DECISION_MATRIX | VSCP_SERVER_CAPABILITY_IP4 |
           /*VSCP_SERVER_CAPABILITY_SSL |*/
           VSCP_SERVER_CAPABILITY_TWO_CONNECTIONS;

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_shutdown
//

static int
vscp_link_callback_shutdown(vscp_link_ctx_t *pctx)
{
  // Check pointers
  if (NULL == pctx) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  return VSCP_ERROR_NOT_SUPPORTED;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_restart
//

static int
vscp_link_callback_restart(vscp_link_ctx_t *pctx)
{
  // Check pointers
  if (NULL == pctx) {
    return VSCP_ERROR_INVALID_POINTER;
  }


  esp_restart();

  return VSCP_ERROR_SUCCESS;
}




// ****************************************************************************
//                       VSCP Link protocol operations table
// ****************************************************************************

const vscp_link_ops_t g_link_ops = {
  // Transport
  .write_client = vscp_link_callback_write_client,
  .disconnect   = vscp_link_callback_disconnect,
  .quit         = vscp_link_callback_quit,
  .welcome      = vscp_link_callback_welcome,

  // Authentication
  .check_user          = vscp_link_callback_check_user,
  .check_password      = vscp_link_callback_check_password,
  .challenge           = vscp_link_callback_challenge,
  .check_authenticated = vscp_link_callback_check_authenticated,
  .check_privilege     = vscp_link_callback_check_privilege,

  // Event I/O
  .send               = vscp_link_callback_send,
  .chkdata            = vscp_link_callback_chkdata,
  .clrall             = vscp_link_callback_clrall,
  .retr               = vscp_link_callback_retr,
  .rcvloop            = vscp_link_callback_rcvloop,
  .enable_rcvloop     = vscp_link_callback_enable_rcvloop,
  .get_rcvloop_status = vscp_link_callback_get_rcvloop_status,

  // Node information
  .get_guid       = vscp_link_callback_get_guid,
  .set_guid       = vscp_link_callback_set_guid,
  .get_version    = vscp_link_callback_get_version,
  .get_channel_id = vscp_link_callback_get_channel_id,
  .wcyd           = vscp_link_callback_wcyd,
  .statistics     = vscp_link_callback_statistics,
  .info           = vscp_link_callback_info,

  // Filter
  .set_filter = vscp_link_callback_set_filter,
  .set_mask   = vscp_link_callback_set_mask,

  // Interfaces
  .get_interface_count = vscp_link_callback_get_interface_count,
  .get_interface       = vscp_link_callback_get_interface,
  .close_interface     = vscp_link_callback_close_interface,

  // Misc
  .help_custom = NULL, // Library standard help texts
  .test        = vscp_link_callback_test,
  .shutdown    = vscp_link_callback_shutdown,
  .restart     = vscp_link_callback_restart,

  // Binary mode
  .binary = NULL,
};
