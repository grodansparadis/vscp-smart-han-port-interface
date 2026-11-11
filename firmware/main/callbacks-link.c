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
#include <nvs_flash.h>
#include <lwip/sockets.h>

#include "main.h"
#include "tcpsrv.h"

#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "tcpsrv-cb";

// Global stuff
extern transport_t tr_twai_rx;
extern transport_t tr_tcpsrv[MAX_TCP_CONNECTIONS];
// from main
extern nvs_handle_t g_nvsHandle;
extern node_persistent_config_t g_persistent;

extern uint32_t
time_us_32(void);

// ****************************************************************************
//                       VSCP Link protocol callbacks
// ****************************************************************************

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_write_client
//

int
vscp_link_callback_welcome(const void *pdata)
{
  if (NULL == pdata) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Get pointer to context
  ctx_t *pctx = (ctx_t *) pdata;

  send(pctx->sock, TCPSRV_WELCOME_MSG, strlen(TCPSRV_WELCOME_MSG), 0);
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_write_client
//

int
vscp_link_callback_write_client(const void *pdata, const char *msg)
{
  if ((NULL == pdata) || (NULL == msg)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Get pointer to context
  ctx_t *pctx = (ctx_t *) pdata;

  send(pctx->sock, (uint8_t *) msg, strlen(msg), 0);
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_quit
//

int
vscp_link_callback_quit(const void *pdata)
{
  if (NULL == pdata) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Get pointer to context
  ctx_t *pctx = (ctx_t *) pdata;

  // Confirm quit
  send(pctx->sock, VSCP_LINK_MSG_GOODBY, strlen(VSCP_LINK_MSG_GOODBY), 0);

  // Disconnect from client
  shutdown(pctx->sock, 0);
  close(pctx->sock);
  pctx->sock = 0;

  // Set context defaults
  // setContextDefaults(pctx);

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_help
//

int
vscp_link_callback_help(const void *pdata, const char *arg)
{
  if ((NULL == pdata) || (NULL == arg)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Get pointer to context
  ctx_t *pctx = (ctx_t *) pdata;

  send(pctx->sock, TCPIPSRV_HELP, strlen(TCPIPSRV_HELP), 0);
  send(pctx->sock, VSCP_LINK_MSG_OK, strlen(VSCP_LINK_MSG_OK), 0);
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_get_interface_count
//

uint16_t
vscp_link_callback_get_interface_count(const void *pdata)
{
  /* Return number of interfaces we support */
  return 2; // see vscp_link_callback_get_interface
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_get_interface
//

int
vscp_link_callback_get_interface(const void *pdata, uint16_t index, struct vscp_interface_info *pif)
{
  if ((NULL == pdata) || (NULL == pif)) {
    return VSCP_ERROR_UNKNOWN_ITEM;
  }

  /*
    We have two interfaces on this device
    00:00 is the CAN4VSCP channel.
    00:01 is the internal interface for the device itself.

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

    case 1: {
      uint8_t guid[16];
      pif->idx  = index;
      pif->type = VSCP_INTERFACE_TYPE_LEVEL1DRV;
      memcpy(guid, g_persistent.guid, 16);
      guid[13] = 0x01; // Interface 0x0001
      memcpy(pif->guid, g_persistent.guid, 16);
      strncpy(pif->description, "Interface for the CAN4VSCP channel", sizeof(pif->description));
    } break;

    default:
      return VSCP_ERROR_UNKNOWN_ITEM;
      break;
  }

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_close_interface
//

int
vscp_link_callback_close_interface(const void *pdata, uint8_t *pguid)
{
  return VSCP_ERROR_NOT_SUPPORTED;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_check_user
//

int
vscp_link_callback_check_user(const void *pdata, const char *arg)
{
  if ((NULL == pdata) || (NULL == arg)) {
    ESP_LOGE(TAG, "Invalid context pointer or user arg\n");
    return VSCP_ERROR_INVALID_POINTER;
  }

  // trim
  const char *p = arg;
  while (*p && isspace((unsigned char) *p)) {
    p++;
  }

  ESP_LOGI(TAG, "Username: %s\n", p);

  // Get pointer to context
  ctx_t *pctx = (ctx_t *) pdata;

  strncpy(pctx->user, p, VSCP_LINK_MAX_USER_NAME_LENGTH);
  ESP_LOGI(TAG, "Username: %s\n", pctx->user);

  send(pctx->sock, VSCP_LINK_MSG_USENAME_OK, strlen(VSCP_LINK_MSG_USENAME_OK), 0);
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_check_password
//

int
vscp_link_callback_check_password(const void *pdata, const char *arg)
{
  if ((NULL == pdata) || (NULL == arg)) {
    ESP_LOGE(TAG, "Invalid context pointer or password arg\n");
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Get pointer to context
  ctx_t *pctx = (ctx_t *) pdata;

  // Must have a username before a password
  if (!strlen(pctx->user)) {
    send(pctx->sock, VSCP_LINK_MSG_NEED_USERNAME, strlen(VSCP_LINK_MSG_NEED_USERNAME), 0);
    ESP_LOGE(TAG, "Password: No username yet\n");
    return VSCP_ERROR_SUCCESS;
  }

  // trim password
  const char *p = arg;
  while (*p && isspace((unsigned char) *p)) {
    p++;
  }

  ESP_LOGI(TAG, "Username:'%s'\n", pctx->user);
  ESP_LOGI(TAG, "Password '%s'\n", p);

  // if (0 == strcmp(pctx->user, "admin") && 0 == strcmp(p, "secret")) {

  if (validate_user(pctx->user, p)) {
    pctx->bValidated = true;
    pctx->privLevel  = 15;
  }
  else {
    pctx->user[0]    = '\0';
    pctx->bValidated = false;
    pctx->privLevel  = 0;
    send(pctx->sock, VSCP_LINK_MSG_PASSWORD_ERROR, strlen(VSCP_LINK_MSG_PASSWORD_ERROR), 0);
    ESP_LOGE(TAG, "Credentials: Invalid\n");
    return VSCP_ERROR_SUCCESS;
  }

  send(pctx->sock, VSCP_LINK_MSG_PASSWORD_OK, strlen(VSCP_LINK_MSG_PASSWORD_OK), 0);
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_challenge
//

int
vscp_link_callback_challenge(const void *pdata, const char *arg)
{
  char buf[80];
  char random_data[32];
  if ((NULL == pdata) || (NULL == arg)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Get pointer to context
  ctx_t *pctx = (ctx_t *) pdata;

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
    random_data[i] = rand() >> 16;
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

  send(pctx->sock, buf, strlen(buf), 0);
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_check_authenticated
//

int
vscp_link_callback_check_authenticated(const void *pdata)
{
  if (NULL == pdata) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Get pointer to context
  ctx_t *pctx = (ctx_t *) pdata;

  if (pctx->bValidated) {
    return VSCP_ERROR_SUCCESS;
  }

  return VSCP_ERROR_INVALID_PERMISSION;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_check_privilege
//

int
vscp_link_callback_check_privilege(const void *pdata, uint8_t priv)
{
  if (NULL == pdata) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Get pointer to context
  ctx_t *pctx = (ctx_t *) pdata;

  if (pctx->privLevel >= priv) {
    return VSCP_ERROR_SUCCESS;
  }

  return VSCP_ERROR_INVALID_PERMISSION;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_challenge
//

int
vscp_link_callback_test(const void *pdata, const char *arg)
{
  if ((NULL == pdata) || (NULL == arg)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Get pointer to context
  ctx_t *pctx = (ctx_t *) pdata;

  send(pctx->sock, VSCP_LINK_MSG_OK, strlen(VSCP_LINK_MSG_OK), 0);
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_send
//

int
vscp_link_callback_send(const void *pdata, vscp_event_t *pev)
{
  if ((NULL == pdata) || (NULL == pev)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Get pointer to context
  ctx_t *pctx = (ctx_t *) pdata;

  // Filter
  // if (!vscp_fwhlp_doLevel2FilterEx(pex, &pctx->filter)) {
  //   return VSCP_ERROR_SUCCESS; // Filter out == OK
  // }

  // Update send statistics
  pctx->statistics.cntTransmitFrames++;
  pctx->statistics.cntTransmitData += pev->sizeData;

  // Mark this event as coming from this interface
  pev->obid = pctx->sock;

  // Check for Level II event
  if (pev->vscp_class > 1024) {}
  // Check for proxy event
  else if (pev->vscp_class > 512) {
    ;
  }
  // Level I event. If addressed to nodeid = 0
  // and VSCP_CLASS1_PROTOCOL it is addressed to us
  // and we should handle it. If not send event
  // on the TWAI interface.
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

  // Write event to receive fifo

  // vscpexentEx *pnew = vscp_fwhlp_mkEventCopy(pex);
  // if (NULL == pnew) {
  //   return VSCP_ERROR_MEMORY;
  // }
  // else {
  //   if (!vscp_fifo_write(&fifoEventsIn, pnew)) {
  //     vscp_fwhlp_deleteEvent(&pnew);
  //     vscp_fwhlp_deleteEvent(&pex);
  //     pctx->statistics.cntOverruns++;
  //     return VSCP_ERROR_TRM_FULL;
  //   }
  // }

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

  // Event is not needed anymore
  // vscp_fwhlp_deleteEvent(&pex);

  // We own the event from now on and must
  // delete it and it's data when we are done
  // with it

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_retr
//

int
vscp_link_callback_retr(const void *pdata, vscp_event_t **pev)
{
  BaseType_t rv;
  can4vscp_frame_t msg = {};

  if ((NULL == pdata) || (NULL == pev)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Get pointer to context
  ctx_t *pctx = (ctx_t *) pdata;

  // Check if there is a TWAI message in the receive queue
  if (pdPASS != (rv = xQueueReceive(tr_tcpsrv[pctx->id].fromcan_queue, &msg, 100))) {
    return VSCP_ERROR_RCV_EMPTY;
  }

  // OK we have a message and should return it as a VSCP event
  // Allocate a new event
  *pev = (vscp_event_t *) malloc(sizeof(vscpEvent));
  if (NULL == *pev) {
    return VSCP_ERROR_MEMORY;
  }

  // Clear event
  memset(*pev, 0, sizeof(vscpEvent));

  // We use socket id as obid
  (*pev)->obid = pctx->sock;

  // Allocate data if the message has data
  if (msg.data_length_code) {
    (*pev)->pdata = (uint8_t *) malloc(msg.data_length_code);
    if (NULL == (*pev)->pdata) {
      free(*pev);
      *pev = NULL;
      return VSCP_ERROR_MEMORY;
    }
    // Copy in data
    (*pev)->sizeData = msg.data_length_code;
    memcpy((*pev)->pdata, msg.data, msg.data_length_code);
  }

  ESP_LOGV(TAG, "--> Event fetched %X", (unsigned int) msg.identifier);

  UBaseType_t cnt = uxQueueMessagesWaiting(tr_tcpsrv[pctx->id].fromcan_queue);
  ESP_LOGV(TAG, "count=%u %d", cnt, rv);

  (*pev)->head       = (msg.identifier >> (26 - 5)) & 0x00e0;
  (*pev)->timestamp  = esp_timer_get_time();  // Microseconds since boot
  (*pev)->vscp_class = (msg.identifier >> 16) & 0x1ff;
  (*pev)->vscp_type  = (msg.identifier >> 8) & 0xff;

  // GUID
  memcpy((*pev)->GUID, g_persistent.guid, 16);

  // Set nickname
  (*pev)->GUID[14] = 0x00; // MSB of Node ID is never used for CAN4VSCP
  (*pev)->GUID[15] = msg.identifier & 0xff;
  (*pev)->sizeData = msg.data_length_code;
  // Copy in data if any
  if (msg.data_length_code) {
    memcpy((*pev)->pdata, msg.data, msg.data_length_code);
  }
  // Time data set to null => first interface with this info should
  // set timing data
  (*pev)->year = (*pev)->month = (*pev)->day = (*pev)->hour = (*pev)->minute = (*pev)->second = 0;

  if (!vscp_fwhlp_doLevel2Filter(*pev, &pctx->filter)) {
    free((*pev)->pdata);
    free(*pev);
    *pev = NULL;
    return VSCP_ERROR_SUCCESS; // Filter out == OK
  }

  // Update receive statistics
  pctx->statistics.cntReceiveFrames++;
  pctx->statistics.cntReceiveData += (*pev)->sizeData;

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_enable_rcvloop
//

int
vscp_link_callback_enable_rcvloop(const void *pdata, int bEnable)
{
  if (NULL == pdata) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Get pointer to context
  ctx_t *pctx = (ctx_t *) pdata;

  pctx->bRcvLoop          = bEnable;
  pctx->last_rcvloop_time = esp_timer_get_time();

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_get_rcvloop_status
//

int
vscp_link_callback_get_rcvloop_status(const void *pdata, int *pRcvLoop)
{
  if (NULL == pdata) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  if (NULL == pRcvLoop) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Get pointer to context
  ctx_t *pctx = (ctx_t *) pdata;

  *pRcvLoop = pctx->bRcvLoop;

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_chkData
//

int
vscp_link_callback_chkData(const void *pdata, uint16_t *pcount)
{
  if (NULL == pdata) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Get pointer to context
  ctx_t *pctx = (ctx_t *) pdata;

  //*pcount     = TRANSMIT_FIFO_SIZE - vscp_fifo_getFree(&pctx->fifoEventsOut);
  *pcount = uxQueueMessagesWaiting(tr_tcpsrv[pctx->id].fromcan_queue);

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_clrAll
//

int
vscp_link_callback_clrAll(const void *pdata)
{
  if (NULL == pdata) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Get pointer to context
  ctx_t *pctx = (ctx_t *) pdata;

  xQueueReset(tr_tcpsrv[pctx->id].fromcan_queue);

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_get_channel_id
//

int
vscp_link_callback_get_channel_id(const void *pdata, uint16_t *pchid)
{
  if ((NULL == pdata) || (NULL == pchid)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Get pointer to context
  ctx_t *pctx = (ctx_t *) pdata;

  *pchid = pctx->sock;

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_get_guid
//

int
vscp_link_callback_get_guid(const void *pdata, uint8_t *pguid)
{
  if ((NULL == pdata) || (NULL == pguid)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  memcpy(pguid, g_persistent.guid, 16);
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_set_guid
//

int
vscp_link_callback_set_guid(const void *pdata, uint8_t *pguid)
{
  if ((NULL == pdata) || (NULL == pguid)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  memcpy(g_persistent.guid, pguid, 16);
  return VSCP_ERROR_NOT_SUPPORTED;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_get_version
//

int
vscp_link_callback_get_version(const void *pdata, uint8_t *pversion)
{
  if ((NULL == pdata) || (NULL == pversion)) {
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

int
vscp_link_callback_setFilter(const void *pdata, vscpEventFilter *pfilter)
{
  if ((NULL == pdata) || (NULL == pfilter)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Get pointer to context
  ctx_t *pctx = (ctx_t *) pdata;

  pctx->filter.filter_class    = pfilter->filter_class;
  pctx->filter.filter_type     = pfilter->filter_type;
  pctx->filter.filter_priority = pfilter->filter_priority;
  memcpy(pctx->filter.filter_GUID, pfilter->filter_GUID, 16);

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_setMask
//

int
vscp_link_callback_setMask(const void *pdata, vscpEventFilter *pfilter)
{
  if ((NULL == pdata) || (NULL == pfilter)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Get pointer to context
  ctx_t *pctx = (ctx_t *) pdata;

  pctx->filter.mask_class    = pfilter->mask_class;
  pctx->filter.mask_type     = pfilter->mask_type;
  pctx->filter.mask_priority = pfilter->mask_priority;
  memcpy(pctx->filter.mask_GUID, pfilter->mask_GUID, 16);

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_statistics
//

int
vscp_link_callback_statistics(const void *pdata, vscp_statistics_t *pStatistics)
{
  if ((NULL == pdata) || (NULL == pStatistics)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Get pointer to context
  ctx_t *pctx = (ctx_t *) pdata;

  memcpy(pStatistics, &pctx->statistics, sizeof(vscp_statistics_t));

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_info
//

int
vscp_link_callback_info(const void *pdata, vscp_status_t *pstatus)
{
  if ((NULL == pdata) || (NULL == pstatus)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Get pointer to context
  ctx_t *pctx = (ctx_t *) pdata;

  memcpy(pstatus, &pctx->status, sizeof(vscp_status_t));

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_rcvloop
//

int
vscp_link_callback_rcvloop(const void *pdata, vscp_event_t **pev)
{
  // BaseType_t rv;
  // twai_message_t msg = {};

  // Check pointer
  if ((NULL == pdata) || (NULL == pev)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Get pointer to context
  ctx_t *pctx = (ctx_t *) pdata;

  // Every second output '+OK\r\n' in rcvloop mode
  if ((esp_timer_get_time() - pctx->last_rcvloop_time) > 1000000) {
    pctx->last_rcvloop_time = esp_timer_get_time();
    return VSCP_ERROR_TIMEOUT;
  }

  return vscp_link_callback_retr(pdata, pev);
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_wcyd
//

int
vscp_link_callback_wcyd(const void *pdata, uint64_t *pwcyd)
{
  // Check pointers
  if ((NULL == pdata) || (NULL == pwcyd)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Get pointer to context
  // ctx_t *pctx = (ctx_t *) pdata;

  // TODO
  *pwcyd = VSCP_SERVER_CAPABILITY_TCPIP | VSCP_SERVER_CAPABILITY_DECISION_MATRIX | VSCP_SERVER_CAPABILITY_IP4 |
           /*VSCP_SERVER_CAPABILITY_SSL |*/
           VSCP_SERVER_CAPABILITY_TWO_CONNECTIONS;

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_shutdown
//

int
vscp_link_callback_shutdown(const void *pdata)
{
  // Check pointers
  if (NULL == pdata) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  return VSCP_ERROR_NOT_SUPPORTED;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_link_callback_restart
//

int
vscp_link_callback_restart(const void *pdata)
{
  // Check pointers
  if (NULL == pdata) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Get pointer to context
  // ctx_t *pctx = (ctx_t *) pdata;

  esp_restart();

  return VSCP_ERROR_SUCCESS;
}

int
vscp_link_callback_bretr(const void *pdata)
{
  // Check pointers
  if (NULL == pdata) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Get pointer to context
  // ctx_t *pctx = (ctx_t *) pdata;

  return VSCP_ERROR_SUCCESS;
}

int
vscp_link_callback_bsend(const void *pdata)
{
  // Check pointers
  if (NULL == pdata) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Get pointer to context
  // ctx_t *pctx = (ctx_t *) pdata;

  return VSCP_ERROR_SUCCESS;
}

int
vscp_link_callback_sec(const void *pdata)
{
  // Check pointers
  if (NULL == pdata) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  // Get pointer to context
  // ctx_t *pctx = (ctx_t *) pdata;

  return VSCP_ERROR_SUCCESS;
}