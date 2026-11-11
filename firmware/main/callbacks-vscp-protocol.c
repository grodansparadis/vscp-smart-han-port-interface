// FILE: callbacks-vscp-protocol.c

// This file holds callbacks for the VSCP protocol

/* ******************************************************************************
 * 	VSCP (Very Simple Control Protocol)
 * 	https://www.vscp.org
 *
 * The MIT License (MIT)
 *
 * Copyright (C) 2000-202 Ake Hedman, Grodans Paradis AB <info@grodansparadis.com>
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

#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// #include <freertos/FreeRTOS.h>
// #include <freertos/task.h>
// #include <freertos/queue.h>
// #include "freertos/semphr.h"
// #include <freertos/event_groups.h>

#include <esp_timer.h>
#include <lwip/sockets.h>

#include "vscp-compiler.h"
#include "vscp-projdefs.h"

#include <vscp-fifo.h>

#include "main.h"
#include "regdefs.h"
#include "tcpsrv.h"

// Defines from demo.c

extern uint8_t device_guid[16];
extern vscp_fifo_t fifoEventsIn;
extern ctx_t gctx[MAX_TCP_CONNECTIONS];
extern struct _eeprom_ eeprom;

// ****************************************************************************
//                        VSCP protocol callbacks
// ****************************************************************************

///////////////////////////////////////////////////////////////////////////////
// vscp_frmw2_callback_get_ms
//

int
vscp_frmw2_callback_get_ms(void* const puserdata, uint32_t *ptime)
{
  if (NULL == ptime) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  *ptime = getMilliSeconds();
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_frmw2_callback_get_guid
//

const uint8_t *
vscp_frmw2_callback_get_guid(void* const puserdata)
{
  return device_guid;
}

#ifdef THIS_FIRMWARE_ENABLE_WRITE_2PROTECTED_LOCATIONS

///////////////////////////////////////////////////////////////////////////////
// vscp_frmw2_callback_write_manufacturer_id
//

int
vscp_frmw2_callback_write_manufacturer_id(void* const puserdata, uint8_t pos, uint8_t val)
{
  if (pos < 4) {
    // TODO // TODO eeprom_write(&eeprom, STDREG_MANUFACTURER_ID0 + pos, val);
  }
  else if (pos < 8) {
    // TODO // TODO eeprom_write(&eeprom, STDREG_MANUFACTURER_SUBID0 + pos - 4, val);
  }

  // Commit changes to 'eeprom'
  // TODO eeprom_commit(&eeprom);

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_frmw2_callback_write_guid
//

int
vscp_frmw2_callback_write_guid(void* const puserdata, uint8_t pos, uint8_t val)
{
  // TODO eeprom_write(&eeprom, STDREG_GUID0 + pos, val);

  // Commit changes to 'eeprom'
  // TODO eeprom_commit(&eeprom);
  

  return VSCP_ERROR_SUCCESS;
}

#endif

///////////////////////////////////////////////////////////////////////////////
// vscp_frmw2_callback_read_user_reg
//

int
vscp_frmw2_callback_read_user_reg(void* const puserdata, uint32_t reg, uint8_t *pval)
{
  // Check pointers (pdata allowed to be NULL)
  if (NULL == pval) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  if (REG_DEVICE_ZONE == reg) {
    *pval = 0; // TODO  eeprom_read(&eeprom, REG_DEVICE_ZONE);
  }
  else if (REG_DEVICE_SUBZONE == reg) {
    *pval = 0; // TODO  eeprom_read(&eeprom, REG_DEVICE_SUBZONE);
  }
  else if (REG_LED_CTRL == reg) {
    *pval = 0; // TODO  eeprom_read(&eeprom, REG_LED_CTRL);
  }
  else if (REG_LED_STATUS == reg) {
    *pval = 0; // TODO gpio_get(LED_PIN);
  }
  else if (REG_LED_BLINK_INTERVAL == reg) {
    *pval = 0; // TODO  eeprom_read(&eeprom, REG_LED_BLINK_INTERVAL);
  }
  else if (REG_IO_CTRL1 == reg) {
    *pval = 0; // TODO  eeprom_read(&eeprom, REG_IO_CTRL1);
  }
  else if (REG_IO_CTRL2 == reg) {
    *pval = 0; // TODO  eeprom_read(&eeprom, REG_IO_CTRL2);
  }
  else if (REG_IO_STATUS == reg) {
    uint32_t all = 0; // TODO  gpio_get_all();
    *pval        = (all >> 2) & 0xff;
  }
  else if (REG_TEMP_CTRL == reg) {
    *pval = 0; // TODO  eeprom_read(&eeprom, REG_TEMP_CTRL);
  }
  else if (REG_TEMP_RAW_MSB == reg) {
    //float temp = read_onboard_temperature();
    //*pval      = (((uint16_t) (100 * temp)) >> 8) & 0xff;
  }
  else if (REG_TEMP_RAW_LSB == reg) {
    //float temp = read_onboard_temperature();
    //*pval      = ((uint16_t) (100 * temp)) & 0xff;
  }
  else if (REG_TEMP_CORR_MSB == reg) {
    *pval = 0; // TODO  eeprom_read(&eeprom, REG_TEMP_CORR_MSB);
  }
  else if (REG_TEMP_CORR_LSB == reg) {
    *pval = 0; // TODO  eeprom_read(&eeprom, REG_TEMP_CORR_LSB);
  }
  else if (REG_TEMP_INTERVAL == reg) {
    *pval = 0; // TODO  eeprom_read(&eeprom, REG_TEMP_INTERVAL);
  }
  else if (REG_ADC0_CTRL == reg) {
    *pval = 0; // TODO  eeprom_read(&eeprom, REG_ADC0_CTRL);
  }
  else if (REG_ADC0_MSB == reg) {
    float adc = 0; // TODO 0; // TODO read_adc(0);
    *pval     = (((uint16_t) (100 * adc)) >> 8) & 0xff;
  }
  else if (REG_ADC0_LSB == reg) {
    float adc = 0; // TODO 0; // TODO read_adc(0);
    *pval     = ((uint16_t) (100 * adc)) & 0xff;
  }
  else if (REG_ADC1_CTRL == reg) {
    *pval = 0; // TODO  eeprom_read(&eeprom, REG_ADC0_CTRL);
  }
  else if (REG_ADC1_MSB == reg) {
    float adc = 0; // TODO read_adc(1);
    *pval     = (((uint16_t) (100 * adc)) >> 8) & 0xff;
  }
  else if (REG_ADC1_LSB == reg) {
    float adc = 0; // TODO read_adc(1);
    *pval     = ((uint16_t) (100 * adc)) & 0xff;
  }
  else if (REG_ADC2_CTRL == reg) {
    *pval = 0; // TODO  eeprom_read(&eeprom, REG_ADC0_CTRL);
  }
  else if (REG_ADC2_MSB == reg) {
    float adc = 0; // TODO 0; // TODO read_adc(2);
    *pval     = (((uint16_t) (100 * adc)) >> 8) & 0xff;
  }
  else if (REG_ADC2_LSB == reg) {
    float adc = 0; // TODO 0; // TODO read_adc(2);
    *pval     = ((uint16_t) (100 * adc)) & 0xff;
  }
  else if ((REG_BOARD_ID0 >= reg) && (REG_BOARD_ID8 <= reg)) {
    // TODO
    // pico_unique_board_id_t boardid;
    // pico_get_unique_board_id(&boardid);
    // *pval = boardid.id[reg - REG_BOARD_ID0];
  }
  else {
    // Invalid register
    return VSCP_ERROR_PARAMETER;
  }

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_frmw2_callback_write_user_reg
//

int
vscp_frmw2_callback_write_user_reg(void* const puserdata, uint32_t reg, uint8_t val)
{
  if (REG_DEVICE_ZONE == reg) {
    // TODO eeprom_write(&eeprom, REG_DEVICE_ZONE, val);
  }
  else if (REG_DEVICE_SUBZONE == reg) {
    // TODO eeprom_write(&eeprom, REG_DEVICE_SUBZONE, val);
  }
  else if (REG_LED_CTRL == reg) {
    // TODO eeprom_write(&eeprom, REG_LED_CTRL, val);
  }
  else if (REG_LED_STATUS == reg) {
    if (val) {
      // TODO gpio_put(LED_PIN, 1);
    }
    else {
      // TODO gpio_put(LED_PIN, 0);
    }
  }
  else if (REG_LED_BLINK_INTERVAL == reg) {
    // TODO eeprom_write(&eeprom, REG_LED_BLINK_INTERVAL, val);
  }
  else if (REG_IO_CTRL1 == reg) {
    // TODO eeprom_write(&eeprom, REG_IO_CTRL1, val);
  }
  else if (REG_IO_CTRL2 == reg) {
    // TODO eeprom_write(&eeprom, REG_IO_CTRL2, val);
  }
  else if (REG_IO_STATUS == reg) {
  }
  else if (REG_TEMP_CTRL == reg) {
    // TODO eeprom_write(&eeprom, REG_TEMP_CTRL, val);
  }
  else if (REG_TEMP_CORR_MSB == reg) {
    // TODO eeprom_write(&eeprom, REG_TEMP_CORR_MSB, val);
  }
  else if (REG_TEMP_CORR_LSB == reg) {
    // TODO eeprom_write(&eeprom, REG_TEMP_CORR_LSB, val);
  }
  else if (REG_TEMP_INTERVAL == reg) {
    // TODO eeprom_write(&eeprom, REG_TEMP_INTERVAL, val);
  }
  else if (REG_ADC0_CTRL == reg) {
    // TODO eeprom_write(&eeprom, REG_ADC0_CTRL, val);
  }
  else if (REG_ADC1_CTRL == reg) {
    // TODO eeprom_write(&eeprom, REG_ADC1_CTRL, val);
  }
  else if (REG_ADC2_CTRL == reg) {
    // TODO eeprom_write(&eeprom, REG_ADC2_CTRL, val);
  }
  else {
    return VSCP_ERROR_PARAMETER;
  }

  // Commit changes to 'eeprom'
  // TODO eeprom_commit(&eeprom);

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_frmw2_callback_enter_bootloader
//

void
vscp_frmw2_callback_enter_bootloader(void *const puserdata)
{
  
}

///////////////////////////////////////////////////////////////////////////////
// vscp_frmw2_callback_report_dmatrix
//

int
vscp_frmw2_callback_report_dmatrix(void* const puserdata)
{
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_frmw2_callback_report_mdf
//

int
vscp_frmw2_callback_report_mdf(void* const puserdata)
{
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_frmw2_callback_report_events_of_interest
//

int
vscp_frmw2_callback_report_events_of_interest(void* const puserdata)
{
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_frmw2_callback_get_timestamp
//

uint64_t
vscp_frmw2_callback_get_timestamp(void* const puserdata)
{
  return esp_timer_get_time();
}

///////////////////////////////////////////////////////////////////////////////
// vscp_frmw2_callback_get_time
//

int
vscp_frmw2_callback_get_time(void* const puserdata, const vscp_event_ex_t *pex)
{
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_frmw2_callback_send_event_ex
//

int
vscp_frmw2_callback_send_event_ex(void *const puserdata, vscp_event_ex_t *pex)
{
  for (int i = 0; i < MAX_TCP_CONNECTIONS; i++) {

    // // Only if user is validated
    // if (gctx[i].bValidated) {
    //   vscp_event_t *pnew = vscp_fwhlp_mkEventCopy(pex);
    //   if (NULL == pnew) {
    //     return VSCP_ERROR_MEMORY;
    //   }
    //   else {
    //     pnew->obid = 0xffffffff; // The device
    //     if (vscp_fifo_write(&gctx[i].fifoEventsOut, pnew)) {
    //       printf("Written to fifo\n");
    //     }
    //     else {
    //       printf("Failed to write to fifo\n");
    //       vscp_fwhlp_deleteEvent(&pnew);
    //     }
    //   }
    // }
  }

  // Remove original event
  // vscp_fwhlp_deleteEvent(&pex);

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_frmw2_callback_send_eventEx
//

int
vscp_frmw2_callback_send_eventEx(void* const puserdata, vscp_event_ex_t *pex)
{
  for (int i = 0; i < MAX_TCP_CONNECTIONS; i++) {
    // // Only if user is validated
    // if (gctx[i].bValidated) {
    //   vscp_event_t *pnew = vscp_fwhlp_mkEventCopy(pex);
    //   if (NULL == pnew) {
    //     return VSCP_ERROR_MEMORY;
    //   }
    //   else {
    //     pnew->obid = 0xffffffff; // The device
    //     if (vscp_fifo_write(&gctx[i].fifoEventsOut, pnew)) {
    //       printf("Written to fifo\n");
    //     }
    //     else {
    //       printf("Failed to write to fifo\n");
    //       vscp_fwhlp_deleteEvent(&pnew);
    //     }
    //   }
    // }
  }

  // Remove original event
  // vscp_fwhlp_deleteEvent(&pex);

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_frmw2_callback_restore_defaults
//

int
vscp_frmw2_callback_restore_defaults(void *const puserdata)
{
  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_frmw2_callback_write_user_id
//

int
vscp_frmw2_callback_write_user_id(void* const puserdata, uint8_t pos, uint8_t val)
{
  // TODO // TODO eeprom_write(&eeprom, STDREG_USER_ID0 + pos, val);

  // Commit changes to 'eeprom'
  // TODO eeprom_commit(&eeprom);

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_frmw2_callback_get_ip_addr
//

int
vscp_frmw2_callback_get_ip_addr(void *const puserdata, uint8_t *pipaddr, uint8_t size)
{
  if (NULL == pipaddr) {
    return VSCP_ERROR_PARAMETER;
  }
  else {
    // memcpy(pipaddr, net_info.ip, 4); TODO
  }

  return VSCP_ERROR_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// vscp_frmw2_callback_set_event_time
//

int
vscp_frmw2_callback_set_event_time(void* const puserdata, vscpEventEx* const pex)
{
  if (NULL == pex) {
    return VSCP_ERROR_PARAMETER;
  }

  return VSCP_ERROR_SUCCESS;
}

#ifdef THIS_FIRMWARE_VSCP_DISCOVER_SERVER

///////////////////////////////////////////////////////////////////////////////
// vscp_frmw2_callback_high_end_server_response
//

int
vscp_frmw2_callback_high_end_server_response(const void *pUserData)
{
  return VSCP_ERROR_SUCCESS;
}

#endif