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

/*
 This is the header file for the VSCP Binary Protocol. This protocol is a simple binary protocol
 designed for efficient communication between a VSCP device and a client over a VSCP supported transport.
 The protocol allows clients to send commands to the device and receive responses, as well as
 send and receive events from the device in real-time.
*/

#ifndef __VSCP_BINARY_H__
#define __VSCP_BINARY_H__

#ifdef __cplusplus
{
#endif

#define VSCP_BINARY_MAX_USERNAME_LENGTH 32
#define VSCP_BINARY_MAX_PASSWORD_LENGTH 32

// Binary command codes
#define VSCP_BINARY_COMMAND_CODE_NOOP      0x0000 /* No operation command */
#define VSCP_BINARY_COMMAND_CODE_QUIT      0x0001 /* Quit connection */
#define VSCP_BINARY_COMMAND_CODE_USER      0x0002 /* Username */
#define VSCP_BINARY_COMMAND_CODE_PASS      0x0003 /* Password */
#define VSCP_BINARY_COMMAND_CODE_CHALLENGE 0x0004 /* Security challenge */
#define VSCP_BINARY_COMMAND_CODE_SEND      0x0005 /* Send event */
#define VSCP_BINARY_COMMAND_CODE_RETR      0x0006 /* Retrive event(s) */
#define VSCP_BINARY_COMMAND_CODE_OPEN      0x0007 /* Open channel */
#define VSCP_BINARY_COMMAND_CODE_CLOSE     0x0008 /* Close channel */
#define VSCP_BINARY_COMMAND_CODE_CHKDATA   0x0009 /* Check if data is available */
#define VSCP_BINARY_COMMAND_CODE_CLEAR     0x000A /* Clear input queue */
#define VSCP_BINARY_COMMAND_CODE_STAT      0x000B /* Get statistical information. */
#define VSCP_BINARY_COMMAND_CODE_INFO      0x000C /* Get status information */
#define VSCP_BINARY_COMMAND_CODE_GETCHID   0x000D /* Get channel id */
#define VSCP_BINARY_COMMAND_CODE_SETGUID   0x000E /* Set GUID for channel (privileged command) */
#define VSCP_BINARY_COMMAND_CODE_GETGUID   0x000F /* Get GUID for channel */
#define VSCP_BINARY_COMMAND_CODE_VERSION   0x0010 /* Get interface version */
#define VSCP_BINARY_COMMAND_CODE_SETFILTER 0x0011 /* Set input filter for channel */
#define VSCP_BINARY_COMMAND_CODE_SETMASK   0x0012 /* Set input mask for channel */
#define VSCP_BINARY_COMMAND_CODE_INTERFACE 0x0013 /* List interfaces */
#define VSCP_BINARY_COMMAND_CODE_TEST      0x001E /* Run test */
#define VSCP_BINARY_COMMAND_CODE_WCYD      0x001F /* WCYD - What Can You Do */
#define VSCP_BINARY_COMMAND_CODE_SHUTDOWN  0x0020 /* Shutdown device (privileged command) */
#define VSCP_BINARY_COMMAND_CODE_RESTART   0x0021 /* Restart device (privileged command) */
#define VSCP_BINARY_COMMAND_CODE_TEXT      0x0022 /* Return to text mode */

#define VSCP_BINARY_COMMAND_CODE_USER_START 0xFF00 /* Start for user command range */

#define VSCP_BINARY_COMMAND_CODE_EVENT_CONFIRM 0xFFFF /* Event confirm command */

  /*!
    @brief Handle a binary command.

    @param pdata Pointer to the context. Only the callbacks know what this is. It can be
                used to store connection specific data for the callbacks.
    @param command The command code.
    @param parg The null terminated command argument string.
    @param len Length of the argument. If argument is string the terminating null is included in the length.
    @return VSCP_ERROR_SUCCESS if all is OK, errorcode otherwise.
  */
  int vscp_handle_binary_command(const void *pdata, uint16_t command, const uint8_t *parg, size_t len);

  /*!
    @brief Handle a binary event.

    @param pdata Pointer to the context. Only the callbacks know what this is. It can be
                used to store connection specific data for the callbacks.
    @param pEvent Pointer to the event structure.
    @return VSCP_ERROR_SUCCESS if all is OK, errorcode otherwise.
  */
  int vscp_handle_binary_event(const void *pdata, vscp_event_t *pEvent);

  ///////////////////////////////////////////////////////////////////////////////
  //                             Callbacks
  ///////////////////////////////////////////////////////////////////////////////

  /**
   * @name Callbacks for a VSCP link protocol implementation
   */

  /*!
   * @fn vscp_binary_callback_reply
   * @brief This callback is executed when a reply should be sent to the client.
   *
   * @param pdata Pointer to context.
   * @param command Command code for which this is a reply
   * @param error Error code for the reply
   * @param parg Null terminated string argument for the reply. Can be NULL if no argument.
   * @param len Length of the argument string including the terminating null. Can be zero if no argument.
   * @return VSCP_ERROR_SUCCESS if all is OK, errorcode otherwise.
   *
   * Send a reply frame to the client. The callback should pack, calculate ad add crc and encrypt the reply
   * frame as needed and send it to the client.
   */
  int vscp_binary_callback_reply(const void *pdata, uint16_t command, uint16_t error, const uint8_t *parg, size_t len);

  /**
   * @fn vscp_binary_callback_quit
   * @brief This callback is executed when the 'quit' command is received.
   *
   * @param pdata Pointer to context.
   * @return VSCP_ERROR_SUCCESS if all is OK, errorcode otherwise.
   *
   * The callback should shutdown the connection with the client and send a positive response if it was successful in
   * doing so and a negative response if not.
   */

  int vscp_binary_callback_quit(const void *pdata);

  /**
   * @fn vscp_binary_callback_write_client
   * @brief Send message to client.
   *
   * @param pdata Pointer to context.
   * @param pmsg Pointer to message to send
   * @return VSCP_ERROR_SUCCESS if all is OK, errorcode otherwise.
   *
   * Send null terminated data to websocket client. The callback should send the data
   * and return a positive response if it was successful in doing so and a
   * negative response if not.
   */

  int vscp_binary_callback_write_client(const void *pdata, const char *msg);

  /**
   * @fn vscp_binary_callback_disconnect_client
   * @brief Disconnect websocket client
   *
   * @param pdata Pointer to context..
   * @return VSCP_ERROR_SUCCESS if all is OK, errorcode otherwise.
   */

  int vscp_binary_callback_disconnect_client(const void *pdata);

  /**
   * @fn vscp_binary_callback_event_received
   * @brief Event has been received from websocket client.
   *
   * @param pdata Pointer to context..
   * @param pex Pointer to received event ex.
   * @return VSCP_ERROR_SUCCESS if all is OK, errorcode otherwise.
   */

  int vscp_binary_callback_event_received(const void *pdata, const vscp_event_t *pev);

  /**
   * @fn vscp_binary_callback_check_user
   * @brief Check username
   *
   * @param pdata Pointer to context.
   * @param user Username to check
   *
   * There is two ways to implement this command handler
   *
   * 1.) Save username without checking it and always respond with
   *     vscp_binary_MSG_USERNAME_OK. This is the preferred way and don't give
   *     information on valid usernames to clients.
   * 2.) Check username and respond with vscp_binary_MSG_USENAME_OK
   *     only if it is valid. Send vscp_binary_MSG_GOODBY and return
   *     VSCP_ERROR_ERROR. In this case the connection will be closed.
   */

  int vscp_binary_callback_user(const void *pdata, const char *user);

  /**
   * @fn vscp_binary_callback_check_password
   * @brief Check password
   *
   * @param pdata Pointer to context.
   * @param password Password to check
   * @return Return VSCP_ERROR_SUCCESS if logged in error code else.
   *
   * This is the point where a client logs in to the system. Write
   * vscp_binary_MSG_NEED_USERNAME to client if no user name has been entered
   * prior to password command.
   * Write vscp_binary_MSG_PASSWORD_ERROR to client if the password is not correct.
   * Write vscp_binary_MSG_PASSWORD_OK to client if logged in.
   */

  int vscp_binary_callback_password(const void *pdata, const char *password);

  /**
   * @fn vscp_binary_callback_challenge
   * @brief Dop challenge sequency
   *
   * @param pdata Pointer to context.
  * @param challenge Pointer to buffer that will receive the challenge.
  * @param challenge_len Size of challenge buffer.
   * @return Return VSCP_ERROR_SUCCESS if logged in error code else.
   *
   */

  int vscp_binary_callback_challenge(const void *pdata, uint8_t *challenge, size_t challenge_len);

  /**
   * @fn vscp_binary_callback_check_authenticated
   * @brief Check if client is authenticated
   *
   *  @param pdata Pointer to context.
   * @return Return VSCP_ERROR_SUCCESS if validated.
   */

  int vscp_binary_callback_check_authenticated(const void *pdata);

  /**
   * @fn vscp_binary_callback_check_privilege
   * @brief Check if client has enough rights to use command
   *
   *  @param pdata Pointer to context.
   *  @param priv Privilege level 0-15 needed to use command.
   * @return Return VSCP_ERROR_SUCCESS if privileged (>= priv).
   */

  //int vscp_binary_callback_check_privilege(const void *pdata, uint8_t priv);

  /**
   * @fn vscp_binary_callback_test
   * @brief Do test command
   *
   *  @param pdata Pointer to context.
   *  @param arg Argument to test command.
   *  @len Length of argument string including terminating null.
   *  @return Return VSCP_ERROR_SUCCESS if logged in error code else.
   */

  int vscp_binary_callback_test(const void *pdata, const uint8_t *arg, size_t len);

  /**
   * @fn vscp_binary_callback_check_data
   * @brief Check if there are events ready to be received
   *
   * @param pdata Pointer to context.
   * @param pcount Pointer to variable that will receive the count of events ready to be received.
   * @return VSCP_ERROR_SUCCESS if all is OK, errorcode otherwise.
   */

  int vscp_binary_callback_check_data(const void *pdata, uint32_t *pcount);

  /**
   * @fn vscp_binary_callback_send
   * @brief Send event ('send').
   *
   * @param pdata Pointer to context.
   * @param pev Pointer to event ex to send. The callback should send the event and return a positive response if it was
   *            successful in doing so and a negative response if not.
   * @return Return VSCP_ERROR_SUCCESS if logged in error code else.
   *
   */

  int vscp_binary_callback_send_event(const void *pdata, const vscp_event_t *pev);

  
  /**
   * @fn vscp_binary_callback_get_event
   * @brief Get event ('retr').
   *
   * @param pdata Pointer to context.
   * @param pev Pointer to event that will get event from input queue if available. If
   *            VSCP_ERROR_SUCCESS is returned this will point to an allocated
                event and it is up to the calling routine to release the memory
                when done with it.
   * @return Return VSCP_ERROR_SUCCESS if logged in error code else.
   *
   * VSCP_ERROR_INVALID_HANDLE - (msg=vscp_binary_MSG_NOT_ACCREDITED) is not logged in.
   * VSCP_ERROR_INVALID_PERMISSION - (msg=vscp_binary_MSG_LOW_PRIVILEGE_ERROR) is returned
   *    if the user is not allowed to use send
   * VSCP_ERROR_RCV_EMPTY - (msg=vscp_binary_MSG_NO_MSG) is returned if no event is available.
   *
   * On a node that can't send events asynchoniously this callback can be used to get events.
   */

  int vscp_binary_callback_get_event(const void *pdata, vscp_event_t *pev);

  /**
   * @fn vscp_binary_callback_get_event
   * @brief Get event ('retr').
   *
   * @param pdata Pointer to context.
   * @param pex Pointer to event ex that will get event from input queue if available. If
   *            VSCP_ERROR_SUCCESS is returned this will point to an allocated
                event and it is up to the calling routine to release the memory
                when done with it.
   * @return Return VSCP_ERROR_SUCCESS if logged in error code else.
   *
   * VSCP_ERROR_INVALID_HANDLE - (msg=vscp_binary_MSG_NOT_ACCREDITED) is not logged in.
   * VSCP_ERROR_INVALID_PERMISSION - (msg=vscp_binary_MSG_LOW_PRIVILEGE_ERROR) is returned
   *    if the user is not allowed to use send
   * VSCP_ERROR_RCV_EMPTY - (msg=vscp_binary_MSG_NO_MSG) is returned if no event is available.
   *
   * On a node that can't send events asynchoniously this callback can be used to get events.
   */

  int vscp_binary_callback_get_eventex(const void *pdata, vscp_event_ex_t *pex);

  /*!
   * @fn vscp_binary_callback_send_async_event
   * @brief Send event to client.
   *
   * @param pdata Pointer to context.
   * @param pev Pointer to event to send. The callback should send the event and return a positive response if it was
   *            successful in doing so and a negative response if not.
   * @return Return VSCP_ERROR_SUCCESS if logged in error code else.
   *
   * This callback is used by the server to send events to the client. The callback should send the event using
   * VSCP frame=0, type=1 (https://grodansparadis.github.io/vscp-doc-spec/#/./vscp_over_binary?id=vscp-frame-type-1)
   */
  int vscp_binary_callback_send_asyncevent(const void *pdata, vscp_event_t *pev);

  /*!
   * @fn vscp_binary_callback_open
   * @brief Open channel for receiving events.
   *
   * @param pdata Pointer to context.
   * @return Return VSCP_ERROR_SUCCESS if logged in error code else.
   *
   * This callback is executed when the client sends the 'open' command. The callback should open the channel for
   * receiving events and return a positive response if the connection was successfully opened, or a negative response
   * if there was a failure.
   *
   * When the connection is opened, the server should start sending events to the client based on the event filter that
   * has been set for the connection. The server should also be prepared to receive commands from the client and respond
   * to them appropriately while the connection is open.
   */

  int vscp_binary_callback_open(const void *pdata);

  /**
   * @fn vscp_binary_callback_close
   * @brief Close channel for receiving events.
   *
   * @param pdata Pointer to context.
   * @return Return VSCP_ERROR_SUCCESS if logged in error code else.
   *
   * This callback is executed when the client sends the 'close' command. The callback should close the channel for
   * receiving events and return a positive response if the connection was successfully closed, or a negative response
   * if there was a failure.
   */

  int vscp_binary_callback_close(const void *pdata);

  /*!
   * @fn vscp_binary_callback_check_open
   * @brief Check if channel is open for receiving events.
   *
   * @param pdata Pointer to context.
   * @return Return true if the channel is open.
   *
   * This callback can be used to check if the channel is open for asynchronious receiving of events. This can be useful
   * in the implementation of the RETR command to determine if events should be sent asynchronously or if the client
   * needs to use the RETR command to retrieve events.
   */
  bool vscp_binary_callback_is_open(const void *pdata);

  /**
   * @fn vscp_binary_callback_clrAll
   * @brief Clear the output queue
   *
   * @param pdata Pointer to context.
   * @return Return VSCP_ERROR_SUCCESS if logged in error code else.
   *
   * On a node that can't send events asynchoniously this callback can be used to clear
   * the events in the input queue.
   */

  int vscp_binary_callback_clrall(const void *pdata);

  /**
   * @fn vscp_binary_callback_get_chid
   * @brief Get channel id
   *
   * @param pdata Pointer to context.
   * @param chid Pointer to variable that will get channel id
   * @return Return VSCP_ERROR_SUCCESS on success, else error code.
   */

  int vscp_binary_callback_get_chid(const void *pdata, uint32_t *pchid);

  /**
   * @fn vscp_binary_callback_set_guid
   * @brief Set device GUID
   *
   * @param pdata Pointer to context.
   * @param pguid Pointer to GUID to set
   * @return Return VSCP_ERROR_SUCCESS on success, else error code.
   */

  int vscp_binary_callback_set_guid(const void *pdata, uint8_t *pguid);

  /**
   * @fn vscp_binary_callback_get_guid
   * @brief Get device GUID
   *
   * @param pdata Pointer to context.
   * @param pguid Pointer to GUID that will get device GUID
   * @return Return VSCP_ERROR_SUCCESS on success, else error code.
   */

  int vscp_binary_callback_get_guid(const void *pdata, uint8_t *pguid);

  /**
   * @fn vscp_binary_callback_get_version
   * @brief Get device version
   *
   * @param pdata Pointer to context.
   * @param pversion Pointer to four byte version array that will get device version
   * @return Return VSCP_ERROR_SUCCESS on success, else error code.
   */

  int vscp_binary_callback_get_version(const void *pdata, uint8_t *pversion);

  /**
   * @fn vscp_binary_callback_setfilter
   * @brief Set filter part of filter
   *
   * @param pdata Pointer to context.
   * @param pfilter Filter data. Mask data not used.
   * @return Return VSCP_ERROR_SUCCESS on success, else error code.
   */

  int vscp_binary_callback_setfilter(const void *pdata, const vscpEventFilter *pfilter);

  /**
   * @fn vscp_binary_callback_setmask
   * @brief Set mask part of filter
   *
   * @param pdata Pointer to context.
   * @param pfilter Mask data. Filter data not used.
   * @return Return VSCP_ERROR_SUCCESS on success, else error code.
   */

  int vscp_binary_callback_setmask(const void *pdata, const vscpEventFilter *pfilter);

  /**
   * @fn vscp_binary_callback_statistics
   * @brief Get statistics info
   *
   * @param pdata Pointer to context.
   * @param pStatistics Pointer to statistics structure
   * @return Return VSCP_ERROR_SUCCESS on success, else error code.
   */

  int vscp_binary_callback_statistics(const void *pdata, vscp_statistics_t *pStatistics);

  /**
   * @fn vscp_binary_callback_info
   * @brief Set mask part of filter
   *
   * @param pdata Pointer to context.
   * @param pStatus Pointer to status structure
   * @return Return VSCP_ERROR_SUCCESS on success, else error code.
   */

  int vscp_binary_callback_info(const void *pdata, vscp_status_t *pstatus);

  /**
   * @fn vscp_binary_callback_get_interface_count
   * @brief Get number of defined interfaces.
   *
   * @param pdata Pointer to context.
   * @return Number of interfaces is returned. If no interfaces are defined
   *         zero is returned (as expected).
   */

  int vscp_binary_callback_get_interface_count(const void *pdata, uint16_t *pcount);

  /**
   * @fn vscp_binary_callback_get_interface
   * @brief Get one interface GUID.
   *
   * @param pdata Pointer to context.
   * @param index Index of interface to get.
   * @param pif Pointer to interface information structure that wil get data for the interface.
   * @return VSCP_ERROR_SUCCESS if an interface is returned. If not VSCP_ERROR_UNKNOWN_ITEM
   *         is returned.
   */

  int vscp_binary_callback_get_interface(const void *pdata, uint16_t index, vscp_interface_info_t *pif);

  /*!
    * @fn vscp_binary_callback_interface_open
    * @brief Open interface
    *
    * @param pdata Pointer to context.
    * @param idx Index of interface to open.
    * @return VSCP_ERROR_SUCCESS if the interface gets opened. If not VSCP_ERROR_UNKNOWN_ITEM
    *         is returned.

  */
  int vscp_binary_callback_interface_open(const void *pdata, uint16_t idx);

  /**
   * @brief Close interface
   *
   * @param pdata Pointer to context
   * @param idx Index of interface to close.
   *
   * @return VSCP_ERROR_SUCCESS if the interface gest closed.
   *  VSCP_ERROR_NOT_SUPPORTED if not supported. Other error
   *  codes if an error occurs.
   */
  int vscp_binary_callback_interface_close(const void *pdata, uint16_t idx);

  /**
   *
   * @fn vscp_binary_callback_rcvloop
   * @brief Callback for active rcvloop
   *
   * @param pdata Pointer to context.
   * @param pex Pointer to pointer of event that will get data if available
   * @return Return VSCP_ERROR_SUCCESS on success, else error code.
   *
   * VSCP_ERROR_SUCCESS - Event is available
   * VSCP_ERROR_RCV_EMPTY - No event available
   * VSCP_ERROR_TIMEOUT - Time to send '+OK\r\n
   * Other error on error condition
   *
   * This function is called periodically when the rcvloop is active. It should send events in the
   * transmit fifo to the client and send a '+OK\r\n' response each second to the client.
   */

  int vscp_binary_callback_rcvloop(const void *pdata, vscp_event_t **pev);

  /**
   * @fn vscp_binary_callback_wcyd
   * @brief Get what can you do info
   *
   * @param pdata Pointer to context.
   * @param pwcyd Pointer to capabilities integer
   * @return Return VSCP_ERROR_SUCCESS on success, else error code.
   */

  int vscp_binary_callback_wcyd(const void *pdata, uint64_t *pwcyd);

  /**
   * @fn vscp_binary_callback_shutdown
   * @brief Shutdown the system to a safe state
   *
   * @param pdata Pointer to context
   * @return Return VSCP_ERROR_SUCCESS on success, else error code.
   *
   * If not implemented just return VSCP_ERROR_SUCCESS. And yes you
   * probably don't want to implement it.
   */

  int vscp_binary_callback_shutdown(const void *pdata);

  /**
   * @fn vscp_binary_callback_restart
   * @brief Restart the system
   *
   * @param pdata Pointer to context
   * @return Return VSCP_ERROR_SUCCESS on success, else error code.
   *
   * If not implemented just return VSCP_ERROR_SUCCESS. And yes you
   * probably don't want to implement it.
   */

  int vscp_binary_callback_restart(const void *pdata);

  /*!
   * @fn vscp_binary_callback_text
   * @brief Return to text mode
   *
   * @param pdata Pointer to context
   * @return Return VSCP_ERROR_SUCCESS on success, else error code.
   *
   * This callback is executed when the client sends the 'text' command.
   * The callback should switch the connection back to text mode and return
   * a positive response if the connection was successfully switched, or a
   * negative response if there was a failure.
   */
  int vscp_binary_callback_text(const void *pdata);

  /**
   * @fn vscp_binary_callback_retreive
   * @brief Get binary encoded frame
   *
   * @param pdata Pointer to context
   * @return Return VSCP_ERROR_SUCCESS on success, else error code.
   */
  int vscp_binary_callback_retreive(const void *pdata);

  /*!
   * @fn vscp_binary_callback_user_command
   * @brief Handle user defined command
   *
   * @param pdata Pointer to context
   * @param command Command code for the user defined command. Will be in the range VSCP_BINARY_COMMAND_CODE_USER_START
   * and up.
   * @param parg Pointer to argument data for the command. Can be NULL if no argument.
   * @param len Length of the argument data. Can be zero if no argument.
   * @return Return VSCP_ERROR_SUCCESS on success, else error code.
   *
   * This callback is executed when a user defined command is received. The callback should handle the command and
   * return a positive response if it was successful in doing so and a negative response if not.
   */
  int vscp_binary_callback_user_command(const void *pdata, uint16_t command, const uint8_t *parg, size_t len);

  // --------------------------------------------------------------------------
  //                                 Binary
  // --------------------------------------------------------------------------

  /**
   * @fn vscp_binary_callback_bsend
   * @brief Send binary encoded frame
   *
   * @param pdata Pointer to context
   * @return Return VSCP_ERROR_SUCCESS on success, else error code.
   */
  int vscp_binary_callback_bsend(const void *pdata);

  /**
   * @fn vscp_binary_callback_brcvloop
   * @brief Enter binary receive loop
   *
   * @param pdata Pointer to context
   * @return Return VSCP_ERROR_SUCCESS on success, else error code.
   */

  int vscp_binary_callback_brcvloop(const void *pdata);

  /**
   * @fn vscp_binary_callback_sec
   * @brief Set security level.
   * Argument is
   * 0 - No security
   * 1 - Encrypt with AES-128
   * 2 - Encrypt with AES-192
   * 3 - Encrypt with AES-256
   *
   * @param pdata Pointer to context
   * @return Return VSCP_ERROR_SUCCESS on success, else error code.
   */

  int vscp_binary_callback_sec(const void *pdata);

  /**
  @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __VSCP_BINARY_H__ */