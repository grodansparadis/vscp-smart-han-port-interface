/*
  File: udpsrv.c

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

  UDP send: echo "Hello from PC" | nc -w1 -u 192.168.1.112 33333
  UDP receive: echo "Hello from PC" | nc -w1 -u 192.168.1.112 33333

  UDP client: nc -u 192.168.1.112 33333
  UDP server: nc -u -l 192.168.1.112 -p 33333

  Config
  ------
  Enable
  ip-address for server
  port
  Encryption
  Encryption key
  Valid client ip's
*/

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <string.h>
#include <sys/param.h>

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include <vscp.h>
#include <vscp-firmware-helper.h>

#include "main.h"
#include "udpsrv.h"

#define BUFFER_SIZE 1024

// ============================================================================
//                              Global Variables
// ============================================================================

// Persistent configuration from main module
extern node_persistent_config_t g_persistent;

extern transport_t tr_udp;

// Logging tag for ESP-IDF logger
static const char *TAG = "udpsrv";

#define UDP_HOST_MAX_LEN 128

#if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
////////////////////////////////////////////////////////////////////////////
// is_ipv4_multicast_addr
//
// Return true if an IPv4 address (network byte order) is in 224.0.0.0/4.

static bool
is_ipv4_multicast_addr(uint32_t addr_net_order)
{
  uint32_t addr_host_order = ntohl(addr_net_order);
  return (addr_host_order >= 0xE0000000UL) && (addr_host_order <= 0xEFFFFFFFUL);
}
#endif

////////////////////////////////////////////////////////////////////////////
// normalize_udp_host
//
// Normalize destination host string by optionally removing udp:// prefix,
// trimming any path part, and stripping a trailing :port section.

static void
normalize_udp_host(char *dst, size_t dstlen, const char *src)
{
  if ((NULL == dst) || (0 == dstlen)) {
    return;
  }

  dst[0] = '\0';
  if (NULL == src) {
    return;
  }

  const char *host = src;
  if (0 == strncmp(host, "udp://", 6)) {
    host += 6;
  }

  size_t i = 0;
  while ((host[i] != '\0') && (host[i] != '/') && (i < (dstlen - 1))) {
    dst[i] = host[i];
    i++;
  }
  dst[i] = '\0';

  char *colon = strchr(dst, ':');
  if (NULL != colon) {
    bool only_digits_after_colon = true;
    for (char *p = colon + 1; *p != '\0'; ++p) {
      if ((*p < '0') || (*p > '9')) {
        only_digits_after_colon = false;
        break;
      }
    }

    if (only_digits_after_colon) {
      *colon = '\0';
    }
  }
}

////////////////////////////////////////////////////////////////////////////
// resolve_udp_ipv4_host
//
// Resolve an IPv4 destination from host string. Accepts IPv4 literals and
// hostnames. Returns false if the host could not be resolved.

static bool
resolve_udp_ipv4_host(struct in_addr *paddr, const char *host)
{
  if ((NULL == paddr) || (NULL == host) || (0 == strlen(host))) {
    return false;
  }

  if (0 != inet_aton(host, paddr)) {
    return true;
  }

  struct addrinfo hints;
  struct addrinfo *result = NULL;
  memset(&hints, 0, sizeof(hints));
  hints.ai_family   = AF_INET;
  hints.ai_socktype = SOCK_DGRAM;

  if (0 != getaddrinfo(host, NULL, &hints, &result)) {
    return false;
  }

  bool bResolved = false;
  if ((NULL != result) && (AF_INET == result->ai_family) && (NULL != result->ai_addr)) {
    struct sockaddr_in *addr4 = (struct sockaddr_in *) result->ai_addr;
    *paddr                    = addr4->sin_addr;
    bResolved                 = true;
  }

  if (NULL != result) {
    freeaddrinfo(result);
  }

  return bResolved;
}

/*
// Host resolution example
struct hostent *he = gethostbyname("myserver.local");
dest_addr.sin_addr = *((struct in_addr *)he->h_addr);


#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

void udp_broadcast_task(void *pvParameters)
{
    const char *message = "Hello from ESP32 UDP Broadcast!";
    const int port = 12345;

    int sock;
    struct sockaddr_in broadcast_addr;
    int broadcast_enable = 1;

    // Create UDP socket
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        printf("Socket creation failed\n");
        vTaskDelete(NULL);
        return;
    }

    // Enable broadcast option (not needed for unicast, but good for broadcast)
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST,
               &broadcast_enable, sizeof(broadcast_enable));

    // Configure broadcast address
    memset(&broadcast_addr, 0, sizeof(broadcast_addr));
    broadcast_addr.sin_family = AF_INET;
    broadcast_addr.sin_port = htons(port);
    broadcast_addr.sin_addr.s_addr = inet_addr("255.255.255.255");
    //broadcast_addr.sin_addr.s_addr = inet_addr("192.168.1.100"); // unicast

    while (1) {

        int err = sendto(sock,
                         message,
                         strlen(message),
                         0,
                         (struct sockaddr *)&broadcast_addr,
                         sizeof(broadcast_addr));

        if (err < 0) {
            printf("Error sending broadcast\n");
        } else {
            printf("Broadcast sent\n");
        }

        vTaskDelay(pdMS_TO_TICKS(2000)); // every 2 seconds
    }

    close(sock);
    vTaskDelete(NULL);
}
*/

////////////////////////////////////////////////////////////////////////////
// udpsrv_broadcast_message
//
// Send raw payload using current UDP destination settings. Supports both
// unicast and broadcast destinations.

esp_err_t
udpsrv_broadcast_message(const uint8_t *payload,
                         size_t payload_len,
                         const char *destination_ip,
                         uint16_t destination_port)
{
  if ((NULL == payload) || (0 == payload_len)) {
    return ESP_ERR_INVALID_ARG;
  }

  int sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock < 0) {
    ESP_LOGE(TAG, "socket() failed: errno=%d", errno);
    return ESP_FAIL;
  }

  int broadcast = 1;
  if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0) {
    ESP_LOGE(TAG, "setsockopt(SO_BROADCAST) failed: errno=%d", errno);
    close(sock);
    return ESP_FAIL;
  }

  struct sockaddr_in udp_addr;
  char normalized_host[UDP_HOST_MAX_LEN] = { 0 };

  memset(&udp_addr, 0, sizeof(udp_addr));
  udp_addr.sin_family = AF_INET;
  uint16_t port       = destination_port ? destination_port : g_persistent.udpPort;
  udp_addr.sin_port   = htons(port ? port : VSCP_DEFAULT_UDP_PORT);

  const char *host = destination_ip;
  if ((NULL == host) || (0 == strlen(host))) {
    host = g_persistent.udpUrl;
  }

  normalize_udp_host(normalized_host, sizeof(normalized_host), host);

  if (!resolve_udp_ipv4_host(&udp_addr.sin_addr, normalized_host)) {
    char fallback_host[UDP_HOST_MAX_LEN] = { 0 };
    normalize_udp_host(fallback_host, sizeof(fallback_host), g_persistent.udpUrl);

    if (!resolve_udp_ipv4_host(&udp_addr.sin_addr, fallback_host)) {
      udp_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    }
  }

  ssize_t sent = sendto(sock, payload, payload_len, 0, (struct sockaddr *) &udp_addr, sizeof(udp_addr));
  close(sock);

  if (sent != (ssize_t) payload_len) {
    ESP_LOGE(TAG, "sendto() failed: errno=%d", errno);
    return ESP_FAIL;
  }

  return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////
// sendEvent
//
// socket - Socket to send event on
// udp_addr - Destination address to send event to
// pstrev - VSCP event on string form to send
// bEncrypt - true if the event should be encrypted with the set key
// nAlgorithm - Encryption algorithm to use (vscp.h)
//

static int __attribute__((unused))
sendEvent(int sock, struct sockaddr_in udp_addr, const char *pstrev, bool bEncrypt, uint8_t nAlgorithm)
{
  int32_t rv;
  uint16_t len             = 0;
  uint8_t buf[BUFFER_SIZE] = { 0 };

  vscp_event_ex_t ex;
  if (VSCP_ERROR_SUCCESS != (rv = vscp_fwhlp_parseStringToEventEx(&ex, pstrev))) {
    fprintf(stderr, "Error parsing event string\n");
    return VSCP_ERROR_SUCCESS;
  }

#ifdef _MULTICAST_DEBUG_
  printf("Parsed event:\n");
  printf("=============\n");
  printf("Class: %d\n", ex.vscp_class);
  printf("Type: %d\n", ex.vscp_type);
  printf("Priority: %d\n", ex.head & 0xE0);
  printf("GUID: ");
  for (int i = 0; i < 16; i++) {
    printf("%02x:", ex.GUID[i]);
  }
  printf("\n");
  printf("Data: ");
  for (int i = 0; i < ex.sizeData; i++) {
    printf("%02x:", ex.data[i]);
  }
  printf("\n");
  printf("----------------------------------------------------\n");
  printf("Timestamp: 0x%08lX\n", (long unsigned int) ex.timestamp);
  printf("Obid: 0x%08lX\n", (long unsigned int) ex.obid);
  printf("Head: %d\n", ex.head);
  printf("Year: %d\n", ex.year);
  printf("Month: %d\n", ex.month);
  printf("Day: %d\n", ex.day);
  printf("Hour: %d\n", ex.hour);
  printf("Minute: %d\n", ex.minute);
  printf("Second: %d\n", ex.second);
  printf("----------------------------------------------------\n");
#endif

  // Calculate needed buffer size
  len = vscp_fwhlp_getFrameSizeFromEventEx(&ex);
  if (VSCP_ERROR_SUCCESS != (rv = vscp_fwhlp_writeEventExToFrame(buf, sizeof(buf), 0, &ex))) {
    fprintf(stderr, "Error writing event to frame. rv=%u\n", (unsigned) rv);
    return VSCP_ERROR_SUCCESS;
  }

#ifdef _MULTICAST_DEBUG_
  printf("Frame size: %d\n", len);
  printf("Frame:\n");
  for (int i = 0; i < len; i++) {
    printf("%02x ", buf[i]);
  }
  printf("\n");
#endif

  // Encrypt frame as needed
  if (g_persistent.bUdpEncrypt) {

    ESP_LOGI(TAG, "Encrypting frame with algorithm %d ", nAlgorithm);

    uint8_t newlen       = 0;
    uint8_t encbuf[1024] = { 0 };

    if (0 == (newlen = vscp_fwhlp_encryptFrame(encbuf, buf, len, g_persistent.pmk, NULL, nAlgorithm))) {
      fprintf(stderr, "Error encrypting frame. newlen = %d\n", newlen);
      return VSCP_ERROR_SUCCESS;
    }

    memcpy(buf, encbuf, newlen);
    buf[0] = (buf[0] & 0xF0) | (VSCP_HLO_ENCRYPTION_AES128 & 0x0F); // Set encryption type
    // Set the new length (may be padded to be modulo 16 + 1)
    len = newlen;

    if (1) {
      printf("Encrypted frame:\n");
      for (int i = 0; i < len; i++) {
        printf("%02x ", buf[i]);
      }
      printf("\nNew length: %d\n", len);
    }
  } // encrypted frame

  if ((sendto(sock, buf, len, 0, (struct sockaddr *) &udp_addr, sizeof(udp_addr))) < len) {
    fprintf(stderr, "send event: Failed %d\n", errno);
    return VSCP_ERROR_ERROR;
  }
  // if ((nsent = sendto(sock, buf, len, s_multicast_ip, s_multicast_port)) < len) {
  //   fprintf(stderr, "send event: Failed %d\n", errno);
  //   return VSCP_ERROR_ERROR;
  // }

  return VSCP_ERROR_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////
// udpsrv_rx_task
//
// This task listens for incoming UDP packets on the configured port. When a
// packet is received, it parse the data and if a valid VSCP frame send it
// on the CAN interface. The task runs indefinitely and handles errors gracefully,
// logging any issues with socket creation, binding, or receiving data.

void
udpsrv_rx_task(void *pvParameters)
{
  int rv = 0;
  uint8_t rx_buffer[BUFFER_SIZE];
  int addr_family = (int) pvParameters;
  int ip_protocol = 0;
  struct sockaddr_in *dest_addr_ip4;
  struct sockaddr_in6 dest_addr;

  if (addr_family == AF_INET) {
    dest_addr_ip4                  = (struct sockaddr_in *) &dest_addr;
    dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr_ip4->sin_family      = AF_INET;
    dest_addr_ip4->sin_port        = htons(VSCP_DEFAULT_UDP_PORT);
    ip_protocol                    = IPPROTO_IP;
  }
  else if (addr_family == AF_INET6) {
    bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
    dest_addr.sin6_family = AF_INET6;
    dest_addr.sin6_port   = htons(VSCP_DEFAULT_UDP_PORT);
    ip_protocol           = IPPROTO_IPV6;
  }
  else {
    ESP_LOGE(TAG, "Unknown address family");
    vTaskDelete(NULL);
    return;
  }

  while (1) {

    int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
      ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
      break;
    }
    ESP_LOGI(TAG, "Socket created");

#if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
  int enable = 1;
  lwip_setsockopt(sock, IPPROTO_IP, IP_PKTINFO, &enable, sizeof(enable));
#endif

#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
    if (addr_family == AF_INET6) {
      // Note that by default IPV6 binds to both protocols, it is must be disabled
      // if both protocols used at the same time (used in CI)
      int opt = 1;
      setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
      setsockopt(sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
    }
#endif
    // Set timeout
    struct timeval timeout;
    timeout.tv_sec  = 10;
    timeout.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

    int flag = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(flag));

    if (addr_family == AF_INET) {
      // ESP_LOGD(TAG, "Binding to %s:%d\n", inet_ntoa(dest_addr_ip4->sin_addr), ntohs(dest_addr_ip4->sin_port));

      int err = bind(sock, (struct sockaddr *) dest_addr_ip4, sizeof(*dest_addr_ip4));
      if (err < 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
      }
    }
    else if (addr_family == AF_INET6) {
      ESP_LOGD(TAG, "Binding to [%s]:%d\n", inet6_ntoa(dest_addr.sin6_addr), ntohs(dest_addr.sin6_port));
      int err = bind(sock, (struct sockaddr *) &dest_addr, sizeof(dest_addr));
      if (err < 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        break;
      }
    }

    struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
    socklen_t socklen = sizeof(source_addr);

#if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
    struct iovec iov;
    struct msghdr msg;
    struct cmsghdr *cmsgtmp;
    u8_t cmsg_buf[CMSG_SPACE(sizeof(struct in_pktinfo))];

    iov.iov_base       = rx_buffer;
    iov.iov_len        = sizeof(rx_buffer);
    msg.msg_control    = cmsg_buf;
    msg.msg_controllen = sizeof(cmsg_buf);
    msg.msg_flags      = 0;
    msg.msg_iov        = &iov;
    msg.msg_iovlen     = 1;
    msg.msg_name       = (struct sockaddr *) &source_addr;
    msg.msg_namelen    = socklen;
#endif

    while (1) {
      ESP_LOGV(TAG, "Waiting for data");
#if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
      memset(cmsg_buf, 0, sizeof(cmsg_buf));
      msg.msg_control    = cmsg_buf;
      msg.msg_controllen = sizeof(cmsg_buf);
      msg.msg_name       = (struct sockaddr *) &source_addr;
      msg.msg_namelen    = socklen;
      int len = recvmsg(sock, &msg, 0);
#else
      int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *) &source_addr, &socklen);
#endif
      // Error occurred during receiving
      if (len < 0) {
        if (errno == EWOULDBLOCK || errno == EAGAIN) {
          ESP_LOGV(TAG, "Receive timeout, no data received");
        }
        else {
          ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
        }
        continue;
      }
      // Data received
      else {
#if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
        bool bDropMulticast = false;
        for (cmsgtmp = CMSG_FIRSTHDR(&msg); cmsgtmp != NULL; cmsgtmp = CMSG_NXTHDR(&msg, cmsgtmp)) {
          if ((cmsgtmp->cmsg_level == IPPROTO_IP) && (cmsgtmp->cmsg_type == IP_PKTINFO)) {
            struct in_pktinfo *pktinfo = (struct in_pktinfo *) CMSG_DATA(cmsgtmp);
            if ((NULL != pktinfo) && is_ipv4_multicast_addr(pktinfo->ipi_addr.s_addr)) {
              bDropMulticast = true;
            }
            break;
          }
        }

        if (bDropMulticast) {
          ESP_LOGD(TAG, "Dropped UDP packet addressed to multicast destination");
          continue;
        }
#endif

        // Get the sender's ip address as string
        //         if (source_addr.ss_family == PF_INET) {
        //           inet_ntoa_r(((struct sockaddr_in *) &source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        // #if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
        //           for (cmsgtmp = CMSG_FIRSTHDR(&msg); cmsgtmp != NULL; cmsgtmp = CMSG_NXTHDR(&msg, cmsgtmp)) {
        //             if (cmsgtmp->cmsg_level == IPPROTO_IP && cmsgtmp->cmsg_type == IP_PKTINFO) {
        //               struct in_pktinfo *pktinfo;
        //               pktinfo = (struct in_pktinfo *) CMSG_DATA(cmsgtmp);
        //               ESP_LOGI(TAG, "dest ip: %s\n", inet_ntoa(pktinfo->ipi_addr));
        //             }
        //           }
        // #endif
        //         }
        //         else if (source_addr.ss_family == PF_INET6) {
        //           inet6_ntoa_r(((struct sockaddr_in6 *) &source_addr)->sin6_addr, addr_str, sizeof(addr_str) -
        //           1);
        //         }

        // rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...
        // ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
        // ESP_LOGI(TAG, "Buffer: [%s]", rx_buffer);

        ESP_LOGI(TAG, "Received %d bytes", len);
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, rx_buffer, len, ESP_LOG_DEBUG);

        // If encrypted frame decrypt it
        if (rx_buffer[0] & 0x0F) {

          ESP_LOGI(TAG, "Encrypted frame detected. Type: %d", rx_buffer[0] & 0x0F);

          uint8_t encbuf[BUFFER_SIZE] = { 0 };
          if (VSCP_ERROR_SUCCESS != vscp_fwhlp_decryptFrame(encbuf,
                                                            rx_buffer,
                                                            len - 16,
                                                            g_persistent.pmk,
                                                            rx_buffer + len - 16,
                                                            VSCP_ENCRYPTION_FROM_TYPE_BYTE)) {
            ESP_LOGE(TAG, "Error decrypting frame. Skipping...");
            continue;
          }
          if (0) {
            printf("Decrypted frame:\n");
            printf("Length: %d\n", len);
            for (int i = 0; i < len; i++) {
              printf("%02x ", encbuf[i]);
            }
            printf("\n");
          }

          // Copy decrypted frame back to buffer
          memcpy(rx_buffer, encbuf, len);

        } // encrypted frame

        // vscp_event_ex_t ex;
        // memset(&ex, 0, sizeof(ex));
        // if (VSCP_ERROR_SUCCESS != (rv = vscp_fwhlp_getEventExFromFrame(&ex, rx_buffer, len))) {
        //   fprintf(stderr, "Error reading event from frame. Check encryption key! rv=%d\n", rv);
        //   continue;
        // }

        // ESP_LOGI(TAG,
        //          "Parsed VSCP event from UDP frame: class=%u type=%u size=%u",
        //          ex.vscp_class,
        //          ex.vscp_type,
        //          ex.sizeData);

        // int err = sendto(sock, rx_buffer, len, 0, (struct sockaddr *) &source_addr, sizeof(source_addr));
        // if (err < 0) {
        //   ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        //   break;
        // }
        vscp_event_t ev;
        memset(&ev, 0, sizeof(ev));
        if (VSCP_ERROR_SUCCESS != (rv = vscp_fwhlp_getEventFromFrame(&ev, rx_buffer, len))) {
          ESP_LOGE(TAG, "Error reading event from frame. rv=%d", rv);
          continue;
        }

        if (0) {
          printf("Event:\n");
          printf("Head: %d\n", ev.head);
          printf("Class: %d\n", ev.vscp_class);
          printf("Type: %d\n", ev.vscp_type);
          printf("Size: %d\n", ev.sizeData);
          for (int i = 0; i < ev.sizeData; i++) {
            printf("%02x ", ev.pdata[i]);
          }
          printf("\n");
          printf("----------------------------------------------------\n");
          printf("Timestamp: 0x%08lX\n", (long unsigned int) ev.timestamp);
          printf("Obid: 0x%08lX\n", (long unsigned int) ev.obid);
          printf("Year: %d\n", ev.year);
          printf("Month: %d\n", ev.month);
          printf("Day: %d\n", ev.day);
          printf("Hour: %d\n", ev.hour);
          printf("Minute: %d\n", ev.minute);
          printf("Second: %d\n", ev.second);
          printf("----------------------------------------------------\n");
        }

        // Add event to CAN queue
        can4vscp_frame_t txmsg = {};
        if (VSCP_ERROR_SUCCESS != (rv = can4vscp_event_to_msg(&txmsg, &ev))) {
          ESP_LOGE(TAG, "Failed to convert VSCP event to CAN message rv=%d", rv);
          if (ev.pdata) {
            free(ev.pdata);
          }
          continue;
        }

        can4vscp_frame_t tx_msg;
        tx_msg.data_length_code = ev.sizeData;
        tx_msg.extd             = 1;
        tx_msg.identifier = ev.GUID[0] + (ev.vscp_type << 8) + (ev.vscp_class << 16) + (((ev.head >> 5) & 7) << 26);
        memcpy(tx_msg.data, ev.pdata, ev.sizeData);

        ESP_LOGI(TAG,
                 "Send multicast event as CAN message to queue: class=%u type=%u size=%u",
                 ev.vscp_class,
                 ev.vscp_type,
                 ev.sizeData);

        esp_err_t err;
        if (ESP_OK != (err = can4vscp_send(&tx_msg, pdMS_TO_TICKS(10)))) {
          ESP_LOGE(TAG, "Failed to send CAN message to queue: %d", err);
        }

        if (ev.pdata) {
          free(ev.pdata);
        }
      }
    } // while

    if (sock != -1) {
      ESP_LOGE(TAG, "Shutting down socket and restarting...");
      shutdown(sock, 0);
      close(sock);
    }
  } // while
  vTaskDelete(NULL);
}

////////////////////////////////////////////////////////////////////////////
// udpsrv_tx_task
//
// This task listens for outgoing UDP packets on the configured port.
//

void
udpsrv_tx_task(void *pvParameters)
{
  int rv;
  can4vscp_frame_t rxmsg = {};

  (void) pvParameters;

  ESP_LOGI(TAG, "UDP TX task started. Destination=%s:%u", g_persistent.udpUrl, g_persistent.udpPort);

  while (1) {

    if (pdPASS != xQueueReceive(tr_udp.fromcan_queue, (void *) &rxmsg, 500)) {
      continue;
    }

    vscp_event_t *pev = NULL;
    if (VSCP_ERROR_SUCCESS != (rv = can4vscp_msg_to_event(&pev, &rxmsg))) {
      ESP_LOGE(TAG, "Failed to convert CAN message to VSCP event rv=%d", rv);
      vscp_fwhlp_deleteEvent(&pev);
      continue;
    }

    uint8_t frame[BUFFER_SIZE] = { 0 };
    uint16_t len               = vscp_fwhlp_getFrameSizeFromEvent(pev);
    if (len > sizeof(frame)) {
      ESP_LOGE(TAG, "VSCP frame too large: len=%u", len);
      vscp_fwhlp_deleteEvent(&pev);
      continue;
    }

    ESP_LOGI(TAG,
             "Received message from CAN queue: class=%u type=%u size=%u",
             pev->vscp_class,
             pev->vscp_type,
             pev->sizeData);

    if (VSCP_ERROR_SUCCESS != (rv = vscp_fwhlp_writeEventToFrame(frame, sizeof(frame), 0, pev))) {
      ESP_LOGE(TAG, "Failed to serialize VSCP event to frame rv=%d", rv);
      vscp_fwhlp_deleteEvent(&pev);
      continue;
    }

    vscp_fwhlp_deleteEvent(&pev);

    uint8_t newlen              = 0;
    uint8_t encbuf[BUFFER_SIZE] = { 0 };

    if (0 == (newlen = vscp_fwhlp_encryptFrame(encbuf, frame, len, g_persistent.pmk, NULL, frame[0] & 0x0F))) {
      ESP_LOGI(TAG, "Error encrypting frame. newlen = %d", newlen);
      continue;
    }

    ESP_LOGV(TAG, "Prepared UDP frame (%d bytes) from CAN message. %02X %02X", len, frame[len - 2], frame[len - 1]);

    if (ESP_OK != udpsrv_broadcast_message(encbuf, newlen, NULL, 0)) {
      ESP_LOGE(TAG, "Failed to send UDP frame");
      continue;
    }

    ESP_LOGV(TAG, "Sent UDP frame (%d bytes)", len);
  }

  vTaskDelete(NULL);
}

////////////////////////////////////////////////////////////////////////////////
// udp_start
// This function creates the UDP server task for both IPv4 and IPv6 (if enabled).
// The task will listen for incoming UDP packets and process them as defined in udpsrv_task.
//

void
udp_start(void)
{
  if (g_persistent.enableUdpRx) {
    xTaskCreate(udpsrv_rx_task, "udpsrv_rx", 4096, (void *) AF_INET, 5, NULL);
#ifdef CONFIG_EXAMPLE_IPV6
    xTaskCreate(udpsrv_rx_task, "udpsrv_rx", 4096, (void *) AF_INET6, 5, NULL);
#endif
  }

  if (g_persistent.enableUdpTx) {
    xTaskCreate(udpsrv_tx_task, "udpsrv_tx", 4096, (void *) AF_INET, 5, NULL);
#ifdef CONFIG_EXAMPLE_IPV6
    xTaskCreate(udpsrv_tx_task, "udpsrv_tx", 4096, (void *) AF_INET6, 5, NULL);
#endif
  }
}