/*
  Web Server

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

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>
#include <sys/param.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <dirent.h>

#include <esp_system.h>
#include <esp_chip_info.h>
#include <esp_flash_spi_init.h>
#include <esp_flash.h>
#include <esp_wifi.h>
#include <esp_mac.h>
#include <esp_ota_ops.h>
#include <esp_timer.h>
#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_http_server.h>

#include <esp_event_base.h>
#include <esp_tls_crypto.h>
#include <esp_vfs.h>
#include <esp_spiffs.h>
#include <network_provisioning/manager.h>
#include <cJSON.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <vscp.h>
#include <vscp-class.h>
#include <vscp-type.h>
#include <vscp-firmware-helper.h>

#include "urldecode.h"

#include "mqtt.h"
#include "main.h"
#include "vscp-ws1.h"
#include "websocksrv.h"
#include "websrv.h"
#include "udpsrv.h"
#include "build/build_counter.h"

#if !CONFIG_HTTPD_WS_SUPPORT
#error This code cannot be used unless HTTPD_WS_SUPPORT is enabled in esp-http-server component configuration
#endif

#ifdef CONFIG_EXAMPLE_PROV_TRANSPORT_BLE
#include <network_provisioning/scheme_ble.h>
#endif /* PROV_EXAMPLE_TRANSPORT_BLE */

#ifdef CONFIG_EXAMPLE_PROV_TRANSPORT_SOFTAP
#include <network_provisioning/scheme_softap.h>
#endif /* PROV_EXAMPLE_TRANSPORT_SOFTAP */

// #define MIN(a, b) (((a) < (b)) ? (a) : (b))
// #define MAX(a, b) (((a) > (b)) ? (a) : (b))

// From app_main
void
startOTA(void);
int
app_initiate_firmware_upload(const char *url);

// External from main
extern nvs_handle_t g_nvsHandle;
extern node_persistent_config_t g_persistent;
extern vprintf_like_t g_stdLogFunc;
extern transport_t tr_websockets;
extern transport_t tr_canapi;
extern transport_t tr_monitor;

// Extern from webnsocksrv
extern httpd_handle_t g_websocket_srv;

#define TAG __func__

static const char *
web_ui_version(const esp_app_desc_t *appDescr)
{
  static char s_ui_ver[64];
  const char *base = "0.0.0";

  if ((NULL != appDescr) && ('\0' != appDescr->version[0])) {
    base = appDescr->version;
  }

  snprintf(s_ui_ver, sizeof(s_ui_ver), "%s+b%d", base, BUILD_COUNTER);
  return s_ui_ver;
}

// Keep a short in-memory rolling log for the web UI.
#define WEB_LOG_RING_SIZE 200
#define WEB_LOG_LINE_MAX  192

typedef struct {
  uint32_t seq;
  char line[WEB_LOG_LINE_MAX];
} web_log_item_t;

static web_log_item_t g_web_log_ring[WEB_LOG_RING_SIZE];
static uint32_t g_web_log_seq = 0;
vprintf_like_t g_stdLogFunc   = NULL;
static SemaphoreHandle_t g_web_log_mutex = NULL;
static volatile bool s_web_log_route_in_progress = false;

static int web_log_vprintf(const char *fmt, va_list args);
static void web_log_capture_line(const char *line);
static esp_err_t logs_get_handler(httpd_req_t *req);
static esp_err_t logstream_get_handler(httpd_req_t *req);
static void web_log_hook_init(void);

// Max length a file path can have on storage
#define FILE_PATH_MAX (ESP_VFS_PATH_MAX + CONFIG_SPIFFS_OBJ_NAME_LEN)

// Chunk buffer size
#define CHUNK_BUFSIZE 8192

#define IS_FILE_EXT(filename, ext) (strcasecmp(&filename[strlen(filename) - sizeof(ext) + 1], ext) == 0)

// Prototypes

///////////////////////////////////////////////////////////////////////////////
// web_log_capture_line
//

static void
web_log_capture_line(const char *line)
{
  if ((NULL == line) || ('\0' == *line) || (NULL == g_web_log_mutex)) {
    return;
  }

  if (pdTRUE != xSemaphoreTake(g_web_log_mutex, pdMS_TO_TICKS(5))) {
    return;
  }

  uint32_t seq = ++g_web_log_seq;
  size_t idx   = (size_t) ((seq - 1U) % WEB_LOG_RING_SIZE);
  g_web_log_ring[idx].seq = seq;

  size_t j = 0;
  for (size_t i = 0; (line[i] != '\0') && (j < (WEB_LOG_LINE_MAX - 1)); ++i) {
    char c = line[i];
    if (('\r' == c) || ('\n' == c)) {
      if ((j > 0) && (' ' != g_web_log_ring[idx].line[j - 1])) {
        g_web_log_ring[idx].line[j++] = ' ';
      }
      continue;
    }
    g_web_log_ring[idx].line[j++] = c;
  }
  g_web_log_ring[idx].line[j] = '\0';

  xSemaphoreGive(g_web_log_mutex);
}

///////////////////////////////////////////////////////////////////////////////
// web_log_route_message
//
// Route log message to configured destination based on logType setting
//

static void
web_log_route_message(const char *message)
{
  if (NULL == message || '\0' == message[0]) {
    return;
  }

  // Route based on configured log type
  switch (g_persistent.logType) {
    
    case LOG_TYPE_UDP:
      // Send log message via configured logging destination.
      udpsrv_broadcast_message((const uint8_t *) message,
                               strlen(message),
                               g_persistent.logUrl,
                               g_persistent.logPort);
      break;

    case LOG_TYPE_MQTT:
      // Send log message via MQTT (if enabled)
      if (g_persistent.enableMqtt && strlen(g_persistent.logMqttTopic)) {
        mqtt_log((char *) message);
      }
      break;

    case LOG_TYPE_STD:
    case LOG_TYPE_TCP:
    case LOG_TYPE_HTTP:
    case LOG_TYPE_VSCP:
    case LOG_TYPE_NONE:
    default:
      // These are handled by the default logging mechanism or not used for syslog
      break;
  }
}

///////////////////////////////////////////////////////////////////////////////
// web_log_vprintf
//

static int
web_log_vprintf(const char *fmt, va_list args)
{
  if (NULL != fmt) {
    char line[WEB_LOG_LINE_MAX] = { 0 };
    va_list copy;
    va_copy(copy, args);
    int n = vsnprintf(line, sizeof(line), fmt, copy);
    va_end(copy);

    if (n > 0) {
      web_log_capture_line(line);

      // Prevent recursive routing when transport code itself logs.
      if (!s_web_log_route_in_progress) {
        s_web_log_route_in_progress = true;
        web_log_route_message(line);
        s_web_log_route_in_progress = false;
      }
    }
  }

  if (NULL != g_stdLogFunc) {
    return g_stdLogFunc(fmt, args);
  }

  return vprintf(fmt, args);
}

///////////////////////////////////////////////////////////////////////////////
// web_log_hook_init
//

static void
web_log_hook_init(void)
{
  if (NULL == g_web_log_mutex) {
    g_web_log_mutex = xSemaphoreCreateMutex();
  }

  if ((NULL != g_web_log_mutex) && (NULL == g_stdLogFunc)) {
    g_stdLogFunc = esp_log_set_vprintf(web_log_vprintf);
    ESP_LOGI(TAG, "Web log capture enabled");
  }
}

///////////////////////////////////////////////////////////////////////////////
// backup_add_hex_field
//

static void
backup_add_hex_field(cJSON *obj, const char *name, const uint8_t *data, size_t len)
{
  if ((NULL == obj) || (NULL == name) || (NULL == data) || (0 == len)) {
    return;
  }

  char *hex = (char *) calloc((len * 2) + 1, 1);
  if (NULL == hex) {
    return;
  }

  for (size_t i = 0; i < len; i++) {
    sprintf(hex + (i * 2), "%02X", data[i]);
  }

  cJSON_AddStringToObject(obj, name, hex);
  free(hex);
}

static uint32_t
backup_secret_seed(const char *field_name)
{
  uint32_t seed     = 2166136261u;
  const char *fixed = "VSCP-CFG-BACKUP-V1";

  for (size_t i = 0; fixed[i] != '\0'; i++) {
    seed ^= (uint8_t) fixed[i];
    seed *= 16777619u;
  }

  if (NULL != field_name) {
    for (size_t i = 0; field_name[i] != '\0'; i++) {
      seed ^= (uint8_t) field_name[i];
      seed *= 16777619u;
    }
  }

  if (0 == seed) {
    seed = 0x1A2B3C4Du;
  }

  return seed;
}

static uint32_t
backup_secret_next(uint32_t *state)
{
  *state ^= (*state << 13);
  *state ^= (*state >> 17);
  *state ^= (*state << 5);
  return *state;
}

static bool
backup_encrypt_to_hex(const char *field_name, const uint8_t *in, size_t in_len, char *out, size_t out_len)
{
  if ((NULL == in) || (NULL == out)) {
    return false;
  }

  if (out_len < ((in_len * 2) + 5)) {
    return false;
  }

  memcpy(out, "enc:", 4);
  uint32_t st = backup_secret_seed(field_name);

  for (size_t i = 0; i < in_len; i++) {
    uint8_t b = in[i] ^ (uint8_t) (backup_secret_next(&st) & 0xFF);
    sprintf(out + 4 + (i * 2), "%02X", b);
  }

  out[4 + (in_len * 2)] = '\0';
  return true;
}

static bool
backup_decrypt_from_hex(const char *field_name, const char *in, uint8_t *out, size_t out_len, size_t *written)
{
  if ((NULL == field_name) || (NULL == in) || (NULL == out)) {
    return false;
  }

  if (0 != strncmp(in, "enc:", 4)) {
    return false;
  }

  const char *hex = in + 4;
  size_t hex_len  = strlen(hex);
  if ((0 == hex_len) || (0 != (hex_len % 2))) {
    return false;
  }

  size_t need = hex_len / 2;
  if (need > out_len) {
    return false;
  }

  uint32_t st = backup_secret_seed(field_name);
  for (size_t i = 0; i < need; i++) {
    char h1 = hex[i * 2];
    char h2 = hex[(i * 2) + 1];
    if (!isxdigit((unsigned char) h1) || !isxdigit((unsigned char) h2)) {
      return false;
    }

    uint8_t hi = (uint8_t) ((h1 <= '9') ? (h1 - '0') : (10 + (tolower((unsigned char) h1) - 'a')));
    uint8_t lo = (uint8_t) ((h2 <= '9') ? (h2 - '0') : (10 + (tolower((unsigned char) h2) - 'a')));
    uint8_t b  = (uint8_t) ((hi << 4) | lo);
    out[i]     = b ^ (uint8_t) (backup_secret_next(&st) & 0xFF);
  }

  if (NULL != written) {
    *written = need;
  }

  return true;
}

static void
backup_add_encrypted_str_field(cJSON *obj, const char *name, const char *plain)
{
  if ((NULL == obj) || (NULL == name) || (NULL == plain)) {
    return;
  }

  size_t plain_len = strlen(plain);
  if (0 == plain_len) {
    cJSON_AddStringToObject(obj, name, "");
    return;
  }

  size_t out_len = (plain_len * 2) + 5;
  char *enc      = (char *) calloc(out_len, 1);
  if (NULL == enc) {
    cJSON_AddStringToObject(obj, name, "");
    return;
  }

  if (backup_encrypt_to_hex(name, (const uint8_t *) plain, plain_len, enc, out_len)) {
    cJSON_AddStringToObject(obj, name, enc);
  }
  else {
    cJSON_AddStringToObject(obj, name, "");
  }

  free(enc);
}

static void
backup_add_encrypted_blob_field(cJSON *obj, const char *name, const uint8_t *data, size_t data_len)
{
  if ((NULL == obj) || (NULL == name) || (NULL == data)) {
    return;
  }

  size_t out_len = (data_len * 2) + 5;
  char *enc      = (char *) calloc(out_len, 1);
  if (NULL == enc) {
    return;
  }

  if (backup_encrypt_to_hex(name, data, data_len, enc, out_len)) {
    cJSON_AddStringToObject(obj, name, enc);
  }

  free(enc);
}

//-----------------------------------------------------------------------------
//                               Start Basic Auth
//-----------------------------------------------------------------------------

///////////////////////////////////////////////////////////////////////////////
// http_auth_basic
//

static char *
http_auth_basic(const char *username, const char *password)
{
  int out;
  char *user_info = NULL;
  char *digest    = NULL;
  size_t n        = 0;
  asprintf(&user_info, "%s:%s", username, password);
  if (!user_info) {
    ESP_LOGE(TAG, "No enough memory for user information");
    return NULL;
  }
  esp_crypto_base64_encode(NULL, 0, &n, (const unsigned char *) user_info, strlen(user_info));

  /* 6: The length of the "Basic " string
   * n: Number of bytes for a base64 encode format
   * 1: Number of bytes for a reserved which be used to fill zero
   */
  digest = calloc(6 + n + 1, 1);
  if (digest) {
    strcpy(digest, "Basic ");
    esp_crypto_base64_encode((unsigned char *) digest + 6,
                             n,
                             (size_t *) &out,
                             (const unsigned char *) user_info,
                             strlen(user_info));
  }
  free(user_info);
  return digest;
}

///////////////////////////////////////////////////////////////////////////////
// info_get_handler
//
// HTTP GET handler for info page
//

static esp_err_t
info_get_handler(httpd_req_t *req)
{
  // esp_err_t rv;
  char *buf;
  char *temp;

  buf = (char *) calloc(CHUNK_BUFSIZE, 1);
  if (NULL == buf) {
    return ESP_ERR_NO_MEM;
  }

  temp = (char *) calloc(80, 1);
  if (NULL == temp) {
    return ESP_ERR_NO_MEM;
  }

  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);

  const esp_app_desc_t *appDescr = esp_app_get_description();

  sprintf(buf, WEBPAGE_START_TEMPLATE, g_persistent.nodeName, "Technical Node Info");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<table style='width:100%%;margin-left: auto; margin-right: auto;'>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // * * * system * * *);
  sprintf(buf, "<tr><td class='infoheader'>System</td><td></td></tr>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<tr style='padding: 25px;'><td class=\"name\">Chip type:</td>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
  switch (chip_info.model) {

    case CHIP_ESP32:
      // printf("ESP32\n");
      sprintf(buf, "<td class=\"prop\">ESP32</td></tr>");
      break;

    case CHIP_ESP32S2:
      // printf("ESP32-S2\n");
      sprintf(buf, "<td class=\"prop\">ESP32-S2</td></tr>");
      break;

    case CHIP_ESP32S3:
      // printf("ESP32-S3\n");
      sprintf(buf, "<td class=\"prop\">ESP32-S3</td></tr>");
      break;

    case CHIP_ESP32C3:
      // printf("ESP32-C3\n");
      sprintf(buf, "<td class=\"prop\">ESP32-C3</td></tr>");
      break;

    case CHIP_ESP32H2:
      // printf("ESP32-H2\n");
      sprintf(buf, "<td class=\"prop\">ESP32-H2</td></tr>");
      break;

    case CHIP_ESP32C2:
      // printf("ESP32-C2\n");
      sprintf(buf, "<td class=\"prop\">ESP32-C2</td></tr>");
      break;

    default:
      // printf("Unknown\n");
      sprintf(buf, "<td class=\"prop\">Unknown</td></tr>");
      break;
  }
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<tr><td class=\"name\">Number of cores:</td><td class=\"prop\">%d</td></tr>", chip_info.cores);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // printf("Number of cores: %d \n", chip_info.cores);

  // Chip comm features
  sprintf(temp,
          "%s%s%s%s",
          (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi " : "",
          (chip_info.features & CHIP_FEATURE_BT) ? "BT " : "",
          (chip_info.features & CHIP_FEATURE_BLE) ? "BLE " : "",
          (chip_info.features & CHIP_FEATURE_IEEE802154) ? "802.15.4 " : "");
  sprintf(buf, "<tr><td class=\"name\">Chip comm features:</td><td class=\"prop\">%s</td></tr>", temp);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  wifi_country_t country;
  esp_wifi_get_country(&country);
  // printf("Wifi country code: %c%c%c\n", country.cc[0],country.cc[1],country.cc[2]);
  sprintf(buf,
          "<tr><td class=\"name\">Wifi country code:</td><td class=\"prop\">%c%c%c</td></tr>",
          country.cc[0],
          country.cc[1],
          country.cc[2]);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(temp, "%s", (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "Yes" : "No");
  sprintf(buf, "<tr><td class=\"name\">Embedded flash:</td><td class=\"prop\">%s</td></tr>", temp);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(temp, "%s", (chip_info.features & CHIP_FEATURE_EMB_PSRAM) ? "Yes" : "No");
  sprintf(buf, "<tr><td class=\"name\">Embedded psram:</td><td class=\"prop\">%s</td></tr>", temp);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // sprintf(temp, "%d", chip_info.revision);
  sprintf(buf, "<tr><td class=\"name\">Silicon revision:</td><td class=\"prop\">%d</td></tr>", chip_info.revision);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  uint32_t chipId;
  esp_flash_read_id(NULL, &chipId);
  // printf("Flash chip id: %04lX\n", chipId);
  sprintf(buf, "<tr><td class=\"name\">Flash chip id:</td><td class=\"prop\">%04lX</td></tr>", chipId);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  uint64_t uniqueId;
  esp_flash_read_unique_chip_id(NULL, &uniqueId);
  // printf("Unique flash chip id: %08llX\n", uniqueId);
  sprintf(buf, "<tr><td class=\"name\">Unique flash chip id:</td><td class=\"prop\">%08llX</td></tr>", uniqueId);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  uint32_t sizeFlash;
  esp_flash_get_size(NULL, &sizeFlash);
  sprintf(temp, "%s", (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "(embedded)" : "(external)");
  // printf("%luMB %s flash\n",
  //        sizeFlash / (1024 * 1024),
  //        (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
  sprintf(buf,
          "<tr><td class=\"name\">Flash size:</td><td class=\"prop\">%s %lu MB</td></tr>",
          temp,
          sizeFlash / (1024 * 1024));
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // get chip id
  // chipId = String((uint32_t) ESP.getEfuseMac(), HEX);
  // chipId.toUpperCase();
  // printf("Chip id: %s\n", chipId.c_str());

  // printf("esp-idf version: %s\n", esp_get_idf_version());
  sprintf(buf, "<tr><td class=\"name\">esp-idf version:</td><td class=\"prop\">%s</td></tr>", esp_get_idf_version());
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // printf("Free heap size: %lu\n", esp_get_free_heap_size());
  sprintf(buf,
          "<tr><td class=\"name\">Free heap size:</td><td class=\"prop\">%lu kB (%lu)</td></tr>",
          esp_get_free_heap_size() / 1024,
          esp_get_free_heap_size());
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // printf("Min free heap size: %lu\n", esp_get_minimum_free_heap_size());
  sprintf(buf,
          "<tr><td class=\"name\">Min free heap size:</td><td class=\"prop\">%lu kB (%lu)</td></tr>",
          esp_get_minimum_free_heap_size() / 1024,
          esp_get_minimum_free_heap_size());
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // printf("Last reset reson: ");
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:
      sprintf(temp, "Reset due to power-on event.\n");
      break;
    case ESP_RST_EXT:
      sprintf(temp, "Reset by external pin (not applicable for ESP32.\n");
      break;
    case ESP_RST_SW:
      sprintf(temp, "Software reset via esp_restart.\n");
      break;
    case ESP_RST_PANIC:
      sprintf(temp, "Software reset due to exception/panic.\n");
      break;
    case ESP_RST_INT_WDT:
      sprintf(temp, "Reset (software or hardware) due to interrupt watchdog.\n");
      break;
    case ESP_RST_TASK_WDT:
      sprintf(temp, "Reset due to task watchdog.\n");
      break;
    case ESP_RST_WDT:
      sprintf(temp, "Reset due to other watchdogs.\n");
      break;
    case ESP_RST_DEEPSLEEP:
      sprintf(temp, "Reset after exiting deep sleep mode.\n");
      break;
    case ESP_RST_BROWNOUT:
      sprintf(temp, "Brownout reset (software or hardware.\n");
      break;
    case ESP_RST_SDIO:
      sprintf(temp, "Reset over SDIO.\n");
      break;
    case ESP_RST_UNKNOWN:
    default:
      sprintf(temp, "Reset reason can not be determined.\n");
      break;
  }
  sprintf(buf, "<tr><td class=\"name\">Last reset reason:</td><td class=\"prop\">%s</td></tr>", temp);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<tr><td class=\"name\">Number of reboots:</td><td class=\"prop\">%lu</td></tr>", g_persistent.bootCnt);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  char *bufguid = (char *) calloc(50, 1);
  if (NULL != bufguid) {
    vscp_fwhlp_writeGuidToString(bufguid, g_persistent.guid);
    sprintf(buf, "<tr><td class=\"name\">GUID:</td><td class=\"prop\">%s</td></tr>", bufguid);
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
    free(bufguid);
  }

  // -------------------------------------------------------------------------

  // * * *  Application * * *
  // sprintf(buf, "<tr><td>Application</td><td></td></tr>");
  sprintf(buf, "<tr><td class='infoheader'>Application</td><td></td></tr>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  int64_t time = esp_timer_get_time();
  int64_t seconds = time / 1000000;
  sprintf(buf,
          "<tr><td class=\"name\">Uptime:</td><td class=\"prop\">%lldT%02lld:%02lld:%02lld</td></tr>",
          seconds / (3600 * 24),
          (seconds % (3600 * 24)) / 3600,
          (seconds % 3600) / 60,
          seconds % 60);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  if (NULL != appDescr) {
    // sprintf(temp,"%s",appDescr->project_name);
    sprintf(buf, "<tr><td class=\"name\">Application:</td><td class=\"prop\">%s</td></tr>", appDescr->project_name);
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

    // sprintf(temp,"Application ver: %s\n",appDescr->version);
        sprintf(buf,
          "<tr><td class=\"name\">Application ver:</td><td class=\"prop\">%s</td></tr>",
          web_ui_version(appDescr));
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

    // sprintf(temp,"Application ver: %s %s\n",appDescr->date,appDescr->time);
    sprintf(buf,
            "<tr><td class=\"name\">Compile time:</td><td class=\"prop\">%s %s</td></tr>",
            appDescr->date,
            appDescr->time);
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

    // sprintf(temp,"idf ver: %s\n",appDescr->idf_ver);
    sprintf(buf, "<tr><td class=\"name\">Compiled w/ idf ver:</td><td class=\"prop\">%s</td></tr>", appDescr->idf_ver);
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
  }

  // * * *  CAN/TWAI * * *
  sprintf(buf, "<tr><td class='infoheader'>CAN/TWAI Interface</td><td></td></tr>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // CAN Mode
  const char *canModeStr = (g_persistent.canMode == 0) ? "Normal" : "Listen Only";
  sprintf(buf, "<tr><td class=\"name\">CAN Mode:</td><td class=\"prop\">%s</td></tr>", canModeStr);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // CAN Speed
  const char *canSpeedStr;
  switch (g_persistent.canSpeed) {
    case 0:
      canSpeedStr = "5 Kbps";
      break;
    case 1:
      canSpeedStr = "10 Kbps";
      break;
    case 2:
      canSpeedStr = "20 Kbps";
      break;
    case 3:
      canSpeedStr = "25 Kbps";
      break;
    case 4:
      canSpeedStr = "50 Kbps";
      break;
    case 5:
      canSpeedStr = "100 Kbps";
      break;
    case 6:
      canSpeedStr = "125 Kbps";
      break;
    case 7:
      canSpeedStr = "250 Kbps";
      break;
    case 8:
      canSpeedStr = "500 Kbps";
      break;
    case 9:
      canSpeedStr = "800 Kbps";
      break;
    case 10:
      canSpeedStr = "1000 Kbps";
      break;
    default:
      canSpeedStr = "Unknown";
      break;
  }
  sprintf(buf, "<tr><td class=\"name\">CAN Speed:</td><td class=\"prop\">%s</td></tr>", canSpeedStr);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // CAN Filter
  sprintf(buf,
          "<tr><td class=\"name\">CAN Filter:</td><td class=\"prop\">0x%08X</td></tr>",
          (unsigned int) g_persistent.canFilter);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // Messages Sent
  sprintf(buf,
          "<tr><td class=\"name\">Messages Sent:</td><td class=\"prop\">%u</td></tr>",
          (unsigned int) g_persistent.nSent);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // Messages Received
  sprintf(buf,
          "<tr><td class=\"name\">Messages Received:</td><td class=\"prop\">%u</td></tr>",
          (unsigned int) g_persistent.nRecv);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // Total Errors
  sprintf(buf,
          "<tr><td class=\"name\">Total Errors:</td><td class=\"prop\">%u</td></tr>",
          (unsigned int) g_persistent.nErr);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // Last Error Code
  sprintf(buf,
          "<tr><td class=\"name\">Last Error Code:</td><td class=\"prop\">0x%08X</td></tr>",
          (unsigned int) g_persistent.lastError);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // * * *  Connection * * *
  sprintf(buf, "<tr><td class='infoheader'>Connection</td><td></td></tr>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  wifi_mode_t mode;
  esp_wifi_get_mode(&mode);
  switch (mode) {

    case WIFI_MODE_STA:
      sprintf(temp, "STA\n");
      break;

    case WIFI_MODE_AP:
      sprintf(temp, "AP\n");
      break;

    case WIFI_MODE_APSTA:
      sprintf(temp, "APSTA\n");
      break;

    case WIFI_MODE_NULL:
    default:
      sprintf(temp, "unknown\n");
      break;
  };
  // sprintf(temp,"Wifi mode: ");
  sprintf(buf, "<tr><td class=\"name\">Wifi mode:</td><td class=\"prop\">%s</td></tr>", temp);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  wifi_sta_list_t sta;
  esp_wifi_ap_get_sta_list(&sta);
  // printf("Stations: %d\n",sta.num);
  sprintf(buf, "<tr><td class=\"name\">Stations:</td><td class=\"prop\">%d</td></tr>", sta.num);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  wifi_ap_record_t ap_info;
  esp_wifi_sta_get_ap_info(&ap_info);
  // printf("bssid: " MACSTR "\n", MAC2STR(ap_info.bssid));
  sprintf(buf, "<tr><td class=\"name\">bssid:</td><td class=\"prop\">" MACSTR "</td></tr>", MAC2STR(ap_info.bssid));
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // printf("ssid: %s\n", ap_info.ssid);
  sprintf(buf, "<tr><td class=\"name\">ssid:</td><td class=\"prop\">%s</td></tr>", ap_info.ssid);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // printf("channel: %d (%d)\n", ap_info.primary, ap_info.second);
  sprintf(buf,
          "<tr><td class=\"name\">channel:</td><td class=\"prop\">%d (%d)</td></tr>",
          ap_info.primary,
          ap_info.second);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // printf("signal strength: %d\n", ap_info.rssi);
  if (ap_info.rssi > -30) {
    sprintf(temp, "Perfect");
  }
  else if (ap_info.rssi > -50) {
    sprintf(temp, "Excellent");
  }
  else if (ap_info.rssi > -60) {
    sprintf(temp, "Good");
  }
  else if (ap_info.rssi > -67) {
    sprintf(temp, "Limited");
  }
  else if (ap_info.rssi > -70) {
    sprintf(temp, "Poor");
  }
  else if (ap_info.rssi > -80) {
    sprintf(temp, "Unstable");
  }
  else {
    sprintf(temp, "Unusable");
  }

  sprintf(buf,
          "<tr><td class=\"name\">signal strength:</td><td class=\"prop\">%d dBm ( %d%% = %s)</td></tr>",
          ap_info.rssi,
          (2 * (ap_info.rssi + 100)),
          temp);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // printf("Mode: 11%s%s%s %s %s",
  //           ap_info.phy_11b ? "b" : "",
  //           ap_info.phy_11g ? "g" : "",
  //           ap_info.phy_11n ? "n" : "",
  //           ap_info.phy_lr ? "lr" : "",
  //           ap_info.wps ? "wps" : "");
  // printf("\nAuth mode of AP: ");
  switch (ap_info.authmode) {

    case WIFI_AUTH_OPEN:
      sprintf(temp, "open\n");
      break;

    case WIFI_AUTH_WEP:
      sprintf(temp, "wep\n");
      break;

    case WIFI_AUTH_WPA_PSK:
      sprintf(temp, "wpa-psk\n");
      break;

    case WIFI_AUTH_WPA2_PSK:
      sprintf(temp, "wpa2-psk\n");
      break;

    case WIFI_AUTH_WPA_WPA2_PSK:
      sprintf(temp, "wpa-wpa2-psk\n");
      break;

    case WIFI_AUTH_WPA2_ENTERPRISE:
      sprintf(temp, "wpa2-enterprise\n");
      break;

    case WIFI_AUTH_WPA3_PSK:
      sprintf(temp, "wpa3-psk\n");
      break;

    case WIFI_AUTH_WPA2_WPA3_PSK:
      sprintf(temp, "wpa2-wpa3-psk\n");
      break;

    case WIFI_AUTH_WAPI_PSK:
      sprintf(temp, "wpa2-wapi-psk\n");
      break;

    case WIFI_AUTH_OWE:
      sprintf(temp, "wpa2-wapi-psk\n");
      break;

    default:
      sprintf(temp, "unknown\n");
      break;
  }

  sprintf(buf, "<tr><td class=\"name\">Auth mode of AP:</td><td class=\"prop\">%s</td></tr>", temp);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  uint8_t mac[6];
  esp_wifi_get_mac(ESP_MAC_WIFI_STA, mac);
  // printf("Wifi STA MAC address: " MACSTR "\n", MAC2STR(mac));
  sprintf(buf,
          "<tr><td class=\"name\">Wifi STA MAC address:</td><td class=\"prop\">" MACSTR "</td></tr>",
          MAC2STR(mac));
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  esp_wifi_get_mac(ESP_MAC_WIFI_SOFTAP, mac);
  // printf("Wifi SOFTAP MAC address: " MACSTR "\n", MAC2STR(mac));
  sprintf(buf,
          "<tr><td class=\"name\">Wifi SOFTAP MAC address:</td><td class=\"prop\">" MACSTR "</td></tr>",
          MAC2STR(mac));
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  esp_netif_t *netif = NULL;
  netif              = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  esp_netif_ip_info_t ifinfo;
  if (NULL != netif) {
    esp_netif_get_ip_info(netif, &ifinfo);
    // printf("IP address (wifi): " IPSTR "\n", IP2STR(&ifinfo.ip));
    sprintf(buf,
            "<tr><td class=\"name\">IP address (wifi):</td><td class=\"prop\">" IPSTR "</td></tr>",
            IP2STR(&ifinfo.ip));
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

    // printf("Subnet Mask: " IPSTR "\n", IP2STR(&ifinfo.netmask));
    sprintf(buf,
            "<tr><td class=\"name\">Subnet Mask:</td><td class=\"prop\">" IPSTR "</td></tr>",
            IP2STR(&ifinfo.netmask));
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

    // printf("Gateway: " IPSTR "\n", IP2STR(&ifinfo.gw));
    sprintf(buf, "<tr><td class=\"name\">Gateway:</td><td class=\"prop\">" IPSTR "</td></tr>", IP2STR(&ifinfo.gw));
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

    esp_netif_dns_info_t dns;
    esp_netif_get_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns);
    // printf("DNS DNS Server1: " IPSTR "\n", IP2STR(&dns.ip.u_addr.ip4));
    sprintf(buf,
            "<tr><td class=\"name\">DNS Server1:</td><td class=\"prop\">" IPSTR "</td></tr>",
            IP2STR(&dns.ip.u_addr.ip4));
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
    esp_netif_get_dns_info(netif, ESP_NETIF_DNS_BACKUP, &dns);

    // printf("DNS Server2: " IPSTR "\n", IP2STR(&dns.ip.u_addr.ip4));
    sprintf(buf,
            "<tr><td class=\"name\">DNS Server2:</td><td class=\"prop\">" IPSTR "</td></tr>",
            IP2STR(&dns.ip.u_addr.ip4));
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
  }

  sprintf(buf, "</table>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, WEBPAGE_END_TEMPLATE, web_ui_version(appDescr), g_persistent.nodeName);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  httpd_resp_send_chunk(req, NULL, 0);

  free(buf);
  free(temp);

  return ESP_OK;
}

// URI handler for getting uploaded files
httpd_uri_t info = { .uri = "/info", .method = HTTP_GET, .handler = info_get_handler, .user_ctx = NULL };

///////////////////////////////////////////////////////////////////////////////
// reset_get_handler
//
// HTTP GET handler for reset of machine
//

static esp_err_t
reset_get_handler(httpd_req_t *req)
{
  const char *resp_str = "<html><head><meta charset='utf-8'><meta http-equiv=\"refresh\" content=\"2;url=index.html\" "
                         "/></head><body><h1>The system is restarting...</h1></body></html>";
  httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

  // Let content render
  vTaskDelay(2000 / portTICK_PERIOD_MS);

  // esp_wifi_disconnect();
  // vTaskDelay(2000 / portTICK_PERIOD_MS);
  esp_restart();
  return ESP_OK;
}

httpd_uri_t reset = { .uri = "/reset", .method = HTTP_GET, .handler = reset_get_handler, .user_ctx = NULL };

///////////////////////////////////////////////////////////////////////////////
// upgrade_get_handler
//
// HTTP GET handler for update of firmware
//

static esp_err_t
send_restart_info_page(httpd_req_t *req, const char *message)
{
  httpd_resp_sendstr_chunk(req, "<html><head><meta charset='utf-8'><meta http-equiv=\"refresh\" content=\"2;url=index.html\" /><style>");
  httpd_resp_sendstr_chunk(req, WEBPAGE_STYLE_CSS);
  httpd_resp_sendstr_chunk(req,
                           "</style></head><body><div style='text-align:left;display:inline-block;color:#eaeaea;min-width:340px;max-width:600px;'>"
                           "<div style='text-align:center;color:#eaeaea;'><h3>");
  httpd_resp_sendstr_chunk(req, g_persistent.nodeName);
  httpd_resp_sendstr_chunk(req,
                           "</h3></div><div style='text-align:center;color:#f7f1a6;'><h4>");
  httpd_resp_sendstr_chunk(req, message);
  httpd_resp_sendstr_chunk(
    req,
    "</h4></div><p class=\"prop\" style='text-align:center;'>The page will reconnect automatically after the restart.</p></div></body></html>");

  return httpd_resp_sendstr_chunk(req, NULL);
}
// - Server upgrade
// - Local upgrade
//

static esp_err_t
upgrade_get_handler(httpd_req_t *req)
{
  // esp_err_t rv;
  char *buf;
  char *temp;

  buf = (char *) calloc(CHUNK_BUFSIZE, 1);
  if (NULL == buf) {
    return ESP_ERR_NO_MEM;
  }

  temp = (char *) calloc(80, 1);
  if (NULL == temp) {
    return ESP_ERR_NO_MEM;
  }

  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);

  const esp_app_desc_t *appDescr = esp_app_get_description();

  sprintf(buf, WEBPAGE_START_TEMPLATE, g_persistent.nodeName, "Upgrade firmware");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<h3>Upgrade from web server</h3>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<div><form id=but3 class=\"button\" action='/upgrdsrv' method='get'><fieldset>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "OTA URL:<input type=\"text\"  id=\"url\" name=\"url\" value=\"%s\" >", DEFAULT_APP_OTA_URL);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<button class=\"bgrn bgrn:hover\" >Start Upgrade</button></fieldset></form></div>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // ----- Local -----

  sprintf(buf,
          "<h3>Upgrade from local file</h3><div> <span style=\"color:red;font-family:verdana;font-size:300%%;\" "
          "id=\"progress\" /></div>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<div><fform id=but3 class=\"button\" action='/upgrdlocal' method='post'><fieldset>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<label for=\"otafile\">OTA firmware file:</label><input type=\"file\" id=\"otafile\" name=\"otafile\" />");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<button class=\"bgrn bgrn:hover\" id=\"upload\" onclick=\"startUpload();\">Start "
          "Upgrade</button></fieldset></fform></div>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, WEBPAGE_END_TEMPLATE, web_ui_version(appDescr), g_persistent.nodeName);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  httpd_resp_send_chunk(req, NULL, 0);

  free(buf);
  free(temp);

  return ESP_OK;
}

// httpd_uri_t upgrade = { .uri = "/upgrade", .method = HTTP_GET, .handler = upgrade_get_handler, .user_ctx = NULL };

///////////////////////////////////////////////////////////////////////////////
// upgrdsrv_get_handler
//

static esp_err_t
upgrdsrv_get_handler(httpd_req_t *req)
{
  esp_err_t ret;
  char *buf = NULL;
  char *req_buf;
  size_t req_buf_len;

  buf = (char *) calloc(CHUNK_BUFSIZE, 1);
  if (NULL == buf) {
    return ESP_ERR_NO_MEM;
  }

  // Read URL query string length and allocate memory for length + 1,
  // extra byte for null termination
  req_buf_len = httpd_req_get_url_query_len(req) + 1;
  if (req_buf_len > 1) {
    req_buf = (char *) malloc(req_buf_len);
    if (httpd_req_get_url_query_str(req, req_buf, req_buf_len) == ESP_OK) {

      ESP_LOGD(TAG, "Found URL query => %s", req_buf);
      char *param = (char *) malloc(WEBPAGE_PARAM_SIZE);
      if (NULL == param) {
        free(req_buf);
        free(buf);
        return ESP_ERR_NO_MEM;
      }

      // URL
      if (ESP_OK == (ret = httpd_query_key_value(req_buf, "url", param, WEBPAGE_PARAM_SIZE))) {
        ESP_LOGD(TAG, "Found query parameter => url=%s", param);
      }
      else {
        ESP_LOGE(TAG, "Error getting url => rv=%d", ret);
      }

      free(param);
    }

    free(req_buf);
  }

  free(buf);

  const char *resp_str = "<html><head><meta charset='utf-8'><meta http-equiv=\"refresh\" content=\"10;url=index.html\" "
                         "/></head><body><h1>Firmware upgrade in progress...</h1></body></html>";
  httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

  // Let content render
  vTaskDelay(2000 / portTICK_PERIOD_MS);

  printf("Start OTA\n");

  // Start the OTA task
  startOTA();

  return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////
// upgrdlocal_post_handler
//
// Handle OTA file upload
// https://github.com/Jeija/esp32-softap-ota
//

esp_err_t
upgrdlocal_post_handler(httpd_req_t *req)
{
  char buf[1000];
  esp_ota_handle_t ota_handle;
  int remaining = req->content_len;

  const esp_partition_t *ota_partition = esp_ota_get_next_update_partition(NULL);
  ESP_ERROR_CHECK(esp_ota_begin(ota_partition, OTA_SIZE_UNKNOWN, &ota_handle));

  while (remaining > 0) {

    ESP_LOGD(TAG, "OTA remaining %d", remaining);

    int recv_len = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)));

    // Timeout Error: Just retry
    if (recv_len == HTTPD_SOCK_ERR_TIMEOUT) {
      continue;
    }
    else if (recv_len <= 0) {
      // Serious Error: Abort OTA
      ESP_LOGE(TAG, "OTA aborted due to protocol error");
      httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Protocol Error");
      return ESP_FAIL;
    }

    // Successful Upload: Flash firmware chunk
    if (esp_ota_write(ota_handle, (const void *) buf, recv_len) != ESP_OK) {
      ESP_LOGE(TAG, "OTA aborted due to flash error");
      httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Flash Error");
      return ESP_FAIL;
    }

    remaining -= recv_len;
  }

  // Validate and switch to new OTA image and reboot
  if (esp_ota_end(ota_handle) != ESP_OK || esp_ota_set_boot_partition(ota_partition) != ESP_OK) {
    ESP_LOGE(TAG, "OTA failed due to image activation error");
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Validation / Activation Error");
    return ESP_FAIL;
  }

  ESP_LOGD(TAG, "OTA finished");

  // httpd_resp_sendstr(req, "Firmware update complete, rebooting now!\n");
  const char *resp_str = "<html><head><meta charset='utf-8'><meta http-equiv=\"refresh\" content=\"2;url=index.html\" "
                         "/></head><body><h1>Firmware update complete, rebooting now!...</h1></body></html>";
  httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

  // Let content render
  vTaskDelay(500 / portTICK_PERIOD_MS);
  esp_restart();

  return ESP_OK;
}

static const httpd_uri_t upgrdlocal = { .uri      = "/upgrdlocal",
                                        .method   = HTTP_POST,
                                        .handler  = upgrdlocal_post_handler,
                                        .user_ctx = NULL };

///////////////////////////////////////////////////////////////////////////////
// upgrdsibling_get_handler
//
static esp_err_t
upgrdsibling_get_handler(httpd_req_t *req)
{
  esp_err_t ret;
  char *buf = NULL;
  char *req_buf;
  size_t req_buf_len;
  char url[DEFAULT_APP_OTA_URL_MAX_SIZE];

  buf = (char *) calloc(CHUNK_BUFSIZE, 1);
  if (NULL == buf) {
    return ESP_ERR_NO_MEM;
  }

  // Read URL query string length and allocate memory for length + 1,
  // extra byte for null termination
  req_buf_len = httpd_req_get_url_query_len(req) + 1;
  if (req_buf_len > 1) {
    req_buf = (char *) malloc(req_buf_len);
    if (httpd_req_get_url_query_str(req, req_buf, req_buf_len) == ESP_OK) {

      ESP_LOGD(TAG, "Found URL query => %s", req_buf);
      char *param = (char *) malloc(WEBPAGE_PARAM_SIZE);
      if (NULL == param) {
        free(req_buf);
        free(buf);
        return ESP_ERR_NO_MEM;
      }

      // URL
      if (ESP_OK == (ret = httpd_query_key_value(req_buf, "url", param, WEBPAGE_PARAM_SIZE))) {

        char *pdecoded = urlDecode(param);
        if (NULL == pdecoded) {
          free(param);
          free(req_buf);
          free(buf);
          return ESP_ERR_NO_MEM;
        }

        memset(url, 0, DEFAULT_APP_OTA_URL_MAX_SIZE);
        strncpy(url, pdecoded, MIN(strlen(pdecoded), (DEFAULT_APP_OTA_URL_MAX_SIZE - 1)));

        ESP_LOGD(TAG, "Found query parameter => url=%s", url);

        free(pdecoded);
      }
      else {
        ESP_LOGE(TAG, "Error getting url => rv=%d", ret);
      }

      free(param);
    }

    free(req_buf);
  }

  free(buf);

  const char *resp_str = "<html><head><meta charset='utf-8'><meta http-equiv=\"refresh\" content=\"10;url=index.html\" "
                         "/></head><body><h1>Firmware upgrade of sibling in progress...</h1></body></html>";
  httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

  // Let content render
  vTaskDelay(2000 / portTICK_PERIOD_MS);

  printf("Start OTA update of beta/gamma node(s)  url=%s\n", url);

  // Start the OTA task

  ret = app_initiate_firmware_upload(url);
  if (ESP_OK != ret) {
    ESP_LOGE(TAG, "Initiation of sibling firmware upload failed ret=%x", ret);
    return ret;
  }

  return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////
// upgrdlocal_post_handler
//
// Handle OTA file upload
// https://github.com/Jeija/esp32-softap-ota
//

esp_err_t
upgrdSiblingslocal_post_handler(httpd_req_t *req)
{
  char buf[1000];
  uint8_t sha_256[32] = { 0 };
  esp_ota_handle_t ota_handle;
  int remaining = req->content_len;

  const esp_partition_t *ota_partition = esp_ota_get_next_update_partition(NULL);
  ESP_ERROR_CHECK(esp_ota_begin(ota_partition, OTA_SIZE_UNKNOWN, &ota_handle));

  while (remaining > 0) {

    ESP_LOGD(TAG, "OTA remaining %d", remaining);

    taskYIELD();

    int recv_len = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)));

    // Timeout Error: Just retry
    if (recv_len == HTTPD_SOCK_ERR_TIMEOUT) {
      continue;
    }
    else if (recv_len <= 0) {
      // Serious Error: Abort OTA
      ESP_LOGE(TAG, "OTA aborted due to protocol error");
      httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Protocol Error");
      return ESP_FAIL;
    }

    // Successful Upload: Flash firmware chunk
    if (esp_ota_write(ota_handle, (const void *) buf, recv_len) != ESP_OK) {
      ESP_LOGE(TAG, "OTA aborted due to flash error");
      httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Flash Error");
      return ESP_FAIL;
    }

    remaining -= recv_len;
  }

  // Validate and switch to new OTA image and reboot
  if (esp_ota_end(ota_handle) != ESP_OK /*|| esp_ota_set_boot_partition(ota_partition) != ESP_OK*/) {
    ESP_LOGE(TAG, "OTA failed due to image activation error");
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Validation / Activation Error");
    return ESP_FAIL;
  }

  ESP_LOGD(TAG, "Send OTA to sibling(s)");

  esp_partition_get_sha256(ota_partition, sha_256);

  // Send new firmware to clients
  // app_firmware_send(req->content_len, sha_256);

  ESP_LOGD(TAG, "Images sent to sibling(s)");

  // httpd_resp_sendstr(req, "Firmware update complete, rebooting now!\n");
  const char *resp_str = "<html><head><meta charset='utf-8'><meta http-equiv=\"refresh\" content=\"2;url=index.html\" "
                         "/></head><body><h1>Firmware update complete, rebooting now!...</h1></body></html>";
  httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

  return ESP_OK;
}

static const httpd_uri_t upgrdsiblinglocal = { .uri      = "/upgrdSiblingLocal",
                                               .method   = HTTP_POST,
                                               .handler  = upgrdSiblingslocal_post_handler,
                                               .user_ctx = NULL };

///////////////////////////////////////////////////////////////////////////////
// hello_get_handler
//
// Copies the full path into destination buffer and returns
// pointer to path (skipping the preceding base path)
//

static const char *
get_path_from_uri(char *dest, const char *base_path, const char *uri, size_t destsize)
{
  const size_t base_pathlen = strlen(base_path);
  size_t pathlen            = strlen(uri);

  const char *quest = strchr(uri, '?');
  if (quest) {
    pathlen = MIN(pathlen, quest - uri);
  }
  const char *hash = strchr(uri, '#');
  if (hash) {
    pathlen = MIN(pathlen, hash - uri);
  }

  if (base_pathlen + pathlen + 1 > destsize) {
    // Full path string won't fit into destination buffer
    return NULL;
  }

  // Construct full path (base + path)
  strcpy(dest, base_path);
  strlcpy(dest + base_pathlen, uri, pathlen + 1);

  // Return pointer to path, skipping the base
  return dest + base_pathlen;
}

///////////////////////////////////////////////////////////////////////////////
// hello_get_handler
//
// An HTTP GET handler
//

static esp_err_t
hello_get_handler(httpd_req_t *req)
{
  char *buf;
  size_t buf_len;

  // Get header value string length and allocate memory for length + 1,
  // extra byte for null termination
  buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
  if (buf_len > 1) {
    buf = (char *) malloc(buf_len);
    // Copy null terminated value string into buffer
    if (httpd_req_get_hdr_value_str(req, "Host", buf, buf_len) == ESP_OK) {
      ESP_LOGD(TAG, "Found header => Host: %s", buf);
    }
    free(buf);
  }

  buf_len = httpd_req_get_hdr_value_len(req, "Test-Header-2") + 1;
  if (buf_len > 1) {
    buf = (char *) malloc(buf_len);
    if (NULL == buf) {
      return ESP_ERR_NO_MEM;
    }
    if (httpd_req_get_hdr_value_str(req, "Test-Header-2", buf, buf_len) == ESP_OK) {
      ESP_LOGD(TAG, "Found header => Test-Header-2: %s", buf);
    }
    free(buf);
  }

  buf_len = httpd_req_get_hdr_value_len(req, "Test-Header-1") + 1;
  if (buf_len > 1) {
    buf = (char *) malloc(buf_len);
    if (httpd_req_get_hdr_value_str(req, "Test-Header-1", buf, buf_len) == ESP_OK) {
      ESP_LOGD(TAG, "Found header => Test-Header-1: %s", buf);
    }
    free(buf);
  }

  // Read URL query string length and allocate memory for length + 1,
  // extra byte for null termination
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char *) malloc(buf_len);
    if (NULL == buf) {
      return ESP_ERR_NO_MEM;
    }
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      ESP_LOGD(TAG, "Found URL query => %s", buf);
      char param[32];
      // Get value of expected key from query string
      if (httpd_query_key_value(buf, "query1", param, sizeof(param)) == ESP_OK) {
        ESP_LOGD(TAG, "Found URL query parameter => query1=%s", param);
      }
      if (httpd_query_key_value(buf, "query3", param, sizeof(param)) == ESP_OK) {
        ESP_LOGD(TAG, "Found URL query parameter => query3=%s", param);
      }
      if (httpd_query_key_value(buf, "query2", param, sizeof(param)) == ESP_OK) {
        ESP_LOGD(TAG, "Found URL query parameter => query2=%s", param);
      }
    }
    free(buf);
  }

  // Set some custom headers
  httpd_resp_set_hdr(req, "Custom-Header-1", "Custom-Value-1");
  httpd_resp_set_hdr(req, "Custom-Header-2", "Custom-Value-2");

  // Send response with custom headers and body set as the
  // string passed in user context
  const char *resp_str = "Be Hungry - Stay Foolish!"; //(const char *) req->user_ctx;
  httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

  // After sending the HTTP response the old HTTP request
  // headers are lost. Check if HTTP request headers can be read now.
  if (httpd_req_get_hdr_value_len(req, "Host") == 0) {
    ESP_LOGD(TAG, "Request headers lost");
  }

  return ESP_OK;
}

static const httpd_uri_t hello = { .uri     = "/hello",
                                   .method  = HTTP_GET,
                                   .handler = hello_get_handler,
                                   // Let's pass response string in user
                                   // context to demonstrate it's usage
                                   .user_ctx = "Be Hungry - Stay Foolish!" };

///////////////////////////////////////////////////////////////////////////////
// mainpg_get_handler
//
// Mainpage for web interface
//

static esp_err_t
mainpg_get_handler(httpd_req_t *req)
{
  char *buf;

  char *req_buf;
  size_t req_buf_len;

  buf = (char *) calloc(CHUNK_BUFSIZE, 1);
  if (NULL == buf) {
    return ESP_ERR_NO_MEM;
  }

  // Get application info data
  const esp_app_desc_t *appDescr = esp_app_get_description();

  // Get header value string length and allocate memory for length + 1,
  // extra byte for null termination
  req_buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
  if (req_buf_len > 1) {
    req_buf = (char *) malloc(req_buf_len);
    // Copy null terminated value string into buffer
    if (httpd_req_get_hdr_value_str(req, "Host", req_buf, req_buf_len) == ESP_OK) {
      ESP_LOGD(TAG, "Found header => Host: %s", req_buf);
    }
    free(req_buf);
  }

  sprintf(buf, WEBPAGE_START_TEMPLATE, g_persistent.nodeName, "Main Page");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<p><form id=but1 class=\"button\" action='config' method='get'><button>Configuration</button></form></p>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<p><form id=but2 class=\"button\" action='info' method='get'><button>Node Information</button></form></p>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

    sprintf(buf,
      "<p><form id=but2a class=\"button\" action='canbus' method='get'><button>CAN Bus Console</button></form></p>");
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

    sprintf(buf,
        "<p><form id=but2b class=\"button\" action='/logs' method='get'><button>Live Logs</button></form></p>");
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<p><form id=but3 class=\"button\" action='upgrade' method='get'><button>Firmware "
          "Upgrade</button></form></p>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<p><form id=but5 class=\"button\" action='reset' method='get'><button name='rst' class='button "
          "bred'>Restart</button></form></p>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, WEBPAGE_END_TEMPLATE_NO_RETURN, web_ui_version(appDescr), g_persistent.nodeName);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  httpd_resp_send_chunk(req, NULL, 0);

  free(buf);

  return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////
// canbus_get_handler
//
// CAN bus web console using direct device HTTP API
//

static esp_err_t
canbus_get_handler(httpd_req_t *req)
{
  char *buf = (char *) calloc(CHUNK_BUFSIZE, 1);
  if (NULL == buf) {
    return ESP_ERR_NO_MEM;
  }

  const esp_app_desc_t *appDescr = esp_app_get_description();

  sprintf(buf, WEBPAGE_START_TEMPLATE, g_persistent.nodeName, "CAN Bus Console");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<p style='font-size:0.95rem;color:#ddd;'>"
          "Direct device API access to TWAI/CAN. This page sends commands straight to the firmware CAN backend."
          "</p>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<fieldset><legend>Interface</legend>"
          "<div class='p'>Mode</div><div class='q'>Direct TWAI/CAN API</div><br style='clear:both'>"
          "<div class='p'>Endpoint</div><div class='q'>/canapi</div><br style='clear:both'>"
          "</fieldset>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<br><fieldset><legend>Send VSCP Event To CAN</legend>"
          "<table style='width:100%%'>"
          "<tr><td>Priority (0-7)</td><td><input id='prio' value='0'></td></tr>"
          "<tr><td>Class</td><td><input id='vclass' value='0'></td></tr>"
          "<tr><td>Type</td><td><input id='vtype' value='1'></td></tr>"
          "<tr><td>Node/Nickname</td><td><input id='nick' value='255'></td></tr>"
      "<tr><td>Data (comma separated)</td><td><input id='data' value='' placeholder='0x01,2,0b00000011'></td></tr>"
          "</table>"
          "<button id='btnSendEvent' type='button' class='bgrn'>Send Event</button>"
          "</fieldset>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<br><fieldset><legend>CAN Bus Node Discovery</legend>"
          "<table style='width:100%%'>"
          "<tr><td>Who Is There target nickname</td><td><input id='whisNick' value='255'></td></tr>"
          "<tr><td>Scan start nickname</td><td><input id='scanStart' value='1'></td></tr>"
          "<tr><td>Scan end nickname</td><td><input id='scanEnd' value='255'></td></tr>"
      "<tr><td>Serial per-node scan</td><td><input id='scanSerial' type='checkbox' checked></td></tr>"
      "<tr><td>Per-node timeout (ms)</td><td><input id='scanPerNodeMs' value='20'></td></tr>"
          "</table>"
          "<button id='btnWhis' type='button'>Send Who Is There</button><br><br>"
          "<button id='btnScan' type='button'>Run Node Scan</button>"
          "</fieldset>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<br><fieldset><legend>TWAI Monitor Filters</legend>"
          "<table style='width:100%%'>"
          "<tr><td>CAN ID min (hex)</td><td><input id='filterIdMin' value='0' placeholder='0x0'></td></tr>"
          "<tr><td>CAN ID max (hex)</td><td><input id='filterIdMax' value='FFFFFFFF' placeholder='0xFFFFFFFF'></td></tr>"
          "<tr><td>VSCP Class</td><td><input id='filterClass' value='' placeholder='(empty=all)'></td></tr>"
          "<tr><td>VSCP Type</td><td><input id='filterType' value='' placeholder='(empty=all)'></td></tr>"
          "<tr><td>Node Nickname</td><td><input id='filterNick' value='' placeholder='(empty=all)'></td></tr>"
          "<tr><td>Extended Frames Only</td><td><input id='filterExtd' type='checkbox' checked></td></tr>"
      "<tr><td>Show transmitted frames</td><td><input id='showTxInLog' type='checkbox'></td></tr>"
          "</table>"
          "</fieldset>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<br><fieldset><legend>API Replies</legend>"
      "<div style='font-size:0.9rem;margin-bottom:6px'>"
      "<span style='color:#66d9ff'>SCAN</span> "
      "<span style='color:#ffd166'>WHO IS THERE</span> "
      "<span style='color:#d6b3ff'>SEND</span> "
      "<span style='color:#ff7b72'>ERROR</span> "
      "<span style='color:#8df186'>TWAI</span>"
      "</div>"
      "<div style='margin-bottom:6px'><button id='btnClearLog' type='button'>Clear Log</button></div>"
      "<div id='log' style='height:260px;overflow:auto;background:#101110;color:#8df186;"
      "font-family:monospace;padding:6px;white-space:pre-wrap;border:1px solid #2f3a2f'></div>"
          "</fieldset>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<script>"
          "(function(){"
          "function el(id){return document.getElementById(id);}"
      "function parseByteToken(tok){"
      "  var s=(tok||'').trim();"
      "  if(!s.length) return null;"
      "  if(/^0[bB][01]+$/.test(s)){var v=0;for(var i=2;i<s.length;i++){v=(v<<1)+(s.charCodeAt(i)-48);}return (v>=0&&v<=255)?v:null;}"
      "  if(/^0[xX][0-9a-fA-F]+$/.test(s)){var v=parseInt(s.substring(2),16);return (v>=0&&v<=255)?v:null;}"
      "  if(/^0[dD][0-9]+$/.test(s)){var v=parseInt(s.substring(2),10);return (v>=0&&v<=255)?v:null;}"
      "  if(/^[0-9]+$/.test(s)){var v=parseInt(s,10);return (v>=0&&v<=255)?v:null;}"
      "  return null;"
      "}"
      "function hexByte(v){var s=(v&255).toString(16).toUpperCase();return (s.length<2?'0':'')+s;}"
      "function buildTxPreview(){"
      "  var prio=parseInt(el('prio').value.trim(),10);"
      "  var vclass=parseInt(el('vclass').value.trim(),10);"
      "  var vtype=parseInt(el('vtype').value.trim(),10);"
      "  var nick=parseInt(el('nick').value.trim(),10);"
      "  if(!(prio>=0&&prio<=7&&vclass>=0&&vclass<=511&&vtype>=0&&vtype<=255&&nick>=0&&nick<=255)){return null;}"
      "  var raw=el('data').value.trim();"
      "  var parts=[];"
      "  if(raw.length){parts=raw.split(/[\\s,;]+/).filter(function(x){return x.length>0;});}"
      "  if(parts.length>8){return null;}"
      "  var bytes=[];"
      "  for(var i=0;i<parts.length;i++){var b=parseByteToken(parts[i]);if(null===b){return null;}bytes.push(b);}"
      "  var canid=(nick + (vtype<<8) + (vclass<<16) + ((prio&7)<<26))>>>0;"
      "  var data=bytes.map(hexByte).join(':');"
      "  return 'TX: id=0x'+canid.toString(16).toUpperCase()+' prio='+prio+' class='+vclass+' type='+vtype+' nick='+nick+' extd=1 dlc='+bytes.length+' data='+data;"
      "}"
      "function log(s,cls){var t=el('log');if(!t)return;var d=document.createElement('div');"
      "d.textContent=(new Date()).toISOString()+\" \"+s;"
      "if('scan'===cls){d.style.color='#66d9ff';}"
      "else if('whis'===cls){d.style.color='#ffd166';}"
      "else if('error'===cls){d.style.color='#ff7b72';}"
      "else if('twai'===cls){d.style.color='#8df186';}"
      "else if('send'===cls){d.style.color='#d6b3ff';}"
      "t.appendChild(d);t.scrollTop=t.scrollHeight;}"
          "function api(q){fetch('/canapi?'+q,{method:'GET'})"
          "  .then(function(r){return r.text().then(function(t){return {ok:r.ok,status:r.status,text:t};});})"
      "  .then(function(res){"
      "    var cls='';"
      "    if(q.indexOf('cmd=scan')===0) cls='scan';"
      "    else if(q.indexOf('cmd=whis')===0) cls='whis';"
      "    else if(q.indexOf('cmd=send')===0) cls='send';"
      "    if(!res.ok) cls='error';"
      "    log('API '+res.status+' '+res.text,cls);"
      "  })"
      "  .catch(function(e){log('API error '+e,'error');});}"
          "el('btnClearLog').onclick=function(){var t=el('log');if(t){t.textContent='';}};"
          "el('btnSendEvent').onclick=function(){"
          "  if(el('showTxInLog').checked){var preview=buildTxPreview();if(preview){log(preview,'send');}else{log('TX preview unavailable: invalid outgoing frame fields','error');}}"
          "  var q='cmd=send'"
          "    +'&prio='+encodeURIComponent(el('prio').value.trim())"
          "    +'&class='+encodeURIComponent(el('vclass').value.trim())"
          "    +'&type='+encodeURIComponent(el('vtype').value.trim())"
          "    +'&nick='+encodeURIComponent(el('nick').value.trim())"
          "    +'&data='+encodeURIComponent(el('data').value.trim());"
          "  api(q);"
          "};"
          "el('btnWhis').onclick=function(){"
          "  api('cmd=whis&nick='+encodeURIComponent(el('whisNick').value.trim()));"
          "};"
          "el('btnScan').onclick=function(){"
          "  var q='cmd=scan&start='+encodeURIComponent(el('scanStart').value.trim())"
          "    +'&end='+encodeURIComponent(el('scanEnd').value.trim())"
          "    +'&mode='+(el('scanSerial').checked?'serial':'burst')"
          "    +'&pernode_ms='+encodeURIComponent(el('scanPerNodeMs').value.trim());"
          "  api(q);"
          "};"
          "log('Starting TWAI bus monitor - polling for CAN frames...','twai');"
          "var twaiRunning=true;"
          "function twaiMonitor(){"
          "  if(!twaiRunning) return;"
          "  var q='cmd=monitor';"
          "  var idMin=el('filterIdMin').value.trim();"
          "  var idMax=el('filterIdMax').value.trim();"
          "  var cls=el('filterClass').value.trim();"
          "  var typ=el('filterType').value.trim();"
          "  var nick=el('filterNick').value.trim();"
          "  var extdOnly=el('filterExtd').checked;"
          "  if(idMin) q+='&id_min='+encodeURIComponent(idMin);"
          "  if(idMax) q+='&id_max='+encodeURIComponent(idMax);"
          "  if(cls) q+='&class='+encodeURIComponent(cls);"
          "  if(typ) q+='&type='+encodeURIComponent(typ);"
          "  if(nick) q+='&nick='+encodeURIComponent(nick);"
          "  if(extdOnly) q+='&extd_only=1';"
          "  fetch('/canapi?'+q,{method:'GET'})"
          "    .then(function(r){return r.text();})"
          "    .then(function(text){"
          "      if(text && text.length>0){"
          "        var lines=text.split('\\n');"
          "        for(var i=0;i<lines.length;i++){"
          "          if(lines[i].length>0){"
          "            log('TWAI: '+lines[i],'twai');"
          "          }"
          "        }"
          "      }"
          "      setTimeout(twaiMonitor,500);"
          "    })"
          "    .catch(function(e){"
          "      log('TWAI monitor error: '+e,'error');"
          "      setTimeout(twaiMonitor,500);"
          "    });"
          "}"
          "twaiMonitor();"
          "})();"
          "</script>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, WEBPAGE_END_TEMPLATE, web_ui_version(appDescr), g_persistent.nodeName);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
  httpd_resp_send_chunk(req, NULL, 0);

  free(buf);
  return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////
// logs_get_handler
//
// Web page that displays device logs in near real-time.
//

static esp_err_t
logs_get_handler(httpd_req_t *req)
{
  char *buf = (char *) calloc(CHUNK_BUFSIZE, 1);
  if (NULL == buf) {
    return ESP_ERR_NO_MEM;
  }

  const esp_app_desc_t *appDescr = esp_app_get_description();

  sprintf(buf, WEBPAGE_START_TEMPLATE, g_persistent.nodeName, "Live Log Console");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<p style='font-size:0.95rem;color:#ddd;'>"
          "Streaming firmware log output from this node in real time."
          "</p>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<fieldset><legend>Log Stream</legend>"
          "<div style='display:flex;gap:8px;margin-bottom:8px'>"
          "<button id='btnPause' type='button'>Pause</button>"
          "<button id='btnClear' type='button'>Clear</button>"
          "</div>"
          "<div id='logbox' style='height:360px;overflow:auto;background:#101110;color:#8df186;"
          "font-family:monospace;padding:6px;white-space:pre-wrap;border:1px solid #2f3a2f'></div>"
          "</fieldset>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<script>"
          "(function(){"
          "var since=0;var paused=false;"
          "function el(id){return document.getElementById(id);}"
          "function addLine(s){var t=el('logbox');if(!t)return;"
          "var d=document.createElement('div');d.textContent=s;t.appendChild(d);"
          "while(t.childNodes.length>800){t.removeChild(t.firstChild);}"
          "t.scrollTop=t.scrollHeight;}"
          "el('btnPause').onclick=function(){paused=!paused;this.textContent=paused?'Resume':'Pause';};"
          "el('btnClear').onclick=function(){var t=el('logbox');if(t){t.textContent='';}};"
          "function poll(){"
          "if(paused){setTimeout(poll,400);return;}"
          "fetch('/logstream?since='+since,{method:'GET'})"
          ".then(function(r){return r.text();})"
          ".then(function(txt){"
          "if(!txt){setTimeout(poll,350);return;}"
          "var lines=txt.split('\\n');"
          "for(var i=0;i<lines.length;i++){"
          "if(lines[i].indexOf('NEXT ')==0){since=parseInt(lines[i].substring(5),10)||since;}"
          "else if(lines[i].length){addLine(lines[i]);}"
          "}"
          "setTimeout(poll,120);"
          "})"
          ".catch(function(e){addLine('logstream error: '+e);setTimeout(poll,700);});"
          "}"
          "poll();"
          "})();"
          "</script>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, WEBPAGE_END_TEMPLATE, web_ui_version(appDescr), g_persistent.nodeName);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
  httpd_resp_send_chunk(req, NULL, 0);

  free(buf);
  return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////
// logstream_get_handler
//

static esp_err_t
logstream_get_handler(httpd_req_t *req)
{
  char qbuf[64] = { 0 };
  char sval[20] = { 0 };
  uint32_t since = 0;

  if ((httpd_req_get_url_query_len(req) > 0) &&
      (ESP_OK == httpd_req_get_url_query_str(req, qbuf, sizeof(qbuf))) &&
      (ESP_OK == httpd_query_key_value(qbuf, "since", sval, sizeof(sval)))) {
    since = (uint32_t) strtoul(sval, NULL, 10);
  }

  httpd_resp_set_type(req, "text/plain; charset=utf-8");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  httpd_resp_set_hdr(req, "Pragma", "no-cache");

  if (NULL == g_web_log_mutex) {
    httpd_resp_sendstr(req, "NEXT 0\n");
    return ESP_OK;
  }

  if (pdTRUE != xSemaphoreTake(g_web_log_mutex, pdMS_TO_TICKS(50))) {
    httpd_resp_sendstr(req, "NEXT 0\n");
    return ESP_OK;
  }

  uint32_t cur = g_web_log_seq;
  uint32_t first_available = (cur > WEB_LOG_RING_SIZE) ? (cur - WEB_LOG_RING_SIZE + 1U) : 1U;
  uint32_t start = since + 1U;
  if (start < first_available) {
    start = first_available;
  }

  char hdr[32] = { 0 };
  snprintf(hdr, sizeof(hdr), "NEXT %lu\n", (unsigned long) cur);
  httpd_resp_send_chunk(req, hdr, HTTPD_RESP_USE_STRLEN);

  for (uint32_t seq = start; seq <= cur; ++seq) {
    size_t idx = (size_t) ((seq - 1U) % WEB_LOG_RING_SIZE);
    if (g_web_log_ring[idx].seq != seq) {
      continue;
    }

    if ('\0' == g_web_log_ring[idx].line[0]) {
      continue;
    }

    httpd_resp_send_chunk(req, g_web_log_ring[idx].line, HTTPD_RESP_USE_STRLEN);
    httpd_resp_send_chunk(req, "\n", 1);
  }

  xSemaphoreGive(g_web_log_mutex);

  httpd_resp_send_chunk(req, NULL, 0);
  return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////
// canapi helpers
//

static bool
canapi_is_digit_string(const char *s)
{
  if ((NULL == s) || ('\0' == *s)) {
    return false;
  }

  for (const char *p = s; *p; ++p) {
    if ((*p < '0') || (*p > '9')) {
      return false;
    }
  }

  return true;
}

static int
canapi_parse_byte_token(const char *tok, size_t tok_len, uint8_t *val)
{
  char tmp[24] = { 0 };

  if ((NULL == tok) || (NULL == val) || (0 == tok_len) || (tok_len >= sizeof(tmp))) {
    return VSCP_ERROR_INVALID_SYNTAX;
  }

  memcpy(tmp, tok, tok_len);
  tmp[tok_len] = '\0';

  // Accept common byte formats: decimal (42), hex (0x2A), binary (0b101010).
  if ((tok_len > 2) && ('0' == tmp[0]) && (('b' == tmp[1]) || ('B' == tmp[1]))) {
    int v = 0;
    char *p = tmp + 2;
    if ('\0' == *p) {
      return VSCP_ERROR_INVALID_SYNTAX;
    }
    while ('\0' != *p) {
      if (('0' != *p) && ('1' != *p)) {
        return VSCP_ERROR_INVALID_SYNTAX;
      }
      v = (v << 1) + (*p - '0');
      if (v > 255) {
        return VSCP_ERROR_INVALID_SYNTAX;
      }
      p++;
    }
    *val = (uint8_t) v;
    return VSCP_ERROR_SUCCESS;
  }

  char *p = tmp;
  int base = 10;
  if ((tok_len > 2) && ('0' == tmp[0]) && (('x' == tmp[1]) || ('X' == tmp[1]))) {
    p = tmp + 2;
    base = 16;
  }
  else if ((tok_len > 2) && ('0' == tmp[0]) && (('d' == tmp[1]) || ('D' == tmp[1]))) {
    p = tmp + 2;
    base = 10;
  }

  if ('\0' == *p) {
    return VSCP_ERROR_INVALID_SYNTAX;
  }

  char *endptr = NULL;
  long v = strtol(p, &endptr, base);
  if ((NULL == endptr) || ('\0' != *endptr) || (v < 0) || (v > 255)) {
    return VSCP_ERROR_INVALID_SYNTAX;
  }

  *val = (uint8_t) v;
  return VSCP_ERROR_SUCCESS;
}

static int
canapi_parse_data_list(const char *src, uint8_t *dst, uint8_t *len)
{
  size_t n = 0;
  const char *p = src;

  if ((NULL == src) || (NULL == dst) || (NULL == len)) {
    return VSCP_ERROR_INVALID_POINTER;
  }

  *len = 0;

  while (*p) {
    while (*p && ((' ' == *p) || (',' == *p) || (';' == *p) || ('\t' == *p) || ('\r' == *p) || ('\n' == *p))) {
      p++;
    }

    if (!*p) {
      break;
    }

    const char *tok = p;
    while (*p && !(' ' == *p || ',' == *p || ';' == *p || '\t' == *p || '\r' == *p || '\n' == *p)) {
      p++;
    }
    size_t tok_len = (size_t) (p - tok);

    if (n >= 8) {
      return VSCP_ERROR_INVALID_SYNTAX;
    }

    if (VSCP_ERROR_SUCCESS != canapi_parse_byte_token(tok, tok_len, &dst[n])) {
      return VSCP_ERROR_INVALID_SYNTAX;
    }
    n++;
  }

  *len = (uint8_t) n;
  return VSCP_ERROR_SUCCESS;
}

static esp_err_t
canapi_send_vscp_as_can(uint8_t prio,
                        uint16_t vscp_class,
                        uint8_t vscp_type,
                        uint8_t nickname,
                        const uint8_t *pdata,
                        uint8_t sizeData)
{
  can4vscp_frame_t msg = { 0 };
  msg.identifier        = (uint32_t) nickname + (((uint32_t) vscp_type) << 8) + (((uint32_t) vscp_class) << 16) +
                   ((((uint32_t) prio) & 0x07u) << 26);
  msg.extd             = 1;
  msg.rtr              = 0;
  msg.data_length_code = sizeData;
  if ((sizeData > 0) && (NULL != pdata)) {
    memcpy(msg.data, pdata, sizeData);
  }
  return can4vscp_send(&msg, pdMS_TO_TICKS(100));
}

static void
canapi_decode_can4vscp_identifier(uint32_t identifier, uint8_t *prio, uint16_t *vscp_class, uint8_t *vscp_type, uint8_t *nickname)
{
  if (NULL != prio) {
    *prio = (uint8_t) ((identifier >> 26) & 0x07u);
  }

  if (NULL != vscp_class) {
    *vscp_class = (uint16_t) ((identifier >> 16) & 0x01ffu);
  }

  if (NULL != vscp_type) {
    *vscp_type = (uint8_t) ((identifier >> 8) & 0xffu);
  }

  if (NULL != nickname) {
    *nickname = (uint8_t) (identifier & 0xffu);
  }
}

static void
canapi_reset_reply_queue(void)
{
  if (NULL == tr_canapi.fromcan_queue) {
    return;
  }

  can4vscp_frame_t rxmsg = { 0 };
  while (pdPASS == xQueueReceive(tr_canapi.fromcan_queue, &rxmsg, 0)) {
    ;
  }
}

static size_t
canapi_append_nick_list(char *dst, size_t dst_len, const bool *seen)
{
  size_t used = 0;

  if ((NULL == dst) || (0 == dst_len) || (NULL == seen)) {
    return 0;
  }

  for (uint16_t nick = 0; nick < 256; ++nick) {
    if (!seen[nick]) {
      continue;
    }

    int n;
    if (used > 0) {
      n = snprintf(dst + used, dst_len - used, ",%u", (unsigned int) nick);
    }
    else {
      n = snprintf(dst + used, dst_len - used, "%u", (unsigned int) nick);
    }

    if ((n <= 0) || ((size_t) n >= (dst_len - used))) {
      break;
    }

    used += (size_t) n;
  }

  return used;
}

///////////////////////////////////////////////////////////////////////////////
// canapi_get_handler
//
// Direct HTTP API for CAN operations
//

static esp_err_t
canapi_get_handler(httpd_req_t *req)
{
  char qbuf[256]  = { 0 };
  char cmd[16]    = { 0 };
  char param[128] = { 0 };

  httpd_resp_set_type(req, "text/plain");

  size_t qlen = httpd_req_get_url_query_len(req) + 1;
  if (qlen <= 1 || qlen > sizeof(qbuf)) {
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_sendstr(req, "ERR: missing query");
    return ESP_OK;
  }

  if (ESP_OK != httpd_req_get_url_query_str(req, qbuf, sizeof(qbuf))) {
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_sendstr(req, "ERR: invalid query");
    return ESP_OK;
  }

  if (ESP_OK != httpd_query_key_value(qbuf, "cmd", cmd, sizeof(cmd))) {
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_sendstr(req, "ERR: missing cmd");
    return ESP_OK;
  }

  for (char *p = cmd; *p; ++p) {
    *p = (char) tolower((unsigned char) *p);
  }

  if (0 == strcmp(cmd, "send")) {
    uint8_t prio;
    uint8_t vtype;
    uint8_t nick;
    uint16_t vclass;
    uint8_t frame_data[8] = { 0 };
    uint8_t frame_data_len = 0;
    char *decoded_data = NULL;

    if ((ESP_OK != httpd_query_key_value(qbuf, "prio", param, sizeof(param))) || !canapi_is_digit_string(param)) {
      httpd_resp_set_status(req, "400 Bad Request");
      httpd_resp_sendstr(req, "ERR: invalid prio");
      return ESP_OK;
    }
    long l = strtol(param, NULL, 10);
    if ((l < 0) || (l > 7)) {
      httpd_resp_set_status(req, "400 Bad Request");
      httpd_resp_sendstr(req, "ERR: prio out of range");
      return ESP_OK;
    }
    prio = (uint8_t) l;

    if ((ESP_OK != httpd_query_key_value(qbuf, "class", param, sizeof(param))) || !canapi_is_digit_string(param)) {
      httpd_resp_set_status(req, "400 Bad Request");
      httpd_resp_sendstr(req, "ERR: invalid class");
      return ESP_OK;
    }
    l = strtol(param, NULL, 10);
    if ((l < 0) || (l > 511)) {
      httpd_resp_set_status(req, "400 Bad Request");
      httpd_resp_sendstr(req, "ERR: class out of range");
      return ESP_OK;
    }
    vclass = (uint16_t) l;

    if ((ESP_OK != httpd_query_key_value(qbuf, "type", param, sizeof(param))) || !canapi_is_digit_string(param)) {
      httpd_resp_set_status(req, "400 Bad Request");
      httpd_resp_sendstr(req, "ERR: invalid type");
      return ESP_OK;
    }
    l = strtol(param, NULL, 10);
    if ((l < 0) || (l > 255)) {
      httpd_resp_set_status(req, "400 Bad Request");
      httpd_resp_sendstr(req, "ERR: type out of range");
      return ESP_OK;
    }
    vtype = (uint8_t) l;

    if ((ESP_OK != httpd_query_key_value(qbuf, "nick", param, sizeof(param))) || !canapi_is_digit_string(param)) {
      httpd_resp_set_status(req, "400 Bad Request");
      httpd_resp_sendstr(req, "ERR: invalid nick");
      return ESP_OK;
    }
    l = strtol(param, NULL, 10);
    if ((l < 0) || (l > 255)) {
      httpd_resp_set_status(req, "400 Bad Request");
      httpd_resp_sendstr(req, "ERR: nick out of range");
      return ESP_OK;
    }
    nick = (uint8_t) l;

    if (ESP_OK == httpd_query_key_value(qbuf, "data", param, sizeof(param))) {
      decoded_data = urlDecode(param);
      const char *datastr = (NULL != decoded_data) ? decoded_data : param;
      if (VSCP_ERROR_SUCCESS != canapi_parse_data_list(datastr, frame_data, &frame_data_len)) {
        if (NULL != decoded_data) {
          free(decoded_data);
        }
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_sendstr(req, "ERR: invalid data");
        return ESP_OK;
      }
    }

    // Push frame to TX queue — twai_transmit_task will call can4vscp_send.
    // This returns immediately without blocking the HTTP task on CAN bus TX.
    can4vscp_frame_t tx_frame = { 0 };
    tx_frame.identifier       = (uint32_t) nick +
                                ((uint32_t) vtype << 8) +
                                ((uint32_t) vclass << 16) +
                                (((uint32_t) prio & 0x07u) << 26);
    tx_frame.extd             = 1;
    tx_frame.rtr              = 0;
    tx_frame.data_length_code = frame_data_len;
    if (frame_data_len > 0) {
      memcpy(tx_frame.data, frame_data, frame_data_len);
    }

    if ((NULL == tr_canapi.tocan_queue) ||
      (pdPASS != xQueueSendToBack(tr_canapi.tocan_queue, &tx_frame, pdMS_TO_TICKS(10)))) {
      if (NULL != decoded_data) {
        free(decoded_data);
      }
      httpd_resp_set_status(req, "500 Internal Server Error");
      httpd_resp_sendstr(req, "ERR: CAN TX queue full");
      return ESP_OK;
    }

    if (NULL != decoded_data) {
      free(decoded_data);
    }

    char rsp[64] = { 0 };
    snprintf(rsp, sizeof(rsp), "OK: SENDEVENT dlc=%u", (unsigned int) frame_data_len);
    httpd_resp_sendstr(req, rsp);
    return ESP_OK;
  }
  else if (0 == strcmp(cmd, "whis")) {
    uint8_t nick = 255;
    bool seen[256] = { 0 };
    uint16_t replies = 0;

    if (ESP_OK == httpd_query_key_value(qbuf, "nick", param, sizeof(param))) {
      if (!canapi_is_digit_string(param)) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_sendstr(req, "ERR: invalid nick");
        return ESP_OK;
      }
      long l = strtol(param, NULL, 10);
      if ((l < 0) || (l > 255)) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_sendstr(req, "ERR: nick out of range");
        return ESP_OK;
      }
      nick = (uint8_t) l;
    }

    if (NULL == tr_canapi.fromcan_queue) {
      httpd_resp_set_status(req, "500 Internal Server Error");
      httpd_resp_sendstr(req, "ERR: CAN reply queue unavailable");
      return ESP_OK;
    }

    canapi_reset_reply_queue();

    uint8_t data[1] = { nick };
    if (ESP_OK != canapi_send_vscp_as_can(0,
                                          VSCP_CLASS1_PROTOCOL,
                                          VSCP_TYPE_PROTOCOL_WHO_IS_THERE,
                                          0,
                                          data,
                                          sizeof(data))) {
      httpd_resp_set_status(req, "500 Internal Server Error");
      httpd_resp_sendstr(req, "ERR: Who Is There send failed");
      return ESP_OK;
    }

    int64_t deadline = esp_timer_get_time() + (int64_t) (1500 * 1000);
    while (esp_timer_get_time() < deadline) {
      can4vscp_frame_t rxmsg = { 0 };
      if (pdPASS != xQueueReceive(tr_canapi.fromcan_queue, &rxmsg, pdMS_TO_TICKS(50))) {
        continue;
      }

      if (!rxmsg.extd) {
        continue;
      }

      uint16_t rx_class = 0;
      uint8_t rx_type = 0;
      uint8_t src_nick = 0;
      canapi_decode_can4vscp_identifier(rxmsg.identifier, NULL, &rx_class, &rx_type, &src_nick);

      if ((VSCP_CLASS1_PROTOCOL == rx_class) && (VSCP_TYPE_PROTOCOL_WHO_IS_THERE_RESPONSE == rx_type)) {
        if (!seen[src_nick]) {
          seen[src_nick] = true;
          replies++;
        }
      }
    }

    char node_list[700] = { 0 };
    canapi_append_nick_list(node_list, sizeof(node_list), seen);

    char rsp[900] = { 0 };
    snprintf(rsp,
             sizeof(rsp),
             "OK: Who Is There target=%u replies=%u nodes=%s",
             (unsigned int) nick,
             (unsigned int) replies,
             (0 == node_list[0]) ? "-" : node_list);
    httpd_resp_sendstr(req, rsp);
    return ESP_OK;
  }
  else if (0 == strcmp(cmd, "scan")) {
    uint8_t start_nick = 1;
    uint8_t end_nick   = 255;
    uint16_t pernode_ms = 20;
    bool serial_mode = true;
    bool seen[256] = { 0 };

    if (ESP_OK == httpd_query_key_value(qbuf, "start", param, sizeof(param))) {
      if (!canapi_is_digit_string(param)) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_sendstr(req, "ERR: invalid start");
        return ESP_OK;
      }
      long l = strtol(param, NULL, 10);
      if ((l < 0) || (l > 255)) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_sendstr(req, "ERR: start out of range");
        return ESP_OK;
      }
      start_nick = (uint8_t) l;
    }

    if (ESP_OK == httpd_query_key_value(qbuf, "end", param, sizeof(param))) {
      if (!canapi_is_digit_string(param)) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_sendstr(req, "ERR: invalid end");
        return ESP_OK;
      }
      long l = strtol(param, NULL, 10);
      if ((l < 0) || (l > 255)) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_sendstr(req, "ERR: end out of range");
        return ESP_OK;
      }
      end_nick = (uint8_t) l;
    }

    if (start_nick > end_nick) {
      httpd_resp_set_status(req, "400 Bad Request");
      httpd_resp_sendstr(req, "ERR: invalid range");
      return ESP_OK;
    }

    if (ESP_OK == httpd_query_key_value(qbuf, "mode", param, sizeof(param))) {
      for (char *p = param; *p; ++p) {
        *p = (char) tolower((unsigned char) *p);
      }

      if (0 == strcmp(param, "serial")) {
        serial_mode = true;
      }
      else if (0 == strcmp(param, "burst")) {
        serial_mode = false;
      }
      else {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_sendstr(req, "ERR: invalid mode");
        return ESP_OK;
      }
    }

    if (ESP_OK == httpd_query_key_value(qbuf, "pernode_ms", param, sizeof(param))) {
      if (!canapi_is_digit_string(param)) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_sendstr(req, "ERR: invalid pernode_ms");
        return ESP_OK;
      }

      long l = strtol(param, NULL, 10);
      if ((l < 1) || (l > 2000)) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_sendstr(req, "ERR: pernode_ms out of range");
        return ESP_OK;
      }
      pernode_ms = (uint16_t) l;
    }

    if (NULL == tr_canapi.fromcan_queue) {
      httpd_resp_set_status(req, "500 Internal Server Error");
      httpd_resp_sendstr(req, "ERR: CAN reply queue unavailable");
      return ESP_OK;
    }

    canapi_reset_reply_queue();

    ESP_LOGI(TAG, "SCAN: Started (serial_mode=%u, pernode_ms=%u, range=%u-%u)",
             serial_mode, pernode_ms, start_nick, end_nick);

    uint16_t sent = 0;
    uint16_t fail = 0;
    uint16_t replies = 0;

    for (uint16_t nick = start_nick; nick <= end_nick; ++nick) {
      uint8_t data[2] = { (uint8_t) nick, VSCP_STD_REGISTER_GUID };
      if (ESP_OK == canapi_send_vscp_as_can(0,
                                            VSCP_CLASS1_PROTOCOL,
                                            VSCP_TYPE_PROTOCOL_READ_REGISTER,
                                            0,
                                            data,
                                            sizeof(data))) {
        sent++;
        ESP_LOGD(TAG, "SCAN: Sent READ_REGISTER 0x%02X to node %u", VSCP_STD_REGISTER_GUID, nick);
      }
      else {
        fail++;
        ESP_LOGW(TAG, "SCAN: Failed to send READ_REGISTER to node %u", nick);
        continue;
      }

      if (!serial_mode) {
        vTaskDelay(pdMS_TO_TICKS(2));
        continue;
      }

      int64_t pernode_deadline = esp_timer_get_time() + ((int64_t) pernode_ms * 1000);
      while (esp_timer_get_time() < pernode_deadline) {
        can4vscp_frame_t rxmsg = { 0 };
        if (pdPASS != xQueueReceive(tr_canapi.fromcan_queue, &rxmsg, pdMS_TO_TICKS(5))) {
          continue;
        }

        ESP_LOGI(TAG, "SCAN serial mode: Received frame id=0x%lX, extd=%u, dlc=%u, data[0]=0x%02X, data[1]=0x%02X",
                 rxmsg.identifier, rxmsg.extd, rxmsg.data_length_code, rxmsg.data[0], rxmsg.data[1]);

        if (!rxmsg.extd) {
          ESP_LOGD(TAG, "SCAN serial mode: Skipping non-extended frame");
          continue;
        }

        uint16_t rx_class = 0;
        uint8_t rx_type = 0;
        uint8_t src_nick = 0;
        canapi_decode_can4vscp_identifier(rxmsg.identifier, NULL, &rx_class, &rx_type, &src_nick);

        ESP_LOGI(TAG, "SCAN serial mode: Decoded - src_nick=%u, type=%u, class=%u, target_nick=%u",
                 src_nick, rx_type, rx_class, (unsigned int) nick);

        if ((VSCP_CLASS1_PROTOCOL == rx_class) &&
            (VSCP_TYPE_PROTOCOL_RW_RESPONSE == rx_type) &&
            (rxmsg.data_length_code >= 2) &&
            (VSCP_STD_REGISTER_GUID == rxmsg.data[0]) &&
            (src_nick >= start_nick) &&
            (src_nick <= end_nick)) {
          ESP_LOGI(TAG, "SCAN serial mode: Found node %u", src_nick);
          if (!seen[src_nick]) {
            seen[src_nick] = true;
            replies++;
          }

          if (src_nick == (uint8_t) nick) {
            break;
          }
        } else {
          ESP_LOGW(TAG, "SCAN serial mode: Frame didn't match criteria (class=%u!=0, type=%u!=10, dlc=%u, data[0]=0x%02X!=0x%02X, src_nick=%u in [%u,%u])",
                   rx_class, rx_type, rxmsg.data_length_code, rxmsg.data[0], VSCP_STD_REGISTER_GUID, src_nick, start_nick, end_nick);
        }
      }
    }

    if (!serial_mode) {
      int64_t deadline = esp_timer_get_time() + (int64_t) (2000 * 1000);
      while (esp_timer_get_time() < deadline) {
        can4vscp_frame_t rxmsg = { 0 };
        if (pdPASS != xQueueReceive(tr_canapi.fromcan_queue, &rxmsg, pdMS_TO_TICKS(50))) {
          continue;
        }

        ESP_LOGI(TAG, "SCAN burst mode: Received frame id=0x%lX, extd=%u, dlc=%u, data[0]=0x%02X",
                 rxmsg.identifier, rxmsg.extd, rxmsg.data_length_code, rxmsg.data[0]);

        if (!rxmsg.extd) {
          ESP_LOGD(TAG, "SCAN burst mode: Skipping non-extended frame");
          continue;
        }

        uint16_t rx_class = 0;
        uint8_t rx_type = 0;
        uint8_t src_nick = 0;
        canapi_decode_can4vscp_identifier(rxmsg.identifier, NULL, &rx_class, &rx_type, &src_nick);

        ESP_LOGI(TAG, "SCAN burst mode: Decoded - src_nick=%u, type=%u, class=%u",
                 src_nick, rx_type, rx_class);

        if ((VSCP_CLASS1_PROTOCOL == rx_class) &&
            (VSCP_TYPE_PROTOCOL_RW_RESPONSE == rx_type) &&
            (rxmsg.data_length_code >= 2) &&
            (VSCP_STD_REGISTER_GUID == rxmsg.data[0]) &&
            (src_nick >= start_nick) &&
            (src_nick <= end_nick)) {
          ESP_LOGI(TAG, "SCAN burst mode: Found node %u", src_nick);
          if (!seen[src_nick]) {
            seen[src_nick] = true;
            replies++;
          }
        }
      }
    }

    char node_list[700] = { 0 };
    canapi_append_nick_list(node_list, sizeof(node_list), seen);

    char rsp[950] = { 0 };
    snprintf(rsp,
             sizeof(rsp),
             "OK: SCAN mode=%s pernode_ms=%u reg=0x%02X range=%u-%u sent=%u failed=%u replies=%u nodes=%s",
             serial_mode ? "serial" : "burst",
             (unsigned int) pernode_ms,
             (unsigned int) VSCP_STD_REGISTER_GUID,
             (unsigned int) start_nick,
             (unsigned int) end_nick,
             (unsigned int) sent,
             (unsigned int) fail,
             (unsigned int) replies,
             (0 == node_list[0]) ? "-" : node_list);
    httpd_resp_sendstr(req, rsp);
    return ESP_OK;
  }
  else if (0 == strcmp(cmd, "monitor")) {
    // Return raw TWAI frames from the input queue with optional filtering
    // Parse filter parameters from query string — reuse the already-populated qbuf.
    
    // Default filter values (no filtering)
    uint32_t id_min = 0;
    uint32_t id_max = 0xFFFFFFFFu;
    uint16_t filter_class = 0;  // 0 = any
    uint8_t filter_type = 0;    // 0 = any
    uint8_t filter_nick = 0;    // 0 = any
    bool filter_extd_only = false;
    
    // Parse individual parameters
    char param_val[32] = { 0 };
    
    // Parse id_min (hex)
    if (httpd_query_key_value(qbuf, "id_min", param_val, sizeof(param_val)) == ESP_OK) {
      id_min = strtoul(param_val, NULL, 16);
    }
    
    // Parse id_max (hex)
    memset(param_val, 0, sizeof(param_val));
    if (httpd_query_key_value(qbuf, "id_max", param_val, sizeof(param_val)) == ESP_OK) {
      id_max = strtoul(param_val, NULL, 16);
    }
    
    // Parse class (decimal)
    memset(param_val, 0, sizeof(param_val));
    if (httpd_query_key_value(qbuf, "class", param_val, sizeof(param_val)) == ESP_OK) {
      filter_class = (uint16_t) strtoul(param_val, NULL, 10);
    }
    
    // Parse type (decimal)
    memset(param_val, 0, sizeof(param_val));
    if (httpd_query_key_value(qbuf, "type", param_val, sizeof(param_val)) == ESP_OK) {
      filter_type = (uint8_t) strtoul(param_val, NULL, 10);
    }
    
    // Parse nickname (decimal)
    memset(param_val, 0, sizeof(param_val));
    if (httpd_query_key_value(qbuf, "nick", param_val, sizeof(param_val)) == ESP_OK) {
      filter_nick = (uint8_t) strtoul(param_val, NULL, 10);
    }
    
    // Parse extd_only (presence = true)
    memset(param_val, 0, sizeof(param_val));
    if (httpd_query_key_value(qbuf, "extd_only", param_val, sizeof(param_val)) == ESP_OK) {
      filter_extd_only = true;
    }
    
    // Heap-allocate to avoid overflowing the httpd task stack.
    // NOTE: use a named constant — sizeof(buf) on a pointer gives pointer size (4),
    //       not the allocation size, which was the original cause of heap corruption.
    #define MONITOR_BUFSIZE 2048
    char *buf = (char *) calloc(MONITOR_BUFSIZE, 1);
    if (NULL == buf) {
      httpd_resp_set_status(req, "500 Internal Server Error");
      httpd_resp_sendstr(req, "ERR: out of memory");
      return ESP_OK;
    }
    size_t offset = 0;
    can4vscp_frame_t rxmsg = { 0 };

    // Drain frames from the dedicated monitor queue (filled by twai_receive_task)
    while ((offset < MONITOR_BUFSIZE - 256) && (NULL != tr_monitor.fromcan_queue) &&
           (pdPASS == xQueueReceive(tr_monitor.fromcan_queue, &rxmsg, 0))) {
      
      // Decode identifier
      uint8_t prio = (uint8_t) ((rxmsg.identifier >> 26) & 0x07u);
      uint16_t vscp_class = (uint16_t) ((rxmsg.identifier >> 16) & 0x01ffu);
      uint8_t vscp_type = (uint8_t) ((rxmsg.identifier >> 8) & 0xffu);
      uint8_t nickname = (uint8_t) (rxmsg.identifier & 0xffu);
      
      // Apply filters
      if (rxmsg.identifier < id_min || rxmsg.identifier > id_max) {
        continue;  // Outside CAN ID range
      }
      if (filter_extd_only && !rxmsg.extd) {
        continue;  // Not extended frame but extended-only filter active
      }
      if (filter_class != 0 && vscp_class != filter_class) {
        continue;  // Class mismatch
      }
      if (filter_type != 0 && vscp_type != filter_type) {
        continue;  // Type mismatch
      }
      if (filter_nick != 0 && nickname != filter_nick) {
        continue;  // Nickname mismatch
      }
      
      // Frame passes all filters - add to output
      int line_len = snprintf(buf + offset, MONITOR_BUFSIZE - offset,
                              "id=0x%lX prio=%u class=%u type=%u nick=%u extd=%u dlc=%u data=",
                              rxmsg.identifier, prio, vscp_class, vscp_type, nickname,
                              rxmsg.extd, rxmsg.data_length_code);
      if (line_len < 0) break;
      offset += (size_t) line_len;

      // Append data bytes
      for (int i = 0; i < rxmsg.data_length_code && i < 8; i++) {
        line_len = snprintf(buf + offset, MONITOR_BUFSIZE - offset, "%02X", rxmsg.data[i]);
        if (line_len < 0) break;
        offset += (size_t) line_len;
        if (i < rxmsg.data_length_code - 1) {
          buf[offset++] = ':';
        }
      }
      buf[offset++] = '\n';
      #undef MONITOR_BUFSIZE
    }

    if (offset > 0) {
      httpd_resp_send(req, buf, offset);
    } else {
      httpd_resp_sendstr(req, "");
    }
    free(buf);
    return ESP_OK;
  }

  httpd_resp_set_status(req, "400 Bad Request");
  httpd_resp_sendstr(req, "ERR: unknown cmd");
  return ESP_OK;
}

// static const httpd_uri_t mainpg = { .uri     = "/index.html",
//                                    .method  = HTTP_GET,
//                                    .handler = mainpg_get_handler,
//                                    .user_ctx = NULL };

///////////////////////////////////////////////////////////////////////////////
// config_get_handler
//

static esp_err_t
config_get_handler(httpd_req_t *req)
{
  // esp_err_t rv;
  char *buf;
  // char *temp;

  char *req_buf;
  size_t req_buf_len;

  buf = (char *) calloc(CHUNK_BUFSIZE, 1);
  if (NULL == buf) {
    return ESP_ERR_NO_MEM;
  }

  // Get application info data
  const esp_app_desc_t *appDescr = esp_app_get_description();

  // Get header value string length and allocate memory for length + 1,
  // extra byte for null termination
  req_buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
  if (req_buf_len > 1) {
    req_buf = (char *) malloc(req_buf_len);
    // Copy null terminated value string into buffer
    if (httpd_req_get_hdr_value_str(req, "Host", req_buf, req_buf_len) == ESP_OK) {
      ESP_LOGD(TAG, "Found header => Host: %s", req_buf);
    }
    free(req_buf);
  }

  sprintf(buf, WEBPAGE_START_TEMPLATE, g_persistent.nodeName, "Configuration");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<p><form id=but1 class=\"button\" action='cfgmodule' method='get'><button>Module</button></form></p>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<p><form id=but2 class=\"button\" action='cfgwifi' method='get'><button>WiFi</button></form></p>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<p><form id=but3 class=\"button\" action='cfgcan' method='get'><button>CAN/TWAI</button></form></p>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<p><form id=but3 class=\"button\" action='cfgweb' method='get'><button>Web interface</button></form></p>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(
    buf,
    "<p><form id=but3 class=\"button\" action='cfgvscplink' method='get'><button>VSCP Link Srv</button></form></p>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<p><form id=but3 class=\"button\" action='cfgmqtt' method='get'><button>MQTT</button></form></p>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<p><form id=but3 class=\"button\" action='cfgmulticast' method='get'><button>Multicast</button></form></p>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<p><form id=but3 class=\"button\" action='cfgudp' method='get'><button>UDP</button></form></p>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(
    buf,
    "<p><form id=but3 class=\"button\" action='cfgwebsockets' method='get'><button>Websockets</button></form></p>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<p><form id=but3 class=\"button\" action='cfglog' method='get'><button>Logging</button></form></p>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(
    buf,
    "<hr /><p><form id=but4 class=\"button\" action='cfgfactorydefaults' method='get'><button name='rst' class='button "
    "bred'>Set factory Defaults</button></form></p>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
  sprintf(buf,
          "<p><form id=but3 class=\"button\" action='cfgbackup' method='get'><button class='button "
          "bgrn'>Backup</button></form></p>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
  sprintf(buf,
          "<p><form id=but3 class=\"button\" action='cfgrestore' method='get'><button class='button "
          "bgrn'>Restore</button></form></p>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, WEBPAGE_END_TEMPLATE, web_ui_version(appDescr), g_persistent.nodeName);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  httpd_resp_send_chunk(req, NULL, 0);

  free(buf);

  return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////
// config_backup_get_handler
//

static esp_err_t
config_backup_get_handler(httpd_req_t *req)
{
  cJSON *root = cJSON_CreateObject();
  if (NULL == root) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Unable to create backup JSON");
    return ESP_FAIL;
  }

  cJSON *persistent = cJSON_AddObjectToObject(root, "persistent");
  if (NULL == persistent) {
    cJSON_Delete(root);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Unable to create backup object");
    return ESP_FAIL;
  }

  cJSON_AddStringToObject(persistent, "nodeName", g_persistent.nodeName);
  cJSON_AddNumberToObject(persistent, "nodeZone", g_persistent.nodeZone);
  cJSON_AddNumberToObject(persistent, "nodeSubzone", g_persistent.nodeSubzone);
  backup_add_hex_field(persistent, "guid", g_persistent.guid, sizeof(g_persistent.guid));
  cJSON_AddNumberToObject(persistent, "encryptLvl", g_persistent.encryptLvl);
  backup_add_encrypted_blob_field(persistent, "pmk", g_persistent.pmk, sizeof(g_persistent.pmk));
  cJSON_AddNumberToObject(persistent, "pmkLen", g_persistent.pmkLen);
  cJSON_AddNumberToObject(persistent, "bootCnt", g_persistent.bootCnt);

  cJSON_AddNumberToObject(persistent, "canMode", g_persistent.canMode);
  cJSON_AddNumberToObject(persistent, "canSpeed", g_persistent.canSpeed);
  cJSON_AddNumberToObject(persistent, "nSent", g_persistent.nSent);
  cJSON_AddNumberToObject(persistent, "nRecv", g_persistent.nRecv);
  cJSON_AddNumberToObject(persistent, "nErr", g_persistent.nErr);
  cJSON_AddNumberToObject(persistent, "lastError", g_persistent.lastError);
  cJSON_AddNumberToObject(persistent, "canFilter", g_persistent.canFilter);
  cJSON_AddNumberToObject(persistent, "bRawCan", g_persistent.bRawCan);

  cJSON_AddNumberToObject(persistent, "logType", g_persistent.logType);
  cJSON_AddNumberToObject(persistent, "logLevel", g_persistent.logLevel);
  cJSON_AddNumberToObject(persistent, "logRetries", g_persistent.logRetries);
  cJSON_AddNumberToObject(persistent, "logPort", g_persistent.logPort);
  cJSON_AddStringToObject(persistent, "logUrl", g_persistent.logUrl);
  cJSON_AddStringToObject(persistent, "logMqttTopic", g_persistent.logMqttTopic);
  cJSON_AddNumberToObject(persistent, "enableMqttLog", g_persistent.enableMqttLog);

  cJSON_AddStringToObject(persistent, "wifiPrimarySsid", g_persistent.wifiPrimarySsid);
  backup_add_encrypted_str_field(persistent, "wifiPrimaryPassword", g_persistent.wifiPrimaryPassword);
  cJSON_AddStringToObject(persistent, "wifiSecondarySsid", g_persistent.wifiSecondarySsid);
  backup_add_encrypted_str_field(persistent, "wifiSecondaryPassword", g_persistent.wifiSecondaryPassword);
  cJSON_AddNumberToObject(persistent, "wifiStaticEnable", g_persistent.wifiStaticEnable);
  cJSON_AddStringToObject(persistent, "wifiStaticIp", g_persistent.wifiStaticIp);
  cJSON_AddStringToObject(persistent, "wifiStaticNetmask", g_persistent.wifiStaticNetmask);
  cJSON_AddStringToObject(persistent, "wifiStaticGateway", g_persistent.wifiStaticGateway);
  cJSON_AddStringToObject(persistent, "wifiStaticDns", g_persistent.wifiStaticDns);

  cJSON_AddNumberToObject(persistent, "webPort", g_persistent.webPort);
  cJSON_AddStringToObject(persistent, "webUser", g_persistent.webUser);
  backup_add_encrypted_str_field(persistent, "webPassword", g_persistent.webPassword);

  cJSON_AddNumberToObject(persistent, "enableVscpLink", g_persistent.enableVscpLink);
  cJSON_AddNumberToObject(persistent, "vscplinkPort", g_persistent.vscplinkPort);
  cJSON_AddStringToObject(persistent, "vscplinkUser", g_persistent.vscplinkUser);
  backup_add_encrypted_str_field(persistent, "vscplinkPw", g_persistent.vscplinkPw);

  cJSON_AddNumberToObject(persistent, "enableMqtt", g_persistent.enableMqtt);
  cJSON_AddNumberToObject(persistent, "enableMqttTls", g_persistent.enableMqttTls);
  cJSON_AddStringToObject(persistent, "mqttUrl", g_persistent.mqttUrl);
  cJSON_AddNumberToObject(persistent, "mqttPort", g_persistent.mqttPort);
  cJSON_AddStringToObject(persistent, "mqttUser", g_persistent.mqttUser);
  backup_add_encrypted_str_field(persistent, "mqttPw", g_persistent.mqttPw);
  cJSON_AddStringToObject(persistent, "mqttPubTopic", g_persistent.mqttPubTopic);
  cJSON_AddStringToObject(persistent, "mqttSubTopic", g_persistent.mqttSubTopic);
  cJSON_AddStringToObject(persistent, "mqttPubLogTopic", g_persistent.mqttPubLogTopic);
  cJSON_AddStringToObject(persistent, "mqttClientId", g_persistent.mqttClientId);
  backup_add_encrypted_str_field(persistent, "mqttCaCert", g_persistent.mqttCaCert);
  backup_add_encrypted_str_field(persistent, "mqttClientCert", g_persistent.mqttClientCert);
  backup_add_encrypted_str_field(persistent, "mqttClientKey", g_persistent.mqttClientKey);
  cJSON_AddNumberToObject(persistent, "mqttQos", g_persistent.mqttQos);
  cJSON_AddNumberToObject(persistent, "mqttRetain", g_persistent.mqttRetain);
  cJSON_AddNumberToObject(persistent, "mqttFormat", g_persistent.mqttFormat);

  cJSON_AddNumberToObject(persistent, "enableMulticast", g_persistent.enableMulticast);
  cJSON_AddStringToObject(persistent, "multicastUrl", g_persistent.multicastUrl);
  cJSON_AddNumberToObject(persistent, "multicastPort", g_persistent.multicastPort);
  cJSON_AddNumberToObject(persistent, "multicastTtl", g_persistent.multicastTtl);
  cJSON_AddNumberToObject(persistent, "bMcastEncrypt", g_persistent.bMcastEncrypt);
  cJSON_AddNumberToObject(persistent, "bHeartbeat", g_persistent.bHeartbeat);

  cJSON_AddNumberToObject(persistent, "enableUdpRx", g_persistent.enableUdpRx);
  cJSON_AddNumberToObject(persistent, "enableUdpTx", g_persistent.enableUdpTx);
  cJSON_AddStringToObject(persistent, "udpUrl", g_persistent.udpUrl);
  cJSON_AddNumberToObject(persistent, "udpPort", g_persistent.udpPort);
  cJSON_AddNumberToObject(persistent, "bUdpEncrypt", g_persistent.bUdpEncrypt);

  cJSON_AddNumberToObject(persistent, "enableWebsock", g_persistent.enableWebsock);
  cJSON_AddNumberToObject(persistent, "websockPort", g_persistent.websockPort);
  cJSON_AddStringToObject(persistent, "websockUser", g_persistent.websockUser);
  backup_add_encrypted_str_field(persistent, "websockPw", g_persistent.websockPw);

  char *json = cJSON_Print(root);
  cJSON_Delete(root);

  if (NULL == json) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Unable to serialize backup JSON");
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"vscp-persistent-backup.json\"");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  esp_err_t rv = httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);

  cJSON_free(json);
  return rv;
}

///////////////////////////////////////////////////////////////////////////////
// backup_parse_hex_field
//

static bool
backup_parse_hex_field(const char *str, uint8_t *out, size_t out_len)
{
  if ((NULL == str) || (NULL == out) || (0 == out_len)) {
    return false;
  }

  size_t hex_count = 0;
  for (size_t i = 0; str[i] != '\0'; i++) {
    if (isxdigit((unsigned char) str[i])) {
      hex_count++;
    }
  }

  if (hex_count != (out_len * 2)) {
    return false;
  }

  size_t idx = 0;
  int hi     = -1;
  for (size_t i = 0; str[i] != '\0'; i++) {
    if (!isxdigit((unsigned char) str[i])) {
      continue;
    }

    int nibble = 0;
    if ((str[i] >= '0') && (str[i] <= '9')) {
      nibble = str[i] - '0';
    }
    else if ((str[i] >= 'a') && (str[i] <= 'f')) {
      nibble = 10 + (str[i] - 'a');
    }
    else {
      nibble = 10 + (str[i] - 'A');
    }

    if (hi < 0) {
      hi = nibble;
    }
    else {
      if (idx >= out_len) {
        return false;
      }
      out[idx++] = (uint8_t) ((hi << 4) | nibble);
      hi         = -1;
    }
  }

  return (idx == out_len);
}

static void backup_json_get_u8(const cJSON *obj, const char *name, uint8_t *target)
{
  const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, name);
  if ((NULL != item) && cJSON_IsNumber(item) && (NULL != target)) {
    *target = (uint8_t) item->valueint;
  }
}

static void backup_json_get_u16(const cJSON *obj, const char *name, uint16_t *target)
{
  const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, name);
  if ((NULL != item) && cJSON_IsNumber(item) && (NULL != target)) {
    *target = (uint16_t) item->valueint;
  }
}

static void backup_json_get_u32(const cJSON *obj, const char *name, uint32_t *target)
{
  const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, name);
  if ((NULL != item) && cJSON_IsNumber(item) && (NULL != target)) {
    *target = (uint32_t) item->valuedouble;
  }
}

static void backup_json_get_str(const cJSON *obj, const char *name, char *target, size_t target_len)
{
  const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, name);
  if ((NULL != item) && cJSON_IsString(item) && (NULL != item->valuestring) && (NULL != target) && (target_len > 0)) {
    strncpy(target, item->valuestring, target_len - 1);
    target[target_len - 1] = '\0';
  }
}

static void
backup_json_get_secret_str(const cJSON *obj, const char *name, char *target, size_t target_len)
{
  const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, name);
  if ((NULL == item) || !cJSON_IsString(item) || (NULL == item->valuestring) || (NULL == target) || (target_len < 2)) {
    return;
  }

  size_t written = 0;
  if (backup_decrypt_from_hex(name, item->valuestring, (uint8_t *) target, target_len - 1, &written)) {
    target[written] = '\0';
    return;
  }

  strncpy(target, item->valuestring, target_len - 1);
  target[target_len - 1] = '\0';
}

static void
backup_json_get_secret_blob(const cJSON *obj, const char *name, uint8_t *target, size_t target_len)
{
  const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, name);
  if ((NULL == item) || !cJSON_IsString(item) || (NULL == item->valuestring) || (NULL == target) || (0 == target_len)) {
    return;
  }

  size_t written = 0;
  if (backup_decrypt_from_hex(name, item->valuestring, target, target_len, &written)) {
    return;
  }

  // Backward compatibility: old backup used plain hex for PMK.
  backup_parse_hex_field(item->valuestring, target, target_len);
}

static esp_err_t save_persistent_to_nvs(void)
{
  esp_err_t rv = ESP_OK;

  rv |= nvs_set_str(g_nvsHandle, "nodeName", g_persistent.nodeName);
  rv |= nvs_set_blob(g_nvsHandle, "guid", g_persistent.guid, sizeof(g_persistent.guid));
  rv |= nvs_set_blob(g_nvsHandle, "pmk", g_persistent.pmk, sizeof(g_persistent.pmk));
  rv |= nvs_set_u8(g_nvsHandle, "encryptLvl", g_persistent.encryptLvl);
  rv |= nvs_set_u8(g_nvsHandle, "zone", g_persistent.nodeZone);
  rv |= nvs_set_u8(g_nvsHandle, "subzone", g_persistent.nodeSubzone);
  rv |= nvs_set_u8(g_nvsHandle, "nodeZone", g_persistent.nodeZone);
  rv |= nvs_set_u8(g_nvsHandle, "nodeSubZone", g_persistent.nodeSubzone);

  rv |= nvs_set_u8(g_nvsHandle, "canMode", g_persistent.canMode);
  rv |= nvs_set_u8(g_nvsHandle, "canSpeed", g_persistent.canSpeed);
  rv |= nvs_set_u32(g_nvsHandle, "canFilter", g_persistent.canFilter);
  rv |= nvs_set_u32(g_nvsHandle, "filter", g_persistent.canFilter);
  rv |= nvs_set_u8(g_nvsHandle, "canRaw", g_persistent.bRawCan);

  rv |= nvs_set_u8(g_nvsHandle, "logType", g_persistent.logType);
  rv |= nvs_set_u8(g_nvsHandle, "logRetries", g_persistent.logRetries);
  rv |= nvs_set_u8(g_nvsHandle, "logLevel", g_persistent.logLevel);
  rv |= nvs_set_u8(g_nvsHandle, "log_level", g_persistent.logLevel);
  rv |= nvs_set_u16(g_nvsHandle, "logPort", g_persistent.logPort);
  rv |= nvs_set_str(g_nvsHandle, "logUrl", g_persistent.logUrl);
  rv |= nvs_set_str(g_nvsHandle, "log_url", g_persistent.logUrl);
  rv |= nvs_set_str(g_nvsHandle, "logMqttTopic", g_persistent.logMqttTopic);
  rv |= nvs_set_u8(g_nvsHandle, "enableMqttLog", g_persistent.enableMqttLog);

  rv |= nvs_set_str(g_nvsHandle, "wifiPriSsid", g_persistent.wifiPrimarySsid);
  rv |= nvs_set_str(g_nvsHandle, "wifiPriPass", g_persistent.wifiPrimaryPassword);
  rv |= nvs_set_str(g_nvsHandle, "wifiSecSsid", g_persistent.wifiSecondarySsid);
  rv |= nvs_set_str(g_nvsHandle, "wifiSecPass", g_persistent.wifiSecondaryPassword);
  rv |= nvs_set_u8(g_nvsHandle, "wifiStaIpEn", g_persistent.wifiStaticEnable);
  rv |= nvs_set_str(g_nvsHandle, "wifiStaIp", g_persistent.wifiStaticIp);
  rv |= nvs_set_str(g_nvsHandle, "wifiStaMask", g_persistent.wifiStaticNetmask);
  rv |= nvs_set_str(g_nvsHandle, "wifiStaGw", g_persistent.wifiStaticGateway);
  rv |= nvs_set_str(g_nvsHandle, "wifiStaDns", g_persistent.wifiStaticDns);

  rv |= nvs_set_u16(g_nvsHandle, "webPort", g_persistent.webPort);
  rv |= nvs_set_str(g_nvsHandle, "webUser", g_persistent.webUser);
  rv |= nvs_set_str(g_nvsHandle, "webPassword", g_persistent.webPassword);

  rv |= nvs_set_u8(g_nvsHandle, "enableVscpLink", g_persistent.enableVscpLink);
  rv |= nvs_set_u16(g_nvsHandle, "vscplinkPort", g_persistent.vscplinkPort);
  rv |= nvs_set_str(g_nvsHandle, "vscplinkUser", g_persistent.vscplinkUser);
  rv |= nvs_set_str(g_nvsHandle, "vscplinkPw", g_persistent.vscplinkPw);

  rv |= nvs_set_u8(g_nvsHandle, "enableMqtt", g_persistent.enableMqtt);
  rv |= nvs_set_u8(g_nvsHandle, "enableMqttTls", g_persistent.enableMqttTls);
  rv |= nvs_set_str(g_nvsHandle, "mqttUrl", g_persistent.mqttUrl);
  rv |= nvs_set_u16(g_nvsHandle, "mqttPort", g_persistent.mqttPort);
  rv |= nvs_set_str(g_nvsHandle, "mqttUser", g_persistent.mqttUser);
  rv |= nvs_set_str(g_nvsHandle, "mqttPw", g_persistent.mqttPw);
  rv |= nvs_set_str(g_nvsHandle, "mqttPubTopic", g_persistent.mqttPubTopic);
  rv |= nvs_set_str(g_nvsHandle, "mqttSubTopic", g_persistent.mqttSubTopic);
  rv |= nvs_set_str(g_nvsHandle, "mqttPubLogTopic", g_persistent.mqttPubLogTopic);
  rv |= nvs_set_str(g_nvsHandle, "mqttClientId", g_persistent.mqttClientId);
  rv |= nvs_set_str(g_nvsHandle, "mqttCaCert", g_persistent.mqttCaCert);
  rv |= nvs_set_str(g_nvsHandle, "mqttClientCert", g_persistent.mqttClientCert);
  rv |= nvs_set_str(g_nvsHandle, "mqttClientKey", g_persistent.mqttClientKey);
  rv |= nvs_set_u8(g_nvsHandle, "mqttQos", g_persistent.mqttQos);
  rv |= nvs_set_u8(g_nvsHandle, "mqttRetain", g_persistent.mqttRetain);
  rv |= nvs_set_u8(g_nvsHandle, "mqttFormat", g_persistent.mqttFormat);

  rv |= nvs_set_u8(g_nvsHandle, "enableMulticast", g_persistent.enableMulticast);
  rv |= nvs_set_str(g_nvsHandle, "multicastUrl", g_persistent.multicastUrl);
  rv |= nvs_set_u16(g_nvsHandle, "multicastPort", g_persistent.multicastPort);
  rv |= nvs_set_u8(g_nvsHandle, "multicastTtl", g_persistent.multicastTtl);
  rv |= nvs_set_u8(g_nvsHandle, "bMcastEncrypt", g_persistent.bMcastEncrypt);
  rv |= nvs_set_u8(g_nvsHandle, "bMcastHbeat", g_persistent.bHeartbeat);

  rv |= nvs_set_u8(g_nvsHandle, "enableUdpRx", g_persistent.enableUdpRx);
  rv |= nvs_set_u8(g_nvsHandle, "enableUdpTx", g_persistent.enableUdpTx);
  rv |= nvs_set_str(g_nvsHandle, "udpUrl", g_persistent.udpUrl);
  rv |= nvs_set_u16(g_nvsHandle, "udpPort", g_persistent.udpPort);
  rv |= nvs_set_u8(g_nvsHandle, "bUdpEncrypt", g_persistent.bUdpEncrypt);

  rv |= nvs_set_u8(g_nvsHandle, "enableWebsock", g_persistent.enableWebsock);
  rv |= nvs_set_u16(g_nvsHandle, "websockPort", g_persistent.websockPort);
  rv |= nvs_set_str(g_nvsHandle, "websockUser", g_persistent.websockUser);
  rv |= nvs_set_str(g_nvsHandle, "websockPw", g_persistent.websockPw);

  if (ESP_OK != rv) {
    return rv;
  }

  return nvs_commit(g_nvsHandle);
}

static esp_err_t
config_restore_get_handler(httpd_req_t *req)
{
  char *buf;
  char *req_buf;
  size_t req_buf_len;

  buf = (char *) calloc(CHUNK_BUFSIZE, 1);
  if (NULL == buf) {
    return ESP_ERR_NO_MEM;
  }

  const esp_app_desc_t *appDescr = esp_app_get_description();

  req_buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
  if (req_buf_len > 1) {
    req_buf = (char *) malloc(req_buf_len);
    if (httpd_req_get_hdr_value_str(req, "Host", req_buf, req_buf_len) == ESP_OK) {
      ESP_LOGD(TAG, "Found header => Host: %s", req_buf);
    }
    free(req_buf);
  }

  sprintf(buf, WEBPAGE_START_TEMPLATE, g_persistent.nodeName, "Restore Configuration");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<div><form id=restoreForm class=\"button\" onsubmit=\"return false;\"><fieldset>"
          "<label for=\"restorefile\">Backup JSON file:</label>"
          "<input type=\"file\" id=\"restorefile\" accept=\"application/json,.json\" />"
          "<button id=\"restorebtn\" class=\"bgrn\" onclick=\"startRestore()\">Restore and Restart</button>"
          "<p id=\"restorestatus\" class=\"prop\"></p>"
          "</fieldset></form></div>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<script>"
          "function startRestore(){"
          "var f=document.getElementById('restorefile').files;"
          "var st=document.getElementById('restorestatus');"
          "if(!f||!f.length){alert('Please select a backup JSON file.');return;}"
          "document.getElementById('restorebtn').disabled=true;"
          "st.textContent='Uploading backup...';"
          "var xhr=new XMLHttpRequest();"
          "xhr.onreadystatechange=function(){"
          "if(xhr.readyState===4){"
          "if(xhr.status===200){"
          "document.open();"
          "document.write(xhr.responseText);"
          "document.close();"
          "}else{"
          "st.textContent='Restore failed: '+xhr.status;"
          "document.getElementById('restorebtn').disabled=false;"
          "}"
          "}"
          "};"
          "xhr.open('POST','/docfgrestore',true);"
          "xhr.setRequestHeader('Content-Type','application/json');"
          "xhr.send(f[0]);"
          "}"
          "</script>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, WEBPAGE_CONFIG_END_TEMPLATE, web_ui_version(appDescr), g_persistent.nodeName);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
  httpd_resp_send_chunk(req, NULL, 0);
  free(buf);

  return ESP_OK;
}

static esp_err_t
do_config_restore_post_handler(httpd_req_t *req)
{
  if ((req->content_len <= 0) || (req->content_len > 32768)) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid backup size");
    return ESP_FAIL;
  }

  char *json_buf = (char *) calloc(req->content_len + 1, 1);
  if (NULL == json_buf) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
    return ESP_FAIL;
  }

  int remaining = req->content_len;
  int offset    = 0;
  while (remaining > 0) {
    int recv_len = httpd_req_recv(req, json_buf + offset, MIN(remaining, 1024));
    if (recv_len <= 0) {
      free(json_buf);
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to receive backup data");
      return ESP_FAIL;
    }
    remaining -= recv_len;
    offset += recv_len;
  }
  json_buf[offset] = '\0';

  cJSON *root = cJSON_Parse(json_buf);
  free(json_buf);
  if (NULL == root) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON backup file");
    return ESP_FAIL;
  }

  const cJSON *persistent = cJSON_GetObjectItemCaseSensitive(root, "persistent");
  if ((NULL == persistent) || !cJSON_IsObject(persistent)) {
    persistent = root;
  }

  backup_json_get_str(persistent, "nodeName", g_persistent.nodeName, sizeof(g_persistent.nodeName));
  backup_json_get_u8(persistent, "nodeZone", &g_persistent.nodeZone);
  backup_json_get_u8(persistent, "nodeSubzone", &g_persistent.nodeSubzone);
  backup_json_get_u8(persistent, "nodeSubZone", &g_persistent.nodeSubzone);

  const cJSON *guid = cJSON_GetObjectItemCaseSensitive(persistent, "guid");
  if ((NULL != guid) && cJSON_IsString(guid) && (NULL != guid->valuestring)) {
    backup_parse_hex_field(guid->valuestring, g_persistent.guid, sizeof(g_persistent.guid));
  }

  backup_json_get_u8(persistent, "encryptLvl", &g_persistent.encryptLvl);

  backup_json_get_secret_blob(persistent, "pmk", g_persistent.pmk, sizeof(g_persistent.pmk));

  backup_json_get_u8(persistent, "pmkLen", &g_persistent.pmkLen);
  backup_json_get_u32(persistent, "bootCnt", &g_persistent.bootCnt);
  backup_json_get_u8(persistent, "canMode", &g_persistent.canMode);
  backup_json_get_u8(persistent, "canSpeed", &g_persistent.canSpeed);
  backup_json_get_u32(persistent, "canFilter", &g_persistent.canFilter);
  backup_json_get_u8(persistent, "bRawCan", &g_persistent.bRawCan);
  backup_json_get_u8(persistent, "logType", &g_persistent.logType);
  backup_json_get_u8(persistent, "logLevel", &g_persistent.logLevel);
  backup_json_get_u8(persistent, "logRetries", &g_persistent.logRetries);
  backup_json_get_u16(persistent, "logPort", &g_persistent.logPort);
  backup_json_get_str(persistent, "logUrl", g_persistent.logUrl, sizeof(g_persistent.logUrl));
  backup_json_get_str(persistent, "logMqttTopic", g_persistent.logMqttTopic, sizeof(g_persistent.logMqttTopic));
  backup_json_get_u8(persistent, "enableMqttLog", &g_persistent.enableMqttLog);
  backup_json_get_str(persistent, "wifiPrimarySsid", g_persistent.wifiPrimarySsid, sizeof(g_persistent.wifiPrimarySsid));
  backup_json_get_secret_str(
    persistent, "wifiPrimaryPassword", g_persistent.wifiPrimaryPassword, sizeof(g_persistent.wifiPrimaryPassword));
  backup_json_get_str(persistent, "wifiSecondarySsid", g_persistent.wifiSecondarySsid, sizeof(g_persistent.wifiSecondarySsid));
  backup_json_get_secret_str(persistent,
                             "wifiSecondaryPassword",
                             g_persistent.wifiSecondaryPassword,
                             sizeof(g_persistent.wifiSecondaryPassword));
  backup_json_get_u8(persistent, "wifiStaticEnable", &g_persistent.wifiStaticEnable);
  backup_json_get_str(persistent, "wifiStaticIp", g_persistent.wifiStaticIp, sizeof(g_persistent.wifiStaticIp));
  backup_json_get_str(persistent, "wifiStaticNetmask", g_persistent.wifiStaticNetmask, sizeof(g_persistent.wifiStaticNetmask));
  backup_json_get_str(persistent, "wifiStaticGateway", g_persistent.wifiStaticGateway, sizeof(g_persistent.wifiStaticGateway));
  backup_json_get_str(persistent, "wifiStaticDns", g_persistent.wifiStaticDns, sizeof(g_persistent.wifiStaticDns));
  backup_json_get_u16(persistent, "webPort", &g_persistent.webPort);
  backup_json_get_str(persistent, "webUser", g_persistent.webUser, sizeof(g_persistent.webUser));
  backup_json_get_secret_str(persistent, "webPassword", g_persistent.webPassword, sizeof(g_persistent.webPassword));
  backup_json_get_u8(persistent, "enableVscpLink", &g_persistent.enableVscpLink);
  backup_json_get_u16(persistent, "vscplinkPort", &g_persistent.vscplinkPort);
  backup_json_get_str(persistent, "vscplinkUser", g_persistent.vscplinkUser, sizeof(g_persistent.vscplinkUser));
  backup_json_get_secret_str(persistent, "vscplinkPw", g_persistent.vscplinkPw, sizeof(g_persistent.vscplinkPw));
  backup_json_get_u8(persistent, "enableMqtt", &g_persistent.enableMqtt);
  backup_json_get_u8(persistent, "enableMqttTls", &g_persistent.enableMqttTls);
  backup_json_get_str(persistent, "mqttUrl", g_persistent.mqttUrl, sizeof(g_persistent.mqttUrl));
  backup_json_get_u16(persistent, "mqttPort", &g_persistent.mqttPort);
  backup_json_get_str(persistent, "mqttUser", g_persistent.mqttUser, sizeof(g_persistent.mqttUser));
  backup_json_get_secret_str(persistent, "mqttPw", g_persistent.mqttPw, sizeof(g_persistent.mqttPw));
  backup_json_get_str(persistent, "mqttPubTopic", g_persistent.mqttPubTopic, sizeof(g_persistent.mqttPubTopic));
  backup_json_get_str(persistent, "mqttSubTopic", g_persistent.mqttSubTopic, sizeof(g_persistent.mqttSubTopic));
  backup_json_get_str(persistent, "mqttPubLogTopic", g_persistent.mqttPubLogTopic, sizeof(g_persistent.mqttPubLogTopic));
  backup_json_get_str(persistent, "mqttClientId", g_persistent.mqttClientId, sizeof(g_persistent.mqttClientId));
  backup_json_get_secret_str(persistent, "mqttCaCert", g_persistent.mqttCaCert, sizeof(g_persistent.mqttCaCert));
  backup_json_get_secret_str(
    persistent, "mqttClientCert", g_persistent.mqttClientCert, sizeof(g_persistent.mqttClientCert));
  backup_json_get_secret_str(
    persistent, "mqttClientKey", g_persistent.mqttClientKey, sizeof(g_persistent.mqttClientKey));
  backup_json_get_u8(persistent, "mqttQos", &g_persistent.mqttQos);
  backup_json_get_u8(persistent, "mqttRetain", &g_persistent.mqttRetain);
  backup_json_get_u8(persistent, "mqttFormat", &g_persistent.mqttFormat);
  backup_json_get_u8(persistent, "enableMulticast", &g_persistent.enableMulticast);
  backup_json_get_str(persistent, "multicastUrl", g_persistent.multicastUrl, sizeof(g_persistent.multicastUrl));
  backup_json_get_u16(persistent, "multicastPort", &g_persistent.multicastPort);
  backup_json_get_u8(persistent, "multicastTtl", &g_persistent.multicastTtl);
  backup_json_get_u8(persistent, "bMcastEncrypt", &g_persistent.bMcastEncrypt);
  backup_json_get_u8(persistent, "bHeartbeat", &g_persistent.bHeartbeat);
  backup_json_get_u8(persistent, "enableUdpRx", &g_persistent.enableUdpRx);
  backup_json_get_u8(persistent, "enableUdpTx", &g_persistent.enableUdpTx);
  backup_json_get_str(persistent, "udpUrl", g_persistent.udpUrl, sizeof(g_persistent.udpUrl));
  backup_json_get_u16(persistent, "udpPort", &g_persistent.udpPort);
  backup_json_get_u8(persistent, "bUdpEncrypt", &g_persistent.bUdpEncrypt);
  backup_json_get_u8(persistent, "enableWebsock", &g_persistent.enableWebsock);
  backup_json_get_u16(persistent, "websockPort", &g_persistent.websockPort);
  backup_json_get_str(persistent, "websockUser", g_persistent.websockUser, sizeof(g_persistent.websockUser));
  backup_json_get_secret_str(persistent, "websockPw", g_persistent.websockPw, sizeof(g_persistent.websockPw));

  cJSON_Delete(root);

  esp_err_t rv = save_persistent_to_nvs();
  if (ESP_OK != rv) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save restored configuration");
    return ESP_FAIL;
  }

  send_restart_info_page(req, "Restore complete. The device is restarting...");
  vTaskDelay(pdMS_TO_TICKS(800));
  esp_restart();
  return ESP_OK;
}

// static const httpd_uri_t config = { .uri     = "/config",
//                                    .method  = HTTP_GET,
//                                    .handler = config_get_handler,
//                                    .user_ctx = NULL };

///////////////////////////////////////////////////////////////////////////////
// config_can_get_handler
//

static esp_err_t
config_can_get_handler(httpd_req_t *req)
{
  char *buf;
  char *req_buf;
  size_t req_buf_len;

  buf = (char *) calloc(CHUNK_BUFSIZE, 1);
  if (NULL == buf) {
    return ESP_ERR_NO_MEM;
  }

  // Get application info data
  const esp_app_desc_t *appDescr = esp_app_get_description();

  // Get header value string length and allocate memory for length + 1,
  // extra byte for null termination
  req_buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
  if (req_buf_len > 1) {
    req_buf = (char *) malloc(req_buf_len);
    // Copy null terminated value string into buffer
    if (httpd_req_get_hdr_value_str(req, "Host", req_buf, req_buf_len) == ESP_OK) {
      ESP_LOGD(TAG, "Found header => Host: %s", req_buf);
    }
    free(req_buf);
  }

  sprintf(buf, WEBPAGE_START_TEMPLATE, g_persistent.nodeName, "CAN/TWAI Configuration");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<div><form id=but3 class=\"button\" action='/docfgcan' method='get'><fieldset>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // CAN Mode
  sprintf(buf, "CAN Mode:<select name=\"mode\">");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<option value=\"0\" %s>Normal</option>", (g_persistent.canMode == 0) ? "selected" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<option value=\"1\" %s>Listen Only</option>", (g_persistent.canMode == 1) ? "selected" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "</select>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // CAN Speed
  sprintf(buf, "<br><br>CAN Speed:<select name=\"speed\">");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<option value=\"0\" %s>5 Kbps</option>", (g_persistent.canSpeed == 0) ? "selected" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<option value=\"1\" %s>10 Kbps</option>", (g_persistent.canSpeed == 1) ? "selected" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<option value=\"2\" %s>20 Kbps</option>", (g_persistent.canSpeed == 2) ? "selected" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<option value=\"3\" %s>25 Kbps</option>", (g_persistent.canSpeed == 3) ? "selected" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<option value=\"4\" %s>50 Kbps</option>", (g_persistent.canSpeed == 4) ? "selected" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<option value=\"5\" %s>100 Kbps</option>", (g_persistent.canSpeed == 5) ? "selected" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<option value=\"6\" %s>125 Kbps (CAN4VSCP)</option>", (g_persistent.canSpeed == 6) ? "selected" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<option value=\"7\" %s>250 Kbps</option>", (g_persistent.canSpeed == 7) ? "selected" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<option value=\"8\" %s>500 Kbps</option>", (g_persistent.canSpeed == 8) ? "selected" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<option value=\"9\" %s>800 Kbps</option>", (g_persistent.canSpeed == 9) ? "selected" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<option value=\"10\" %s>1000 Kbps</option>", (g_persistent.canSpeed == 10) ? "selected" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "</select>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // CAN Filter (hex format)
  sprintf(buf,
          "<br><br>CAN Filter (hex):<input type=\"text\" name=\"filter\" value=\"%08X\" maxlength=\"8\" size=\"10\">",
          (unsigned int) g_persistent.canFilter);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // Statistics (read-only)
  sprintf(buf, "<br><br><fieldset><legend>Statistics (read-only)</legend>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "Messages Sent: %u<br>", (unsigned int) g_persistent.nSent);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "Messages Received: %u<br>", (unsigned int) g_persistent.nRecv);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "Errors: %u<br>", (unsigned int) g_persistent.nErr);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "Last Error Code: 0x%08X", (unsigned int) g_persistent.lastError);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "</fieldset>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<br><button class=\"bgrn bgrn:hover\">Save</button></fieldset></form></div>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, WEBPAGE_CONFIG_END_TEMPLATE, web_ui_version(appDescr), g_persistent.nodeName);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  httpd_resp_send_chunk(req, NULL, 0);

  free(buf);

  return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////
// config_module_get_handler
//

static esp_err_t
config_module_get_handler(httpd_req_t *req)
{
  // esp_err_t rv;
  char *buf;
  // char *temp;

  char *req_buf;
  size_t req_buf_len;

  buf = (char *) calloc(CHUNK_BUFSIZE, 1);
  if (NULL == buf) {
    return ESP_ERR_NO_MEM;
  }

  // Get application info data
  const esp_app_desc_t *appDescr = esp_app_get_description();

  // Get header value string length and allocate memory for length + 1,
  // extra byte for null termination
  req_buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
  if (req_buf_len > 1) {
    req_buf = (char *) malloc(req_buf_len);
    // Copy null terminated value string into buffer
    if (httpd_req_get_hdr_value_str(req, "Host", req_buf, req_buf_len) == ESP_OK) {
      ESP_LOGD(TAG, "Found header => Host: %s", req_buf);
    }
    free(req_buf);
  }

  sprintf(buf, WEBPAGE_START_TEMPLATE, g_persistent.nodeName, "Module Configuration");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<div><form id=but3 class=\"button\" action='/docfgmodule' method='get'><fieldset>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "Module name:<input type=\"text\" name=\"node_name\" maxlength=\"32\" size=\"20\" value=\"%s\" >",
          g_persistent.nodeName);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // TODO
  // char *pmkstr = (char *)malloc(65);
  // for (int i = 0; i < sizeof(g_persistent.pmk); i++) {
  //   sprintf(pmkstr + 2 * i, "%02X", g_persistent.pmk[i]);
  // }
  // sprintf(buf,
  //         "Primay key (32 bytes hex):<input type=\"password\" name=\"key\" maxlength=\"64\" size=\"20\" value=\"%s\"
  //         >",
  //         >", pmkstr);
  // httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
  // free(pmkstr);

  // sprintf(buf,
  //         "Startup delay:<input type=\"text\" name=\"strtdly\" value=\"%d\" maxlength=\"2\" size=\"4\">",
  //         g_persistent.startDelay);
  // httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // Encryption level
  sprintf(buf, "<br>Encryption level:<select name=\"encryptlvl\">");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<option value=\"0\" %s>None</option>",
          (g_persistent.encryptLvl == VSCP_ENCRYPTION_NONE) ? "selected" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<option value=\"1\" %s>AES-128</option>",
          (g_persistent.encryptLvl == VSCP_ENCRYPTION_AES128) ? "selected" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<option value=\"2\" %s>AES-192</option>", (g_persistent.encryptLvl == 2) ? "selected" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<option value=\"3\" %s>AES-256</option>",
          (g_persistent.encryptLvl == VSCP_ENCRYPTION_AES256) ? "selected" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "</select>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  char *pmkstr = (char *) calloc(sizeof(g_persistent.pmk) * 2 + 1, 1);
  for (int i = 0; i < sizeof(g_persistent.pmk); i++) {
    sprintf(pmkstr + 2 * i, "%02X", g_persistent.pmk[i]);
  }
  sprintf(
    buf,
    "Primary security key (16/24/32 hex bytes):<input type=\"password\" name=\"key\" maxlength=\"32\" value=\"%s\" >",
    pmkstr);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
  free(pmkstr);

  sprintf(buf,
          "Module zone:<input type=\"text\" name=\"zone\" maxlength=\"32\" size=\"20\" value=\"%d\" >",
          g_persistent.nodeZone);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "Module sub-zone:<input type=\"text\" name=\"subzone\" maxlength=\"32\" size=\"20\" value=\"%d\" >",
          g_persistent.nodeSubzone);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<button class=\"bgrn bgrn:hover\">Save</button></fieldset></form></div>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, WEBPAGE_CONFIG_END_TEMPLATE, web_ui_version(appDescr), g_persistent.nodeName);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  httpd_resp_send_chunk(req, NULL, 0);

  free(buf);

  return ESP_OK;
}

// static const httpd_uri_t cfgModule = { .uri     = "/cfgmodule",
//                                    .method  = HTTP_GET,
//                                    .handler = config_module_get_handler,
//                                    .user_ctx = NULL };

///////////////////////////////////////////////////////////////////////////////
// do_config_module_get_handler
//

static esp_err_t
do_config_module_get_handler(httpd_req_t *req)
{
  esp_err_t rv;
  char *buf;
  size_t buf_len;

  // Read URL query string length and allocate memory for length + 1,
  // extra byte for null termination
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char *) malloc(buf_len);
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {

      ESP_LOGD(TAG, "Found URL query => %s", buf);
      char *param = (char *) malloc(WEBPAGE_PARAM_SIZE);
      if (NULL == param) {
        return ESP_ERR_NO_MEM;
        free(param);
      }

      // name
      if (ESP_OK == (rv = httpd_query_key_value(buf, "node_name", param, WEBPAGE_PARAM_SIZE))) {
        char *pdecoded = urlDecode(param);
        if (NULL == pdecoded) {
          free(param);
          free(buf);
          return ESP_ERR_NO_MEM;
        }
        ESP_LOGD(TAG, "Found query parameter => name=%s", pdecoded);
        strncpy(g_persistent.nodeName, pdecoded, 31);
        free(pdecoded);

        // setAccessPointParameters();

        // Write changed value to persistent storage
        rv = nvs_set_str(g_nvsHandle, "nodeName", g_persistent.nodeName);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update node name [%s]", esp_err_to_name(rv));
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting node name => rv=%d", rv);
      }

      // key
      if (ESP_OK == (rv = httpd_query_key_value(buf, "key", param, WEBPAGE_PARAM_SIZE))) {
        // uint8_t key_info[DEFAULT_KEY_LEN];
        ESP_LOGD(TAG, "Found query parameter => key");
        memset(g_persistent.pmk, 0, DEFAULT_KEY_LEN);
        rv = vscp_fwhlp_hex2bin(g_persistent.pmk, DEFAULT_KEY_LEN, param);
        if (rv == -1) {
          ESP_LOGE(TAG, "Failed to write key. rv=%d", rv);
        }

        if (rv < 16 || (rv > 16 && rv < 24) || (rv > 24 && rv < 32) || rv > 32) {
          ESP_LOGE(TAG, "Key length invalid. Must be 16, 24 or 32 bytes. rv=%d", rv);
          // Invalid key length
          ESP_LOGE(TAG, "Invalid key size. size=%d", rv);
        }
        else {
          g_persistent.pmkLen = 16; // rv; // Save the keylength (16/24/32 bytes)

          // Write changed value to persistent storage
          rv = nvs_set_blob(g_nvsHandle, "pmk", g_persistent.pmk, sizeof(g_persistent.pmk));
          if (rv != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write node pmk to nvs. rv=%d", rv);
          }
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting node_name => rv=%d", rv);
      }

      // Encryption level
      if (ESP_OK == (rv = httpd_query_key_value(buf, "encryptlvl", param, WEBPAGE_PARAM_SIZE))) {
        ESP_LOGD(TAG, "Found query parameter => encryptlvl=%s", param);
        g_persistent.encryptLvl = (uint8_t) atoi(param);
      }
      else {
        ESP_LOGE(TAG, "Error getting encryption level => rv=%d", rv);
      }
      // const char *encryptLvlStr;
      // switch (g_persistent.encryptLvl) {
      //   case 0:
      //     encryptLvlStr = "None";
      //     break;
      //   case 1:
      //     encryptLvlStr = "AES128";
      //     break;
      //     break;
      //   case 2:
      //     encryptLvlStr = "AES192";
      //     break;
      //   case 3:
      //     encryptLvlStr = "AES256";
      //     break;
      //   default:
      //     encryptLvlStr = "Unknown";
      //     break;
      // }
      // sprintf(buf, "<tr><td class=\"name\">Encryption Level:</td><td class=\"prop\">%s</td></tr>", encryptLvlStr);
      // httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

      // Write changed value to persistent storage
      rv = nvs_set_u8(g_nvsHandle, "encryptLvl", g_persistent.encryptLvl);
      if (rv != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update encryption level [%s]", esp_err_to_name(rv));
      }

      // zone
      if (ESP_OK == (rv = httpd_query_key_value(buf, "zone", param, WEBPAGE_PARAM_SIZE))) {
        g_persistent.nodeZone = (uint8_t) vscp_fwhlp_readStringValue(param);

        // Write changed value to persistent storage
        rv = nvs_set_u8(g_nvsHandle, "nodeZone", g_persistent.nodeZone);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update node zone [%s]", esp_err_to_name(rv));
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting node zone => rv=%d", rv);
      }

      // subzone
      if (ESP_OK == (rv = httpd_query_key_value(buf, "subzone", param, WEBPAGE_PARAM_SIZE))) {
        g_persistent.nodeSubzone = (uint8_t) vscp_fwhlp_readStringValue(param);

        // Write changed value to persistent storage
        rv = nvs_set_u8(g_nvsHandle, "nodeSubZone", g_persistent.nodeSubzone);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update node subzone [%s]", esp_err_to_name(rv));
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting node subzone => rv=%d", rv);
      }

      rv = nvs_commit(g_nvsHandle);
      if (rv != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit updates to nvs\n");
      }

      rv = wcang_reconfigure_wifi_sta();
      if (rv != ESP_OK) {
        ESP_LOGW(TAG, "WiFi runtime reconfiguration failed: %s", esp_err_to_name(rv));
      }

      free(param);
    }

    free(buf);
  }
  const char *resp_str =
    "<html><head><meta charset='utf-8'><meta http-equiv=\"refresh\" content=\"1;url=cfgmodule\" "
    "/><style>" WEBPAGE_STYLE_CSS "</style></head><body><h2 class=\"name\">saving module data...</h2></body></html>";
  httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

  return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////
// do_config_can_get_handler
//

static esp_err_t
do_config_can_get_handler(httpd_req_t *req)
{
  esp_err_t rv;
  char *buf;
  size_t buf_len;

  // Read URL query string length and allocate memory for length + 1,
  // extra byte for null termination
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = malloc(buf_len);
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {

      ESP_LOGD(TAG, "Found URL query => %s", buf);
      char *param = (char *) malloc(WEBPAGE_PARAM_SIZE);
      if (NULL == param) {
        free(buf);
        return ESP_ERR_NO_MEM;
      }

      // CAN Mode
      if (ESP_OK == (rv = httpd_query_key_value(buf, "mode", param, WEBPAGE_PARAM_SIZE))) {
        ESP_LOGD(TAG, "Found query parameter => mode=%s", param);
        g_persistent.canMode = (uint8_t) atoi(param);
        // Write changed value to persistent storage
        rv = nvs_set_u8(g_nvsHandle, "canMode", g_persistent.canMode);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update CAN mode [%s]", esp_err_to_name(rv));
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting CAN mode => rv=%d", rv);
      }

      // CAN Speed
      if (ESP_OK == (rv = httpd_query_key_value(buf, "speed", param, WEBPAGE_PARAM_SIZE))) {
        ESP_LOGD(TAG, "Found query parameter => speed=%s", param);
        g_persistent.canSpeed = (uint8_t) atoi(param);
        // Write changed value to persistent storage
        rv = nvs_set_u8(g_nvsHandle, "canSpeed", g_persistent.canSpeed);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update CAN speed [%s]", esp_err_to_name(rv));
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting CAN speed => rv=%d", rv);
      }

      // CAN Filter
      if (ESP_OK == (rv = httpd_query_key_value(buf, "filter", param, WEBPAGE_PARAM_SIZE))) {
        ESP_LOGD(TAG, "Found query parameter => filter=%s", param);
        uint32_t filter_val    = (uint32_t) strtoul(param, NULL, 16);
        g_persistent.canFilter = filter_val;
        // Write changed value to persistent storage
        rv = nvs_set_u32(g_nvsHandle, "filter", g_persistent.canFilter);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update CAN filter [%s]", esp_err_to_name(rv));
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting CAN filter => rv=%d", rv);
      }

      rv = nvs_commit(g_nvsHandle);
      if (rv != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit updates to nvs\n");
      }

      free(param);
    }

    free(buf);
  }
  const char *resp_str =
    "<html><head><meta charset='utf-8'><meta http-equiv=\"refresh\" content=\"1;url=cfgcan\" "
    "/><style>" WEBPAGE_STYLE_CSS "</style></head><body><h2 class=\"name\">saving CAN/TWAI data...</h2></body></html>";
  httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

  return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////
// print_auth_mode
//

static void
print_auth_mode(httpd_req_t *req, char *buf, int authmode)
{
  switch (authmode) {
    case WIFI_AUTH_OPEN:
      sprintf(buf, "<b>Authmode</b> WIFI_AUTH_OPEN<br>");
      break;
    case WIFI_AUTH_OWE:
      sprintf(buf, "<b>Authmode</b> WIFI_AUTH_OWE<br>");
      break;
    case WIFI_AUTH_WEP:
      sprintf(buf, "<b>Authmode</b> WIFI_AUTH_WEP<br>");
      break;
    case WIFI_AUTH_WPA_PSK:
      sprintf(buf, "<b>Authmode</b> WIFI_AUTH_WPA_PSK<br>");
      break;
    case WIFI_AUTH_WPA2_PSK:
      sprintf(buf, "<b>Authmode</b> WIFI_AUTH_WPA2_PSK<br>");
      break;
    case WIFI_AUTH_WPA_WPA2_PSK:
      sprintf(buf, "<b>Authmode</b> WIFI_AUTH_WPA_WPA2_PSK<br>");
      break;
    case WIFI_AUTH_WPA2_ENTERPRISE:
      sprintf(buf, "<b>Authmode</b> WIFI_AUTH_WPA2_ENTERPRISE<br>");
      break;
    case WIFI_AUTH_WPA3_PSK:
      sprintf(buf, "<b>Authmode</b> WIFI_AUTH_WPA3_PSK<br>");
      break;
    case WIFI_AUTH_WPA2_WPA3_PSK:
      sprintf(buf, "<b>Authmode</b> WIFI_AUTH_WPA2_WPA3_PSK<br>");
      break;
    default:
      sprintf(buf, "<b>Authmode</b> WIFI_AUTH_UNKNOWN<br>");
      break;
  }

  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
}

///////////////////////////////////////////////////////////////////////////////
// print_cipher_type
//

static void
print_cipher_type(httpd_req_t *req, char *buf, int pairwise_cipher, int group_cipher)
{
  switch (pairwise_cipher) {
    case WIFI_CIPHER_TYPE_NONE:
      sprintf(buf, "<b>Pairwise Cipher</b> WIFI_CIPHER_TYPE_NONE<br>");
      break;
    case WIFI_CIPHER_TYPE_WEP40:
      sprintf(buf, "<b>Pairwise Cipher</b> WIFI_CIPHER_TYPE_WEP40<br>");
      break;
    case WIFI_CIPHER_TYPE_WEP104:
      sprintf(buf, "<b>Pairwise Cipher</b> WIFI_CIPHER_TYPE_WEP104<br>");
      break;
    case WIFI_CIPHER_TYPE_TKIP:
      sprintf(buf, "<b>Pairwise Cipher</b> WIFI_CIPHER_TYPE_TKIP<br>");
      break;
    case WIFI_CIPHER_TYPE_CCMP:
      sprintf(buf, "<b>Pairwise Cipher</b> WIFI_CIPHER_TYPE_CCMP<br>");
      break;
    case WIFI_CIPHER_TYPE_TKIP_CCMP:
      sprintf(buf, "<b>Pairwise Cipher</b> WIFI_CIPHER_TYPE_TKIP_CCMP<br>");
      break;
    default:
      sprintf(buf, "<b>Pairwise Cipher</b> WIFI_CIPHER_TYPE_UNKNOWN<br>");
      break;
  }

  switch (group_cipher) {
    case WIFI_CIPHER_TYPE_NONE:
      sprintf(buf, "<b>Group Cipher</b> WIFI_CIPHER_TYPE_NONE<br>");
      break;
    case WIFI_CIPHER_TYPE_WEP40:
      sprintf(buf, "<b>Group Cipher</b> WIFI_CIPHER_TYPE_WEP40<br>");
      break;
    case WIFI_CIPHER_TYPE_WEP104:
      sprintf(buf, "<b>Group Cipher</b> WIFI_CIPHER_TYPE_WEP104<br>");
      break;
    case WIFI_CIPHER_TYPE_TKIP:
      sprintf(buf, "<b>Group Cipher</b> WIFI_CIPHER_TYPE_TKIP<br>");
      break;
    case WIFI_CIPHER_TYPE_CCMP:
      sprintf(buf, "<b>Group Cipher</b> WIFI_CIPHER_TYPE_CCMP<br>");
      break;
    case WIFI_CIPHER_TYPE_TKIP_CCMP:
      sprintf(buf, "<b>Group Cipher</b> WIFI_CIPHER_TYPE_TKIP_CCMP<br>");
      break;
    default:
      sprintf(buf, "<b>Group Cipher</b> WIFI_CIPHER_TYPE_UNKNOWN<br>");
      break;
  }
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
}

///////////////////////////////////////////////////////////////////////////////
// config_wifi_get_handler
//

static esp_err_t
config_wifi_get_handler(httpd_req_t *req)
{
  esp_err_t rv = ESP_OK;
  char *buf;
  char *req_buf;
  size_t req_buf_len;

  buf = (char *) calloc(CHUNK_BUFSIZE, 1);
  if (NULL == buf) {
    return ESP_ERR_NO_MEM;
  }

  // Get application info data
  const esp_app_desc_t *appDescr = esp_app_get_description();

  // Get header value string length and allocate memory for length + 1,
  // extra byte for null termination
  req_buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
  if (req_buf_len > 1) {
    req_buf = (char *) malloc(req_buf_len);
    // Copy null terminated value string into buffer
    if (httpd_req_get_hdr_value_str(req, "Host", req_buf, req_buf_len) == ESP_OK) {
      ESP_LOGD(TAG, "Found header => Host: %s", req_buf);
    }
    free(req_buf);
  }

  sprintf(buf, WEBPAGE_START_TEMPLATE, g_persistent.nodeName, "Wifi Configuration");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<div><form id=but3 class=\"button\" action='/docfgwifi' method='get'><fieldset>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // Display current WiFi configuration
  sprintf(buf, "<fieldset><legend>Current WiFi Configuration</legend>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "Primary SSID:<input type=\"text\" name=\"primary_ssid\" value=\"%s\" maxlength=\"32\" size=\"25\">",
          g_persistent.wifiPrimarySsid);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<br>Primary Password:<input type=\"password\" name=\"primary_password\" value=\"%s\" maxlength=\"64\" "
          "size=\"25\">",
          g_persistent.wifiPrimaryPassword);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(
    buf,
    "<br><br>Secondary SSID:<input type=\"text\" name=\"secondary_ssid\" value=\"%s\" maxlength=\"32\" size=\"25\">",
    g_persistent.wifiSecondarySsid);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<br>Secondary Password:<input type=\"password\" name=\"secondary_password\" value=\"%s\" maxlength=\"64\" "
          "size=\"25\">",
          g_persistent.wifiSecondaryPassword);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<br><br>IP Assignment:<select name=\"static_ip_enable\">"
          "<option value=\"0\" %s>DHCP</option>"
          "<option value=\"1\" %s>Static</option></select>",
          g_persistent.wifiStaticEnable ? "" : "selected",
          g_persistent.wifiStaticEnable ? "selected" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<br>Static IP:<input type=\"text\" name=\"static_ip\" value=\"%s\" maxlength=\"15\" size=\"25\">",
          g_persistent.wifiStaticIp);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<br>Netmask:<input type=\"text\" name=\"static_netmask\" value=\"%s\" maxlength=\"15\" size=\"25\">",
          g_persistent.wifiStaticNetmask);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<br>Gateway:<input type=\"text\" name=\"static_gateway\" value=\"%s\" maxlength=\"15\" size=\"25\">",
          g_persistent.wifiStaticGateway);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<br>DNS:<input type=\"text\" name=\"static_dns\" value=\"%s\" maxlength=\"15\" size=\"25\">",
          g_persistent.wifiStaticDns);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "</fieldset><br>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<fieldset><legend>Available WiFi Networks (Click SSID to copy to Primary/Secondary)</legend>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  uint16_t count = 5;
  wifi_ap_record_t ap_info[5];
  uint16_t ap_count = 0;
  memset(ap_info, 0, sizeof(ap_info));

  // Define scan configuration
  wifi_scan_config_t scan_config = {
    .ssid        = NULL,
    .bssid       = NULL,
    .channel     = 0, // Scan all channels
    .show_hidden = false,
    .scan_type   = WIFI_SCAN_TYPE_ACTIVE
    // .scan_time = {
    //     .active = {
    //         .min = 100,
    //         .max = 400
    //     }
    //}
  };

  ESP_LOGI(TAG, "WiFi scan start");

  // Start WiFi scan with configuration
  rv = esp_wifi_scan_start(&scan_config, true);
  // rv = esp_wifi_scan_start(NULL, true);
  if (rv != ESP_OK) {
    ESP_LOGE(TAG, "WiFi scan failed: %s", esp_err_to_name(rv));
    sprintf(buf, "WiFi scan failed: %s<br>", esp_err_to_name(rv));
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
    rv = ESP_FAIL; // Abort further processing on failure
    goto wifi_scan_done;
  }

  // Retrieve scan results
  rv = esp_wifi_scan_get_ap_records(&count, ap_info);
  if (rv != ESP_OK) {
    ESP_LOGE(TAG, "Failed to get AP records: %s", esp_err_to_name(rv));
    sprintf(buf, "Failed to get AP records: %s<br>", esp_err_to_name(rv));
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
    rv = ESP_FAIL; // Abort further processing on failure
    goto wifi_scan_done;
  }

  ESP_LOGI(TAG, "Number of access points found: %u", count);

  rv = esp_wifi_scan_get_ap_num(&ap_count);
  if (rv != ESP_OK) {
    ESP_LOGE(TAG, "Failed to get AP count: %s", esp_err_to_name(rv));
    sprintf(buf, "Failed to get AP count: %s<br>", esp_err_to_name(rv));
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
    rv = ESP_FAIL; // Abort further processing on failure
    goto wifi_scan_done;
  }

  ap_count = count;

  // Log and send total AP count
  // if (ap_count == 0) {
  //   ESP_LOGW(TAG, "No access points found");
  //   sprintf(buf, "<b>No access points found</b><br>");
  //   httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
  //   rv = ESP_FAIL; // Abort further processing on failure
  //   goto wifi_scan_done;
  // }

  sprintf(buf, "<b>Total APs scanned</b> = %u<br><br>", ap_count);
  ESP_LOGI(TAG, "Total APs scanned = %u", ap_count);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // Send details of each AP
  for (int i = 0; (i < 5) && (i < ap_count); i++) {
    sprintf(buf, "<b>SSID</b> = %s<br>", ap_info[i].ssid);
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
    sprintf(buf, "<b>RSSI</b> = %d<br>", ap_info[i].rssi);
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
    print_auth_mode(req, buf, ap_info[i].authmode);
    if (ap_info[i].authmode != WIFI_AUTH_WEP) {
      print_cipher_type(req, buf, ap_info[i].pairwise_cipher, ap_info[i].group_cipher);
    }
    sprintf(buf, "Channel = %d<br><hr>", ap_info[i].primary);
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
  }

wifi_scan_done:

  sprintf(buf, "</fieldset><br>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<button class=\"bgrn bgrn:hover\">Save Configuration</button>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "</fieldset></form></div>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<br><div><form id=rescan class=\"button\" action='/cfgwifi' method='get'>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<button class=\"button\">Rescan Networks</button></form></div>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, WEBPAGE_CONFIG_END_TEMPLATE, web_ui_version(appDescr), g_persistent.nodeName);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  httpd_resp_send_chunk(req, NULL, 0);

  free(buf);

  return rv;
}

///////////////////////////////////////////////////////////////////////////////
// do_config_wifi_get_handler
//

static esp_err_t
do_config_wifi_get_handler(httpd_req_t *req)
{
  esp_err_t rv;
  char *buf;
  size_t buf_len;

  // Read URL query string length and allocate memory for length + 1,
  // extra byte for null termination
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char *) malloc(buf_len);
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {

      ESP_LOGD(TAG, "Found URL query => %s", buf);
      char *param = malloc(WEBPAGE_PARAM_SIZE);
      if (NULL == param) {
        return ESP_ERR_NO_MEM;
        free(param);
      }

      // Primary SSID
      if (ESP_OK == (rv = httpd_query_key_value(buf, "primary_ssid", param, WEBPAGE_PARAM_SIZE))) {
        char *pdecoded = urlDecode(param);
        if (NULL == pdecoded) {
          free(param);
          free(buf);
          return ESP_ERR_NO_MEM;
        }
        ESP_LOGD(TAG, "Found query parameter => primary_ssid=%s", pdecoded);
        strncpy(g_persistent.wifiPrimarySsid, pdecoded, sizeof(g_persistent.wifiPrimarySsid) - 1);
        g_persistent.wifiPrimarySsid[sizeof(g_persistent.wifiPrimarySsid) - 1] = '\0';
        free(pdecoded);
        // Write changed value to persistent storage
        rv = nvs_set_str(g_nvsHandle, "wifiPriSsid", g_persistent.wifiPrimarySsid);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update primary SSID [%s]", esp_err_to_name(rv));
        }
      }

      // Primary Password
      if (ESP_OK == (rv = httpd_query_key_value(buf, "primary_password", param, WEBPAGE_PARAM_SIZE))) {
        char *pdecoded = urlDecode(param);
        if (NULL == pdecoded) {
          free(param);
          free(buf);
          return ESP_ERR_NO_MEM;
        }
        ESP_LOGD(TAG, "Found query parameter => primary_password");
        strncpy(g_persistent.wifiPrimaryPassword, pdecoded, sizeof(g_persistent.wifiPrimaryPassword) - 1);
        g_persistent.wifiPrimaryPassword[sizeof(g_persistent.wifiPrimaryPassword) - 1] = '\0';
        free(pdecoded);
        // Write changed value to persistent storage
        rv = nvs_set_str(g_nvsHandle, "wifiPriPass", g_persistent.wifiPrimaryPassword);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update primary password [%s]", esp_err_to_name(rv));
        }
      }

      // Secondary SSID
      if (ESP_OK == (rv = httpd_query_key_value(buf, "secondary_ssid", param, WEBPAGE_PARAM_SIZE))) {
        char *pdecoded = urlDecode(param);
        if (NULL == pdecoded) {
          free(param);
          free(buf);
          return ESP_ERR_NO_MEM;
        }
        ESP_LOGD(TAG, "Found query parameter => secondary_ssid=%s", pdecoded);
        strncpy(g_persistent.wifiSecondarySsid, pdecoded, sizeof(g_persistent.wifiSecondarySsid) - 1);
        g_persistent.wifiSecondarySsid[sizeof(g_persistent.wifiSecondarySsid) - 1] = '\0';
        free(pdecoded);
        // Write changed value to persistent storage
        rv = nvs_set_str(g_nvsHandle, "wifiSecSsid", g_persistent.wifiSecondarySsid);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update secondary SSID [%s]", esp_err_to_name(rv));
        }
      }

      // Secondary Password
      if (ESP_OK == (rv = httpd_query_key_value(buf, "secondary_password", param, WEBPAGE_PARAM_SIZE))) {
        char *pdecoded = urlDecode(param);
        if (NULL == pdecoded) {
          free(param);
          free(buf);
          return ESP_ERR_NO_MEM;
        }
        ESP_LOGD(TAG, "Found query parameter => secondary_password");
        strncpy(g_persistent.wifiSecondaryPassword, pdecoded, sizeof(g_persistent.wifiSecondaryPassword) - 1);
        g_persistent.wifiSecondaryPassword[sizeof(g_persistent.wifiSecondaryPassword) - 1] = '\0';
        free(pdecoded);
        // Write changed value to persistent storage
        rv = nvs_set_str(g_nvsHandle, "wifiSecPass", g_persistent.wifiSecondaryPassword);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update secondary password [%s]", esp_err_to_name(rv));
        }
      }

      // IP assignment mode (0 = DHCP, 1 = static)
      if (ESP_OK == (rv = httpd_query_key_value(buf, "static_ip_enable", param, WEBPAGE_PARAM_SIZE))) {
        g_persistent.wifiStaticEnable = (0 == strcmp(param, "1")) ? 1 : 0;
        rv                            = nvs_set_u8(g_nvsHandle, "wifiStaIpEn", g_persistent.wifiStaticEnable);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update static IP enable [%s]", esp_err_to_name(rv));
        }
      }

      // Static IP address
      if (ESP_OK == (rv = httpd_query_key_value(buf, "static_ip", param, WEBPAGE_PARAM_SIZE))) {
        char *pdecoded = urlDecode(param);
        if (NULL == pdecoded) {
          free(param);
          free(buf);
          return ESP_ERR_NO_MEM;
        }
        strncpy(g_persistent.wifiStaticIp, pdecoded, sizeof(g_persistent.wifiStaticIp) - 1);
        g_persistent.wifiStaticIp[sizeof(g_persistent.wifiStaticIp) - 1] = '\0';
        free(pdecoded);
        rv = nvs_set_str(g_nvsHandle, "wifiStaIp", g_persistent.wifiStaticIp);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update static IP [%s]", esp_err_to_name(rv));
        }
      }

      // Static netmask
      if (ESP_OK == (rv = httpd_query_key_value(buf, "static_netmask", param, WEBPAGE_PARAM_SIZE))) {
        char *pdecoded = urlDecode(param);
        if (NULL == pdecoded) {
          free(param);
          free(buf);
          return ESP_ERR_NO_MEM;
        }
        strncpy(g_persistent.wifiStaticNetmask, pdecoded, sizeof(g_persistent.wifiStaticNetmask) - 1);
        g_persistent.wifiStaticNetmask[sizeof(g_persistent.wifiStaticNetmask) - 1] = '\0';
        free(pdecoded);
        rv = nvs_set_str(g_nvsHandle, "wifiStaMask", g_persistent.wifiStaticNetmask);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update static netmask [%s]", esp_err_to_name(rv));
        }
      }

      // Static gateway
      if (ESP_OK == (rv = httpd_query_key_value(buf, "static_gateway", param, WEBPAGE_PARAM_SIZE))) {
        char *pdecoded = urlDecode(param);
        if (NULL == pdecoded) {
          free(param);
          free(buf);
          return ESP_ERR_NO_MEM;
        }
        strncpy(g_persistent.wifiStaticGateway, pdecoded, sizeof(g_persistent.wifiStaticGateway) - 1);
        g_persistent.wifiStaticGateway[sizeof(g_persistent.wifiStaticGateway) - 1] = '\0';
        free(pdecoded);
        rv = nvs_set_str(g_nvsHandle, "wifiStaGw", g_persistent.wifiStaticGateway);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update static gateway [%s]", esp_err_to_name(rv));
        }
      }

      // Static DNS server
      if (ESP_OK == (rv = httpd_query_key_value(buf, "static_dns", param, WEBPAGE_PARAM_SIZE))) {
        char *pdecoded = urlDecode(param);
        if (NULL == pdecoded) {
          free(param);
          free(buf);
          return ESP_ERR_NO_MEM;
        }
        strncpy(g_persistent.wifiStaticDns, pdecoded, sizeof(g_persistent.wifiStaticDns) - 1);
        g_persistent.wifiStaticDns[sizeof(g_persistent.wifiStaticDns) - 1] = '\0';
        free(pdecoded);
        rv = nvs_set_str(g_nvsHandle, "wifiStaDns", g_persistent.wifiStaticDns);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update static DNS [%s]", esp_err_to_name(rv));
        }
      }

      rv = nvs_commit(g_nvsHandle);
      if (rv != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit updates to nvs\n");
      }

      free(param);
    }

    free(buf);
  }
  const char *resp_str =
    "<html><head><meta charset='utf-8'><meta http-equiv=\"refresh\" content=\"2;url=cfgwifi\" "
    "/><style>" WEBPAGE_STYLE_CSS "</style></head><body><h2 class=\"name\">Saving WiFi configuration...<br>"
    "Device will apply network settings and reconnect.</h2></body></html>";
  httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

  return ESP_OK;
}

// ----------------------------------------------------------------------------

///////////////////////////////////////////////////////////////////////////////
// config_vscplink_get_handler
//

static esp_err_t
config_vscplink_get_handler(httpd_req_t *req)
{
  // esp_err_t rv;
  char *buf;
  // char *temp;

  char *req_buf;
  size_t req_buf_len;

  buf = (char *) calloc(CHUNK_BUFSIZE, 1);
  if (NULL == buf) {
    return ESP_ERR_NO_MEM;
  }

  // Get application info data
  const esp_app_desc_t *appDescr = esp_app_get_description();

  // Get header value string length and allocate memory for length + 1,
  // extra byte for null termination
  req_buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
  if (req_buf_len > 1) {
    req_buf = (char *) malloc(req_buf_len);
    // Copy null terminated value string into buffer
    if (httpd_req_get_hdr_value_str(req, "Host", req_buf, req_buf_len) == ESP_OK) {
      ESP_LOGD(TAG, "Found header => Host: %s", req_buf);
    }
    free(req_buf);
  }

  sprintf(buf, WEBPAGE_START_TEMPLATE, g_persistent.nodeName, "VSCP Link Srv Configuration");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<div><form id=but3 class=\"button\" action='/docfgvscplink' method='get'><fieldset>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // Enable
  sprintf(buf, "<input type=\"checkbox\" name=\"enable\" value=\"true\" ");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
  sprintf(buf, "%s><label for=\"lr\"> Enable</label>", g_persistent.enableVscpLink ? "checked" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<br>Port:<input type=\"text\" name=\"port\" value=\"%d\" >", g_persistent.vscplinkPort);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "Username:<input type=\"text\" name=\"user\" value=\"%s\" >", g_persistent.vscplinkUser);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "Password:<input type=\"password\" name=\"password\" value=\"%s\" >", g_persistent.vscplinkPw);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<button class=\"bgrn bgrn:hover\">Save</button></fieldset></form></div>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, WEBPAGE_CONFIG_END_TEMPLATE, web_ui_version(appDescr), g_persistent.nodeName);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  httpd_resp_send_chunk(req, NULL, 0);

  free(buf);

  return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////
// do_config_vscplink_get_handler
//

static esp_err_t
do_config_vscplink_get_handler(httpd_req_t *req)
{
  esp_err_t rv;
  char *buf;
  size_t buf_len;

  // Read URL query string length and allocate memory for length + 1,
  // extra byte for null termination
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char *) malloc(buf_len);
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {

      ESP_LOGD(TAG, "Found URL query => %s", buf);
      char *param = malloc(WEBPAGE_PARAM_SIZE);
      if (NULL == param) {
        return ESP_ERR_NO_MEM;
        free(param);
      }

      // Enable
      if (ESP_OK == (rv = httpd_query_key_value(buf, "enable", param, WEBPAGE_PARAM_SIZE))) {
        ESP_LOGD(TAG, "Found query parameter => enable=%s", param);
        if (NULL != strstr(param, "true")) {
          g_persistent.enableVscpLink = true;
        }
      }
      else {
        g_persistent.enableVscpLink = false;
      }

      // Write changed value to persistent storage
      rv = nvs_set_u8(g_nvsHandle, "vscplinkEnable", g_persistent.enableVscpLink);
      if (rv != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update vscp link srv enable [%s]", esp_err_to_name(rv));
      }

      // port
      if (ESP_OK == (rv = httpd_query_key_value(buf, "port", param, WEBPAGE_PARAM_SIZE))) {
        ESP_LOGD(TAG, "Found query parameter => port=%s", param);
        g_persistent.vscplinkPort = atoi(param);
        // Write changed value to persistent storage
        rv = nvs_set_u16(g_nvsHandle, "vscplinkPort", g_persistent.vscplinkPort);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update VSCP link srv port [%s]", esp_err_to_name(rv));
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting VSCP link srv port => rv=%d", rv);
      }

      // username
      if (ESP_OK == (rv = httpd_query_key_value(buf, "user", param, WEBPAGE_PARAM_SIZE))) {
        char *pdecoded = urlDecode(param);
        if (NULL == pdecoded) {
          free(param);
          free(buf);
          return ESP_ERR_NO_MEM;
        }
        ESP_LOGD(TAG, "Found query parameter => user=%s", pdecoded);
        strncpy(g_persistent.vscplinkUser, pdecoded, sizeof(g_persistent.vscplinkUser) - 1);
        // Write changed value to persistent storage
        rv = nvs_set_str(g_nvsHandle, "vscplinkUser", g_persistent.vscplinkUser);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update VSCP link srv user [%s]", esp_err_to_name(rv));
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting VSCP link srv username => rv=%d", rv);
      }

      // password
      if (ESP_OK == (rv = httpd_query_key_value(buf, "password", param, WEBPAGE_PARAM_SIZE))) {
        char *pdecoded = urlDecode(param);
        if (NULL == pdecoded) {
          free(param);
          free(buf);
          return ESP_ERR_NO_MEM;
        }
        ESP_LOGD(TAG, "Found query parameter => password=%s", pdecoded);
        strncpy(g_persistent.vscplinkPw, pdecoded, sizeof(g_persistent.vscplinkPw) - 1);
        // Write changed value to persistent storage
        rv = nvs_set_str(g_nvsHandle, "vscplinkPw", g_persistent.vscplinkPw);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update VSCP link srv password err=%02X [%s]", rv, esp_err_to_name(rv));
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting VSCP link srv password => rv=%d", rv);
      }

      // key
      if (ESP_OK == (rv = httpd_query_key_value(buf, "key", param, WEBPAGE_PARAM_SIZE))) {
        ESP_LOGD(TAG, "Found query parameter => key=%s", param);
        memset(g_persistent.pmk, 0, 32);
        vscp_fwhlp_hex2bin(g_persistent.pmk, 32, param);

        // Write changed value to persistent storage
        rv = nvs_set_blob(g_nvsHandle, "vscp_key", g_persistent.pmk, sizeof(g_persistent.pmk));
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to write VSCP link key to nvs. rv=%d", rv);
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting VSCP link key => rv=%d", rv);
      }

      rv = nvs_commit(g_nvsHandle);
      if (rv != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit updates to nvs\n");
      }

      free(param);
    }

    free(buf);
  }
  const char *resp_str =
    "<html><head><meta charset='utf-8'><meta http-equiv=\"refresh\" content=\"1;url=cfgvscplink\" "
    "/><style>" WEBPAGE_STYLE_CSS "</style></head><body><h2 class=\"name\">saving module data...</h2></body></html>";
  httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

  return ESP_OK;
}

// ----------------------------------------------------------------------------

///////////////////////////////////////////////////////////////////////////////
// config_mqtt_get_handler
//

static esp_err_t
config_mqtt_get_handler(httpd_req_t *req)
{
  char *buf;

  char *req_buf;
  size_t req_buf_len;

  buf = (char *) calloc(CHUNK_BUFSIZE, 1);
  if (NULL == buf) {
    return ESP_ERR_NO_MEM;
  }

  // Get application info data
  const esp_app_desc_t *appDescr = esp_app_get_description();

  // Get header value string length and allocate memory for length + 1,
  // extra byte for null termination
  req_buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
  if (req_buf_len > 1) {
    req_buf = (char *) malloc(req_buf_len);
    // Copy null terminated value string into buffer
    if (httpd_req_get_hdr_value_str(req, "Host", req_buf, req_buf_len) == ESP_OK) {
      ESP_LOGD(TAG, "Found header => Host: %s", req_buf);
    }
    free(req_buf);
  }

  sprintf(buf, WEBPAGE_START_TEMPLATE, g_persistent.nodeName, "MQTT Configuration");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<div><form id=but3 class=\"button\" action='/docfgmqtt' method='get'><fieldset>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // Enable
  sprintf(buf, "<input type=\"checkbox\" name=\"enable\" value=\"true\" ");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
  sprintf(buf, "%s><label for=\"lr\"> Enable</label>", g_persistent.enableMqtt ? "checked" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<br><br>Host: (without mqtt:// or other prefix)<input type=\"text\" name=\"url\" value=\"%s\" >",
          g_persistent.mqttUrl);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "Port:<input type=\"text\" name=\"port\" value=\"%d\" >", g_persistent.mqttPort);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "Client id:<input type=\"text\" name=\"client\" value=\"%s\" >", g_persistent.mqttClientId);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "Username:<input type=\"text\" name=\"user\" value=\"%s\" >", g_persistent.mqttUser);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "Password:<input type=\"password\" name=\"password\" value=\"%s\" >", g_persistent.mqttPw);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "Subscribe:<input type=\"text\" name=\"sub\" value=\"%s\" >", g_persistent.mqttSubTopic);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "Publish:<input type=\"text\" name=\"pub\" value=\"%s\" >", g_persistent.mqttPubTopic);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // MQTT format
  sprintf(buf, "<br>MQTT format:<select name=\"format\">");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<option value=\"0\" %s>JSON</option>", (g_persistent.mqttFormat == MQTT_FORMAT_JSON) ? "selected" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<option value=\"1\" %s>XML</option>", (g_persistent.mqttFormat == MQTT_FORMAT_XML) ? "selected" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<option value=\"2\" %s>String</option>",
          (g_persistent.mqttFormat == MQTT_FORMAT_STRING) ? "selected" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<option value=\"3\" %s>Binary</option>",
          (g_persistent.mqttFormat == MQTT_FORMAT_BINARY) ? "selected" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "</select>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // QoS selection
  sprintf(buf, "<br>QoS:<select name=\"qos\">");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
  sprintf(buf, "<option value=\"0\" %s>0 - At most once</option>", g_persistent.mqttQos == 0 ? "selected" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
  sprintf(buf, "<option value=\"1\" %s>1 - At least once</option>", g_persistent.mqttQos == 1 ? "selected" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
  sprintf(buf, "<option value=\"2\" %s>2 - Exactly once</option>", g_persistent.mqttQos == 2 ? "selected" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
  sprintf(buf, "</select>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // Retain flag
  sprintf(buf,
          " <input type=\"checkbox\" name=\"retain\" value=\"true\" %s><label> Retain</label>",
          g_persistent.mqttRetain ? "checked" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<br><br><fieldset><legend>TLS/SSL Configuration</legend>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // Enable TLS
  sprintf(buf, "<input type=\"checkbox\" name=\"enable_tls\" value=\"true\" ");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
  sprintf(buf, "%s><label for=\"tls\"> Enable TLS/SSL</label>", g_persistent.enableMqttTls ? "checked" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<br><br>CA Certificate (PEM):<br><textarea name=\"ca_cert\" rows=\"6\" cols=\"60\">%s</textarea>",
          g_persistent.mqttCaCert);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<br>Client Certificate (PEM):<br><textarea name=\"client_cert\" rows=\"6\" cols=\"60\">%s</textarea>",
          g_persistent.mqttClientCert);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<br>Client Private Key (PEM):<br><textarea name=\"client_key\" rows=\"6\" cols=\"60\">%s</textarea>",
          g_persistent.mqttClientKey);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "</fieldset>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<button class=\"bgrn bgrn:hover\">Save</button></fieldset></form></div>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, WEBPAGE_CONFIG_END_TEMPLATE, web_ui_version(appDescr), g_persistent.nodeName);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  httpd_resp_send_chunk(req, NULL, 0);

  free(buf);

  return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////
// do_config_mqtt_get_handler
//

static esp_err_t
do_config_mqtt_get_handler(httpd_req_t *req)
{
  esp_err_t rv;
  char *buf;
  size_t buf_len;

  // Read URL query string length and allocate memory for length + 1,
  // extra byte for null termination
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = malloc(buf_len);
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {

      ESP_LOGD(TAG, "Found URL query => %s", buf);
      char *param = (char *) malloc(WEBPAGE_PARAM_SIZE);
      if (NULL == param) {
        return ESP_ERR_NO_MEM;
        free(param);
      }

      // Enable
      if (ESP_OK == (rv = httpd_query_key_value(buf, "enable", param, WEBPAGE_PARAM_SIZE))) {
        ESP_LOGD(TAG, "Found query parameter => enable=%s", param);
        if (NULL != strstr(param, "true")) {
          g_persistent.enableMqtt = true;
        }
      }
      else {
        g_persistent.enableMqtt = false;
      }

      // Write changed value to persistent storage
      nvs_set_u8(g_nvsHandle, "enableMqtt", g_persistent.enableMqtt);
      if (rv != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update MQTT enable [%s]", esp_err_to_name(rv));
      }

      // url
      if (ESP_OK == (rv = httpd_query_key_value(buf, "url", param, WEBPAGE_PARAM_SIZE))) {
        char *pdecoded = urlDecode(param);
        if (NULL == pdecoded) {
          free(param);
          free(buf);
          return ESP_ERR_NO_MEM;
        }
        ESP_LOGD(TAG, "Found query parameter => url=%s", pdecoded);
        strncpy(g_persistent.mqttUrl, pdecoded, sizeof(g_persistent.mqttUrl) - 1);
        free(pdecoded);
        // Write changed value to persistent storage
        rv = nvs_set_str(g_nvsHandle, "mqttUrl", g_persistent.mqttUrl);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update MQTT url [%s]", esp_err_to_name(rv));
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting MQTT URL => rv=%d", rv);
      }

      // port
      if (ESP_OK == (rv = httpd_query_key_value(buf, "port", param, WEBPAGE_PARAM_SIZE))) {
        ESP_LOGD(TAG, "Found query parameter => port=%s", param);
        g_persistent.mqttPort = atoi(param);
        // Write changed value to persistent storage
        rv = nvs_set_u16(g_nvsHandle, "mqttPort", g_persistent.mqttPort);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update MQTT port [%s]", esp_err_to_name(rv));
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting MQTT port => rv=%d", rv);
      }

      // clientid
      if (ESP_OK == (rv = httpd_query_key_value(buf, "client", param, WEBPAGE_PARAM_SIZE))) {
        char *pdecoded = urlDecode(param);
        if (NULL == pdecoded) {
          free(param);
          free(buf);
          return ESP_ERR_NO_MEM;
        }
        ESP_LOGD(TAG, "Found query parameter => clientid=%s", pdecoded);
        strncpy(g_persistent.mqttClientId, pdecoded, sizeof(g_persistent.mqttClientId) - 1);
        // Write changed value to persistent storage
        rv = nvs_set_str(g_nvsHandle, "mqttClientId", g_persistent.mqttClientId);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update MQTT clientid [%s]", esp_err_to_name(rv));
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting MQTT clientid => rv=%d", rv);
      }

      // user
      if (ESP_OK == (rv = httpd_query_key_value(buf, "user", param, WEBPAGE_PARAM_SIZE))) {
        char *pdecoded = urlDecode(param);
        if (NULL == pdecoded) {
          free(param);
          free(buf);
          return ESP_ERR_NO_MEM;
        }
        ESP_LOGD(TAG, "Found query parameter => user=%s", pdecoded);
        strncpy(g_persistent.mqttUser, pdecoded, sizeof(g_persistent.mqttUser) - 1);
        // Write changed value to persistent storage
        rv = nvs_set_str(g_nvsHandle, "mqttUser", g_persistent.mqttUser);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update MQTT user [%s]", esp_err_to_name(rv));
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting MQTT user => rv=%d", rv);
      }

      // password
      if (ESP_OK == (rv = httpd_query_key_value(buf, "password", param, WEBPAGE_PARAM_SIZE))) {
        char *pdecoded = urlDecode(param);
        if (NULL == pdecoded) {
          free(param);
          free(buf);
          return ESP_ERR_NO_MEM;
        }
        ESP_LOGD(TAG, "Found query parameter => password=%s", pdecoded);
        strncpy(g_persistent.mqttPw, pdecoded, sizeof(g_persistent.mqttPw) - 1);
        // Write changed value to persistent storage
        rv = nvs_set_str(g_nvsHandle, "mqttPw", g_persistent.mqttPw);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update MQTT password [%s]", esp_err_to_name(rv));
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting MQTT password => rv=%d", rv);
      }

      // Subscribe
      if (ESP_OK == (rv = httpd_query_key_value(buf, "sub", param, WEBPAGE_PARAM_SIZE))) {
        char *pdecoded = urlDecode(param);
        if (NULL == pdecoded) {
          free(param);
          free(buf);
          return ESP_ERR_NO_MEM;
        }
        ESP_LOGD(TAG, "Found query parameter => sub=%s", pdecoded);
        strncpy(g_persistent.mqttSubTopic, pdecoded, sizeof(g_persistent.mqttSubTopic) - 1);
        free(pdecoded);
        // Write changed value to persistent storage
        rv = nvs_set_str(g_nvsHandle, "mqttSubTopic", g_persistent.mqttSubTopic);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update MQTT sub [%s]", esp_err_to_name(rv));
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting MQTT sub => rv=%d", rv);
      }

      // Publish
      if (ESP_OK == (rv = httpd_query_key_value(buf, "pub", param, WEBPAGE_PARAM_SIZE))) {
        char *pdecoded = urlDecode(param);
        if (NULL == pdecoded) {
          free(param);
          free(buf);
          return ESP_ERR_NO_MEM;
        }
        ESP_LOGD(TAG, "Found query parameter => pub=%s", pdecoded);
        strncpy(g_persistent.mqttPubTopic, pdecoded, sizeof(g_persistent.mqttPubTopic) - 1);
        free(pdecoded);
        // Write changed value to persistent storage
        rv = nvs_set_str(g_nvsHandle, "mqttPubTopic", g_persistent.mqttPubTopic);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update MQTT pub [%s]", esp_err_to_name(rv));
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting MQTT pub => rv=%d", rv);
      }

      // QoS
      if (ESP_OK == (rv = httpd_query_key_value(buf, "qos", param, WEBPAGE_PARAM_SIZE))) {
        ESP_LOGD(TAG, "Found query parameter => qos=%s", param);
        g_persistent.mqttQos = atoi(param);
        // Clamp to valid range (0-2)
        if (g_persistent.mqttQos > 2) {
          g_persistent.mqttQos = 0;
        }
        // Write changed value to persistent storage
        rv = nvs_set_u8(g_nvsHandle, "mqttQos", g_persistent.mqttQos);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update MQTT QoS [%s]", esp_err_to_name(rv));
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting MQTT QoS => rv=%d", rv);
      }

      // Retain
      if (ESP_OK == (rv = httpd_query_key_value(buf, "retain", param, WEBPAGE_PARAM_SIZE))) {
        ESP_LOGD(TAG, "Found query parameter => retain=%s", param);
        if (NULL != strstr(param, "true")) {
          g_persistent.mqttRetain = true;
        }
      }
      else {
        g_persistent.mqttRetain = false;
      }

      // MQTT format
      if (ESP_OK == (rv = httpd_query_key_value(buf, "format", param, WEBPAGE_PARAM_SIZE))) {
        ESP_LOGD(TAG, "Found query parameter => format=%s", param);
        g_persistent.mqttFormat = atoi(param);
        // Clamp to valid range (0-3)
        if (g_persistent.mqttFormat > 3) {
          g_persistent.mqttFormat = MQTT_FORMAT_JSON;
        }
        // Write changed value to persistent storage
        rv = nvs_set_u8(g_nvsHandle, "mqttFormat", g_persistent.mqttFormat);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update MQTT format [%s]", esp_err_to_name(rv));
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting MQTT format => rv=%d", rv);
      }

      // Enable TLS
      if (ESP_OK == (rv = httpd_query_key_value(buf, "enable_tls", param, WEBPAGE_PARAM_SIZE))) {
        ESP_LOGD(TAG, "Found query parameter => enable_tls=%s", param);
        if (NULL != strstr(param, "true")) {
          g_persistent.enableMqttTls = true;
        }
      }
      else {
        g_persistent.enableMqttTls = false;
      }

      // Write changed value to persistent storage
      rv = nvs_set_u8(g_nvsHandle, "enableMqttTls", g_persistent.enableMqttTls);
      if (rv != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update MQTT TLS enable [%s]", esp_err_to_name(rv));
      }

      // CA Certificate
      if (ESP_OK == (rv = httpd_query_key_value(buf, "ca_cert", param, WEBPAGE_PARAM_SIZE))) {
        char *pdecoded = urlDecode(param);
        if (NULL == pdecoded) {
          free(param);
          free(buf);
          return ESP_ERR_NO_MEM;
        }
        ESP_LOGD(TAG, "Found query parameter => ca_cert (length=%d)", strlen(pdecoded));
        strncpy(g_persistent.mqttCaCert, pdecoded, sizeof(g_persistent.mqttCaCert) - 1);
        free(pdecoded);
        // Write changed value to persistent storage
        rv = nvs_set_str(g_nvsHandle, "mqttCaCert", g_persistent.mqttCaCert);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update MQTT CA cert [%s]", esp_err_to_name(rv));
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting MQTT CA cert => rv=%d", rv);
      }

      // Client Certificate
      if (ESP_OK == (rv = httpd_query_key_value(buf, "client_cert", param, WEBPAGE_PARAM_SIZE))) {
        char *pdecoded = urlDecode(param);
        if (NULL == pdecoded) {
          free(param);
          free(buf);
          return ESP_ERR_NO_MEM;
        }
        ESP_LOGD(TAG, "Found query parameter => client_cert (length=%d)", strlen(pdecoded));
        strncpy(g_persistent.mqttClientCert, pdecoded, sizeof(g_persistent.mqttClientCert) - 1);
        free(pdecoded);
        // Write changed value to persistent storage
        rv = nvs_set_str(g_nvsHandle, "mqttClientCert", g_persistent.mqttClientCert);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update MQTT client cert [%s]", esp_err_to_name(rv));
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting MQTT client cert => rv=%d", rv);
      }

      // Client Private Key
      if (ESP_OK == (rv = httpd_query_key_value(buf, "client_key", param, WEBPAGE_PARAM_SIZE))) {
        char *pdecoded = urlDecode(param);
        if (NULL == pdecoded) {
          free(param);
          free(buf);
          return ESP_ERR_NO_MEM;
        }
        ESP_LOGD(TAG, "Found query parameter => client_key (length=%d)", strlen(pdecoded));
        strncpy(g_persistent.mqttClientKey, pdecoded, sizeof(g_persistent.mqttClientKey) - 1);
        free(pdecoded);
        // Write changed value to persistent storage
        rv = nvs_set_str(g_nvsHandle, "mqttClientKey", g_persistent.mqttClientKey);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update MQTT client key [%s]", esp_err_to_name(rv));
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting MQTT client key => rv=%d", rv);
      }

      // Write changed value to persistent storage
      rv = nvs_set_u8(g_nvsHandle, "mqttRetain", g_persistent.mqttRetain);
      if (rv != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update MQTT retain [%s]", esp_err_to_name(rv));
      }

      rv = nvs_commit(g_nvsHandle);
      if (rv != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit updates to nvs\n");
      }

      free(param);
    }

    free(buf);
  }
  const char *resp_str =
    "<html><head><meta charset='utf-8'><meta http-equiv=\"refresh\" content=\"1;url=cfgmqtt\" "
    "/><style>" WEBPAGE_STYLE_CSS "</style></head><body><h2 class=\"name\">saving module data...</h2></body></html>";
  httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

  return ESP_OK;
}

// ----------------------------------------------------------------------------

///////////////////////////////////////////////////////////////////////////////
// config_multicast_get_handler
//

static esp_err_t
config_multicast_get_handler(httpd_req_t *req)
{
  // esp_err_t rv;
  char *buf;
  // char *temp;

  char *req_buf;
  size_t req_buf_len;

  buf = (char *) calloc(CHUNK_BUFSIZE, 1);
  if (NULL == buf) {
    return ESP_ERR_NO_MEM;
  }

  // Get application info data
  const esp_app_desc_t *appDescr = esp_app_get_description();

  // Get header value string length and allocate memory for length + 1,
  // extra byte for null termination
  req_buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
  if (req_buf_len > 1) {
    req_buf = (char *) malloc(req_buf_len);
    // Copy null terminated value string into buffer
    if (httpd_req_get_hdr_value_str(req, "Host", req_buf, req_buf_len) == ESP_OK) {
      ESP_LOGD(TAG, "Found header => Host: %s", req_buf);
    }
    free(req_buf);
  }

  sprintf(buf, WEBPAGE_START_TEMPLATE, g_persistent.nodeName, "Multicast Configuration");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<div><form id=but3 class=\"button\" action='/docfgmulticast' method='get'><fieldset>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // Enable
  sprintf(buf, "<input type=\"checkbox\" name=\"enable\" value=\"true\" ");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
  sprintf(buf, "%s><label for=\"lr\"> Enable</label>", g_persistent.enableMulticast ? "checked" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf,
          "<br><br>Multicast IP address (224.0.23.158):<input type=\"text\" name=\"url\" value=\"%s\" >",
          g_persistent.multicastUrl);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "Port (9598):<input type=\"text\" name=\"port\" value=\"%d\" >", g_persistent.multicastPort);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "TTL:<input type=\"text\" name=\"ttl\" value=\"%d\" >", g_persistent.multicastTtl);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // bMcastEncrypt
  sprintf(buf, "<input type=\"checkbox\" name=\"bMcastEncrypt\" value=\"true\" ");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
  sprintf(buf, "%s><label for=\"lr\"> Enable Encryption</label>", g_persistent.bMcastEncrypt ? "checked" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // bMcastHbeat
  sprintf(buf, "<br><input type=\"checkbox\" name=\"bMcastHbeat\" value=\"true\" ");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
  sprintf(buf,
          "%s><label for=\"lr\"> Enable VSCP heartbeat on port 9598</label>",
          g_persistent.bHeartbeat ? "checked" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<button class=\"bgrn bgrn:hover\">Save</button></fieldset></form></div>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, WEBPAGE_CONFIG_END_TEMPLATE, web_ui_version(appDescr), g_persistent.nodeName);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  httpd_resp_send_chunk(req, NULL, 0);

  free(buf);

  return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////
// do_config_multicast_get_handler
//

static esp_err_t
do_config_multicast_get_handler(httpd_req_t *req)
{
  esp_err_t rv;
  char *buf;
  size_t buf_len;

  // Read URL query string length and allocate memory for length + 1,
  // extra byte for null termination
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = malloc(buf_len);
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {

      ESP_LOGD(TAG, "Found URL query => %s", buf);
      char *param = (char *) malloc(WEBPAGE_PARAM_SIZE);
      if (NULL == param) {
        return ESP_ERR_NO_MEM;
        free(param);
      }

      // Enable
      if (ESP_OK == (rv = httpd_query_key_value(buf, "enable", param, WEBPAGE_PARAM_SIZE))) {
        ESP_LOGD(TAG, "Found query parameter => enable=%s", param);
        if (NULL != strstr(param, "true")) {
          g_persistent.enableMulticast = true;
        }
      }
      else {
        g_persistent.enableMulticast = false;
      }

      // Write changed value to persistent storage
      nvs_set_u8(g_nvsHandle, "enableMulticast", g_persistent.enableMulticast);
      if (rv != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update multicast enable [%s]", esp_err_to_name(rv));
      }

      // url
      if (ESP_OK == (rv = httpd_query_key_value(buf, "url", param, WEBPAGE_PARAM_SIZE))) {
        char *pdecoded = urlDecode(param);
        if (NULL == pdecoded) {
          free(param);
          free(buf);
          return ESP_ERR_NO_MEM;
        }
        ESP_LOGD(TAG, "Found query parameter => url=%s", pdecoded);
        strncpy(g_persistent.multicastUrl, pdecoded, sizeof(g_persistent.multicastUrl) - 1);
        free(pdecoded);
        // Write changed value to persistent storage
        rv = nvs_set_str(g_nvsHandle, "multicastUrl", g_persistent.multicastUrl);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update multicast address [%s]", esp_err_to_name(rv));
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting multicast URL => rv=%d", rv);
      }

      // port
      if (ESP_OK == (rv = httpd_query_key_value(buf, "port", param, WEBPAGE_PARAM_SIZE))) {
        ESP_LOGD(TAG, "Found query parameter => port=%s", param);
        g_persistent.multicastPort = atoi(param);
        // Write changed value to persistent storage
        rv = nvs_set_u16(g_nvsHandle, "multicastPort", g_persistent.multicastPort);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update multicast port [%s]", esp_err_to_name(rv));
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting multicast port => rv=%d", rv);
      }

      // ttl
      if (ESP_OK == (rv = httpd_query_key_value(buf, "ttl", param, WEBPAGE_PARAM_SIZE))) {
        ESP_LOGD(TAG, "Found query parameter => ttl=%s", param);
        g_persistent.multicastTtl = atoi(param);
        // Write changed value to persistent storage
        rv = nvs_set_u8(g_nvsHandle, "multicastTtl", g_persistent.multicastTtl);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update multicast ttl [%s]", esp_err_to_name(rv));
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting multicast ttl => rv=%d", rv);
      }

      // EnableEncryption
      if (ESP_OK == (rv = httpd_query_key_value(buf, "bMcastEncrypt", param, WEBPAGE_PARAM_SIZE))) {
        ESP_LOGD(TAG, "Found query parameter => bMcastEncrypt=%s", param);
        if (NULL != strstr(param, "true")) {
          g_persistent.bMcastEncrypt = true;
        }
      }
      else {
        g_persistent.bMcastEncrypt = false;
      }
      // Write changed value to persistent storage
      nvs_set_u8(g_nvsHandle, "bMcastEncrypt", g_persistent.bMcastEncrypt);
      if (rv != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update multicast encryption enable [%s]", esp_err_to_name(rv));
      }

      // bMcastHbeat
      if (ESP_OK == (rv = httpd_query_key_value(buf, "bMcastHbeat", param, WEBPAGE_PARAM_SIZE))) {
        ESP_LOGD(TAG, "Found query parameter => bMcastHbeat=%s", param);
        if (NULL != strstr(param, "true")) {
          g_persistent.bHeartbeat = true;
        }
      }
      else {
        g_persistent.bHeartbeat = false;
      }
      // Write changed value to persistent storage
      nvs_set_u8(g_nvsHandle, "bMcastHbeat", g_persistent.bHeartbeat);
      if (rv != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update multicast heartbeat enable [%s]", esp_err_to_name(rv));
      }

      rv = nvs_commit(g_nvsHandle);
      if (rv != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit updates to nvs\n");
      }

      free(param);
    }

    free(buf);
  }
  const char *resp_str =
    "<html><head><meta charset='utf-8'><meta http-equiv=\"refresh\" content=\"1;url=cfgmulticast\" "
    "/><style>" WEBPAGE_STYLE_CSS "</style></head><body><h2 class=\"name\">saving module data...</h2></body></html>";
  httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

  return ESP_OK;
}

// ----------------------------------------------------------------------------

///////////////////////////////////////////////////////////////////////////////
// config_udp_get_handler
//

static esp_err_t
config_udp_get_handler(httpd_req_t *req)
{
  // esp_err_t rv;
  char *buf;
  // char *temp;

  char *req_buf;
  size_t req_buf_len;

  buf = (char *) calloc(CHUNK_BUFSIZE, 1);
  if (NULL == buf) {
    return ESP_ERR_NO_MEM;
  }

  // Get application info data
  const esp_app_desc_t *appDescr = esp_app_get_description();

  // Get header value string length and allocate memory for length + 1,
  // extra byte for null termination
  req_buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
  if (req_buf_len > 1) {
    req_buf = (char *) malloc(req_buf_len);
    // Copy null terminated value string into buffer
    if (httpd_req_get_hdr_value_str(req, "Host", req_buf, req_buf_len) == ESP_OK) {
      ESP_LOGD(TAG, "Found header => Host: %s", req_buf);
    }
    free(req_buf);
  }

  sprintf(buf, WEBPAGE_START_TEMPLATE, g_persistent.nodeName, "UDP Configuration");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<div><form id=but3 class=\"button\" action='/docfgudp' method='get'><fieldset>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // EnableUdpRx
  sprintf(buf, "<input type=\"checkbox\" name=\"enablerx\" value=\"true\" ");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
    sprintf(buf,
      "%s><label for=\"lr\"> Enable RX</label><br>",
      g_persistent.enableUdpRx ? "checked" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // EnableUdpTx
  sprintf(buf, "<input type=\"checkbox\" name=\"enabletx\" value=\"true\" ");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
    sprintf(buf,
      "%s><label for=\"lr\"> Enable TX</label><br><br>",
      g_persistent.enableUdpTx ? "checked" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(
    buf,
    "Host <small style=\"display:inline;margin:0\"> (IPv4, hostname or udp://host)</small>:<input type=\"text\" name=\"url\" value=\"%s\" >",
    g_persistent.udpUrl);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

    sprintf(buf,
      "Port <small style=\"display:inline;margin:0\"> (default:33333)</small>:<input type=\"text\" name=\"port\" value=\"%d\" >",
      g_persistent.udpPort);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // bUdpEncrypt
  sprintf(buf, "<input type=\"checkbox\" name=\"enableencryption\" value=\"true\" ");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
  sprintf(buf, "%s><label for=\"lr\"> Enable Encryption</label>", g_persistent.bUdpEncrypt ? "checked" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<button class=\"bgrn bgrn:hover\">Save</button></fieldset></form></div>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, WEBPAGE_CONFIG_END_TEMPLATE, web_ui_version(appDescr), g_persistent.nodeName);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  httpd_resp_send_chunk(req, NULL, 0);

  free(buf);

  return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////
// do_config_udp_get_handler
//

static esp_err_t
do_config_udp_get_handler(httpd_req_t *req)
{
  esp_err_t rv;
  char *buf;
  size_t buf_len;

  // Read URL query string length and allocate memory for length + 1,
  // extra byte for null termination
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = malloc(buf_len);
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {

      ESP_LOGD(TAG, "Found URL query => %s", buf);
      char *param = (char *) malloc(WEBPAGE_PARAM_SIZE);
      if (NULL == param) {
        return ESP_ERR_NO_MEM;
        free(param);
      }

      // EnableRx
      if (ESP_OK == (rv = httpd_query_key_value(buf, "enablerx", param, WEBPAGE_PARAM_SIZE))) {
        ESP_LOGD(TAG, "Found query parameter => enablerx=%s", param);
        if (NULL != strstr(param, "true")) {
          g_persistent.enableUdpRx = true;
        }
      }
      else {
        g_persistent.enableUdpRx = false;
      }

      // Write changed value to persistent storage
      nvs_set_u8(g_nvsHandle, "enableUdpRx", g_persistent.enableUdpRx);
      if (rv != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update UDP rx enable [%s]", esp_err_to_name(rv));
      }

      // EnableTx
      if (ESP_OK == (rv = httpd_query_key_value(buf, "enabletx", param, WEBPAGE_PARAM_SIZE))) {
        ESP_LOGD(TAG, "Found query parameter => enabletx=%s", param);
        if (NULL != strstr(param, "true")) {
          g_persistent.enableUdpTx = true;
        }
      }
      else {
        g_persistent.enableUdpTx = false;
      }

      // Write changed value to persistent storage
      nvs_set_u8(g_nvsHandle, "enableUdpTx", g_persistent.enableUdpTx);
      if (rv != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update UDP tx enable [%s]", esp_err_to_name(rv));
      }

      // url
      if (ESP_OK == (rv = httpd_query_key_value(buf, "url", param, WEBPAGE_PARAM_SIZE))) {
        char *pdecoded = urlDecode(param);
        if (NULL == pdecoded) {
          free(param);
          free(buf);
          return ESP_ERR_NO_MEM;
        }
        ESP_LOGD(TAG, "Found query parameter => url=%s", pdecoded);
        strncpy(g_persistent.udpUrl, pdecoded, sizeof(g_persistent.udpUrl) - 1);
        free(pdecoded);
        // Write changed value to persistent storage
        rv = nvs_set_str(g_nvsHandle, "udpUrl", g_persistent.udpUrl);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update UDP url[%s]", esp_err_to_name(rv));
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting MQTT URL => rv=%d", rv);
      }

      // port
      if (ESP_OK == (rv = httpd_query_key_value(buf, "port", param, WEBPAGE_PARAM_SIZE))) {
        ESP_LOGD(TAG, "Found query parameter => port=%s", param);
        g_persistent.udpPort = atoi(param);
        // Write changed value to persistent storage
        rv = nvs_set_u16(g_nvsHandle, "udpPort", g_persistent.udpPort);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update UDP port [%s]", esp_err_to_name(rv));
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting MQTT port => rv=%d", rv);
      }

      // EnableEncryption
      if (ESP_OK == (rv = httpd_query_key_value(buf, "enableencryption", param, WEBPAGE_PARAM_SIZE))) {
        ESP_LOGD(TAG, "Found query parameter => enableencryption=%s", param);
        if (NULL != strstr(param, "true")) {
          g_persistent.bUdpEncrypt = true;
        }
      }
      else {
        g_persistent.bUdpEncrypt = false;
      }
      // Write changed value to persistent storage
      nvs_set_u8(g_nvsHandle, "bUdpEncrypt", g_persistent.bUdpEncrypt);
      if (rv != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update UDP encryption enable [%s]", esp_err_to_name(rv));
      }

      rv = nvs_commit(g_nvsHandle);
      if (rv != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit updates to nvs\n");
      }

      free(param);
    }

    free(buf);
  }
  const char *resp_str =
    "<html><head><meta charset='utf-8'><meta http-equiv=\"refresh\" content=\"1;url=cfgudp\" "
    "/><style>" WEBPAGE_STYLE_CSS "</style></head><body><h2 class=\"name\">saving module data...</h2></body></html>";
  httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

  return ESP_OK;
}

// ----------------------------------------------------------------------------

///////////////////////////////////////////////////////////////////////////////
// config_websockets_get_handler
//

static esp_err_t
config_websockets_get_handler(httpd_req_t *req)
{
  // esp_err_t rv;
  char *buf;
  // char *temp;

  char *req_buf;
  size_t req_buf_len;

  buf = (char *) calloc(CHUNK_BUFSIZE, 1);
  if (NULL == buf) {
    return ESP_ERR_NO_MEM;
  }

  // Get application info data
  const esp_app_desc_t *appDescr = esp_app_get_description();

  // Get header value string length and allocate memory for length + 1,
  // extra byte for null termination
  req_buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
  if (req_buf_len > 1) {
    req_buf = (char *) malloc(req_buf_len);
    // Copy null terminated value string into buffer
    if (httpd_req_get_hdr_value_str(req, "Host", req_buf, req_buf_len) == ESP_OK) {
      ESP_LOGD(TAG, "Found header => Host: %s", req_buf);
    }
    free(req_buf);
  }

  sprintf(buf, WEBPAGE_START_TEMPLATE, g_persistent.nodeName, "Websockets Configuration");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<div><form id=but3 class=\"button\" action='/docfgwebsockets' method='get'><fieldset>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // Enable
  sprintf(buf, "<input type=\"checkbox\" name=\"enable\" value=\"true\" ");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
  sprintf(buf, "%s><label for=\"lr\"> Enable</label>", g_persistent.enableWebsock ? "checked" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<br>Port:<input type=\"text\" name=\"port\" value=\"%d\" >", g_persistent.websockPort);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "Username:<input type=\"text\" name=\"user\" value=\"%s\" >", g_persistent.websockUser);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "Password:<input type=\"password\" name=\"password\" value=\"%s\" >", g_persistent.websockPw);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<button class=\"bgrn bgrn:hover\">Save</button></fieldset></form></div>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, WEBPAGE_CONFIG_END_TEMPLATE, web_ui_version(appDescr), g_persistent.nodeName);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  httpd_resp_send_chunk(req, NULL, 0);

  free(buf);

  return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////
// do_config_websockets_get_handler
//

static esp_err_t
do_config_websockets_get_handler(httpd_req_t *req)
{
  esp_err_t rv;
  char *buf;
  size_t buf_len;

  // Read URL query string length and allocate memory for length + 1,
  // extra byte for null termination
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = malloc(buf_len);
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {

      ESP_LOGD(TAG, "Found URL query => %s", buf);
      char *param = (char *) malloc(WEBPAGE_PARAM_SIZE);
      if (NULL == param) {
        return ESP_ERR_NO_MEM;
        free(param);
      }

      // Enable
      if (ESP_OK == (rv = httpd_query_key_value(buf, "enable", param, WEBPAGE_PARAM_SIZE))) {
        ESP_LOGD(TAG, "Found query parameter => enable=%s", param);
        if (NULL != strstr(param, "true")) {
          g_persistent.enableWebsock = true;
        }
      }
      else {
        g_persistent.enableWebsock = false;
      }

      // Write changed value to persistent storage
      nvs_set_u8(g_nvsHandle, "enableWebsock", g_persistent.enableWebsock);
      if (rv != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update Websockets enable [%s]", esp_err_to_name(rv));
      }

      // port
      if (ESP_OK == (rv = httpd_query_key_value(buf, "port", param, WEBPAGE_PARAM_SIZE))) {
        ESP_LOGD(TAG, "Found query parameter => port=%s", param);
        g_persistent.websockPort = atoi(param);
        // Write changed value to persistent storage
        rv = nvs_set_u16(g_nvsHandle, "websockPort", g_persistent.websockPort);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update Websockets port [%s]", esp_err_to_name(rv));
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting Websockets port => rv=%d", rv);
      }

      // user
      if (ESP_OK == (rv = httpd_query_key_value(buf, "user", param, WEBPAGE_PARAM_SIZE))) {
        char *pdecoded = urlDecode(param);
        if (NULL == pdecoded) {
          free(param);
          free(buf);
          return ESP_ERR_NO_MEM;
        }
        ESP_LOGD(TAG, "Found query parameter => user=%s", pdecoded);
        strncpy(g_persistent.websockUser, pdecoded, sizeof(g_persistent.websockUser) - 1);
        // Write changed value to persistent storage
        rv = nvs_set_str(g_nvsHandle, "websockUser", g_persistent.websockUser);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update Websockets user[%s]", esp_err_to_name(rv));
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting Websockets user => rv=%d", rv);
      }

      // password
      if (ESP_OK == (rv = httpd_query_key_value(buf, "password", param, WEBPAGE_PARAM_SIZE))) {
        char *pdecoded = urlDecode(param);
        if (NULL == pdecoded) {
          free(param);
          free(buf);
          return ESP_ERR_NO_MEM;
        }
        ESP_LOGD(TAG, "Found query parameter => password=%s", pdecoded);
        strncpy(g_persistent.websockPw, pdecoded, sizeof(g_persistent.websockPw) - 1);
        // Write changed value to persistent storage
        rv = nvs_set_str(g_nvsHandle, "websockPw", g_persistent.websockPw);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update Websockets password [%s]", esp_err_to_name(rv));
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting Websockets password => rv=%d", rv);
      }

      rv = nvs_commit(g_nvsHandle);
      if (rv != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit updates to nvs\n");
      }

      free(param);
    }

    free(buf);
  }
  const char *resp_str =
    "<html><head><meta charset='utf-8'><meta http-equiv=\"refresh\" content=\"1;url=cfgwebsockets\" "
    "/><style>" WEBPAGE_STYLE_CSS "</style></head><body><h2 class=\"name\">saving module data...</h2></body></html>";
  httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

  return ESP_OK;
}

// ----------------------------------------------------------------------------

///////////////////////////////////////////////////////////////////////////////
// config_web_get_handler
//

static esp_err_t
config_web_get_handler(httpd_req_t *req)
{
  // esp_err_t rv;
  char *buf;
  // char *temp;

  char *req_buf;
  size_t req_buf_len;

  buf = (char *) calloc(CHUNK_BUFSIZE, 1);
  if (NULL == buf) {
    return ESP_ERR_NO_MEM;
  }

  // Get application info data
  const esp_app_desc_t *appDescr = esp_app_get_description();

  // Get header value string length and allocate memory for length + 1,
  // extra byte for null termination
  req_buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
  if (req_buf_len > 1) {
    req_buf = (char *) malloc(req_buf_len);
    // Copy null terminated value string into buffer
    if (httpd_req_get_hdr_value_str(req, "Host", req_buf, req_buf_len) == ESP_OK) {
      ESP_LOGD(TAG, "Found header => Host: %s", req_buf);
    }
    free(req_buf);
  }

  sprintf(buf, WEBPAGE_START_TEMPLATE, g_persistent.nodeName, "Module web interface");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<div><form id=but3 class=\"button\" action='/docfgweb' method='get'><fieldset>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "Port:<input type=\"text\" name=\"port\" value=\"%d\" >", g_persistent.webPort);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "Username:<input type=\"text\" name=\"user\" value=\"%s\" >", g_persistent.webUser);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "Password:<input type=\"password\" name=\"password\" value=\"%s\" >", g_persistent.webPassword);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<button class=\"bgrn bgrn:hover\">Save</button></fieldset></form></div>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, WEBPAGE_CONFIG_END_TEMPLATE, web_ui_version(appDescr), g_persistent.nodeName);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  httpd_resp_send_chunk(req, NULL, 0);

  free(buf);

  return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////
// do_config_web_get_handler
//

static esp_err_t
do_config_web_get_handler(httpd_req_t *req)
{
  esp_err_t rv;
  char *buf;
  size_t buf_len;

  // Read URL query string length and allocate memory for length + 1,
  // extra byte for null termination
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char *) malloc(buf_len);
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {

      ESP_LOGD(TAG, "Found URL query => %s", buf);
      char *param = malloc(WEBPAGE_PARAM_SIZE);
      if (NULL == param) {
        return ESP_ERR_NO_MEM;
        free(param);
      }

      // port
      if (ESP_OK == (rv = httpd_query_key_value(buf, "port", param, WEBPAGE_PARAM_SIZE))) {
        ESP_LOGD(TAG, "Found query parameter => port=%s", param);
        g_persistent.webPort = atoi(param);
        // Write changed value to persistent storage
        rv = nvs_set_u16(g_nvsHandle, "webPort", g_persistent.webPort);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update Web interface port");
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting Web interface port => rv=%d", rv);
      }

      // user
      if (ESP_OK == (rv = httpd_query_key_value(buf, "user", param, WEBPAGE_PARAM_SIZE))) {
        char *pdecoded = urlDecode(param);
        if (NULL == pdecoded) {
          free(param);
          free(buf);
          return ESP_ERR_NO_MEM;
        }
        ESP_LOGD(TAG, "Found query parameter => user=%s", pdecoded);
        strncpy(g_persistent.webUser, pdecoded, sizeof(g_persistent.webUser) - 1);
        // Write changed value to persistent storage
        rv = nvs_set_str(g_nvsHandle, "webUser", g_persistent.webUser);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update Web interface user");
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting Web interface user => rv=%d", rv);
      }

      // password
      if (ESP_OK == (rv = httpd_query_key_value(buf, "password", param, WEBPAGE_PARAM_SIZE))) {
        char *pdecoded = urlDecode(param);
        if (NULL == pdecoded) {
          free(param);
          free(buf);
          return ESP_ERR_NO_MEM;
        }
        ESP_LOGD(TAG, "Found query parameter => password=%s", pdecoded);
        strncpy(g_persistent.webPassword, pdecoded, sizeof(g_persistent.webPassword) - 1);
        // Write changed value to persistent storage
        rv = nvs_set_str(g_nvsHandle, "webPassword", g_persistent.webPassword);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update Web interface password");
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting Web interface password => rv=%d", rv);
      }

      rv = nvs_commit(g_nvsHandle);
      if (rv != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit updates to nvs\n");
      }

      free(param);
    }

    free(buf);
  }
  const char *resp_str =
    "<html><head><meta charset='utf-8'><meta http-equiv=\"refresh\" content=\"1;url=cfgweb\" "
    "/><style>" WEBPAGE_STYLE_CSS "</style></head><body><h2 class=\"name\">saving module data...</h2></body></html>";
  httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

  return ESP_OK;
}

// ----------------------------------------------------------------------------

///////////////////////////////////////////////////////////////////////////////
// config_log_get_handler
//

static esp_err_t
config_log_get_handler(httpd_req_t *req)
{
  // esp_err_t rv;
  char *buf;
  // char *temp;
  uint8_t uiLogType;

  char *req_buf;
  size_t req_buf_len;

  buf = (char *) calloc(CHUNK_BUFSIZE, 1);
  if (NULL == buf) {
    return ESP_ERR_NO_MEM;
  }

  // Get application info data
  const esp_app_desc_t *appDescr = esp_app_get_description();

  // Get header value string length and allocate memory for length + 1,
  // extra byte for null termination
  req_buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
  if (req_buf_len > 1) {
    req_buf = (char *) malloc(req_buf_len);
    // Copy null terminated value string into buffer
    if (httpd_req_get_hdr_value_str(req, "Host", req_buf, req_buf_len) == ESP_OK) {
      ESP_LOGD(TAG, "Found header => Host: %s", req_buf);
    }
    free(req_buf);
  }

  sprintf(buf, WEBPAGE_START_TEMPLATE, g_persistent.nodeName, "Logging Configuration");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  uiLogType = g_persistent.logType;
  if ((LOG_TYPE_STD != uiLogType) && (LOG_TYPE_UDP != uiLogType) && (LOG_TYPE_MQTT != uiLogType)) {
    uiLogType = LOG_TYPE_STD;
  }

  sprintf(buf, "<div><form id=but3 class=\"button\" action='/docfglog' method='get'><fieldset>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<br /><br />Log to:<select name=\"type\">");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
    sprintf(buf, "<option value=\"1\" %s>default</option>", (LOG_TYPE_STD == uiLogType) ? "selected" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
    sprintf(buf, "<option value=\"2\" %s>UDP</option>", (LOG_TYPE_UDP == uiLogType) ? "selected" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
  sprintf(buf, "<option value=\"5\" %s>MQTT</option>", 
      (LOG_TYPE_MQTT == uiLogType) ? "selected" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
  sprintf(buf, "</select>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "Log level:<select name=\"level\">");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
  sprintf(buf, "<option value=\"1\" %s>error</option>", (ESP_LOG_ERROR == g_persistent.logLevel) ? "selected" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
  sprintf(buf, "<option value=\"2\" %s>warning</option>", (ESP_LOG_WARN == g_persistent.logLevel) ? "selected" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
  sprintf(buf, "<option value=\"3\" %s>info</option>", (ESP_LOG_INFO == g_persistent.logLevel) ? "selected" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
  sprintf(buf, "<option value=\"4\" %s>  debug</option>", (ESP_LOG_DEBUG == g_persistent.logLevel) ? "selected" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
  sprintf(buf, "<option value=\"5\" %s>verbose</option>", (ESP_LOG_VERBOSE == g_persistent.logLevel) ? "selected" : "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
  sprintf(buf, "</select>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "Destination (IP Addr):<input type=\"text\" name=\"url\" value=\"%s\" >", g_persistent.logUrl);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "Port:<input type=\"text\" name=\"port\" value=\"%d\" >", g_persistent.logPort);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "MQTT log Topic:<input type=\"text\" name=\"topic\" value=\"%s\" >", g_persistent.logMqttTopic);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // Show warnings if MQTT is not properly configured
  if (!g_persistent.enableMqtt) {
    sprintf(buf, "<br /><small style=\"color:#ff9800;\">⚠ MQTT is not enabled. Please enable MQTT in the MQTT configuration.</small>");
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
  }
  else if (!strlen(g_persistent.logMqttTopic)) {
    sprintf(buf, "<br /><small style=\"color:#ff9800;\">⚠ MQTT log topic is not configured. Please set a topic for MQTT logging.</small>");
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
  }

  sprintf(buf, "<br /><button class=\"bgrn bgrn:hover\">Save</button></fieldset></form></div>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

    sprintf(buf,
      "<script>"
      "(function(){"
      "var typeSel=document.querySelector('select[name=\\\"type\\\"]');"
      "if(!typeSel)return;"
      "function sync(){"
      "var isDefault=(typeSel.value==='1');"
      "var isUdp=(typeSel.value==='2');"
      "var urlEl=document.querySelector('[name=\\\"url\\\"]');"
      "var portEl=document.querySelector('[name=\\\"port\\\"]');"
      "var topicEl=document.querySelector('[name=\\\"topic\\\"]');"
      "if(urlEl){urlEl.disabled=isDefault;}"
      "if(portEl){portEl.disabled=isDefault;}"
      "if(topicEl){topicEl.disabled=(isDefault||isUdp);}"
      "}"
      "typeSel.addEventListener('change',sync);"
      "sync();"
      "})();"
      "</script>");
    httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, WEBPAGE_CONFIG_END_TEMPLATE, web_ui_version(appDescr), g_persistent.nodeName);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  httpd_resp_send_chunk(req, NULL, 0);

  free(buf);

  return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////
// do_config_log_get_handler
//

static esp_err_t
do_config_log_get_handler(httpd_req_t *req)
{
  esp_err_t rv;
  char *buf;
  size_t buf_len;

  // Read URL query string length and allocate memory for length + 1,
  // extra byte for null termination
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char *) malloc(buf_len);
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {

      ESP_LOGD(TAG, "Found URL query => %s", buf);
      char *param = (char *) malloc(WEBPAGE_PARAM_SIZE);
      if (NULL == param) {
        return ESP_ERR_NO_MEM;
        free(param);
      }

      // type
      if (ESP_OK == (rv = httpd_query_key_value(buf, "type", param, WEBPAGE_PARAM_SIZE))) {
        int type;
        ESP_LOGD(TAG, "Found query parameter => type=%s", param);
        type = atoi(param);
        if ((LOG_TYPE_STD == type) || (LOG_TYPE_UDP == type) || (LOG_TYPE_MQTT == type)) {
          g_persistent.logType = type;
        }
        else {
          g_persistent.logType = LOG_TYPE_STD;
        }

        rv = nvs_set_u8(g_nvsHandle, "logType", g_persistent.logType);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update log type");
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting log type => rv=%d", rv);
      }

      // level
      if (ESP_OK == (rv = httpd_query_key_value(buf, "level", param, WEBPAGE_PARAM_SIZE))) {
        ESP_LOGD(TAG, "Found query parameter => level=%s", param);
        g_persistent.logLevel = atoi(param);

        rv = nvs_set_u8(g_nvsHandle, "log_level", g_persistent.logLevel);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update log level");
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting log level => rv=%d", rv);
      }

      // port
      if (ESP_OK == (rv = httpd_query_key_value(buf, "port", param, WEBPAGE_PARAM_SIZE))) {
        ESP_LOGD(TAG, "Found query parameter => port=%s", param);
        g_persistent.logPort = atoi(param);

        rv = nvs_set_u16(g_nvsHandle, "logPort", g_persistent.logPort);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to update log port [%s]", esp_err_to_name(rv));
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting log port => rv=%d", rv);
      }

      // url
      if (ESP_OK == (rv = httpd_query_key_value(buf, "url", param, WEBPAGE_PARAM_SIZE))) {
        ESP_LOGD(TAG, "Found query parameter => url=%s", param);
        strncpy(g_persistent.logUrl, param, sizeof(g_persistent.logUrl));

        rv = nvs_set_str(g_nvsHandle, "logUrl", g_persistent.logUrl);
        if (rv == ESP_OK) {
          // Keep legacy key in sync for backward compatibility.
          rv = nvs_set_str(g_nvsHandle, "log_url", g_persistent.logUrl);
        }
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to save log URL");
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting log port => rv=%d", rv);
      }

      // MQTT topic
      if (ESP_OK == (rv = httpd_query_key_value(buf, "topic", param, WEBPAGE_PARAM_SIZE))) {
        char *pdecoded = urlDecode(param);
        if (NULL == pdecoded) {
          free(param);
          free(buf);
          return ESP_ERR_NO_MEM;
        }
        ESP_LOGD(TAG, "Found query parameter => topic=%s", pdecoded);
        strncpy(g_persistent.logMqttTopic, pdecoded, sizeof(g_persistent.logMqttTopic));
        free(pdecoded);

        rv = nvs_set_str(g_nvsHandle, "logMqttTopic", g_persistent.logMqttTopic);
        if (rv != ESP_OK) {
          ESP_LOGE(TAG, "Failed to save logMqtt topic");
        }
      }
      else {
        ESP_LOGE(TAG, "Error getting log topic => rv=%d", rv);
      }

      // MQTT logging follows selected logging type
      g_persistent.enableMqttLog = (LOG_TYPE_MQTT == g_persistent.logType) ? 1 : 0;
      rv = nvs_set_u8(g_nvsHandle, "enableMqttLog", g_persistent.enableMqttLog);
      if (rv != ESP_OK) {
        ESP_LOGE(TAG, "Failed to update MQTT logging enable");
      }

      free(param);
    }

    free(buf);
  }
  const char *resp_str =
    "<html><head><meta charset='utf-8'><meta http-equiv=\"refresh\" content=\"1;url=cfglog\" "
    "/><style>" WEBPAGE_STYLE_CSS "</style></head><body><h2 class=\"name\">saving module data...</h2></body></html>";
  httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

  return ESP_OK;
}

//---------------------------------------------------------------------------

///////////////////////////////////////////////////////////////////////////////
// config_factory_defaults_get_handler
//

static esp_err_t
config_factory_defaults_get_handler(httpd_req_t *req)
{
  // esp_err_t rv;
  char *buf;
  // char *temp;

  char *req_buf;
  size_t req_buf_len;

  buf = (char *) calloc(CHUNK_BUFSIZE, 1);
  if (NULL == buf) {
    return ESP_ERR_NO_MEM;
  }

  // Get application info data
  const esp_app_desc_t *appDescr = esp_app_get_description();

  // Get header value string length and allocate memory for length + 1,
  // extra byte for null termination
  req_buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
  if (req_buf_len > 1) {
    req_buf = (char *) malloc(req_buf_len);
    // Copy null terminated value string into buffer
    if (httpd_req_get_hdr_value_str(req, "Host", req_buf, req_buf_len) == ESP_OK) {
      ESP_LOGD(TAG, "Found header => Host: %s", req_buf);
    }
    free(req_buf);
  }

  sprintf(buf, WEBPAGE_START_TEMPLATE, g_persistent.nodeName, "Factory Defaults");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

    sprintf(buf,
      "<div><form id=but3 class=\"button\" action='/docfgfactorydefaults' method='get' "
      "onsubmit=\"return confirm('Restore factory defaults and restart the device?');\"><fieldset>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  // Enable
  sprintf(buf, "<input type=\"checkbox\" name=\"yesimsure\" value=\"true\" ");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);
  sprintf(buf, "%s><label for=\"lr\"> Yes I am sure</label>", "");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, "<button class=\"bred bred:hover\">Restore factory defaults</button></fieldset></form></div>");
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  sprintf(buf, WEBPAGE_CONFIG_END_TEMPLATE, web_ui_version(appDescr), g_persistent.nodeName);
  httpd_resp_send_chunk(req, buf, HTTPD_RESP_USE_STRLEN);

  httpd_resp_send_chunk(req, NULL, 0);

  free(buf);

  return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////
// do_config_factory_defaults_get_handler
//

static esp_err_t
do_config_factory_defaults_get_handler(httpd_req_t *req)
{
  esp_err_t rv;
  char *buf;
  size_t buf_len;

  ESP_LOGE(TAG, "HANDLER => do_config_factory_defaults_get_handler");
  // Read URL query string length and allocate memory for length + 1,
  // extra byte for null termination
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char *) malloc(buf_len);
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {

      ESP_LOGW(TAG, "Found URL query => %s", buf);
      char *param = malloc(WEBPAGE_PARAM_SIZE);
      if (NULL == param) {
        free(param);
        free(buf);
        return ESP_ERR_NO_MEM;
      }

      // Yes-I'm-Sure
      if (ESP_OK == (rv = httpd_query_key_value(buf, "yesimsure", param, WEBPAGE_PARAM_SIZE))) {
        ESP_LOGW(TAG, "Found query parameter => yesimsure=%s", param);
        if (NULL != strstr(param, "true")) {
          ESP_LOGW(TAG, "Restoring factory defaults...");
          esp_err_t erase_rv = nvs_erase_all(g_nvsHandle);
          esp_err_t commit_rv = nvs_commit(g_nvsHandle);
          if ((ESP_OK == erase_rv) && (ESP_OK == commit_rv)) {
            send_restart_info_page(req, "Restoring factory defaults. The device is restarting...");
            vTaskDelay(pdMS_TO_TICKS(800));
            esp_restart();
            free(param);
            free(buf);
            return ESP_OK;
          }

          ESP_LOGE(TAG,
                   "Failed to restore factory defaults erase=%s commit=%s",
                   esp_err_to_name(erase_rv),
                   esp_err_to_name(commit_rv));
        }
      }

      free(param);
    }
    free(buf);
  }

  ESP_LOGW(TAG, "NOT restoring factory defaults...");
  const char *resp_str = "<html><head><meta charset='utf-8'><meta http-equiv=\"refresh\" content=\"1;url=config\" "
                         "/><style>" WEBPAGE_STYLE_CSS
                         "</style></head><body><h2 class=\"name\">No factory default. Returning...</h2></body></html>";
  httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

  return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////
// echo_post_handler
//
// An HTTP POST handler
//

static esp_err_t
echo_post_handler(httpd_req_t *req)
{
  char buf[100];
  int ret, remaining = req->content_len;

  while (remaining > 0) {
    // Read the data for the request
    if ((ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)))) <= 0) {
      if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
        // Retry receiving if timeout occurred
        continue;
      }
      return ESP_FAIL;
    }

    // Send back the same data
    httpd_resp_send_chunk(req, buf, ret);
    remaining -= ret;

    // Log data received
    ESP_LOGD(TAG, "=========== RECEIVED DATA ==========");
    ESP_LOGD(TAG, "%.*s", ret, buf);
    ESP_LOGD(TAG, "====================================");
  }

  // End response
  httpd_resp_send_chunk(req, NULL, 0);
  return ESP_OK;
}

static const httpd_uri_t echo = { .uri = "/echo", .method = HTTP_POST, .handler = echo_post_handler, .user_ctx = NULL };

static const httpd_uri_t cfgrestorepost = {
  .uri      = "/docfgrestore",
  .method   = HTTP_POST,
  .handler  = do_config_restore_post_handler,
  .user_ctx = NULL
};

///////////////////////////////////////////////////////////////////////////////
// http_404_error_handler
//
// This handler allows the custom error handling functionality to be
// tested from client side. For that, when a PUT request 0 is sent to
// URI /ctrl, the /hello and /echo URIs are unregistered and following
// custom error handler http_404_error_handler() is registered.
// Afterwards, when /hello or /echo is requested, this custom error
// handler is invoked which, after sending an error message to client,
// either closes the underlying socket (when requested URI is /echo)
// or keeps it open (when requested URI is /hello). This allows the
// client to infer if the custom error handler is functioning as expected
// by observing the socket state.
//

esp_err_t
http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
  if (strcmp("/hello", req->uri) == 0) {
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/hello URI is not available");
    // Return ESP_OK to keep underlying socket open
    return ESP_OK;
  }
  else if (strcmp("/echo", req->uri) == 0) {
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/echo URI is not available");
    // Return ESP_FAIL to close underlying socket
    return ESP_FAIL;
  }
  // For any other URI send 404 and close socket
  httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Some 404 error message");
  return ESP_FAIL;
}

///////////////////////////////////////////////////////////////////////////////
// ctrl_put_handler
//
// An HTTP PUT handler. This demonstrates realtime
// registration and deregistration of URI handlers
//

static esp_err_t __attribute__((unused))
ctrl_put_handler(httpd_req_t *req)
{
  char buf;
  int ret;

  if ((ret = httpd_req_recv(req, &buf, 1)) <= 0) {
    if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
      httpd_resp_send_408(req);
    }
    return ESP_FAIL;
  }

  if (buf == '0') {
    // URI handlers can be unregistered using the uri string
    ESP_LOGD(TAG, "Unregistering /hello and /echo URIs");
    httpd_unregister_uri(req->handle, "/hello");
    httpd_unregister_uri(req->handle, "/echo");
    // Register the custom error handler
    httpd_register_err_handler(req->handle, HTTPD_404_NOT_FOUND, http_404_error_handler);
  }
  else {
    ESP_LOGD(TAG, "Registering /hello and /echo URIs");
    httpd_register_uri_handler(req->handle, &hello);
    httpd_register_uri_handler(req->handle, &echo);
    // Unregister custom error handler
    httpd_register_err_handler(req->handle, HTTPD_404_NOT_FOUND, NULL);
  }

  // Respond with empty body
  httpd_resp_send(req, NULL, 0);
  return ESP_OK;
}

// static const httpd_uri_t ctrl = { .uri = "/ctrl", .method = HTTP_PUT, .handler = ctrl_put_handler, .user_ctx =
// NULL
// };

///////////////////////////////////////////////////////////////////////////////
// set_content_type_from_file
//
// Set HTTP response content type according to file extension
//

static esp_err_t
set_content_type_from_file(httpd_req_t *req, const char *filename)
{
  if (IS_FILE_EXT(filename, ".gz")) {
    return httpd_resp_set_type(req, "application/gzip");
  }
  else if (IS_FILE_EXT(filename, ".html")) {
    return httpd_resp_set_type(req, "text/html");
  }
  else if (IS_FILE_EXT(filename, ".css")) {
    return httpd_resp_set_type(req, "text/css");
  }
  else if (IS_FILE_EXT(filename, ".jpeg")) {
    return httpd_resp_set_type(req, "image/jpeg");
  }
  else if (IS_FILE_EXT(filename, ".png")) {
    return httpd_resp_set_type(req, "image/png");
  }
  else if (IS_FILE_EXT(filename, ".ico")) {
    return httpd_resp_set_type(req, "image/x-icon");
  }
  else if (IS_FILE_EXT(filename, ".js")) {
    return httpd_resp_set_type(req, "text/javascript");
  }
  // For any other type always set as plain text
  return httpd_resp_set_type(req, "text/plain");
}

///////////////////////////////////////////////////////////////////////////////
// default_get_handler
//
// Handler to download a file kept on the server
//

static esp_err_t
default_get_handler(httpd_req_t *req)
{
  char filepath[FILE_PATH_MAX];
  FILE *fd = NULL;
  struct stat file_stat;
  char *buf      = NULL;
  size_t buf_len = 0;

  ESP_LOGD(TAG, "uri : [%s]", req->uri);

  //---------------------------------------------------------------------------

  ESP_LOGD(TAG, "default_get_handler");

  buf_len = httpd_req_get_hdr_value_len(req, "Authorization") + 1;
  if (buf_len > 1) {
    buf = calloc(buf_len, 1);
    if (!buf) {
      ESP_LOGE(TAG, "No enough memory for basic authorization");
      return ESP_ERR_NO_MEM;
    }

    if (httpd_req_get_hdr_value_str(req, "Authorization", buf, buf_len) == ESP_OK) {
      ESP_LOGD(TAG, "Found header => Authorization: %s", buf);
    }
    else {
      ESP_LOGE(TAG, "No auth value received");
    }

    char *auth_credentials = http_auth_basic(DEFAULT_VSCP_LINK_USER, DEFAULT_VSCP_LINK_PASSWORD);
    if (!auth_credentials) {
      ESP_LOGE(TAG, "No enough memory for basic authorization credentials");
      free(buf);
      return ESP_ERR_NO_MEM;
    }

    if (strncmp(auth_credentials, buf, buf_len)) {
      ESP_LOGE(TAG, "Not authenticated");
      httpd_resp_set_status(req, HTTPD_401);
      httpd_resp_set_type(req, "application/json");
      httpd_resp_set_hdr(req, "Connection", "keep-alive");
      httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"Alpha\"");
      httpd_resp_send(req, NULL, 0);
    }
    else {
      ESP_LOGD(TAG, "------> Authenticated!");
      /*char *basic_auth_resp = NULL;
      httpd_resp_set_status(req, HTTPD_200);
      httpd_resp_set_type(req, "application/json");
      httpd_resp_set_hdr(req, "Connection", "keep-alive");
      asprintf(&basic_auth_resp, "{\"authenticated\": true,\"user\": \"%s\"}", basic_auth_info->username);
      if (!basic_auth_resp) {
        ESP_LOGE(TAG, "No enough memory for basic authorization response");
        free(auth_credentials);
        free(buf)
        return ESP_ERR_NO_MEM;
      }
      httpd_resp_send(req, basic_auth_resp, strlen(basic_auth_resp));
      free(basic_auth_resp); */
    }
    free(auth_credentials);
    free(buf);
  }
  else {
    ESP_LOGE(TAG, "No auth header received.");
    httpd_resp_set_status(req, HTTPD_401);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Connection", "keep-alive");
    httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"Alpha\"");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
  }

  // -----------------------------------------------------------------------------

  // if (0 == strncmp(req->uri, "/hello", 6)) {
  //   ESP_LOGV(TAG, "--------- HELLO ---------\n");
  //   return hello_get_handler(req);
  // }

  // if (0 == strncmp(req->uri, "/echo", 5)) {
  //   ESP_LOGV(TAG, "--------- ECHO ---------\n");
  //   return echo_post_handler(req);
  // }

  // if (0 == strncmp(req->uri, "/ctrl", 5)) {
  //   ESP_LOGV(TAG, "--------- CTRL ---------\n");
  //   return ctrl_put_handler(req);
  // }

  if (0 == strncmp(req->uri, "/index.html", 11)) {
    ESP_LOGV(TAG, "--------- index ---------\n");
    return mainpg_get_handler(req);
  }

  if ((0 == strncmp(req->uri, "/", 1)) && (1 == strlen(req->uri))) {
    ESP_LOGV(TAG, "--------- index /---------\n");
    return mainpg_get_handler(req);
  }

  // ---------------------------------------------------------------

  if (0 == strncmp(req->uri, "/config", 7)) {
    ESP_LOGV(TAG, "--------- config ---------\n");
    return config_get_handler(req);
  }

  if (0 == strncmp(req->uri, "/canbus", 7)) {
    ESP_LOGV(TAG, "--------- canbus ---------\n");
    return canbus_get_handler(req);
  }

  if (0 == strncmp(req->uri, "/logstream", 10)) {
    return logstream_get_handler(req);
  }

  if ((0 == strncmp(req->uri, "/logs", 5)) &&
      (('\0' == req->uri[5]) || ('/' == req->uri[5]) || ('?' == req->uri[5]))) {
    ESP_LOGV(TAG, "--------- logs ---------\n");
    return logs_get_handler(req);
  }

  if (0 == strncmp(req->uri, "/canapi", 7)) {
    ESP_LOGV(TAG, "--------- canapi ---------\n");
    return canapi_get_handler(req);
  }

  if (0 == strncmp(req->uri, "/cfgmodule", 10)) {
    ESP_LOGV(TAG, "--------- cfgmodule ---------\n");
    return config_module_get_handler(req);
  }

  if (0 == strncmp(req->uri, "/docfgmodule", 12)) {
    ESP_LOGV(TAG, "--------- docfgmodule ---------\n");
    return do_config_module_get_handler(req);
  }

  if (0 == strncmp(req->uri, "/cfgcan", 7)) {
    ESP_LOGV(TAG, "--------- cfgcan ---------\n");
    return config_can_get_handler(req);
  }

  if (0 == strncmp(req->uri, "/docfgcan", 9)) {
    ESP_LOGV(TAG, "--------- docfgcan ---------\n");
    return do_config_can_get_handler(req);
  }

  if (0 == strncmp(req->uri, "/cfgwifi", 8)) {
    ESP_LOGV(TAG, "--------- cfgwifi ---------\n");
    return config_wifi_get_handler(req);
  }

  if (0 == strncmp(req->uri, "/docfgwifi", 10)) {
    ESP_LOGV(TAG, "--------- docfgwifi ---------\n");
    return do_config_wifi_get_handler(req);
  }

  if (0 == strncmp(req->uri, "/cfgfactorydefaults", 8)) {
    ESP_LOGV(TAG, "--------- docfgfactorydefaults ---------\n");
    return config_factory_defaults_get_handler(req);
  }

  if (0 == strncmp(req->uri, "/cfgbackup", 10)) {
    ESP_LOGV(TAG, "--------- cfgbackup ---------\n");
    return config_backup_get_handler(req);
  }

  if (0 == strncmp(req->uri, "/cfgrestore", 11)) {
    ESP_LOGV(TAG, "--------- cfgrestore ---------\n");
    return config_restore_get_handler(req);
  }

  if (0 == strncmp(req->uri, "/docfgfactorydefaults", 10)) {
    ESP_LOGV(TAG, "--------- docfgfactorydefaults ---------\n");
    return do_config_factory_defaults_get_handler(req);
  }

  if (0 == strncmp(req->uri, "/cfgvscplink", 11)) {
    ESP_LOGV(TAG, "--------- cfgvscplink ---------\n");
    return config_vscplink_get_handler(req);
  }

  if (0 == strncmp(req->uri, "/docfgvscplink", 13)) {
    ESP_LOGV(TAG, "--------- docfgvscplink ---------\n");
    return do_config_vscplink_get_handler(req);
  }

  if (0 == strncmp(req->uri, "/cfgmqtt", 8)) {
    ESP_LOGV(TAG, "--------- cfgmqtt ---------\n");
    return config_mqtt_get_handler(req);
  }

  if (0 == strncmp(req->uri, "/docfgmqtt", 10)) {
    ESP_LOGV(TAG, "--------- docfgmqtt ---------\n");
    return do_config_mqtt_get_handler(req);
  }

  if (0 == strncmp(req->uri, "/cfgwebsockets", 14)) {
    ESP_LOGV(TAG, "--------- cfgwebsockets ---------\n");
    return config_websockets_get_handler(req);
  }

  if (0 == strncmp(req->uri, "/docfgwebsockets", 16)) {
    ESP_LOGV(TAG, "--------- docfgwebsockets ---------\n");
    return do_config_websockets_get_handler(req);
  }

  if (0 == strncmp(req->uri, "/cfgweb", 7)) {
    ESP_LOGV(TAG, "--------- cfgweb ---------\n");
    return config_web_get_handler(req);
  }

  if (0 == strncmp(req->uri, "/docfgweb", 9)) {
    ESP_LOGV(TAG, "--------- docfgweb ---------\n");
    return do_config_web_get_handler(req);
  }

  if (0 == strncmp(req->uri, "/cfglog", 7)) {
    ESP_LOGV(TAG, "--------- cfglog ---------\n");
    return config_log_get_handler(req);
  }

  if (0 == strncmp(req->uri, "/docfglog", 9)) {
    ESP_LOGV(TAG, "--------- docfglog ---------\n");
    return do_config_log_get_handler(req);
  }

  if (0 == strncmp(req->uri, "/cfgmulticast", 13)) {
    ESP_LOGV(TAG, "--------- cfgmulticast ---------\n");
    return config_multicast_get_handler(req);
  }

  if (0 == strncmp(req->uri, "/docfgmulticast", 15)) {
    ESP_LOGV(TAG, "--------- docfgmulticast ---------\n");
    return do_config_multicast_get_handler(req);
  }

  if (0 == strncmp(req->uri, "/cfgudp", 7)) {
    ESP_LOGV(TAG, "--------- cfgudp ---------\n");
    return config_udp_get_handler(req);
  }

  if (0 == strncmp(req->uri, "/docfgudp", 9)) {
    ESP_LOGV(TAG, "--------- docfgudp ---------\n");
    return do_config_udp_get_handler(req);
  }

  // ---------------------------------------------------------------

  if (0 == strncmp(req->uri, "/info", 5)) {
    ESP_LOGV(TAG, "--------- info ---------\n");
    return info_get_handler(req);
  }

  if (0 == strncmp(req->uri, "/reset", 6)) {
    ESP_LOGV(TAG, "--------- reset ---------\n");
    return reset_get_handler(req);
  }

  if (0 == strncmp(req->uri, "/upgrade", 8)) {
    ESP_LOGV(TAG, "--------- Upgrade main ---------\n");
    return upgrade_get_handler(req);
  }

  if (0 == strncmp(req->uri, "/upgrdsrv", 9)) {
    ESP_LOGV(TAG, "--------- Upgrade server ---------\n");
    return upgrdsrv_get_handler(req);
  }

  if (0 == strncmp(req->uri, "/upgrdlocal", 11)) {
    ESP_LOGV(TAG, "--------- Upgrade local ---------\n");
    return upgrdlocal_post_handler(req);
  }

  if (0 == strncmp(req->uri, "/upgrdSibling", 13)) {
    ESP_LOGV(TAG, "--------- Upgrade sibling(s) from server ---------\n");
    return upgrdsibling_get_handler(req);
  }

  if (0 == strncmp(req->uri, "/upgrdSiblingLocal", 18)) {
    ESP_LOGV(TAG, "--------- Upgrade sibling(s) local ---------\n");
    return upgrdSiblingslocal_post_handler(req);
  }

  return ESP_OK;

  // ------------------------------------------------------------------------------------------

  // If name has trailing '/', respond with directory contents
  if (0 == strcmp(req->uri, "/")) {
    ESP_LOGD(TAG, "Set default uri");
    strcpy((char *) req->uri, "/index.html");
  }

  const char *filename = get_path_from_uri(filepath, "/spiffs", req->uri, sizeof(filepath));
  if (!filename) {
    ESP_LOGE(TAG, "Filename is too long");
    // Respond with 500 Internal Server Error
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");
    return ESP_FAIL;
  }

  if (stat(filepath, &file_stat) == -1) {
    // If file not present on SPIFFS check if URI
    // corresponds to one of the hardcoded paths
    if (strcmp(filename, "/index.html") == 0) {
      // return index_html_get_handler(req);
    }
    else if (strcmp(filename, "/favicon.ico") == 0) {
      // return favicon_get_handler(req);
    }
    ESP_LOGE(TAG, "Failed to stat file : %s", filepath);
    // Respond with 404 Not Found
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File does not exist");
    return ESP_FAIL;
  }

  fd = fopen(filepath, "r");
  if (!fd) {
    ESP_LOGE(TAG, "Failed to read existing file : %s", filepath);
    // Respond with 500 Internal Server Error
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read existing file");
    return ESP_FAIL;
  }

  ESP_LOGD(TAG, "Sending file : %s (%ld bytes)...", filename, file_stat.st_size);
  set_content_type_from_file(req, filename);

  // Retrieve the pointer to chunk buffer for temporary storage
  char *chunk = (char *) req->user_ctx;
  size_t chunksize;
  do {
    // Read file in chunks into the chund buffer
    memset(chunk, 0, sizeof(req->user_ctx));
    chunksize = fread(chunk, 1, CHUNK_BUFSIZE, fd);

    if (chunksize > 0) {
      // Send the buffer contents as HTTP response chunk
      if (httpd_resp_send_chunk(req, chunk, chunksize) != ESP_OK) {
        fclose(fd);
        ESP_LOGE(TAG, "File sending failed!");
        // Abort sending file
        httpd_resp_sendstr_chunk(req, NULL);
        // Respond with 500 Internal Server Error
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");
        return ESP_FAIL;
      }
    }

    // Keep looping till the whole file is sent
  } while (chunksize != 0);

  // Close file after sending complete
  fclose(fd);
  ESP_LOGD(TAG, "File sending complete");

  /* Respond with an empty chunk to signal HTTP response completion */
#ifdef CONFIG_EXAMPLE_HTTPD_CONN_CLOSE_HEADER
  httpd_resp_set_hdr(req, "Connection", "close");
#endif
  httpd_resp_send_chunk(req, NULL, 0);
  return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////
// start_webserver
//

httpd_handle_t
start_webserver(void)
{
  httpd_handle_t srv     = NULL;
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  web_log_hook_init();
  cfg.server_port = g_persistent.webPort;

  // 4096 is to low for OTA
  cfg.stack_size = 1024 * 8;

  cfg.lru_purge_enable = true;
  // Use the URI wildcard matching function in order to
  // allow the same handler to respond to multiple different
  // target URIs which match the wildcard scheme
  cfg.uri_match_fn = httpd_uri_match_wildcard;

  cfg.max_uri_handlers = 20;
  cfg.max_open_sockets  = 7; 

  // extern const unsigned char servercert_start[] asm("_binary_servercert_pem_start");
  // extern const unsigned char servercert_end[] asm("_binary_servercert_pem_end");
  // cfg.servercert     = servercert_start;
  // cfg.servercert_len = servercert_end - servercert_start;

  // extern const unsigned char prvtkey_pem_start[] asm("_binary_prvtkey_pem_start");
  // extern const unsigned char prvtkey_pem_end[] asm("_binary_prvtkey_pem_end");
  // cfg.prvtkey_pem = prvtkey_pem_start;
  // cfg.prvtkey_len = prvtkey_pem_end - prvtkey_pem_start;

  // Start the httpd server
  ESP_LOGD(TAG, "Starting httpd server on port: '%d'", cfg.server_port);
  if (httpd_start(&srv, &cfg) == ESP_OK) {

    // Set URI handlers
    ESP_LOGD(TAG, "Registering URI handlers");

    // URI handler for getting uploaded files
    // httpd_uri_t file_spiffs = { .uri      = "/*", // Match all URIs of type /path/to/file
    //                             .method   = HTTP_GET,
    //                             .handler  = spiffs_get_handler,
    //                             .user_ctx = NULL };

    httpd_uri_t dflt = { .uri      = "/*", // Match all URIs of type /path/to/file
                         .method   = HTTP_GET,
                         .handler  = default_get_handler,
                         .user_ctx = NULL };

    httpd_register_uri_handler(srv, &hello);
    // httpd_register_uri_handler(srv, &echo);
    //  httpd_register_uri_handler(srv, &ctrl);
    //  httpd_register_uri_handler(srv, &mainpg);

    if (g_persistent.enableWebsock) {
      if (g_persistent.websockPort == g_persistent.webPort) {
        ESP_LOGW(TAG, "WebSocket port equals web port (%d); registering /ws1 on web server", g_persistent.webPort);
        if (ESP_OK != wss_register_handlers(srv)) {
          // We still continue letting just the webserver run
        }
        ESP_LOGI(TAG, "WebSocket server running on same port as web server");
        g_websocket_srv = NULL;
      }
      else {
        g_websocket_srv = wss_start_websocket_server();
      }
    }

    httpd_register_uri_handler(srv, &dflt);

    httpd_register_uri_handler(srv, &upgrdlocal);
    httpd_register_uri_handler(srv, &upgrdsiblinglocal);
    httpd_register_uri_handler(srv, &cfgrestorepost);

    // httpd_register_uri_handler(srv, &config);
    //  httpd_register_uri_handler(srv, &cfgModule);

    // httpd_register_uri_handler(srv, &info);
    // httpd_register_uri_handler(srv, &reset);

    // httpd_register_uri_handler(srv, &upgrade);
    // httpd_register_uri_handler(srv, &upgrade_local);

    // httpd_register_basic_auth(srv);
    // httpd_register_uri_handler(srv, &file_spiffs);

    return srv;
  }

  ESP_LOGD(TAG, "Error starting server!");
  return NULL;
}

///////////////////////////////////////////////////////////////////////////////
// stop_webserver
//

esp_err_t
stop_webserver(httpd_handle_t server)
{
  if (g_websocket_srv) {
    ESP_LOGD(TAG, "Stopping websocket server");
    httpd_stop(g_websocket_srv);
    g_websocket_srv = NULL;
  }

  // Stop the httpd server
  return httpd_stop(server);
}

///////////////////////////////////////////////////////////////////////////////
// disconnect_handler
//

static void __attribute__((unused))
disconnect_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
  httpd_handle_t *server = (httpd_handle_t *) arg;
  if (*server) {
    ESP_LOGD(TAG, "Stopping webserver");
    if (stop_webserver(*server) == ESP_OK) {
      *server = NULL;
    }
    else {
      ESP_LOGE(TAG, "Failed to stop http server");
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
// connect_handler
//

static void __attribute__((unused))
connect_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
  httpd_handle_t *server = (httpd_handle_t *) arg;
  if (*server == NULL) {
    ESP_LOGD(TAG, "Starting webserver");
    *server = start_webserver();
  }
}
