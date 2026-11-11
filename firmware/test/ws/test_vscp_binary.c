#define _POSIX_C_SOURCE 200809L

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <openssl/evp.h>
#include <openssl/rand.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#define DEFAULT_URL "ws://192.168.1.104:8884/ws1"
#define DEFAULT_USERNAME "vscp"
#define DEFAULT_PASSWORD "secret"
#define DEFAULT_KEY_HEX "A4A86F7D7E119BA3F0CD06881E371B98"

#define WS_OPCODE_TEXT 0x1
#define WS_OPCODE_BINARY 0x2
#define WS_OPCODE_CLOSE 0x8
#define WS_OPCODE_PING 0x9
#define WS_OPCODE_PONG 0xA

#define CMD_NOOP      0x0000
#define CMD_QUIT      0x0001
#define CMD_USER      0x0002
#define CMD_PASS      0x0003
#define CMD_CHALLENGE 0x0004
#define CMD_SEND      0x0005
#define CMD_RETR      0x0006
#define CMD_OPEN      0x0007
#define CMD_CLOSE     0x0008
#define CMD_CHKDATA   0x0009
#define CMD_CLEAR     0x000A
#define CMD_STAT      0x000B
#define CMD_INFO      0x000C
#define CMD_GETCHID   0x000D
#define CMD_SETGUID   0x000E
#define CMD_GETGUID   0x000F
#define CMD_VERSION   0x0010
#define CMD_SETFILTER 0x0011
#define CMD_SETMASK   0x0012
#define CMD_INTERFACE 0x0013
#define CMD_TEST      0x001E
#define CMD_WCYD      0x001F
#define CMD_SHUTDOWN  0x0020
#define CMD_RESTART   0x0021
#define CMD_TEXT      0x0022

#define VSCP_ENCRYPTION_AES128 0x01
#define VSCP_BINARY_EVENT_HEADER_LENGTH 35
#define VSCP_HEADER16_FRAME_VERSION_UNIX_NS 0x0100
#define SAMPLE_EVENT_CLASS 20
#define SAMPLE_EVENT_TYPE 3

#define MAX_TEXT_FRAME 4096

typedef struct {
  char host[256];
  int port;
  char path[512];
} ws_url_t;

typedef struct {
  int timeout_ms;
  int async_events;
  char url[512];
  char username[128];
  char password[128];
  char key_hex[65];
} test_config_t;

static void
fatal(const char *msg)
{
  fprintf(stderr, "\nFAIL: %s\n", msg);
  exit(1);
}

static void
fatal_errno(const char *msg)
{
  fprintf(stderr, "\nFAIL: %s: %s\n", msg, strerror(errno));
  exit(1);
}

static int
is_hex_string(const char *s, size_t expected_len)
{
  if ((NULL == s) || (strlen(s) != expected_len)) {
    return 0;
  }

  for (const char *p = s; *p; ++p) {
    if (!((*p >= '0' && *p <= '9') || (*p >= 'a' && *p <= 'f') || (*p >= 'A' && *p <= 'F'))) {
      return 0;
    }
  }

  return 1;
}

static int
hex_to_bytes(const char *hex, uint8_t *out, size_t out_len)
{
  size_t hex_len = strlen(hex);
  if ((hex_len % 2) != 0 || out_len < (hex_len / 2)) {
    return -1;
  }

  for (size_t i = 0; i < hex_len / 2; ++i) {
    unsigned int v;
    if (1 != sscanf(hex + (2 * i), "%2x", &v)) {
      return -1;
    }
    out[i] = (uint8_t) v;
  }

  return (int) (hex_len / 2);
}

static char *
bytes_to_hex(const uint8_t *buf, size_t len)
{
  char *out = (char *) calloc(1, (len * 2) + 1);
  if (NULL == out) {
    return NULL;
  }

  for (size_t i = 0; i < len; ++i) {
    snprintf(out + (i * 2), 3, "%02x", buf[i]);
  }

  return out;
}

static uint64_t
unix_time_ns(void)
{
  struct timespec ts;
  if (0 != clock_gettime(CLOCK_REALTIME, &ts)) {
    return 0;
  }

  return ((uint64_t) ts.tv_sec * 1000000000ULL) + (uint64_t) ts.tv_nsec;
}

static uint16_t
crc_ccitt(const uint8_t *data, size_t len)
{
  uint16_t crc = 0xFFFF;

  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t) data[i] << 8;
    for (int b = 0; b < 8; ++b) {
      if (crc & 0x8000) {
        crc = (uint16_t) ((crc << 1) ^ 0x1021);
      }
      else {
        crc <<= 1;
      }
    }
  }

  return crc;
}

static const char *
getenv_default(const char *name, const char *fallback)
{
  const char *v = getenv(name);
  if ((NULL == v) || ('\0' == *v)) {
    return fallback;
  }

  return v;
}

static int
read_exact_timeout(int fd, void *buf, size_t len, int timeout_ms)
{
  uint8_t *p = (uint8_t *) buf;
  size_t got = 0;

  while (got < len) {
    struct pollfd pfd = { .fd = fd, .events = POLLIN };
    int pr = poll(&pfd, 1, timeout_ms);
    if (pr < 0) {
      return -1;
    }
    if (0 == pr) {
      errno = ETIMEDOUT;
      return -1;
    }

    ssize_t n = recv(fd, p + got, len - got, 0);
    if (n <= 0) {
      return -1;
    }
    got += (size_t) n;
  }

  return 0;
}

static int
write_all(int fd, const void *buf, size_t len)
{
  const uint8_t *p = (const uint8_t *) buf;
  size_t sent = 0;

  while (sent < len) {
    ssize_t n = send(fd, p + sent, len - sent, 0);
    if (n <= 0) {
      return -1;
    }
    sent += (size_t) n;
  }

  return 0;
}

static int
parse_url(const char *url, ws_url_t *out)
{
  if ((NULL == url) || (NULL == out)) {
    return -1;
  }

  if (0 != strncmp(url, "ws://", 5)) {
    fprintf(stderr, "Only ws:// URLs are supported by this C test\n");
    return -1;
  }

  const char *p = url + 5;
  const char *slash = strchr(p, '/');
  const char *host_end = slash ? slash : (p + strlen(p));
  const char *colon = NULL;

  for (const char *q = p; q < host_end; ++q) {
    if (':' == *q) {
      colon = q;
      break;
    }
  }

  memset(out, 0, sizeof(*out));

  if (colon) {
    size_t host_len = (size_t) (colon - p);
    size_t port_len = (size_t) (host_end - colon - 1);
    if ((host_len == 0) || (host_len >= sizeof(out->host)) || (port_len == 0) || (port_len > 6)) {
      return -1;
    }

    memcpy(out->host, p, host_len);
    out->host[host_len] = '\0';

    char port_buf[8] = { 0 };
    memcpy(port_buf, colon + 1, port_len);
    out->port = atoi(port_buf);
  }
  else {
    size_t host_len = (size_t) (host_end - p);
    if ((host_len == 0) || (host_len >= sizeof(out->host))) {
      return -1;
    }

    memcpy(out->host, p, host_len);
    out->host[host_len] = '\0';
    out->port = 80;
  }

  if (slash) {
    if (strlen(slash) >= sizeof(out->path)) {
      return -1;
    }
    strcpy(out->path, slash);
  }
  else {
    strcpy(out->path, "/");
  }

  return 0;
}

static int
tcp_connect_host(const char *host, int port)
{
  char port_str[16];
  snprintf(port_str, sizeof(port_str), "%d", port);

  struct addrinfo hints;
  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;

  struct addrinfo *res = NULL;
  int rv = getaddrinfo(host, port_str, &hints, &res);
  if (0 != rv) {
    return -1;
  }

  int fd = -1;
  for (struct addrinfo *it = res; it; it = it->ai_next) {
    fd = socket(it->ai_family, it->ai_socktype, it->ai_protocol);
    if (fd < 0) {
      continue;
    }

    if (0 == connect(fd, it->ai_addr, it->ai_addrlen)) {
      break;
    }

    close(fd);
    fd = -1;
  }

  freeaddrinfo(res);
  return fd;
}

static int
ws_send_frame(int fd, uint8_t opcode, const uint8_t *payload, size_t payload_len)
{
  uint8_t header[14];
  size_t hlen = 0;

  header[hlen++] = (uint8_t) (0x80 | (opcode & 0x0F));

  if (payload_len <= 125) {
    header[hlen++] = (uint8_t) (0x80 | payload_len);
  }
  else if (payload_len <= 0xFFFF) {
    header[hlen++] = 0x80 | 126;
    header[hlen++] = (uint8_t) ((payload_len >> 8) & 0xFF);
    header[hlen++] = (uint8_t) (payload_len & 0xFF);
  }
  else {
    header[hlen++] = 0x80 | 127;
    for (int i = 7; i >= 0; --i) {
      header[hlen++] = (uint8_t) ((payload_len >> (8 * i)) & 0xFF);
    }
  }

  uint8_t mask[4];
  if (1 != RAND_bytes(mask, sizeof(mask))) {
    return -1;
  }

  memcpy(header + hlen, mask, sizeof(mask));
  hlen += sizeof(mask);

  uint8_t *masked = NULL;
  if (payload_len > 0) {
    masked = (uint8_t *) malloc(payload_len);
    if (NULL == masked) {
      return -1;
    }

    for (size_t i = 0; i < payload_len; ++i) {
      masked[i] = payload[i] ^ mask[i % 4];
    }
  }

  int rc = 0;
  if (0 != write_all(fd, header, hlen)) {
    rc = -1;
    goto done;
  }

  if ((payload_len > 0) && (0 != write_all(fd, masked, payload_len))) {
    rc = -1;
    goto done;
  }

done:
  free(masked);
  return rc;
}

static int
ws_recv_frame(int fd, uint8_t *opcode, uint8_t **payload, size_t *payload_len, int timeout_ms)
{
  uint8_t h2[2];
  if (0 != read_exact_timeout(fd, h2, sizeof(h2), timeout_ms)) {
    return -1;
  }

  uint8_t fin = (uint8_t) ((h2[0] >> 7) & 0x01);
  (void) fin;
  uint8_t op = (uint8_t) (h2[0] & 0x0F);
  uint8_t masked = (uint8_t) ((h2[1] >> 7) & 0x01);
  uint64_t len = (uint64_t) (h2[1] & 0x7F);

  if (126 == len) {
    uint8_t ext[2];
    if (0 != read_exact_timeout(fd, ext, sizeof(ext), timeout_ms)) {
      return -1;
    }
    len = ((uint64_t) ext[0] << 8) | ext[1];
  }
  else if (127 == len) {
    uint8_t ext[8];
    if (0 != read_exact_timeout(fd, ext, sizeof(ext), timeout_ms)) {
      return -1;
    }
    len = 0;
    for (int i = 0; i < 8; ++i) {
      len = (len << 8) | ext[i];
    }
  }

  uint8_t mask[4] = { 0 };
  if (masked) {
    if (0 != read_exact_timeout(fd, mask, sizeof(mask), timeout_ms)) {
      return -1;
    }
  }

  if (len > (16 * 1024 * 1024)) {
    errno = EMSGSIZE;
    return -1;
  }

  uint8_t *buf = NULL;
  if (len > 0) {
    buf = (uint8_t *) malloc((size_t) len);
    if (NULL == buf) {
      return -1;
    }
    if (0 != read_exact_timeout(fd, buf, (size_t) len, timeout_ms)) {
      free(buf);
      return -1;
    }

    if (masked) {
      for (size_t i = 0; i < (size_t) len; ++i) {
        buf[i] ^= mask[i % 4];
      }
    }
  }

  *opcode = op;
  *payload = buf;
  *payload_len = (size_t) len;
  return 0;
}

static int
ws_recv_message(int fd, uint8_t *out_opcode, uint8_t **out_payload, size_t *out_len, int timeout_ms)
{
  for (;;) {
    uint8_t opcode;
    uint8_t *payload = NULL;
    size_t len = 0;

    if (0 != ws_recv_frame(fd, &opcode, &payload, &len, timeout_ms)) {
      return -1;
    }

    if (WS_OPCODE_PING == opcode) {
      ws_send_frame(fd, WS_OPCODE_PONG, payload, len);
      free(payload);
      continue;
    }

    if (WS_OPCODE_CLOSE == opcode) {
      free(payload);
      errno = ECONNRESET;
      return -1;
    }

    *out_opcode = opcode;
    *out_payload = payload;
    *out_len = len;
    return 0;
  }
}

static int
websocket_connect(const ws_url_t *url, int timeout_ms)
{
  int fd = tcp_connect_host(url->host, url->port);
  if (fd < 0) {
    return -1;
  }

  uint8_t key_raw[16];
  if (1 != RAND_bytes(key_raw, sizeof(key_raw))) {
    close(fd);
    return -1;
  }

  char sec_key[64] = { 0 };
  EVP_EncodeBlock((unsigned char *) sec_key, key_raw, (int) sizeof(key_raw));

  char req[2048];
  int req_len = snprintf(req,
                         sizeof(req),
                         "GET %s HTTP/1.1\r\n"
                         "Host: %s:%d\r\n"
                         "Upgrade: websocket\r\n"
                         "Connection: Upgrade\r\n"
                         "Sec-WebSocket-Key: %s\r\n"
                         "Sec-WebSocket-Version: 13\r\n"
                         "\r\n",
                         url->path,
                         url->host,
                         url->port,
                         sec_key);
  if ((req_len <= 0) || (req_len >= (int) sizeof(req))) {
    close(fd);
    errno = EINVAL;
    return -1;
  }

  if (0 != write_all(fd, req, (size_t) req_len)) {
    close(fd);
    return -1;
  }

  char resp[4096] = { 0 };
  size_t used = 0;

  while (used + 1 < sizeof(resp)) {
    struct pollfd pfd = { .fd = fd, .events = POLLIN };
    int pr = poll(&pfd, 1, timeout_ms);
    if (pr <= 0) {
      close(fd);
      errno = (0 == pr) ? ETIMEDOUT : errno;
      return -1;
    }

    ssize_t n = recv(fd, resp + used, sizeof(resp) - used - 1, 0);
    if (n <= 0) {
      close(fd);
      errno = ECONNRESET;
      return -1;
    }

    used += (size_t) n;
    resp[used] = '\0';

    if (strstr(resp, "\r\n\r\n")) {
      break;
    }
  }

  if (NULL == strstr(resp, " 101 ")) {
    close(fd);
    errno = EPROTO;
    return -1;
  }

  return fd;
}

static int
send_text_frame(int fd, const char *text)
{
  return ws_send_frame(fd, WS_OPCODE_TEXT, (const uint8_t *) text, strlen(text));
}

static int
send_binary_frame(int fd, const uint8_t *buf, size_t len)
{
  return ws_send_frame(fd, WS_OPCODE_BINARY, buf, len);
}

static int
extract_sid_from_text(const char *msg, char sid_out[33])
{
  if ((NULL == msg) || ('+' != msg[0])) {
    return 0;
  }

  const char *p = msg;
  while (*p) {
    const char *start = p;
    while ((*p != '\0') && (*p != ';')) {
      ++p;
    }

    size_t token_len = (size_t) (p - start);
    if (32 == token_len) {
      char token[33];
      memcpy(token, start, 32);
      token[32] = '\0';

      if (is_hex_string(token, 32)) {
        strcpy(sid_out, token);
        return 1;
      }
    }

    if (';' == *p) {
      ++p;
    }
  }

  return 0;
}

static int
is_positive_for(const char *msg, const char *cmd)
{
  if ((NULL == msg) || ('+' != msg[0])) {
    return 0;
  }

  size_t cmd_len = strlen(cmd);
  if (strlen(msg) < (2 + cmd_len)) {
    return 0;
  }

  return (0 == strncmp(msg + 2, cmd, cmd_len));
}

static int
is_negative_reply(const char *msg)
{
  return (NULL != msg) && ('-' == msg[0]);
}

static int
encrypt_auth_credentials(const char *username,
                         const char *password,
                         const char *sid_hex,
                         const char *key_hex,
                         char **out_hex)
{
  uint8_t iv[16];
  uint8_t key[16];

  if (!is_hex_string(sid_hex, 32) || !is_hex_string(key_hex, 32)) {
    return -1;
  }

  if ((16 != hex_to_bytes(sid_hex, iv, sizeof(iv))) || (16 != hex_to_bytes(key_hex, key, sizeof(key)))) {
    return -1;
  }

  size_t plain_len = strlen(username) + 1 + strlen(password);
  uint8_t *plain = (uint8_t *) malloc(plain_len + 1);
  if (NULL == plain) {
    return -1;
  }

  snprintf((char *) plain, plain_len + 1, "%s:%s", username, password);

  EVP_CIPHER_CTX *ctx = EVP_CIPHER_CTX_new();
  if (NULL == ctx) {
    free(plain);
    return -1;
  }

  int rc = -1;
  int outl1 = 0, outl2 = 0;
  size_t max_cipher = plain_len + 16;
  uint8_t *cipher = (uint8_t *) malloc(max_cipher);
  if (NULL == cipher) {
    goto done;
  }

  if (1 != EVP_EncryptInit_ex(ctx, EVP_aes_128_cbc(), NULL, key, iv)) {
    free(cipher);
    goto done;
  }

  if (1 != EVP_EncryptUpdate(ctx, cipher, &outl1, plain, (int) plain_len)) {
    free(cipher);
    goto done;
  }

  if (1 != EVP_EncryptFinal_ex(ctx, cipher + outl1, &outl2)) {
    free(cipher);
    goto done;
  }

  *out_hex = bytes_to_hex(cipher, (size_t) (outl1 + outl2));
  free(cipher);
  rc = (*out_hex != NULL) ? 0 : -1;

done:
  EVP_CIPHER_CTX_free(ctx);
  free(plain);
  return rc;
}

static int
build_binary_command_frame(uint16_t command, const uint8_t *arg, size_t arg_len, uint8_t **out_frame, size_t *out_len)
{
  size_t frame_len = 1 + 2 + arg_len + 2;
  uint8_t *frame = (uint8_t *) calloc(1, frame_len);
  if (NULL == frame) {
    return -1;
  }

  frame[0] = 0xE0;
  frame[1] = (uint8_t) ((command >> 8) & 0xFF);
  frame[2] = (uint8_t) (command & 0xFF);

  if ((arg_len > 0) && (NULL != arg)) {
    memcpy(frame + 3, arg, arg_len);
  }

  uint16_t crc = crc_ccitt(frame + 1, frame_len - 3);
  frame[frame_len - 2] = (uint8_t) ((crc >> 8) & 0xFF);
  frame[frame_len - 1] = (uint8_t) (crc & 0xFF);

  *out_frame = frame;
  *out_len = frame_len;
  return 0;
}

static int
encrypt_binary_frame(const uint8_t *frame,
                     size_t frame_len,
                     const char *key_hex,
                     uint8_t **out_enc,
                     size_t *out_enc_len)
{
  if ((NULL == frame) || (frame_len < 1) || !is_hex_string(key_hex, 32)) {
    return -1;
  }

  uint8_t key[16];
  if (16 != hex_to_bytes(key_hex, key, sizeof(key))) {
    return -1;
  }

  const uint8_t *payload = frame + 1;
  size_t payload_len = frame_len - 1;
  size_t padded_len = ((payload_len + 15) / 16) * 16;
  if (0 == padded_len) {
    padded_len = 16;
  }

  uint8_t *padded = (uint8_t *) calloc(1, padded_len);
  if (NULL == padded) {
    return -1;
  }

  memcpy(padded, payload, payload_len);

  uint8_t iv[16];
  if (1 != RAND_bytes(iv, sizeof(iv))) {
    free(padded);
    return -1;
  }

  EVP_CIPHER_CTX *ctx = EVP_CIPHER_CTX_new();
  if (NULL == ctx) {
    free(padded);
    return -1;
  }

  int rc = -1;
  int outl1 = 0, outl2 = 0;
  uint8_t *cipher = (uint8_t *) malloc(padded_len + 16);
  if (NULL == cipher) {
    goto done;
  }

  if (1 != EVP_EncryptInit_ex(ctx, EVP_aes_128_cbc(), NULL, key, iv)) {
    free(cipher);
    goto done;
  }

  if (1 != EVP_CIPHER_CTX_set_padding(ctx, 0)) {
    free(cipher);
    goto done;
  }

  if (1 != EVP_EncryptUpdate(ctx, cipher, &outl1, padded, (int) padded_len)) {
    free(cipher);
    goto done;
  }

  if (1 != EVP_EncryptFinal_ex(ctx, cipher + outl1, &outl2)) {
    free(cipher);
    goto done;
  }

  size_t cipher_len = (size_t) (outl1 + outl2);
  size_t final_len = 1 + cipher_len + 16;
  uint8_t *enc = (uint8_t *) malloc(final_len);
  if (NULL == enc) {
    free(cipher);
    goto done;
  }

  enc[0] = (uint8_t) ((frame[0] & 0xF0) | (VSCP_ENCRYPTION_AES128 & 0x0F));
  memcpy(enc + 1, cipher, cipher_len);
  memcpy(enc + 1 + cipher_len, iv, 16);

  *out_enc = enc;
  *out_enc_len = final_len;
  free(cipher);
  rc = 0;

done:
  EVP_CIPHER_CTX_free(ctx);
  free(padded);
  return rc;
}

static int
build_sample_event_frame(uint8_t **out_frame, size_t *out_len)
{
  static const uint8_t sample_data[] = { 0x01, 0x02, 0x03 };
  size_t frame_len = 1 + VSCP_BINARY_EVENT_HEADER_LENGTH + sizeof(sample_data) + 2;
  uint8_t *frame = (uint8_t *) calloc(1, frame_len);
  if (NULL == frame) {
    return -1;
  }

  uint64_t ts = unix_time_ns();
  frame[0] = 0x00;
  frame[1] = (uint8_t) ((VSCP_HEADER16_FRAME_VERSION_UNIX_NS >> 8) & 0xFF);
  frame[2] = (uint8_t) (VSCP_HEADER16_FRAME_VERSION_UNIX_NS & 0xFF);

  for (int i = 0; i < 8; ++i) {
    int shift = 56 - (i * 8);
    frame[3 + i] = (uint8_t) ((ts >> shift) & 0xFF);
  }

  frame[11] = 0;
  frame[12] = 0;
  frame[13] = 0;
  frame[14] = (uint8_t) ((SAMPLE_EVENT_CLASS >> 8) & 0xFF);
  frame[15] = (uint8_t) (SAMPLE_EVENT_CLASS & 0xFF);
  frame[16] = (uint8_t) ((SAMPLE_EVENT_TYPE >> 8) & 0xFF);
  frame[17] = (uint8_t) (SAMPLE_EVENT_TYPE & 0xFF);
  frame[33] = 0x01;
  frame[34] = 0;
  frame[35] = (uint8_t) sizeof(sample_data);
  memcpy(frame + 36, sample_data, sizeof(sample_data));

  uint16_t crc = crc_ccitt(frame + 1, frame_len - 3);
  frame[frame_len - 2] = (uint8_t) ((crc >> 8) & 0xFF);
  frame[frame_len - 1] = (uint8_t) (crc & 0xFF);

  *out_frame = frame;
  *out_len = frame_len;
  return 0;
}

static int
parse_binary_reply(const uint8_t *buf, size_t len, uint16_t *command, uint16_t *error)
{
  if ((NULL == buf) || (len < 7)) {
    return -1;
  }

  uint8_t frame_type = (uint8_t) (buf[0] & 0xF0);
  if (0xF0 != frame_type) {
    return -1;
  }

  uint16_t cmd = (uint16_t) (((uint16_t) buf[1] << 8) | buf[2]);
  uint16_t err = (uint16_t) (((uint16_t) buf[3] << 8) | buf[4]);
  uint16_t rx_crc = (uint16_t) (((uint16_t) buf[5] << 8) | buf[6]);
  uint16_t ex_crc = crc_ccitt(buf + 1, len - 3);

  if (rx_crc != ex_crc) {
    return -1;
  }

  *command = cmd;
  *error = err;
  return 0;
}

static int
wait_for_binary_reply(int fd, uint16_t expected_cmd, const char *step_name, int timeout_ms)
{
  for (;;) {
    uint8_t opcode;
    uint8_t *payload = NULL;
    size_t len = 0;

    if (0 != ws_recv_message(fd, &opcode, &payload, &len, timeout_ms)) {
      return -1;
    }

    if (WS_OPCODE_TEXT == opcode) {
      printf("  <- %.*s\n", (int) len, (char *) payload);
      free(payload);
      continue;
    }

    if (WS_OPCODE_BINARY != opcode) {
      free(payload);
      continue;
    }

    uint8_t frame_type = (uint8_t) (payload[0] & 0xF0);
    if (0x00 == frame_type) {
      printf("  <- [async event while waiting for %s, %zu bytes]\n", step_name, len);
      free(payload);
      continue;
    }

    uint16_t cmd = 0, err = 0;
    if (0 != parse_binary_reply(payload, len, &cmd, &err)) {
      free(payload);
      return -1;
    }

    printf("  <- [binary reply, %zu bytes] cmd=0x%04X err=0x%04X\n", len, cmd, err);
    free(payload);

    if (cmd != expected_cmd) {
      continue;
    }

    if (0x0000 != err) {
      fprintf(stderr, "FAIL: %s failed, error=0x%04X\n", step_name, err);
      return -1;
    }

    return 0;
  }
}

static int
wait_for_async_events(int fd, int count, const char *label, int timeout_ms)
{
  printf("  Waiting for %d asynchronous event(s) in %s...\n", count, label);
  int received = 0;

  while (received < count) {
    uint8_t opcode;
    uint8_t *payload = NULL;
    size_t len = 0;

    if (0 != ws_recv_message(fd, &opcode, &payload, &len, timeout_ms * 4)) {
      return -1;
    }

    if (WS_OPCODE_TEXT == opcode) {
      char tmp[MAX_TEXT_FRAME];
      size_t n = (len < (sizeof(tmp) - 1)) ? len : (sizeof(tmp) - 1);
      memcpy(tmp, payload, n);
      tmp[n] = '\0';
      printf("  <- %s\n", tmp);
      if ((n > 0) && ('+' != tmp[0]) && ('-' != tmp[0])) {
        ++received;
        printf("  <- [async event %d/%d, text]\n", received, count);
      }
      free(payload);
      continue;
    }

    if (WS_OPCODE_BINARY == opcode) {
      uint8_t frame_type = (uint8_t) (payload[0] & 0xF0);
      if (0x00 == frame_type) {
        ++received;
        printf("  <- [async event %d/%d, %zu bytes]\n", received, count, len);
      }
      else {
        uint16_t cmd = 0, err = 0;
        if (0 == parse_binary_reply(payload, len, &cmd, &err)) {
          printf("  <- [binary reply while waiting events] cmd=0x%04X err=0x%04X\n", cmd, err);
        }
      }
    }

    free(payload);
  }

  return 0;
}

static int
scenario_text_then_binary(const test_config_t *cfg)
{
  printf("\nScenario A: text AUTH/OPEN then binary NOOP\n");

  ws_url_t url;
  if (0 != parse_url(cfg->url, &url)) {
    fprintf(stderr, "Invalid URL: %s\n", cfg->url);
    return -1;
  }

  int fd = websocket_connect(&url, cfg->timeout_ms);
  if (fd < 0) {
    return -1;
  }

  char sid[33] = { 0 };
  int authenticated = 0;
  int opened = 0;

  printf("  -> C;CHALLENGE\n");
  if (0 != send_text_frame(fd, "C;CHALLENGE")) {
    close(fd);
    return -1;
  }

  while (!opened) {
    uint8_t opcode;
    uint8_t *payload = NULL;
    size_t len = 0;

    if (0 != ws_recv_message(fd, &opcode, &payload, &len, cfg->timeout_ms)) {
      close(fd);
      return -1;
    }

    if (WS_OPCODE_TEXT != opcode) {
      free(payload);
      continue;
    }

    char msg[MAX_TEXT_FRAME];
    size_t n = (len < (sizeof(msg) - 1)) ? len : (sizeof(msg) - 1);
    memcpy(msg, payload, n);
    msg[n] = '\0';
    free(payload);

    printf("  <- %s\n", msg);

    if ((sid[0] == '\0') && extract_sid_from_text(msg, sid)) {
      printf("  SID received: %s\n", sid);
      char *auth_hex = NULL;
      if (0 != encrypt_auth_credentials(cfg->username, cfg->password, sid, cfg->key_hex, &auth_hex)) {
        close(fd);
        return -1;
      }

      char auth_cmd[1024];
      snprintf(auth_cmd, sizeof(auth_cmd), "C;AUTH;%s;%s", sid, auth_hex);
      printf("  -> %s\n", auth_cmd);
      free(auth_hex);

      if (0 != send_text_frame(fd, auth_cmd)) {
        close(fd);
        return -1;
      }
      continue;
    }

    if (!authenticated && is_positive_for(msg, "AUTH")) {
      authenticated = 1;
      printf("  -> C;OPEN\n");
      if (0 != send_text_frame(fd, "C;OPEN")) {
        close(fd);
        return -1;
      }
      continue;
    }

    if (is_positive_for(msg, "OPEN")) {
      opened = 1;
      continue;
    }

    if (is_negative_reply(msg)) {
      close(fd);
      return -1;
    }
  }

  uint8_t *noop = NULL;
  size_t noop_len = 0;
  if (0 != build_binary_command_frame(CMD_NOOP, NULL, 0, &noop, &noop_len)) {
    close(fd);
    return -1;
  }

  printf("  -> [binary NOOP frame, %zu bytes]\n", noop_len);
  if ((0 != send_binary_frame(fd, noop, noop_len)) ||
      (0 != wait_for_binary_reply(fd, CMD_NOOP, "NOOP", cfg->timeout_ms)) ||
      (0 != wait_for_async_events(fd, cfg->async_events, "scenario A", cfg->timeout_ms))) {
    free(noop);
    close(fd);
    return -1;
  }
  free(noop);

  uint8_t *quit = NULL;
  size_t quit_len = 0;
  if (0 != build_binary_command_frame(CMD_QUIT, NULL, 0, &quit, &quit_len)) {
    close(fd);
    return -1;
  }

  printf("  -> [binary QUIT frame, %zu bytes]\n", quit_len);
  int rc = 0;
  if ((0 != send_binary_frame(fd, quit, quit_len)) ||
      (0 != wait_for_binary_reply(fd, CMD_QUIT, "QUIT", cfg->timeout_ms))) {
    rc = -1;
  }

  free(quit);
  ws_send_frame(fd, WS_OPCODE_CLOSE, NULL, 0);
  close(fd);
  return rc;
}

static int
send_plain_binary_command(int fd, uint16_t cmd, const uint8_t *arg, size_t arg_len, const char *label, int timeout_ms)
{
  uint8_t *frame = NULL;
  size_t frame_len = 0;
  if (0 != build_binary_command_frame(cmd, arg, arg_len, &frame, &frame_len)) {
    return -1;
  }

  printf("  -> [binary %s]\n", label);
  int rc = 0;
  if ((0 != send_binary_frame(fd, frame, frame_len)) ||
      (0 != wait_for_binary_reply(fd, cmd, label, timeout_ms))) {
    rc = -1;
  }

  free(frame);
  return rc;
}

static int
send_encrypted_binary_command(int fd,
                              uint16_t cmd,
                              const uint8_t *arg,
                              size_t arg_len,
                              const char *key_hex,
                              const char *label,
                              int timeout_ms)
{
  uint8_t *frame = NULL;
  size_t frame_len = 0;
  if (0 != build_binary_command_frame(cmd, arg, arg_len, &frame, &frame_len)) {
    return -1;
  }

  uint8_t *enc = NULL;
  size_t enc_len = 0;
  if (0 != encrypt_binary_frame(frame, frame_len, key_hex, &enc, &enc_len)) {
    free(frame);
    return -1;
  }

  printf("  -> [encrypted binary %s]\n", label);
  int rc = 0;
  if ((0 != send_binary_frame(fd, enc, enc_len)) ||
      (0 != wait_for_binary_reply(fd, cmd, label, timeout_ms))) {
    rc = -1;
  }

  free(enc);
  free(frame);
  return rc;
}

static int
scenario_binary_only_common(const test_config_t *cfg, int encrypted)
{
  printf("\nScenario %c: %sbinary USER/PASS/OPEN/NOOP/QUIT\n",
         encrypted ? 'C' : 'B',
         encrypted ? "encrypted " : "");

  ws_url_t url;
  if (0 != parse_url(cfg->url, &url)) {
    fprintf(stderr, "Invalid URL: %s\n", cfg->url);
    return -1;
  }

  int fd = websocket_connect(&url, cfg->timeout_ms);
  if (fd < 0) {
    return -1;
  }

  printf("  Connected, waiting for initial text greeting/challenge...\n");
  uint8_t opcode;
  uint8_t *payload = NULL;
  size_t len = 0;
  if (0 == ws_recv_message(fd, &opcode, &payload, &len, cfg->timeout_ms)) {
    if (WS_OPCODE_TEXT == opcode) {
      printf("  <- %.*s\n", (int) len, (char *) payload);
    }
    free(payload);
  }

  uint8_t user_arg[256];
  size_t user_len = strlen(cfg->username);
  if (user_len + 1 > sizeof(user_arg)) {
    close(fd);
    return -1;
  }
  memcpy(user_arg, cfg->username, user_len);
  user_arg[user_len] = '\0';

  uint8_t pass_arg[256];
  size_t pass_len = strlen(cfg->password);
  if (pass_len + 1 > sizeof(pass_arg)) {
    close(fd);
    return -1;
  }
  memcpy(pass_arg, cfg->password, pass_len);
  pass_arg[pass_len] = '\0';

  int rc = 0;
  if (encrypted) {
    rc |= send_encrypted_binary_command(fd,
                                        CMD_USER,
                                        user_arg,
                                        user_len + 1,
                                        cfg->key_hex,
                                        "USER",
                                        cfg->timeout_ms);
    rc |= send_encrypted_binary_command(fd,
                                        CMD_PASS,
                                        pass_arg,
                                        pass_len + 1,
                                        cfg->key_hex,
                                        "PASS",
                                        cfg->timeout_ms);
    rc |= send_encrypted_binary_command(fd, CMD_OPEN, NULL, 0, cfg->key_hex, "OPEN", cfg->timeout_ms);
  }
  else {
    rc |= send_plain_binary_command(fd, CMD_USER, user_arg, user_len + 1, "USER", cfg->timeout_ms);
    rc |= send_plain_binary_command(fd, CMD_PASS, pass_arg, pass_len + 1, "PASS", cfg->timeout_ms);
    rc |= send_plain_binary_command(fd, CMD_OPEN, NULL, 0, "OPEN", cfg->timeout_ms);
  }

  if (encrypted) {
    /* 16-byte all-zero GUID for SETGUID; 21-byte all-zero filter/mask for SETFILTER/SETMASK
       (1 priority + 2 class + 2 type + 16 GUID).  INTERFACE subcommand 0x00 = get count. */
    static const uint8_t zero_guid[16]   = { 0 };
    static const uint8_t zero_filter[21] = { 0 };
    static const uint8_t iface_count[1]  = { 0x00 };

    rc |= send_encrypted_binary_command(fd, CMD_GETCHID,   NULL,        0,                   cfg->key_hex, "GETCHID",   cfg->timeout_ms);
    rc |= send_encrypted_binary_command(fd, CMD_GETGUID,   NULL,        0,                   cfg->key_hex, "GETGUID",   cfg->timeout_ms);
    rc |= send_encrypted_binary_command(fd, CMD_SETGUID,   zero_guid,   16,                  cfg->key_hex, "SETGUID",   cfg->timeout_ms);
    rc |= send_encrypted_binary_command(fd, CMD_VERSION,   NULL,        0,                   cfg->key_hex, "VERSION",   cfg->timeout_ms);
    rc |= send_encrypted_binary_command(fd, CMD_STAT,      NULL,        0,                   cfg->key_hex, "STAT",      cfg->timeout_ms);
    rc |= send_encrypted_binary_command(fd, CMD_INFO,      NULL,        0,                   cfg->key_hex, "INFO",      cfg->timeout_ms);
    rc |= send_encrypted_binary_command(fd, CMD_WCYD,      NULL,        0,                   cfg->key_hex, "WCYD",      cfg->timeout_ms);
    rc |= send_encrypted_binary_command(fd, CMD_CHKDATA,   NULL,        0,                   cfg->key_hex, "CHKDATA",   cfg->timeout_ms);
    rc |= send_encrypted_binary_command(fd, CMD_CLEAR,     NULL,        0,                   cfg->key_hex, "CLEAR",     cfg->timeout_ms);
    rc |= send_encrypted_binary_command(fd, CMD_SETFILTER, zero_filter, sizeof(zero_filter), cfg->key_hex, "SETFILTER", cfg->timeout_ms);
    rc |= send_encrypted_binary_command(fd, CMD_SETMASK,   zero_filter, sizeof(zero_filter), cfg->key_hex, "SETMASK",   cfg->timeout_ms);
    rc |= send_encrypted_binary_command(fd, CMD_INTERFACE, iface_count, sizeof(iface_count), cfg->key_hex, "INTERFACE", cfg->timeout_ms);
    rc |= send_encrypted_binary_command(fd, CMD_TEST,      NULL,        0,                   cfg->key_hex, "TEST",      cfg->timeout_ms);
    rc |= send_encrypted_binary_command(fd, CMD_CHALLENGE, NULL,        0,                   cfg->key_hex, "CHALLENGE", cfg->timeout_ms);
  }

  uint8_t *event_frame = NULL;
  size_t event_len = 0;
  if (0 != build_sample_event_frame(&event_frame, &event_len)) {
    close(fd);
    return -1;
  }

  if (encrypted) {
    rc |= send_encrypted_binary_command(fd, CMD_SEND, event_frame, event_len, cfg->key_hex, "SEND", cfg->timeout_ms);
    rc |= send_encrypted_binary_command(fd, CMD_NOOP, NULL, 0, cfg->key_hex, "NOOP", cfg->timeout_ms);
  }
  else {
    rc |= send_plain_binary_command(fd, CMD_SEND, event_frame, event_len, "SEND", cfg->timeout_ms);
    rc |= send_plain_binary_command(fd, CMD_NOOP, NULL, 0, "NOOP", cfg->timeout_ms);
  }
  free(event_frame);

  if (0 == rc) {
    rc |= wait_for_async_events(fd, cfg->async_events, encrypted ? "scenario C" : "scenario B", cfg->timeout_ms);
  }

  if (encrypted) {
    /* CLOSE / RETR / OPEN cycle */
    rc |= send_encrypted_binary_command(fd,     CMD_CLOSE,    NULL, 0, cfg->key_hex, "CLOSE",        cfg->timeout_ms);
    rc |= send_encrypted_binary_command(fd,     CMD_RETR,     NULL, 0, cfg->key_hex, "RETR",        cfg->timeout_ms);
    rc |= send_encrypted_binary_command(fd,     CMD_OPEN,     NULL, 0, cfg->key_hex, "OPEN(reopen)", cfg->timeout_ms);
    rc |= send_encrypted_binary_command(fd,     CMD_SHUTDOWN, NULL, 0, cfg->key_hex, "SHUTDOWN",     cfg->timeout_ms);
    rc |= send_encrypted_binary_command(fd,     CMD_TEXT,     NULL, 0, cfg->key_hex, "TEXT",        cfg->timeout_ms);
     /* CMD_RESTART: server replies then calls esp_restart() -- device reboots.
       Send this last; QUIT is intentionally omitted because the connection will drop. */
     rc |= send_encrypted_binary_command(fd,     CMD_RESTART,  NULL, 0, cfg->key_hex, "RESTART",      cfg->timeout_ms);
  }
  else {
    rc |= send_plain_binary_command(fd, CMD_QUIT, NULL, 0, "QUIT", cfg->timeout_ms);
  }

  ws_send_frame(fd, WS_OPCODE_CLOSE, NULL, 0);
  close(fd);
  return (0 == rc) ? 0 : -1;
}

static void
print_usage(const char *argv0)
{
  printf("Usage: %s [url] [--url <url>]\n", argv0);
  printf("Env: WS_BINARY_URL, WS1_URL, WS_TIMEOUT_S, WS_ASYNC_EVENTS\n");
  printf("Auth env: VSCP_USERNAME, VSCP_PASSWORD, VSCP_KEY16\n");
}

static void
load_config(int argc, char **argv, test_config_t *cfg)
{
  memset(cfg, 0, sizeof(*cfg));

  const char *timeout_env = getenv_default("WS_TIMEOUT_S", "5");
  cfg->timeout_ms = (int) (atof(timeout_env) * 1000.0);
  if (cfg->timeout_ms <= 0) {
    cfg->timeout_ms = 5000;
  }

  const char *async_env = getenv_default("WS_ASYNC_EVENTS", "3");
  cfg->async_events = atoi(async_env);
  if (cfg->async_events <= 0) {
    cfg->async_events = 3;
  }

  const char *env_url = getenv("WS_BINARY_URL");
  if ((NULL == env_url) || ('\0' == *env_url)) {
    env_url = getenv("WS1_URL");
  }
  if ((NULL == env_url) || ('\0' == *env_url)) {
    env_url = DEFAULT_URL;
  }
  strncpy(cfg->url, env_url, sizeof(cfg->url) - 1);

  strncpy(cfg->username, getenv_default("VSCP_USERNAME", DEFAULT_USERNAME), sizeof(cfg->username) - 1);
  strncpy(cfg->password, getenv_default("VSCP_PASSWORD", DEFAULT_PASSWORD), sizeof(cfg->password) - 1);
  strncpy(cfg->key_hex, getenv_default("VSCP_KEY16", DEFAULT_KEY_HEX), sizeof(cfg->key_hex) - 1);

  for (int i = 1; i < argc; ++i) {
    if ((0 == strcmp(argv[i], "--help")) || (0 == strcmp(argv[i], "-h"))) {
      print_usage(argv[0]);
      exit(0);
    }

    if ((0 == strcmp(argv[i], "--url")) && ((i + 1) < argc)) {
      strncpy(cfg->url, argv[++i], sizeof(cfg->url) - 1);
      continue;
    }

    if (0 == strncmp(argv[i], "--url=", 6)) {
      strncpy(cfg->url, argv[i] + 6, sizeof(cfg->url) - 1);
      continue;
    }

    if (argv[i][0] != '-') {
      strncpy(cfg->url, argv[i], sizeof(cfg->url) - 1);
    }
  }

  if (!is_hex_string(cfg->key_hex, 32)) {
    fatal("Invalid VSCP_KEY16: expected 32 hex chars");
  }
}

int
main(int argc, char **argv)
{
  test_config_t cfg;
  load_config(argc, argv, &cfg);

  OpenSSL_add_all_algorithms();

  printf("Connecting to %s\n", cfg.url);

  if (0 != scenario_text_then_binary(&cfg)) {
    fatal_errno("Scenario A failed");
  }

  if (0 != scenario_binary_only_common(&cfg, 0)) {
    fatal_errno("Scenario B failed");
  }

  if (0 != scenario_binary_only_common(&cfg, 1)) {
    fatal_errno("Scenario C failed");
  }

  printf("\nPASS: Binary NOOP test passed in all scenarios (text-assisted, binary-only, encrypted-binary)\n");
  return 0;
}
