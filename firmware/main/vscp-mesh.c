/*
  VSCP mesh transport helper for compact framed event transport.
*/

#include <string.h>
#include <stdlib.h>

#include <esp_log.h>
#include <esp_timer.h>

#include <vscp.h>
#include <vscp-firmware-helper.h>

#include "vscp-mesh.h"

#define TAG "vscp-mesh"

#define VSCP_MESH_REASSEMBLY_SLOTS      8u
#define VSCP_MESH_MAX_FRAGMENTS         32u
#define VSCP_MESH_MAX_FRAGMENT_PAYLOAD  255u
#define VSCP_MESH_MAX_REASSEMBLED_FRAME 640u
#define VSCP_MESH_DUP_CACHE_SIZE        16u

typedef struct {
  bool used;
  uint16_t src_nick;
  uint32_t msg_id;
  uint32_t ts_ms;
} vscp_mesh_dup_cache_t;

typedef struct {
  bool active;
  bool has_last;
  uint16_t src_nick;
  uint32_t msg_id;
  uint32_t started_ms;
  uint8_t frag_cnt;
  uint8_t last_idx;
  uint16_t frag_lens[VSCP_MESH_MAX_FRAGMENTS];
  uint32_t recv_mask[(VSCP_MESH_MAX_FRAGMENTS + 31u) / 32u];
  uint8_t frame[VSCP_MESH_MAX_REASSEMBLED_FRAME];
} vscp_mesh_reasm_slot_t;

typedef struct {
  vscp_mesh_config_t cfg;
  uint32_t msg_id;
  vscp_mesh_tx_packet_cb_t tx_cb;
  void *tx_ctx;
  vscp_mesh_reasm_slot_t slots[VSCP_MESH_REASSEMBLY_SLOTS];
  vscp_mesh_dup_cache_t dup[VSCP_MESH_DUP_CACHE_SIZE];
} vscp_mesh_state_t;

static vscp_mesh_state_t g_mesh;

static uint32_t
vscp_mesh_now_ms(void)
{
  return (uint32_t) (esp_timer_get_time() / 1000ULL);
}

static uint16_t
vscp_mesh_crc16_ccitt(const uint8_t *data, size_t len)
{
  uint16_t crc = 0xFFFFu;

  if (NULL == data) {
    return 0;
  }

  for (size_t i = 0; i < len; ++i) {
    crc ^= ((uint16_t) data[i] << 8);
    for (int b = 0; b < 8; ++b) {
      if (crc & 0x8000u) {
        crc = (uint16_t) ((crc << 1) ^ 0x1021u);
      }
      else {
        crc <<= 1;
      }
    }
  }

  return crc;
}

static void
vscp_mesh_put_u16_le(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t) (v & 0xFFu);
  p[1] = (uint8_t) ((v >> 8) & 0xFFu);
}

static void
vscp_mesh_put_u32_le(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t) (v & 0xFFu);
  p[1] = (uint8_t) ((v >> 8) & 0xFFu);
  p[2] = (uint8_t) ((v >> 16) & 0xFFu);
  p[3] = (uint8_t) ((v >> 24) & 0xFFu);
}

static uint16_t
vscp_mesh_get_u16_le(const uint8_t *p)
{
  return (uint16_t) p[0] | ((uint16_t) p[1] << 8);
}

static uint32_t
vscp_mesh_get_u32_le(const uint8_t *p)
{
  return (uint32_t) p[0] | ((uint32_t) p[1] << 8) | ((uint32_t) p[2] << 16) | ((uint32_t) p[3] << 24);
}

static bool
vscp_mesh_is_dup(uint16_t src_nick, uint32_t msg_id)
{
  uint32_t now = vscp_mesh_now_ms();

  for (size_t i = 0; i < VSCP_MESH_DUP_CACHE_SIZE; ++i) {
    if (g_mesh.dup[i].used && g_mesh.dup[i].src_nick == src_nick && g_mesh.dup[i].msg_id == msg_id) {
      if ((now - g_mesh.dup[i].ts_ms) < 60000u) {
        return true;
      }
      g_mesh.dup[i].used = false;
      break;
    }
  }

  return false;
}

static void
vscp_mesh_mark_dup(uint16_t src_nick, uint32_t msg_id)
{
  uint32_t now      = vscp_mesh_now_ms();
  size_t insert_idx = 0;
  uint32_t oldest   = 0;
  bool found_free   = false;

  for (size_t i = 0; i < VSCP_MESH_DUP_CACHE_SIZE; ++i) {
    if (!g_mesh.dup[i].used) {
      insert_idx = i;
      found_free = true;
      break;
    }

    if ((0 == i) || (g_mesh.dup[i].ts_ms < oldest)) {
      oldest = g_mesh.dup[i].ts_ms;
      insert_idx = i;
    }
  }

  g_mesh.dup[insert_idx].used = true;
  g_mesh.dup[insert_idx].src_nick = src_nick;
  g_mesh.dup[insert_idx].msg_id = msg_id;
  g_mesh.dup[insert_idx].ts_ms = now;

  (void) found_free;
}

static vscp_mesh_reasm_slot_t *
vscp_mesh_get_slot(uint16_t src_nick, uint32_t msg_id)
{
  uint32_t now = vscp_mesh_now_ms();

  for (size_t i = 0; i < VSCP_MESH_REASSEMBLY_SLOTS; ++i) {
    if (g_mesh.slots[i].active && g_mesh.slots[i].src_nick == src_nick && g_mesh.slots[i].msg_id == msg_id) {
      return &g_mesh.slots[i];
    }
  }

  for (size_t i = 0; i < VSCP_MESH_REASSEMBLY_SLOTS; ++i) {
    if (g_mesh.slots[i].active &&
        (now - g_mesh.slots[i].started_ms) > g_mesh.cfg.reassembly_timeout_ms) {
      memset(&g_mesh.slots[i], 0, sizeof(vscp_mesh_reasm_slot_t));
    }
  }

  for (size_t i = 0; i < VSCP_MESH_REASSEMBLY_SLOTS; ++i) {
    if (!g_mesh.slots[i].active) {
      g_mesh.slots[i].active = true;
      g_mesh.slots[i].src_nick = src_nick;
      g_mesh.slots[i].msg_id = msg_id;
      g_mesh.slots[i].started_ms = now;
      return &g_mesh.slots[i];
    }
  }

  return NULL;
}

void
vscp_mesh_default_config(vscp_mesh_config_t *pcfg)
{
  if (NULL == pcfg) {
    return;
  }

  memset(pcfg, 0, sizeof(vscp_mesh_config_t));
  pcfg->src_nick = 0;
  pcfg->default_dst_nick = VSCP_MESH_ADDR_BROADCAST;
  pcfg->ttl_hops = VSCP_MESH_DEFAULT_TTL_HOPS;
  pcfg->fragment_payload_size = VSCP_MESH_DEFAULT_FRAGMENT_PAYLOAD;
  pcfg->reassembly_timeout_ms = VSCP_MESH_DEFAULT_REASSEMBLY_TIMEOUT;
}

int
vscp_mesh_init(const vscp_mesh_config_t *pcfg)
{
  vscp_mesh_default_config(&g_mesh.cfg);

  if (NULL != pcfg) {
    g_mesh.cfg = *pcfg;
  }

  if (0 == g_mesh.cfg.fragment_payload_size) {
    g_mesh.cfg.fragment_payload_size = VSCP_MESH_DEFAULT_FRAGMENT_PAYLOAD;
  }

  if (0 == g_mesh.cfg.reassembly_timeout_ms) {
    g_mesh.cfg.reassembly_timeout_ms = VSCP_MESH_DEFAULT_REASSEMBLY_TIMEOUT;
  }

  if (0 == g_mesh.cfg.ttl_hops) {
    g_mesh.cfg.ttl_hops = VSCP_MESH_DEFAULT_TTL_HOPS;
  }

  if ((uint32_t) g_mesh.cfg.fragment_payload_size * VSCP_MESH_MAX_FRAGMENTS > VSCP_MESH_MAX_REASSEMBLED_FRAME) {
    return VSCP_ERROR_PARAMETER;
  }

  memset(g_mesh.slots, 0, sizeof(g_mesh.slots));
  memset(g_mesh.dup, 0, sizeof(g_mesh.dup));

  return VSCP_ERROR_SUCCESS;
}

void
vscp_mesh_set_tx_callback(vscp_mesh_tx_packet_cb_t cb, void *ctx)
{
  g_mesh.tx_cb = cb;
  g_mesh.tx_ctx = ctx;
}

uint32_t
vscp_mesh_next_message_id(void)
{
  g_mesh.msg_id++;
  if (0 == g_mesh.msg_id) {
    g_mesh.msg_id = 1;
  }
  return g_mesh.msg_id;
}

uint8_t
vscp_mesh_qos_from_priority(uint8_t priority)
{
  if (priority <= 1u) {
    return VSCP_MESH_QOS_PRIORITY;
  }

  if (priority <= 4u) {
    return VSCP_MESH_QOS_RETRY;
  }

  return VSCP_MESH_QOS_BEST_EFFORT;
}

int
vscp_mesh_encode_header(uint8_t *out, size_t out_len, const vscp_mesh_header_t *phdr)
{
  if ((NULL == out) || (NULL == phdr) || (out_len < VSCP_MESH_HEADER_SIZE)) {
    return VSCP_ERROR_PARAMETER;
  }

  out[0] = phdr->ver;
  out[1] = phdr->flags;
  out[2] = phdr->ttl_hops;
  out[3] = phdr->qos;
  vscp_mesh_put_u32_le(out + 4, phdr->msg_id);
  vscp_mesh_put_u16_le(out + 8, phdr->src_nick);
  vscp_mesh_put_u16_le(out + 10, phdr->dst_nick);
  vscp_mesh_put_u16_le(out + 12, phdr->vscp_class);
  vscp_mesh_put_u16_le(out + 14, phdr->vscp_type);
  out[16] = phdr->frag_idx;
  out[17] = phdr->frag_cnt;
  vscp_mesh_put_u16_le(out + 18, phdr->payload_len);

  return VSCP_ERROR_SUCCESS;
}

int
vscp_mesh_decode_header(vscp_mesh_header_t *phdr, const uint8_t *packet, size_t packet_len)
{
  if ((NULL == phdr) || (NULL == packet) || (packet_len < VSCP_MESH_HEADER_SIZE)) {
    return VSCP_ERROR_PARAMETER;
  }

  memset(phdr, 0, sizeof(vscp_mesh_header_t));
  phdr->ver = packet[0];
  phdr->flags = packet[1];
  phdr->ttl_hops = packet[2];
  phdr->qos = packet[3];
  phdr->msg_id = vscp_mesh_get_u32_le(packet + 4);
  phdr->src_nick = vscp_mesh_get_u16_le(packet + 8);
  phdr->dst_nick = vscp_mesh_get_u16_le(packet + 10);
  phdr->vscp_class = vscp_mesh_get_u16_le(packet + 12);
  phdr->vscp_type = vscp_mesh_get_u16_le(packet + 14);
  phdr->frag_idx = packet[16];
  phdr->frag_cnt = packet[17];
  phdr->payload_len = vscp_mesh_get_u16_le(packet + 18);

  if (phdr->ver != VSCP_MESH_VERSION) {
    return VSCP_ERROR_INVALID_FRAME;
  }

  if (packet_len < (VSCP_MESH_HEADER_SIZE + (size_t) phdr->payload_len)) {
    return VSCP_ERROR_INVALID_FRAME;
  }

  return VSCP_ERROR_SUCCESS;
}

int
vscp_mesh_send_eventex(const vscp_event_ex_t *pex, uint16_t dst_nick)
{
  if (NULL == pex) {
    return VSCP_ERROR_PARAMETER;
  }

  if (NULL == g_mesh.tx_cb) {
    return VSCP_ERROR_UNSUPPORTED;
  }

  size_t frame_len = vscp_fwhlp_getFrameSizeFromEventEx((vscp_event_ex_t *) pex);
  if ((0 == frame_len) || (frame_len > VSCP_MESH_MAX_REASSEMBLED_FRAME)) {
    return VSCP_ERROR_PARAMETER;
  }

  uint8_t *frame = (uint8_t *) malloc(frame_len);
  if (NULL == frame) {
    return VSCP_ERROR_MEMORY;
  }

  int rv = vscp_fwhlp_writeEventExToFrame(frame, frame_len, 0, pex);
  if (VSCP_ERROR_SUCCESS != rv) {
    free(frame);
    return rv;
  }

  const uint8_t frag_size = g_mesh.cfg.fragment_payload_size;
  const uint8_t frag_cnt = (uint8_t) ((frame_len + frag_size - 1u) / frag_size);
  if ((0 == frag_cnt) || (frag_cnt > VSCP_MESH_MAX_FRAGMENTS)) {
    free(frame);
    return VSCP_ERROR_PARAMETER;
  }

  const uint8_t priority = (uint8_t) ((pex->head >> 5) & 0x07u);
  const uint8_t qos = vscp_mesh_qos_from_priority(priority);
  const uint32_t msg_id = vscp_mesh_next_message_id();

  size_t offset = 0;
  for (uint8_t idx = 0; idx < frag_cnt; ++idx) {
    const bool is_last = (idx == (uint8_t) (frag_cnt - 1u));
    const size_t frag_payload = (size_t) ((frame_len - offset) > frag_size ? frag_size : (frame_len - offset));

    vscp_mesh_header_t hdr = { 0 };
    hdr.ver = VSCP_MESH_VERSION;
    hdr.flags = 0;
    if (frag_cnt > 1u) {
      hdr.flags |= VSCP_MESH_FLAG_IS_FRAGMENT;
    }
    if (qos > VSCP_MESH_QOS_BEST_EFFORT) {
      hdr.flags |= VSCP_MESH_FLAG_ACK_REQ;
    }
    if (is_last) {
      hdr.flags |= VSCP_MESH_FLAG_IS_LAST;
    }
    hdr.ttl_hops = g_mesh.cfg.ttl_hops;
    hdr.qos = qos;
    hdr.msg_id = msg_id;
    hdr.src_nick = g_mesh.cfg.src_nick;
    hdr.dst_nick = dst_nick;
    hdr.vscp_class = pex->vscp_class;
    hdr.vscp_type = pex->vscp_type;
    hdr.frag_idx = idx;
    hdr.frag_cnt = frag_cnt;
    hdr.payload_len = (uint16_t) frag_payload;

    uint8_t packet[VSCP_MESH_HEADER_SIZE + VSCP_MESH_MAX_FRAGMENT_PAYLOAD + 2];
    rv = vscp_mesh_encode_header(packet, sizeof(packet), &hdr);
    if (VSCP_ERROR_SUCCESS != rv) {
      free(frame);
      return rv;
    }

    memcpy(packet + VSCP_MESH_HEADER_SIZE, frame + offset, frag_payload);
    size_t packet_len = VSCP_MESH_HEADER_SIZE + frag_payload;

    if (is_last) {
      uint16_t crc = vscp_mesh_crc16_ccitt(frame, frame_len);
      packet[packet_len + 0] = (uint8_t) (crc & 0xFFu);
      packet[packet_len + 1] = (uint8_t) ((crc >> 8) & 0xFFu);
      packet_len += 2;
    }

    rv = g_mesh.tx_cb(packet, packet_len, g_mesh.tx_ctx);
    if (VSCP_ERROR_SUCCESS != rv) {
      free(frame);
      return rv;
    }

    offset += frag_payload;
  }

  free(frame);
  return VSCP_ERROR_SUCCESS;
}

vscp_mesh_rx_result_t
vscp_mesh_receive_packet(const uint8_t *packet,
                        size_t packet_len,
                        vscp_event_ex_t *out_eventex,
                        uint16_t *psrc_nick,
                        uint32_t *pmsg_id)
{
  if ((NULL == packet) || (NULL == out_eventex)) {
    return VSCP_MESH_RX_ERROR;
  }

  vscp_mesh_header_t hdr;
  if (VSCP_ERROR_SUCCESS != vscp_mesh_decode_header(&hdr, packet, packet_len)) {
    return VSCP_MESH_RX_ERROR;
  }

  if ((0 == hdr.frag_cnt) || (hdr.frag_cnt > VSCP_MESH_MAX_FRAGMENTS) || (hdr.frag_idx >= hdr.frag_cnt)) {
    return VSCP_MESH_RX_ERROR;
  }

  if (vscp_mesh_is_dup(hdr.src_nick, hdr.msg_id)) {
    return VSCP_MESH_RX_DUPLICATE;
  }

  vscp_mesh_reasm_slot_t *pslot = vscp_mesh_get_slot(hdr.src_nick, hdr.msg_id);
  if (NULL == pslot) {
    return VSCP_MESH_RX_ERROR;
  }

  if (0 == pslot->frag_cnt) {
    pslot->frag_cnt = hdr.frag_cnt;
  }
  else if (pslot->frag_cnt != hdr.frag_cnt) {
    memset(pslot, 0, sizeof(vscp_mesh_reasm_slot_t));
    return VSCP_MESH_RX_ERROR;
  }

  const size_t payload_off = VSCP_MESH_HEADER_SIZE;
  const size_t payload_len = hdr.payload_len;
  const size_t slot_off = (size_t) hdr.frag_idx * g_mesh.cfg.fragment_payload_size;

  if ((slot_off + payload_len) > VSCP_MESH_MAX_REASSEMBLED_FRAME) {
    memset(pslot, 0, sizeof(vscp_mesh_reasm_slot_t));
    return VSCP_MESH_RX_ERROR;
  }

  const uint32_t bit_word = (uint32_t) hdr.frag_idx / 32u;
  const uint32_t bit_mask = 1u << ((uint32_t) hdr.frag_idx % 32u);

  if (0u == (pslot->recv_mask[bit_word] & bit_mask)) {
    memcpy(pslot->frame + slot_off, packet + payload_off, payload_len);
    pslot->frag_lens[hdr.frag_idx] = (uint16_t) payload_len;
    pslot->recv_mask[bit_word] |= bit_mask;
  }

  if (hdr.flags & VSCP_MESH_FLAG_IS_LAST) {
    pslot->has_last = true;
    pslot->last_idx = hdr.frag_idx;
  }

  for (uint8_t i = 0; i < pslot->frag_cnt; ++i) {
    const uint32_t w = (uint32_t) i / 32u;
    const uint32_t m = 1u << ((uint32_t) i % 32u);
    if (0u == (pslot->recv_mask[w] & m)) {
      return VSCP_MESH_RX_INCOMPLETE;
    }
  }

  if (!pslot->has_last) {
    return VSCP_MESH_RX_INCOMPLETE;
  }

  size_t total_len = ((size_t) pslot->last_idx * g_mesh.cfg.fragment_payload_size) + pslot->frag_lens[pslot->last_idx];
  if ((0 == total_len) || (total_len > VSCP_MESH_MAX_REASSEMBLED_FRAME)) {
    memset(pslot, 0, sizeof(vscp_mesh_reasm_slot_t));
    return VSCP_MESH_RX_ERROR;
  }

  if (total_len >= 3) {
    uint16_t recv_crc = (uint16_t) pslot->frame[total_len - 2] | ((uint16_t) pslot->frame[total_len - 1] << 8);
    uint16_t calc_crc = vscp_mesh_crc16_ccitt(pslot->frame, total_len - 2);
    if (recv_crc == calc_crc) {
      total_len -= 2;
    }
  }

  int rv = vscp_fwhlp_getEventExFromFrame(out_eventex, pslot->frame, total_len);
  if (VSCP_ERROR_SUCCESS != rv) {
    ESP_LOGW(TAG, "Failed to parse reassembled frame rv=%d", rv);
    memset(pslot, 0, sizeof(vscp_mesh_reasm_slot_t));
    return VSCP_MESH_RX_ERROR;
  }

  if (NULL != psrc_nick) {
    *psrc_nick = hdr.src_nick;
  }

  if (NULL != pmsg_id) {
    *pmsg_id = hdr.msg_id;
  }

  vscp_mesh_mark_dup(hdr.src_nick, hdr.msg_id);
  memset(pslot, 0, sizeof(vscp_mesh_reasm_slot_t));
  return VSCP_MESH_RX_COMPLETE;
}
