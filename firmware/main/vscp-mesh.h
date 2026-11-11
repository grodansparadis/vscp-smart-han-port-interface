/*
  VSCP mesh transport helper for compact framed event transport.
*/

#ifndef __VSCP_MESH_H__
#define __VSCP_MESH_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include <vscp.h>

#define VSCP_MESH_VERSION                    1u
#define VSCP_MESH_HEADER_SIZE                20u
#define VSCP_MESH_DEFAULT_FRAGMENT_PAYLOAD   80u
#define VSCP_MESH_DEFAULT_TTL_HOPS           5u
#define VSCP_MESH_DEFAULT_REASSEMBLY_TIMEOUT 8000u

#define VSCP_MESH_ADDR_BROADCAST 0xFFFFu

#define VSCP_MESH_FLAG_ACK_REQ     0x01u
#define VSCP_MESH_FLAG_IS_FRAGMENT 0x02u
#define VSCP_MESH_FLAG_IS_LAST     0x04u
#define VSCP_MESH_FLAG_ENCRYPTED   0x08u
#define VSCP_MESH_FLAG_ACK         0x10u

#define VSCP_MESH_QOS_BEST_EFFORT 0u
#define VSCP_MESH_QOS_RETRY       1u
#define VSCP_MESH_QOS_PRIORITY    2u

typedef struct {
  uint8_t ver;
  uint8_t flags;
  uint8_t ttl_hops;
  uint8_t qos;
  uint32_t msg_id;
  uint16_t src_nick;
  uint16_t dst_nick;
  uint16_t vscp_class;
  uint16_t vscp_type;
  uint8_t frag_idx;
  uint8_t frag_cnt;
  uint16_t payload_len;
} vscp_mesh_header_t;

typedef struct {
  uint16_t src_nick;
  uint16_t default_dst_nick;
  uint8_t ttl_hops;
  uint8_t fragment_payload_size;
  uint32_t reassembly_timeout_ms;
} vscp_mesh_config_t;

typedef int (*vscp_mesh_tx_packet_cb_t)(const uint8_t *packet, size_t packet_len, void *ctx);

typedef enum {
  VSCP_MESH_RX_ERROR      = -1,
  VSCP_MESH_RX_INCOMPLETE = 0,
  VSCP_MESH_RX_COMPLETE   = 1,
  VSCP_MESH_RX_DUPLICATE  = 2
} vscp_mesh_rx_result_t;

void
vscp_mesh_default_config(vscp_mesh_config_t *pcfg);
int
vscp_mesh_init(const vscp_mesh_config_t *pcfg);
void
vscp_mesh_set_tx_callback(vscp_mesh_tx_packet_cb_t cb, void *ctx);

uint32_t
vscp_mesh_next_message_id(void);
uint8_t
vscp_mesh_qos_from_priority(uint8_t priority);

int
vscp_mesh_encode_header(uint8_t *out, size_t out_len, const vscp_mesh_header_t *phdr);
int
vscp_mesh_decode_header(vscp_mesh_header_t *phdr, const uint8_t *packet, size_t packet_len);

int
vscp_mesh_send_eventex(const vscp_event_ex_t *pex, uint16_t dst_nick);
vscp_mesh_rx_result_t
vscp_mesh_receive_packet(const uint8_t *packet,
                         size_t packet_len,
                         vscp_event_ex_t *out_eventex,
                         uint16_t *psrc_nick,
                         uint32_t *pmsg_id);

#ifdef __cplusplus
}
#endif

#endif
