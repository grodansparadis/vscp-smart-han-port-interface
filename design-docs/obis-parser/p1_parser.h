/*
 * p1_parser.h
 *
 * Streaming-friendly parser for IEC 62056-21 / DSMR "P1" smart meter
 * telegrams (the ASCII protocol used on the P1 port of Dutch/Belgian
 * electricity meters, also used more generally for OBIS-coded ASCII
 * meter output).
 *
 * Designed for embedded targets:
 *   - no dynamic memory allocation
 *   - fixed-size buffers (sized via the #defines below)
 *   - no dependency on anything beyond <stdint.h>/<stdbool.h>/<stddef.h>
 *
 * A telegram looks like:
 *
 *   /ISK5\2M550T-1012
 *
 *   1-3:0.2.8(50)
 *   0-0:1.0.0(230101120000W)
 *   0-0:96.1.1(4530303236303030303331303231363139)
 *   1-0:1.8.1(000123.456*kWh)
 *   1-0:1.8.2(000234.567*kWh)
 *   1-0:1.7.0(00.424*kW)
 *   0-1:24.2.1(230101120000W)(00123.456*m3)
 *   !6A9B
 *
 * Each data line is: <OBIS reference>(<value>)[(<value2>)...]
 * The telegram is terminated by a line starting with '!' followed by
 * a 4-hex-digit CRC16 (CRC16/ARC) checksum computed over every byte
 * from the leading '/' up to and including the '!'.
 */

#ifndef P1_PARSER_H
#define P1_PARSER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Tunables --------------------------------------------------- */

#define P1_OBIS_MAX_LEN         16   /* "1-0:99.99.99.255" + NUL      */
#define P1_VALUE_MAX_LEN        36   /* e.g. "000123.456*kWh"         */
#define P1_MAX_VALUES_PER_LINE  4    /* enough for gas (ts + value)   */

/* ---- Types -------------------------------------------------------*/

typedef struct {
    char    obis[P1_OBIS_MAX_LEN];
    char    values[P1_MAX_VALUES_PER_LINE][P1_VALUE_MAX_LEN];
    uint8_t value_count;
} p1_field_t;

typedef enum {
    P1_OK = 0,
    P1_ERR_NO_START,        /* no leading '/' found                  */
    P1_ERR_NO_END,          /* no '!' + CRC found                    */
    P1_ERR_CRC_MISMATCH,    /* CRC16 did not match                   */
    P1_ERR_TRUNCATED,       /* buffer ended before a full telegram   */
} p1_status_t;

/* Called once per parsed data line while walking a telegram. */
typedef void (*p1_field_cb)(const p1_field_t *field, void *user_ctx);

/* ---- Line-level API ---------------------------------------------
 * Parses a single line (no trailing \r\n required, but tolerated).
 * Returns true and fills *out_field if the line was a valid OBIS
 * data line ("<obis>(<value>)..."). Returns false for header lines
 * ('/...'), the end marker ('!...'), blank lines, or malformed input.
 */
bool p1_parse_line(const char *line, p1_field_t *out_field);

/* ---- Telegram-level API -------------------------------------------
 * Parses a complete, buffered telegram: verifies the CRC16 and then
 * invokes cb() for every valid data line found. len is the number of
 * valid bytes in buf (buf need not be NUL-terminated).
 */
p1_status_t p1_parse_telegram(const char *buf, size_t len,
                               p1_field_cb cb, void *user_ctx);

/* ---- CRC16/ARC (poly 0xA001, init 0x0000, no xorout) -------------
 * This is the checksum variant used by the DSMR/P1 spec.
 */
uint16_t p1_crc16(const uint8_t *data, size_t len);

/* ---- OBIS lookup table --------------------------------------------
 * Returns a short human-readable description for well-known DSMR/P1
 * OBIS reference codes, or NULL if the code isn't in the table.
 * The comparison ignores a trailing ".255" (the common "no channel"
 * suffix), so "1-0:1.8.1" and "1-0:1.8.1.255" both match.
 */
const char *p1_obis_describe(const char *obis);

#ifdef __cplusplus
}
#endif

#endif /* P1_PARSER_H */
