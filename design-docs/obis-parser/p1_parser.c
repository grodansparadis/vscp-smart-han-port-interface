#include "p1_parser.h"
#include <string.h>
#include <ctype.h>

/* ================= CRC16/ARC ===================================== */

uint16_t p1_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0x0000;

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x0001) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001);
            } else {
                crc = (uint16_t)(crc >> 1);
            }
        }
    }
    return crc;
}

/* ================= small string helpers ============================ */

static size_t p1_rtrim(char *s, size_t len)
{
    while (len > 0 && (s[len - 1] == '\r' || s[len - 1] == '\n' ||
                        s[len - 1] == ' '  || s[len - 1] == '\t')) {
        len--;
    }
    s[len] = '\0';
    return len;
}

static int p1_hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/* ================= line parser ===================================== */

bool p1_parse_line(const char *line, p1_field_t *out_field)
{
    if (line == NULL || out_field == NULL) {
        return false;
    }

    char buf[P1_OBIS_MAX_LEN + (P1_MAX_VALUES_PER_LINE * (P1_VALUE_MAX_LEN + 2)) + 8];
    size_t len = strlen(line);
    if (len >= sizeof(buf)) {
        len = sizeof(buf) - 1;
    }
    memcpy(buf, line, len);
    buf[len] = '\0';
    len = p1_rtrim(buf, len);

    if (len == 0 || buf[0] == '/' || buf[0] == '!') {
        return false; /* header line, end marker, or blank line */
    }

    const char *paren = strchr(buf, '(');
    if (paren == NULL || paren == buf) {
        return false; /* no OBIS/value structure on this line */
    }

    size_t obis_len = (size_t)(paren - buf);
    if (obis_len >= P1_OBIS_MAX_LEN) {
        return false; /* OBIS code longer than we expect - reject rather
                          than silently truncate/misreport */
    }
    memcpy(out_field->obis, buf, obis_len);
    out_field->obis[obis_len] = '\0';

    out_field->value_count = 0;
    const char *p = paren;

    while (*p == '(' && out_field->value_count < P1_MAX_VALUES_PER_LINE) {
        const char *close = strchr(p, ')');
        if (close == NULL) {
            break; /* unterminated group - stop, keep what we have */
        }
        size_t vlen = (size_t)(close - p - 1);
        if (vlen >= P1_VALUE_MAX_LEN) {
            vlen = P1_VALUE_MAX_LEN - 1; /* truncate defensively */
        }
        char *dst = out_field->values[out_field->value_count];
        memcpy(dst, p + 1, vlen);
        dst[vlen] = '\0';
        out_field->value_count++;

        p = close + 1;
    }

    return out_field->value_count > 0;
}

/* ================= telegram parser =================================== */

p1_status_t p1_parse_telegram(const char *buf, size_t len,
                               p1_field_cb cb, void *user_ctx)
{
    if (buf == NULL || len == 0) {
        return P1_ERR_TRUNCATED;
    }

    /* Locate the leading '/' identification line. */
    size_t start = 0;
    while (start < len && buf[start] != '/') {
        start++;
    }
    if (start == len) {
        return P1_ERR_NO_START;
    }

    /* Locate the '!' end marker after start. */
    size_t bang = start;
    while (bang < len && buf[bang] != '!') {
        bang++;
    }
    if (bang == len) {
        return P1_ERR_NO_END;
    }
    if (bang + 4 >= len) {
        return P1_ERR_TRUNCATED; /* not enough room for 4 CRC hex digits */
    }

    /* Parse the 4 hex digits following '!'. */
    uint16_t telegram_crc = 0;
    for (int i = 0; i < 4; i++) {
        int nib = p1_hex_nibble(buf[bang + 1 + i]);
        if (nib < 0) {
            return P1_ERR_TRUNCATED;
        }
        telegram_crc = (uint16_t)((telegram_crc << 4) | (uint16_t)nib);
    }

    /* CRC covers everything from '/' through '!' inclusive. */
    uint16_t computed_crc = p1_crc16((const uint8_t *)&buf[start], bang - start + 1);
    if (computed_crc != telegram_crc) {
        return P1_ERR_CRC_MISMATCH;
    }

    /* Walk lines between the header and the '!' end marker. */
    size_t i = start;
    /* Skip the identification line itself (up to its newline). */
    while (i < bang && buf[i] != '\n') {
        i++;
    }
    if (i < bang) {
        i++; /* move past '\n' */
    }

    char line[P1_OBIS_MAX_LEN + (P1_MAX_VALUES_PER_LINE * (P1_VALUE_MAX_LEN + 2)) + 8];

    while (i < bang) {
        size_t line_len = 0;
        while (i < bang && buf[i] != '\n' && line_len < sizeof(line) - 1) {
            line[line_len++] = buf[i++];
        }
        line[line_len] = '\0';
        if (i < bang && buf[i] == '\n') {
            i++;
        }

        p1_field_t field;
        if (p1_parse_line(line, &field) && cb != NULL) {
            cb(&field, user_ctx);
        }
    }

    return P1_OK;
}

/* ================= OBIS lookup table ================================== */

typedef struct {
    const char *code;   /* canonical form, no trailing .255 */
    const char *desc;
} p1_obis_entry_t;

static const p1_obis_entry_t p1_obis_table[] = {
    { "1-3:0.2.8",     "DSMR version" },
    { "0-0:1.0.0",     "Timestamp" },
    { "0-0:96.1.1",    "Equipment identifier (meter serial)" },
    { "1-0:1.8.1",     "Energy delivered to client, tariff 1 (kWh)" },
    { "1-0:1.8.2",     "Energy delivered to client, tariff 2 (kWh)" },
    { "1-0:2.8.1",     "Energy delivered by client, tariff 1 (kWh)" },
    { "1-0:2.8.2",     "Energy delivered by client, tariff 2 (kWh)" },
    { "0-0:96.14.0",   "Tariff indicator electricity" },
    { "1-0:1.7.0",     "Actual power delivered (kW)" },
    { "1-0:2.7.0",     "Actual power received (kW)" },
    { "0-0:96.7.21",   "Number of power failures" },
    { "0-0:96.7.9",    "Number of long power failures" },
    { "1-0:99.97.0",   "Power failure event log" },
    { "1-0:32.32.0",   "Number of voltage sags L1" },
    { "1-0:52.32.0",   "Number of voltage sags L2" },
    { "1-0:72.32.0",   "Number of voltage sags L3" },
    { "1-0:32.36.0",   "Number of voltage swells L1" },
    { "1-0:52.36.0",   "Number of voltage swells L2" },
    { "1-0:72.36.0",   "Number of voltage swells L3" },
    { "0-0:96.13.0",   "Text message" },
    { "1-0:32.7.0",    "Instantaneous voltage L1 (V)" },
    { "1-0:52.7.0",    "Instantaneous voltage L2 (V)" },
    { "1-0:72.7.0",    "Instantaneous voltage L3 (V)" },
    { "1-0:31.7.0",    "Instantaneous current L1 (A)" },
    { "1-0:51.7.0",    "Instantaneous current L2 (A)" },
    { "1-0:71.7.0",    "Instantaneous current L3 (A)" },
    { "1-0:21.7.0",    "Instantaneous active power+ L1 (kW)" },
    { "1-0:41.7.0",    "Instantaneous active power+ L2 (kW)" },
    { "1-0:61.7.0",    "Instantaneous active power+ L3 (kW)" },
    { "1-0:22.7.0",    "Instantaneous active power- L1 (kW)" },
    { "1-0:42.7.0",    "Instantaneous active power- L2 (kW)" },
    { "1-0:62.7.0",    "Instantaneous active power- L3 (kW)" },
    { "0-1:24.1.0",    "Device type (gas meter)" },
    { "0-1:96.1.0",    "Gas meter equipment identifier" },
    { "0-1:24.2.1",    "Gas meter reading (timestamp + m3)" },
    { "0-1:24.4.0",    "Gas valve state" },
};

const char *p1_obis_describe(const char *obis)
{
    if (obis == NULL) {
        return NULL;
    }

    char canon[P1_OBIS_MAX_LEN];
    size_t len = strlen(obis);
    if (len >= sizeof(canon)) {
        len = sizeof(canon) - 1;
    }
    memcpy(canon, obis, len);
    canon[len] = '\0';

    /* Strip a trailing ".255" channel suffix, if present. */
    static const char suffix[] = ".255";
    size_t slen = sizeof(suffix) - 1;
    if (len > slen && strcmp(canon + len - slen, suffix) == 0) {
        canon[len - slen] = '\0';
    }

    size_t table_size = sizeof(p1_obis_table) / sizeof(p1_obis_table[0]);
    for (size_t i = 0; i < table_size; i++) {
        if (strcmp(canon, p1_obis_table[i].code) == 0) {
            return p1_obis_table[i].desc;
        }
    }
    return NULL;
}
