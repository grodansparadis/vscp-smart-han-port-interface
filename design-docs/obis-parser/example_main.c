#include <stdio.h>
#include "p1_parser.h"

/* Sample DSMR telegram (CRLF line endings, as real meters send). */
static const char sample_telegram[] =
    "/ISK5\\2M550T-1012\r\n"
    "\r\n"
    "1-3:0.2.8(50)\r\n"
    "0-0:1.0.0(230101120000W)\r\n"
    "0-0:96.1.1(4530303236303030303331303231363139)\r\n"
    "1-0:1.8.1(000123.456*kWh)\r\n"
    "1-0:1.8.2(000234.567*kWh)\r\n"
    "1-0:1.7.0(00.424*kW)\r\n"
    "0-1:24.1.0(003)\r\n"
    "0-1:96.1.0(4730303139303030303331323334353637)\r\n"
    "0-1:24.2.1(230101120000W)(00123.456*m3)\r\n"
    "!7397\r\n";

static void on_field(const p1_field_t *field, void *ctx)
{
    (void)ctx;
    const char *desc = p1_obis_describe(field->obis);

    printf("%-16s %-24s :", field->obis, desc ? desc : "(unknown OBIS code)");
    for (uint8_t i = 0; i < field->value_count; i++) {
        printf(" [%s]", field->values[i]);
    }
    printf("\n");
}

int main(void)
{
    p1_status_t status = p1_parse_telegram(
        sample_telegram, sizeof(sample_telegram) - 1, on_field, NULL);

    if (status != P1_OK) {
        printf("Parse failed, status=%d\n", status);
        return 1;
    }
    return 0;
}
