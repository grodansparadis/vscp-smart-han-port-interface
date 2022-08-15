/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/* Attributes State Machine */
enum
{
    IDX_SVC,        // Heart Rate Service index
    IDX_CHAR_A,     // A characteristic index
    IDX_CHAR_VAL_A, // A characteristic value index
    IDX_CHAR_CFG_A, // A characteristic notifications configuration (CCC) index

    IDX_CHAR_B,     // B characteristic index
    IDX_CHAR_VAL_B, // B characteristic value index

    IDX_CHAR_C,     // C characteristic index
    IDX_CHAR_VAL_C, // C characteristic value index

    IDX_CHAR_TEMP,     // Chip temperature characteristic index
    IDX_CHAR_VAL_TEMP, // Chip temperature characteristic value index
    IDX_CHAR_CFG_TEMP, // A characteristic notifications configuration (CCC) index

    HRS_IDX_NB,     // Number of table elements.
};

typedef struct {
    uint64_t event_count;
} timing_queue_element_t;
