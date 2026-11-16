#pragma once
#include <stdio.h>
#include "gc_protocol.h"

/* Format a payload like:  S Y X B A | L R Z ^ v > < | joy 128 128 | c 128 128 | L  20 R  18 */
static inline void gc_format_payload(const GC_Payload *p, char *buf, size_t len)
{
    snprintf(buf, len,
             "%c%c%c%c%c %c%c%c %c%c%c%c | joy %3u %3u | c %3u %3u | L %3u R %3u",
             (p->byte0 & START_BTN)   ? 'S' : '-',
             (p->byte0 & Y_BTN)       ? 'Y' : '-',
             (p->byte0 & X_BTN)       ? 'X' : '-',
             (p->byte0 & B_BTN)       ? 'B' : '-',
             (p->byte0 & A_BTN)       ? 'A' : '-',
             (p->byte1 & L_BTN)       ? 'L' : '-',
             (p->byte1 & R_BTN)       ? 'R' : '-',
             (p->byte1 & Z_BTN)       ? 'Z' : '-',
             (p->byte1 & D_UP_BTN)    ? '^' : '-',
             (p->byte1 & D_DOWN_BTN)  ? 'v' : '-',
             (p->byte1 & D_RIGHT_BTN) ? '>' : '-',
             (p->byte1 & D_LEFT_BTN)  ? '<' : '-',
             p->joy_X_val, p->joy_Y_val,
             p->C_stick_X_val, p->C_stick_Y_val,
             p->left_trigger_val, p->right_trigger_val);
}