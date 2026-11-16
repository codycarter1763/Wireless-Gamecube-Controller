#pragma once
#include <stdint.h>

/**
 * Gamecube button bit masks
 */
// Byte 0
#define START_BTN 0x10
#define Y_BTN 0x08
#define X_BTN 0x04
#define B_BTN 0x02
#define A_BTN 0x01

// Byte 1
#define L_BTN 0x40
#define R_BTN 0x20
#define Z_BTN 0x10
#define D_UP_BTN 0x08
#define D_DOWN_BTN 0x04
#define D_RIGHT_BTN 0x02
#define D_LEFT_BTN 0x01

/**
 * 24-bit poll command; OR in GC_RUMBLE_BIT to run the motor
 */
#define GC_POLL       0x400300UL
#define GC_RUMBLE_BIT 0x000001UL

/**
 * Status bits
 */
#define GC_BYTE0_NEED_ORIGIN 0x20   // set -> console should send GC_CMD_ORIGIN
#define GC_BYTE1_ALWAYS_ONE  0x80   // always high in a valid response

/**
 * Joybus command bytes (first byte on the wire)
 */
#define GC_CMD_ID          0x00   // 1 byte  -> 3-byte reply
#define GC_CMD_POLL        0x40   // 3 bytes -> 8-byte reply (GC_Payload)
#define GC_CMD_ORIGIN      0x41   // 1 byte  -> 10-byte reply (GC_Origin)
#define GC_CMD_RECALIBRATE 0x42   // 3 bytes -> 10-byte reply (GC_Origin)
#define GC_CMD_RESET       0xFF   // 1 byte  -> 3-byte reply (same as ID)

/**
 * ID reply for a standard wired controller.
 * Commonly cited value - verify against your pad with a logic analyzer.
 */
#define GC_ID_BYTE0 0x09
#define GC_ID_BYTE1 0x00
#define GC_ID_BYTE2 0x03

// Gamecube protocol payload
typedef struct __attribute__((packed)) {
    uint8_t byte0;
    uint8_t byte1;
    uint8_t joy_X_val;
    uint8_t joy_Y_val;
    uint8_t C_stick_X_val;
    uint8_t C_stick_Y_val;
    uint8_t left_trigger_val;
    uint8_t right_trigger_val;
} GC_Payload;

/**
 * Origin / recalibrate response (10 bytes)
 */
typedef struct __attribute__((packed)) {
    GC_Payload state;
    uint8_t    reserved[2];     // trailing bytes, normally 0
} GC_Origin;

/**
 * ESP-NOW packets
 */
typedef enum {
    GC_PKT_INPUT  = 0,          // data.state holds latest poll
    GC_PKT_ORIGIN = 1,          // data holds origin reply
} GC_PacketType;

// Controller side -> receiver side
typedef struct __attribute__((packed)) {
    uint8_t   type;             // GC_PacketType
    uint8_t   seq;              // detect drops/reordering
    GC_Origin data;             // input uses first 8 bytes
} GC_TxPacket;

// Receiver side -> controller side
typedef struct __attribute__((packed)) {
    uint8_t rumble;             // last rumble bit seen from console
} GC_RxPacket;

/**
 * Basic sanity check on a poll response (from J. Ward's checks)
 */
static inline int gc_payload_valid(const GC_Payload *p) {
    if (!(p->byte1 & GC_BYTE1_ALWAYS_ONE)) return 0;
    if ((p->byte1 & D_UP_BTN) && (p->byte1 & D_DOWN_BTN)) return 0;
    if ((p->byte1 & D_LEFT_BTN) && (p->byte1 & D_RIGHT_BTN)) return 0;
    return 1;
}