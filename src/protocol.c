#include "protocol.h"

#include <string.h>

// Nibble-table CRC: 32 bytes of flash, ~2x faster than bitwise. Fine for an M0.
static const uint16_t crc_nib[16] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
};

uint16_t proto_crc16(const uint8_t *p, uint32_t len, uint16_t crc)
{
    while (len--) {
        crc = (uint16_t)((crc << 4) ^ crc_nib[(crc >> 12) ^ (*p >> 4)]);
        crc = (uint16_t)((crc << 4) ^ crc_nib[(crc >> 12) ^ (*p & 0x0F)]);
        p++;
    }
    return crc;
}

uint32_t proto_frame(uint8_t *out, uint8_t type, const void *payload, uint8_t len)
{
    out[0] = PROTO_SYNC0;
    out[1] = PROTO_SYNC1;
    out[2] = type;
    out[3] = len;
    memcpy(&out[4], payload, len);
    uint16_t crc = proto_crc16(&out[2], 2u + len, 0xFFFF);
    out[4 + len] = (uint8_t)crc;
    out[5 + len] = (uint8_t)(crc >> 8);
    return len + PROTO_OVERHEAD;
}
