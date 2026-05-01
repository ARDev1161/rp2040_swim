#include "usb_protocol.h"

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"

static bool read_exact(uint8_t *dst, size_t len) {
    for (size_t i = 0; i < len; i++) {
        int c = getchar_timeout_us(500000);
        if (c == PICO_ERROR_TIMEOUT) {
            return false;
        }
        dst[i] = (uint8_t)c;
    }
    return true;
}

uint32_t rpsw_crc32(const uint8_t *data, size_t len) {
    uint32_t crc = 0xffffffffu;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; bit++) {
            uint32_t mask = 0u - (crc & 1u);
            crc = (crc >> 1) ^ (0xedb88320u & mask);
        }
    }
    return ~crc;
}

bool rpsw_read_frame(rpsw_frame_t *frame) {
    uint8_t header[10];
    if (!read_exact(header, sizeof(header))) {
        return false;
    }

    uint32_t magic = rpsw_get_u32le(header);
    if (magic != RPSW_MAGIC || header[4] != RPSW_PROTOCOL_VERSION) {
        return false;
    }

    frame->version = header[4];
    frame->command = header[5];
    frame->sequence = rpsw_get_u16le(&header[6]);
    frame->length = rpsw_get_u16le(&header[8]);
    if (frame->length > RPSW_MAX_PAYLOAD) {
        return false;
    }
    if (!read_exact(frame->payload, frame->length)) {
        return false;
    }

    uint8_t crc_bytes[4];
    if (!read_exact(crc_bytes, sizeof(crc_bytes))) {
        return false;
    }

    uint8_t crc_input[10 + RPSW_MAX_PAYLOAD];
    memcpy(crc_input, header, sizeof(header));
    memcpy(crc_input + sizeof(header), frame->payload, frame->length);
    uint32_t expected = rpsw_get_u32le(crc_bytes);
    return rpsw_crc32(crc_input, sizeof(header) + frame->length) == expected;
}

void rpsw_write_response(uint8_t command, uint16_t sequence, rpsw_status_t status,
                         const uint8_t *payload, uint16_t payload_len) {
    uint8_t header[12];
    rpsw_put_u32le(&header[0], RPSW_MAGIC);
    header[4] = RPSW_PROTOCOL_VERSION;
    header[5] = command | 0x80u;
    rpsw_put_u16le(&header[6], sequence);
    rpsw_put_u16le(&header[8], (uint16_t)(payload_len + 2u));
    rpsw_put_u16le(&header[10], (uint16_t)status);

    uint32_t crc = 0xffffffffu;
    uint8_t crc_buf[12 + RPSW_MAX_PAYLOAD];
    memcpy(crc_buf, header, sizeof(header));
    if (payload_len > 0 && payload != NULL) {
        memcpy(crc_buf + sizeof(header), payload, payload_len);
    }
    crc = rpsw_crc32(crc_buf, sizeof(header) + payload_len);

    for (size_t i = 0; i < sizeof(header); i++) {
        putchar_raw(header[i]);
    }
    for (uint16_t i = 0; i < payload_len; i++) {
        putchar_raw(payload[i]);
    }
    uint8_t crc_bytes[4];
    rpsw_put_u32le(crc_bytes, crc);
    for (size_t i = 0; i < sizeof(crc_bytes); i++) {
        putchar_raw(crc_bytes[i]);
    }
    stdio_flush();
}

const char *rpsw_status_text(rpsw_status_t status) {
    switch (status) {
    case RPSW_OK: return "ok";
    case RPSW_ERR_BAD_FRAME: return "bad frame";
    case RPSW_ERR_BAD_CRC: return "bad crc";
    case RPSW_ERR_BAD_COMMAND: return "bad command";
    case RPSW_ERR_BAD_ARGUMENT: return "bad argument";
    case RPSW_ERR_SWIM_TIMEOUT: return "swim timeout";
    case RPSW_ERR_SWIM_NACK: return "swim nack";
    case RPSW_ERR_TARGET: return "target error";
    case RPSW_ERR_UNSUPPORTED: return "unsupported";
    case RPSW_ERR_FLASH_GUARD: return "flash operation guarded";
    case RPSW_ERR_INTERNAL: return "internal error";
    default: return "unknown";
    }
}
