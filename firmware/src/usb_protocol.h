#ifndef USB_PROTOCOL_H
#define USB_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define RPSW_MAGIC 0x53575052u /* "RPWS" little-endian on wire */
#define RPSW_PROTOCOL_VERSION 1u
#define RPSW_MAX_PAYLOAD 1024u

typedef enum {
    CMD_GET_VERSION = 0x01,
    CMD_SET_PINS = 0x02,
    CMD_SET_SPEED = 0x03,
    CMD_ENTER_SWIM = 0x04,
    CMD_RESET_TARGET = 0x05,
    CMD_SWIM_READ = 0x06,
    CMD_SWIM_WRITE = 0x07,
    CMD_MEMORY_READ = 0x08,
    CMD_MEMORY_WRITE = 0x09,
    CMD_FLASH_ERASE = 0x0A,
    CMD_FLASH_WRITE_BLOCK = 0x0B,
    CMD_FLASH_VERIFY = 0x0C,
    CMD_GET_LAST_ERROR = 0x0D,
    CMD_DEBUG_WAVEFORM = 0x0E,
    CMD_GET_SWIM_DEBUG = 0x0F,
    CMD_ENTRY_WAVEFORM = 0x10,
    CMD_OPTION_WRITE_BYTE = 0x11,
    CMD_RELEASE_TARGET = 0x12
} rpsw_command_t;

typedef enum {
    RPSW_OK = 0,
    RPSW_ERR_BAD_FRAME = 1,
    RPSW_ERR_BAD_CRC = 2,
    RPSW_ERR_BAD_COMMAND = 3,
    RPSW_ERR_BAD_ARGUMENT = 4,
    RPSW_ERR_SWIM_TIMEOUT = 5,
    RPSW_ERR_SWIM_NACK = 6,
    RPSW_ERR_TARGET = 7,
    RPSW_ERR_UNSUPPORTED = 8,
    RPSW_ERR_FLASH_GUARD = 9,
    RPSW_ERR_INTERNAL = 10,
} rpsw_status_t;

typedef struct {
    uint8_t version;
    uint8_t command;
    uint16_t sequence;
    uint16_t length;
    uint8_t payload[RPSW_MAX_PAYLOAD];
} rpsw_frame_t;

typedef rpsw_status_t (*rpsw_handler_t)(const rpsw_frame_t *request,
                                        uint8_t *response,
                                        uint16_t *response_len);

uint32_t rpsw_crc32(const uint8_t *data, size_t len);
bool rpsw_read_frame(rpsw_frame_t *frame);
void rpsw_write_response(uint8_t command, uint16_t sequence, rpsw_status_t status,
                         const uint8_t *payload, uint16_t payload_len);
const char *rpsw_status_text(rpsw_status_t status);

static inline uint16_t rpsw_get_u16le(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline uint32_t rpsw_get_u32le(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline void rpsw_put_u16le(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xffu);
    p[1] = (uint8_t)(v >> 8);
}

static inline void rpsw_put_u32le(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xffu);
    p[1] = (uint8_t)((v >> 8) & 0xffu);
    p[2] = (uint8_t)((v >> 16) & 0xffu);
    p[3] = (uint8_t)((v >> 24) & 0xffu);
}

#endif
