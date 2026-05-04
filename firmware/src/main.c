#include <stdio.h>
#include <string.h>

#include "board_rp2040_zero.h"
#include "pico/stdlib.h"
#include "stm8_dm.h"
#include "stm8_flash.h"
#include "swim_link.h"
#include "swim_phy.h"
#include "usb_protocol.h"

static char g_last_error[96] = "none";

static uint16_t bounded_strlen(const char *s, uint16_t max_len) {
    uint16_t len = 0;
    while (len < max_len && s[len] != '\0') {
        len++;
    }
    return len;
}

static void set_last_error(rpsw_status_t status) {
    const char *flash_error = stm8_flash_last_error();
    if (flash_error != NULL && flash_error[0] != '\0') {
        snprintf(g_last_error, sizeof(g_last_error), "%s: %s",
                 rpsw_status_text(status), flash_error);
    } else {
        snprintf(g_last_error, sizeof(g_last_error), "%s", rpsw_status_text(status));
    }
}

static rpsw_status_t handle_frame(const rpsw_frame_t *request,
                                  uint8_t *response,
                                  uint16_t *response_len) {
    *response_len = 0;
    switch (request->command) {
    case CMD_GET_VERSION:
        response[0] = RP2040_SWIM_FW_VERSION_MAJOR;
        response[1] = RP2040_SWIM_FW_VERSION_MINOR;
        response[2] = RP2040_SWIM_FW_VERSION_PATCH;
        response[3] = RPSW_PROTOCOL_VERSION;
        memcpy(&response[4], RP2040_SWIM_FW_NAME, sizeof(RP2040_SWIM_FW_NAME));
        *response_len = 4u + (uint16_t)sizeof(RP2040_SWIM_FW_NAME);
        return RPSW_OK;

    case CMD_SET_PINS:
        if (request->length < 3) {
            return RPSW_ERR_BAD_ARGUMENT;
        }
        swim_phy_set_pins(request->payload[0], request->payload[1],
                          request->payload[2] != 0u);
        return RPSW_OK;

    case CMD_SET_SPEED:
        if (request->length < 1 || request->payload[0] > SWIM_SPEED_HIGH) {
            return RPSW_ERR_BAD_ARGUMENT;
        }
        return swim_phy_set_speed((swim_speed_t)request->payload[0])
                   ? RPSW_OK
                   : RPSW_ERR_UNSUPPORTED;

    case CMD_ENTER_SWIM:
        return swim_link_enter();

    case CMD_RESET_TARGET:
        swim_phy_release();
        swim_phy_nrst_assert();
        sleep_ms(20);
        swim_phy_nrst_release();
        swim_phy_release_target();
        return RPSW_OK;

    case CMD_RELEASE_TARGET:
        swim_phy_release_target();
        return RPSW_OK;

    case CMD_SWIM_READ:
    case CMD_MEMORY_READ: {
        if (request->length < 6) {
            return RPSW_ERR_BAD_ARGUMENT;
        }
        uint32_t address = rpsw_get_u32le(&request->payload[0]);
        uint16_t len = rpsw_get_u16le(&request->payload[4]);
        if (len > RPSW_MAX_PAYLOAD) {
            return RPSW_ERR_BAD_ARGUMENT;
        }
        rpsw_status_t st = stm8_dm_memory_read(address, response, len);
        if (st == RPSW_OK) {
            *response_len = len;
        }
        return st;
    }

    case CMD_SWIM_WRITE:
    case CMD_MEMORY_WRITE: {
        if (request->length < 6) {
            return RPSW_ERR_BAD_ARGUMENT;
        }
        uint32_t address = rpsw_get_u32le(&request->payload[0]);
        uint16_t len = rpsw_get_u16le(&request->payload[4]);
        if (request->length != (uint16_t)(6u + len)) {
            return RPSW_ERR_BAD_ARGUMENT;
        }
        return stm8_dm_memory_write(address, &request->payload[6], len);
    }

    case CMD_FLASH_ERASE:
        if (request->length < 8) {
            return RPSW_ERR_BAD_ARGUMENT;
        }
        return stm8_flash_erase_range(rpsw_get_u32le(&request->payload[0]),
                                      rpsw_get_u32le(&request->payload[4]));

    case CMD_FLASH_WRITE_BLOCK: {
        if (request->length < 6) {
            return RPSW_ERR_BAD_ARGUMENT;
        }
        uint16_t len = rpsw_get_u16le(&request->payload[4]);
        if (request->length != (uint16_t)(6u + len)) {
            return RPSW_ERR_BAD_ARGUMENT;
        }
        return stm8_flash_write_block(rpsw_get_u32le(&request->payload[0]),
                                      &request->payload[6],
                                      len);
    }

    case CMD_OPTION_WRITE_BYTE:
        if (request->length < 5) {
            return RPSW_ERR_BAD_ARGUMENT;
        }
        return stm8_flash_write_option_byte(rpsw_get_u32le(&request->payload[0]),
                                            request->payload[4]);

    case CMD_FLASH_VERIFY:
        return RPSW_ERR_UNSUPPORTED;

    case CMD_GET_LAST_ERROR:
        *response_len = bounded_strlen(g_last_error, sizeof(g_last_error));
        memcpy(response, g_last_error, *response_len);
        return RPSW_OK;

    case CMD_DEBUG_WAVEFORM:
        swim_phy_debug_waveform();
        return RPSW_OK;

    case CMD_ENTRY_WAVEFORM: {
        if (request->length < 4) {
            return RPSW_ERR_BAD_ARGUMENT;
        }
        uint32_t delay_ms = rpsw_get_u32le(&request->payload[0]);
        if (delay_ms > 60000u) {
            return RPSW_ERR_BAD_ARGUMENT;
        }
        sleep_ms(delay_ms);
        swim_phy_reset_timing();
        swim_phy_release();
        swim_phy_nrst_assert();
        sleep_ms(2);
        rpsw_status_t st = swim_phy_entry_sequence_um0470();
        swim_phy_release();
        swim_phy_nrst_release();
        return st;
    }

    case CMD_GET_SWIM_DEBUG: {
        swim_phy_debug_t debug = swim_phy_get_debug();

        response[0] = debug.synced ? 1u : 0u;
        response[1] = (uint8_t)debug.speed;
        response[2] = debug.swim_csr_valid ? 1u : 0u;
        response[3] = debug.swim_csr;

        rpsw_put_u32le(&response[4], debug.last_sync_low_us);
        rpsw_put_u32le(&response[8], debug.last_sync_low_ns);
        rpsw_put_u32le(&response[12], debug.derived_tswim_ns);
        rpsw_put_u32le(&response[16], debug.sync_low_loop_count);

        response[20] = debug.phy_backend;
        response[21] = debug.pio_init_ok ? 1u : 0u;
        response[22] = debug.entry_slow_pulses;
        response[23] = debug.entry_fast_pulses;
        rpsw_put_u32le(&response[24], debug.entry_protocol_us);

        /*
         * Extended enter/debug fields.
         *
         * Keep old fields at 0..28 stable enough for older host tools, append new
         * fields after entry_protocol_us.
         */
        response[28] = (uint8_t)debug.enter_stage;
        response[29] = debug.comm_reset_sent ? 1u : 0u;
        response[30] = debug.second_sync_seen ? 1u : 0u;
        response[31] = 0u; /* reserved/alignment */

        rpsw_put_u32le(&response[32], debug.comm_reset_low_us);
        rpsw_put_u32le(&response[36], debug.comm_reset_low_ns);

        uint16_t pio_error_len = bounded_strlen(debug.pio_error, sizeof(debug.pio_error));
        if (pio_error_len > 64u) {
            pio_error_len = 64u;
        }

        response[40] = (uint8_t)pio_error_len;
        memcpy(&response[41], debug.pio_error, pio_error_len);
        *response_len = (uint16_t)(41u + pio_error_len);
        return RPSW_OK;
    }

    default:
        return RPSW_ERR_BAD_COMMAND;
    }
}

int main(void) {
    stdio_init_all();
    swim_phy_config_t config = {
        .swim_pin = RP2040_SWIM_DEFAULT_SWIM_PIN,
        .nrst_pin = RP2040_SWIM_DEFAULT_NRST_PIN,
        .internal_pullup = false,
        .active_drive_high = false,
        .speed = SWIM_SPEED_LOW,
    };
    swim_phy_init(&config);

    uint8_t response[RPSW_MAX_PAYLOAD];
    while (true) {
        rpsw_frame_t frame;
        if (!rpsw_read_frame(&frame)) {
            tight_loop_contents();
            continue;
        }

        uint16_t response_len = 0;
        rpsw_status_t status = handle_frame(&frame, response, &response_len);
        if (status != RPSW_OK) {
            set_last_error(status);
            response_len = 0;
        }
        rpsw_write_response(frame.command, frame.sequence, status, response, response_len);
    }
}
