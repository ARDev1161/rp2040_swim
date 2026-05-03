#include "swim_pio_rx.h"

#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "swim_rx_width.pio.h"
#include "swim_rx_decode.pio.h"

#define SWIM_RX_LOOP_CYCLES 2u
#define SWIM_RX_BURST_DEBUG_WIDTHS 12u

static PIO g_rx_pio = pio0;
static uint g_rx_sm = 0;
static uint g_frame_sm = 0;
static uint g_rx_offset = 0;
static uint g_rx_pin = 0;
static bool g_rx_initialized = false;

typedef enum {
    SWIM_RX_PROGRAM_WIDTH = 0,
    SWIM_RX_PROGRAM_DECODE = 1,
} swim_rx_program_mode_t;

static swim_rx_program_mode_t g_rx_program_mode = SWIM_RX_PROGRAM_WIDTH;

static uint32_t g_burst_requested_count = 0;
static uint32_t g_burst_captured_count = 0;
static uint32_t g_burst_timeout_index = 0xffffffffu;
static uint32_t g_burst_first_low_us[SWIM_RX_BURST_DEBUG_WIDTHS] = {0};
static uint32_t g_burst_first_low_ns[SWIM_RX_BURST_DEBUG_WIDTHS] = {0};

static uint32_t g_frame_last_max_loop_count = 0;

/* max_loop_count from the last arm call, needed to convert remaining X to loops_used */
static uint32_t g_rx_last_max_loop_count = 0;

static void restart_rx_sm_at_program_start(void) {
    pio_sm_restart(g_rx_pio, g_rx_sm);
    pio_sm_exec(g_rx_pio, g_rx_sm, pio_encode_jmp(g_rx_offset));
}

static void configure_width_sms(void) {
    pio_sm_config c = swim_rx_width_program_get_default_config(g_rx_offset);
    sm_config_set_in_pins(&c, g_rx_pin);
    sm_config_set_jmp_pin(&c, g_rx_pin);
    sm_config_set_clkdiv(&c, 1.0f);
    pio_sm_init(g_rx_pio, g_rx_sm, g_rx_offset, &c);
    pio_sm_set_enabled(g_rx_pio, g_rx_sm, false);

    pio_sm_config frame_config = swim_rx_width_program_get_default_config(g_rx_offset);
    sm_config_set_in_pins(&frame_config, g_rx_pin);
    sm_config_set_jmp_pin(&frame_config, g_rx_pin);
    sm_config_set_in_shift(&frame_config, false, false, 32);
    sm_config_set_clkdiv(&frame_config, 1.0f);
    pio_sm_init(g_rx_pio, g_frame_sm, g_rx_offset, &frame_config);
    pio_sm_set_enabled(g_rx_pio, g_frame_sm, false);
}

static void configure_decode_sm(void) {
    pio_sm_config c = swim_rx_decode_program_get_default_config(g_rx_offset);
    sm_config_set_in_pins(&c, g_rx_pin);
    sm_config_set_jmp_pin(&c, g_rx_pin);
    sm_config_set_in_shift(&c, false, false, 32);
    sm_config_set_clkdiv(&c, 1.0f);
    pio_sm_init(g_rx_pio, g_rx_sm, g_rx_offset, &c);
    pio_sm_set_enabled(g_rx_pio, g_rx_sm, false);

    pio_sm_set_enabled(g_rx_pio, g_frame_sm, false);
    pio_sm_clear_fifos(g_rx_pio, g_frame_sm);
}

static rpsw_status_t switch_rx_program(swim_rx_program_mode_t mode) {
    if (!g_rx_initialized) {
        return RPSW_ERR_INTERNAL;
    }
    if (g_rx_program_mode == mode) {
        return RPSW_OK;
    }

    pio_sm_set_enabled(g_rx_pio, g_rx_sm, false);
    pio_sm_set_enabled(g_rx_pio, g_frame_sm, false);
    pio_sm_clear_fifos(g_rx_pio, g_rx_sm);
    pio_sm_clear_fifos(g_rx_pio, g_frame_sm);

    if (g_rx_program_mode == SWIM_RX_PROGRAM_WIDTH) {
        pio_remove_program(g_rx_pio, &swim_rx_width_program, g_rx_offset);
    } else {
        pio_remove_program(g_rx_pio, &swim_rx_decode_program, g_rx_offset);
    }

    if (mode == SWIM_RX_PROGRAM_WIDTH) {
        if (!pio_can_add_program(g_rx_pio, &swim_rx_width_program)) {
            return RPSW_ERR_INTERNAL;
        }
        g_rx_offset = pio_add_program(g_rx_pio, &swim_rx_width_program);
        configure_width_sms();
    } else {
        if (!pio_can_add_program(g_rx_pio, &swim_rx_decode_program)) {
            return RPSW_ERR_INTERNAL;
        }
        g_rx_offset = pio_add_program(g_rx_pio, &swim_rx_decode_program);
        configure_decode_sm();
    }

    g_rx_program_mode = mode;
    return RPSW_OK;
}

rpsw_status_t swim_pio_rx_init(PIO pio, uint swim_pin) {
    if (swim_pin >= NUM_BANK0_GPIOS) {
        return RPSW_ERR_BAD_ARGUMENT;
    }

    if (g_rx_initialized) {
        if (g_rx_pio == pio && g_rx_pin == swim_pin) {
            return RPSW_OK;
        }
        return RPSW_ERR_INTERNAL;
    }

    g_rx_pin = swim_pin;
    g_rx_pio = pio;

    int sm = pio_claim_unused_sm(g_rx_pio, false);
    if (sm < 0) {
        return RPSW_ERR_INTERNAL;
    }
    int frame_sm = pio_claim_unused_sm(g_rx_pio, false);
    if (frame_sm < 0) {
        pio_sm_unclaim(g_rx_pio, (uint)sm);
        return RPSW_ERR_INTERNAL;
    }

    if (!pio_can_add_program(g_rx_pio, &swim_rx_width_program)) {
        pio_sm_unclaim(g_rx_pio, (uint)sm);
        pio_sm_unclaim(g_rx_pio, (uint)frame_sm);
        return RPSW_ERR_INTERNAL;
    }

    g_rx_sm = (uint)sm;
    g_frame_sm = (uint)frame_sm;
    g_rx_offset = pio_add_program(g_rx_pio, &swim_rx_width_program);

    gpio_init(g_rx_pin);
    gpio_set_dir(g_rx_pin, GPIO_IN);

    pio_gpio_init(g_rx_pio, g_rx_pin);
    pio_sm_set_consecutive_pindirs(g_rx_pio, g_rx_sm, g_rx_pin, 1, false);

    configure_width_sms();
    g_rx_program_mode = SWIM_RX_PROGRAM_WIDTH;

    g_rx_initialized = true;
    return RPSW_OK;
}

uint32_t swim_pio_rx_ns_to_max_loops(uint32_t max_ns) {
    uint32_t clk_hz = clock_get_hz(clk_sys);
    uint64_t cycles = ((uint64_t)clk_hz * (uint64_t)max_ns + 999999999ull) / 1000000000ull;
    uint64_t loops = (cycles + SWIM_RX_LOOP_CYCLES - 1u) / SWIM_RX_LOOP_CYCLES;
    if (loops == 0) {
        loops = 1;
    }
    if (loops > 0x7fffffffu) {
        loops = 0x7fffffffu;
    }
    return (uint32_t)loops;
}

uint32_t swim_pio_rx_loops_to_ns(uint32_t loops) {
    uint32_t clk_hz = clock_get_hz(clk_sys);
    uint64_t cycles = (uint64_t)loops * SWIM_RX_LOOP_CYCLES;
    uint64_t ns = (cycles * 1000000000ull + (clk_hz / 2u)) / clk_hz;
    if (ns > UINT32_MAX) {
        return UINT32_MAX;
    }
    return (uint32_t)ns;
}

rpsw_status_t swim_pio_rx_arm_after_tx_done(uint32_t max_loop_count) {
    if (!g_rx_initialized) {
        return RPSW_ERR_INTERNAL;
    }
    rpsw_status_t mode_st = switch_rx_program(SWIM_RX_PROGRAM_WIDTH);
    if (mode_st != RPSW_OK) {
        return mode_st;
    }
    if (max_loop_count == 0 || max_loop_count > 0x7fffffffu) {
        return RPSW_ERR_BAD_ARGUMENT;
    }
    g_rx_last_max_loop_count = max_loop_count;

    pio_sm_set_enabled(g_rx_pio, g_rx_sm, false);
    pio_sm_clear_fifos(g_rx_pio, g_rx_sm);
    restart_rx_sm_at_program_start();

    /*
     * Important: clear IRQ0 before arming. TX waveform will set IRQ0 after
     * releasing SWIM, so RX starts waiting for target response immediately.
     */
    pio_interrupt_clear(g_rx_pio, 0);
    pio_sm_put_blocking(g_rx_pio, g_rx_sm, 0u);              // count_minus_1
    pio_sm_put_blocking(g_rx_pio, g_rx_sm, max_loop_count);
    pio_sm_set_enabled(g_rx_pio, g_rx_sm, true);

    return RPSW_OK;
}

rpsw_status_t swim_pio_rx_arm_now(uint32_t max_loop_count) {
    rpsw_status_t st = swim_pio_rx_arm_after_tx_done(max_loop_count);
    if (st != RPSW_OK) {
        return st;
    }

    /*
     * RX program waits for IRQ0 before waiting for falling edge.
     * For direct bit reads there is no preceding TX waveform, so wake it now.
     */
    pio_sm_exec(g_rx_pio, g_rx_sm, pio_encode_irq_set(false, 0));
    return RPSW_OK;
}

rpsw_status_t swim_pio_rx_get_width(swim_pio_rx_width_t *out, uint32_t timeout_us) {
    if (out == NULL) {
        return RPSW_ERR_BAD_ARGUMENT;
    }
    if (!g_rx_initialized) {
        return RPSW_ERR_INTERNAL;
    }

    absolute_time_t deadline = make_timeout_time_us(timeout_us);

    while (pio_sm_is_rx_fifo_empty(g_rx_pio, g_rx_sm)) {
        if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
            out->low_ticks = 0;
            out->low_ns = 0;
            out->low_us = 0;
            out->loops_used = 0;
            out->timeout = true;
            pio_sm_set_enabled(g_rx_pio, g_rx_sm, false);
            return RPSW_ERR_SWIM_TIMEOUT;
        }
        tight_loop_contents();
    }

    uint32_t remaining = pio_sm_get_blocking(g_rx_pio, g_rx_sm);

    uint32_t loops_used = 0;
    if (g_rx_last_max_loop_count >= remaining) {
        loops_used = g_rx_last_max_loop_count - remaining;
    }

    out->loops_used = loops_used;
    out->low_ticks = loops_used;
    out->low_ns = swim_pio_rx_loops_to_ns(loops_used);
    out->low_us = (out->low_ns + 999u) / 1000u;
    out->timeout = false;

    pio_sm_set_enabled(g_rx_pio, g_rx_sm, false);
    return RPSW_OK;
}

rpsw_status_t swim_pio_rx_arm_width_burst_after_tx_done(uint32_t count, uint32_t max_loop_count) {
    if (!g_rx_initialized || count == 0u) {
        return RPSW_ERR_BAD_ARGUMENT;
    }
    rpsw_status_t mode_st = switch_rx_program(SWIM_RX_PROGRAM_WIDTH);
    if (mode_st != RPSW_OK) {
        return mode_st;
    }
    if (max_loop_count == 0u || max_loop_count > 0x7fffffffu) {
        return RPSW_ERR_BAD_ARGUMENT;
    }

    g_frame_last_max_loop_count = max_loop_count;

    pio_sm_set_enabled(g_rx_pio, g_frame_sm, false);
    pio_sm_clear_fifos(g_rx_pio, g_frame_sm);
    pio_sm_restart(g_rx_pio, g_frame_sm);
    pio_sm_exec(g_rx_pio, g_frame_sm, pio_encode_jmp(g_rx_offset));

    pio_interrupt_clear(g_rx_pio, 0);

    pio_sm_put_blocking(g_rx_pio, g_frame_sm, count - 1u);
    pio_sm_put_blocking(g_rx_pio, g_frame_sm, max_loop_count);

    pio_sm_set_enabled(g_rx_pio, g_frame_sm, true);
    return RPSW_OK;
}

rpsw_status_t swim_pio_rx_get_width_burst(swim_pio_rx_width_t *out,
                                          uint32_t count,
                                          uint32_t timeout_us) {
    if (out == NULL || count == 0u) {
        return RPSW_ERR_BAD_ARGUMENT;
    }

    g_burst_requested_count = count;
    g_burst_captured_count = 0u;
    g_burst_timeout_index = 0xffffffffu;

    for (uint32_t i = 0; i < SWIM_RX_BURST_DEBUG_WIDTHS; ++i) {
        g_burst_first_low_us[i] = 0u;
        g_burst_first_low_ns[i] = 0u;
    }

    absolute_time_t deadline = make_timeout_time_us(timeout_us);

    for (uint32_t i = 0; i < count; ++i) {
        while (pio_sm_is_rx_fifo_empty(g_rx_pio, g_frame_sm)) {
            if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
                g_burst_timeout_index = i;
                pio_sm_set_enabled(g_rx_pio, g_frame_sm, false);
                return RPSW_ERR_SWIM_TIMEOUT;
            }
            tight_loop_contents();
        }

        uint32_t remaining = pio_sm_get_blocking(g_rx_pio, g_frame_sm);
        uint32_t loops_used = 0u;
        if (g_frame_last_max_loop_count >= remaining) {
            loops_used = g_frame_last_max_loop_count - remaining;
        }

        out[i].loops_used = loops_used;
        out[i].low_ticks = loops_used;
        out[i].low_ns = swim_pio_rx_loops_to_ns(loops_used);
        out[i].low_us = (out[i].low_ns + 999u) / 1000u;
        out[i].timeout = false;

        g_burst_captured_count = i + 1u;

        if (i < SWIM_RX_BURST_DEBUG_WIDTHS) {
            g_burst_first_low_us[i] = out[i].low_us;
            g_burst_first_low_ns[i] = out[i].low_ns;
        }
    }

    pio_sm_set_enabled(g_rx_pio, g_frame_sm, false);
    return RPSW_OK;
}

rpsw_status_t swim_pio_rx_get_width_burst_partial(swim_pio_rx_width_t *out,
                                                  uint32_t max_count,
                                                  uint32_t timeout_us,
                                                  uint32_t *captured_count) {
    if (out == NULL || max_count == 0u || captured_count == NULL) {
        return RPSW_ERR_BAD_ARGUMENT;
    }

    *captured_count = 0u;

    g_burst_requested_count = max_count;
    g_burst_captured_count = 0u;
    g_burst_timeout_index = 0xffffffffu;

    for (uint32_t i = 0; i < SWIM_RX_BURST_DEBUG_WIDTHS; ++i) {
        g_burst_first_low_us[i] = 0u;
        g_burst_first_low_ns[i] = 0u;
    }

    absolute_time_t deadline = make_timeout_time_us(timeout_us);

    for (uint32_t i = 0; i < max_count; ++i) {
        while (pio_sm_is_rx_fifo_empty(g_rx_pio, g_frame_sm)) {
            if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
                g_burst_timeout_index = i;
                pio_sm_set_enabled(g_rx_pio, g_frame_sm, false);

                if (*captured_count > 0u) {
                    return RPSW_OK;
                }

                return RPSW_ERR_SWIM_TIMEOUT;
            }
            tight_loop_contents();
        }

        uint32_t remaining = pio_sm_get_blocking(g_rx_pio, g_frame_sm);
        uint32_t loops_used = 0u;
        if (g_frame_last_max_loop_count >= remaining) {
            loops_used = g_frame_last_max_loop_count - remaining;
        }

        out[i].loops_used = loops_used;
        out[i].low_ticks = loops_used;
        out[i].low_ns = swim_pio_rx_loops_to_ns(loops_used);
        out[i].low_us = (out[i].low_ns + 999u) / 1000u;
        out[i].timeout = false;

        *captured_count = i + 1u;
        g_burst_captured_count = *captured_count;

        if (i < SWIM_RX_BURST_DEBUG_WIDTHS) {
            g_burst_first_low_us[i] = out[i].low_us;
            g_burst_first_low_ns[i] = out[i].low_ns;
        }

        /*
         * Partial capture is a stream capture, not one dense packet.
         * After each captured low pulse, restart the timeout window so gaps between
         * target/host response groups do not prematurely end the capture.
         */
        deadline = make_timeout_time_us(timeout_us);
    }

    pio_sm_set_enabled(g_rx_pio, g_frame_sm, false);
    return RPSW_OK;
}

rpsw_status_t swim_pio_rx_arm_decode_bits_after_tx_done(uint32_t bit_count,
                                                        uint32_t threshold_loop_count) {
    if (bit_count == 0u || bit_count > 31u || threshold_loop_count == 0u) {
        return RPSW_ERR_BAD_ARGUMENT;
    }
    if (!g_rx_initialized) {
        return RPSW_ERR_INTERNAL;
    }

    rpsw_status_t mode_st = switch_rx_program(SWIM_RX_PROGRAM_DECODE);
    if (mode_st != RPSW_OK) {
        return mode_st;
    }

    pio_sm_set_enabled(g_rx_pio, g_rx_sm, false);
    pio_sm_clear_fifos(g_rx_pio, g_rx_sm);
    pio_sm_restart(g_rx_pio, g_rx_sm);
    pio_sm_exec(g_rx_pio, g_rx_sm, pio_encode_jmp(g_rx_offset));

    pio_interrupt_clear(g_rx_pio, 0);
    pio_sm_put_blocking(g_rx_pio, g_rx_sm, bit_count - 1u);
    pio_sm_put_blocking(g_rx_pio, g_rx_sm, threshold_loop_count);
    pio_sm_set_enabled(g_rx_pio, g_rx_sm, true);
    return RPSW_OK;
}

rpsw_status_t swim_pio_rx_get_decoded_bits(uint32_t timeout_us, uint32_t bit_count, uint32_t *bits) {
    if (bits == NULL || bit_count == 0u || bit_count > 31u) {
        return RPSW_ERR_BAD_ARGUMENT;
    }
    if (!g_rx_initialized) {
        return RPSW_ERR_INTERNAL;
    }

    *bits = 0u;
    absolute_time_t deadline = make_timeout_time_us(timeout_us);
    while (pio_sm_is_rx_fifo_empty(g_rx_pio, g_rx_sm)) {
        if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
            pio_sm_set_enabled(g_rx_pio, g_rx_sm, false);
            return RPSW_ERR_SWIM_TIMEOUT;
        }
        tight_loop_contents();
    }

    uint32_t raw = pio_sm_get_blocking(g_rx_pio, g_rx_sm);
    pio_sm_set_enabled(g_rx_pio, g_rx_sm, false);

    if (bit_count < 32u) {
        raw &= ((1u << bit_count) - 1u);
    }
    *bits = raw;
    return RPSW_OK;
}


uint32_t swim_pio_rx_burst_requested_count(void) {
    return g_burst_requested_count;
}

uint32_t swim_pio_rx_burst_captured_count(void) {
    return g_burst_captured_count;
}

uint32_t swim_pio_rx_burst_timeout_index(void) {
    return g_burst_timeout_index;
}

uint32_t swim_pio_rx_burst_first_low_us(uint32_t index) {
    if (index >= SWIM_RX_BURST_DEBUG_WIDTHS) {
        return 0u;
    }
    return g_burst_first_low_us[index];
}

uint32_t swim_pio_rx_burst_first_low_ns(uint32_t index) {
    if (index >= SWIM_RX_BURST_DEBUG_WIDTHS) {
        return 0u;
    }
    return g_burst_first_low_ns[index];
}
