#include "swim_pio_waveform.h"

#include <stdio.h>
#include <string.h>

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "swim_waveform.pio.h"

#define SWIM_PIO_TICKS_PER_US 10u
#define SWIM_PIO_SEGMENT_OVERHEAD_TICKS 7u
#define SWIM_PIO_MAX_SEGMENTS 32u

typedef struct {
    PIO pio;
    uint sm;
    uint offset;
    int dma_channel;
    uint swim_pin;
    bool initialized;
    bool program_loaded;
    bool internal_pullup;
    char error[64];
} swim_pio_state_t;

static swim_pio_state_t g_pio = {
    .pio = pio0,
    .sm = 0,
    .offset = 0,
    .dma_channel = -1,
    .swim_pin = 0,
    .initialized = false,
    .program_loaded = false,
    .internal_pullup = false,
    .error = "not initialized",
};

static void set_error(const char *message) {
    snprintf(g_pio.error, sizeof(g_pio.error), "%s", message);
}

static void release_pin(uint pin, bool pullup) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    if (pullup) {
        gpio_pull_up(pin);
    } else {
        gpio_disable_pulls(pin);
    }
}

static bool try_claim_pio(PIO pio) {
    int sm = pio_claim_unused_sm(pio, false);
    if (sm < 0) {
        return false;
    }
    if (!pio_can_add_program(pio, &swim_waveform_program)) {
        pio_sm_unclaim(pio, (uint)sm);
        return false;
    }
    g_pio.pio = pio;
    g_pio.sm = (uint)sm;
    g_pio.offset = pio_add_program(pio, &swim_waveform_program);
    g_pio.program_loaded = true;
    return true;
}

rpsw_status_t swim_pio_waveform_init(uint swim_pin, bool enable_internal_pullup) {
    if (swim_pin >= NUM_BANK0_GPIOS) {
        set_error("bad SWIM GPIO");
        return RPSW_ERR_BAD_ARGUMENT;
    }

    if (!g_pio.initialized) {
        if (!try_claim_pio(pio0) && !try_claim_pio(pio1)) {
            set_error("no PIO state machine/program space");
            release_pin(swim_pin, enable_internal_pullup);
            return RPSW_ERR_INTERNAL;
        }
        g_pio.dma_channel = dma_claim_unused_channel(false);
        if (g_pio.dma_channel < 0) {
            pio_sm_unclaim(g_pio.pio, g_pio.sm);
            set_error("no DMA channel");
            release_pin(swim_pin, enable_internal_pullup);
            return RPSW_ERR_INTERNAL;
        }
        g_pio.initialized = true;
    }

    g_pio.swim_pin = swim_pin;
    g_pio.internal_pullup = enable_internal_pullup;

    gpio_init(swim_pin);
    gpio_put(swim_pin, 0);
    gpio_set_dir(swim_pin, GPIO_IN);
    if (enable_internal_pullup) {
        gpio_pull_up(swim_pin);
    } else {
        gpio_disable_pulls(swim_pin);
    }
    pio_gpio_init(g_pio.pio, swim_pin);
    pio_sm_set_enabled(g_pio.pio, g_pio.sm, false);
    pio_sm_clear_fifos(g_pio.pio, g_pio.sm);
    pio_sm_restart(g_pio.pio, g_pio.sm);
    pio_sm_set_consecutive_pindirs(g_pio.pio, g_pio.sm, swim_pin, 1, false);

    pio_sm_config config = swim_waveform_program_get_default_config(g_pio.offset);
    sm_config_set_set_pins(&config, swim_pin, 1);
    sm_config_set_out_shift(&config, true, false, 32);
    sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_TX);
    float div = (float)clock_get_hz(clk_sys) / (float)(SWIM_PIO_TICKS_PER_US * 1000000u);
    sm_config_set_clkdiv(&config, div);
    pio_sm_init(g_pio.pio, g_pio.sm, g_pio.offset, &config);
    pio_sm_set_enabled(g_pio.pio, g_pio.sm, true);
    set_error("ok");
    return RPSW_OK;
}

bool swim_pio_waveform_available(void) {
    return g_pio.initialized;
}

const char *swim_pio_waveform_error(void) {
    return g_pio.error;
}

static rpsw_status_t encode_segments(const swim_segment_t *segments, size_t count,
                                     uint32_t *commands, uint32_t *total_us) {
    if (segments == NULL || commands == NULL || total_us == NULL || count == 0 ||
        count > SWIM_PIO_MAX_SEGMENTS) {
        return RPSW_ERR_BAD_ARGUMENT;
    }

    uint64_t total = 0;
    for (size_t i = 0; i < count; i++) {
        if (segments[i].level != SWIM_SEG_RELEASE && segments[i].level != SWIM_SEG_LOW) {
            return RPSW_ERR_BAD_ARGUMENT;
        }
        if (segments[i].duration_us == 0) {
            return RPSW_ERR_BAD_ARGUMENT;
        }
        uint64_t requested_ticks = (uint64_t)segments[i].duration_us * SWIM_PIO_TICKS_PER_US;
        if (requested_ticks <= SWIM_PIO_SEGMENT_OVERHEAD_TICKS ||
            requested_ticks > 0x7fffffffull) {
            return RPSW_ERR_BAD_ARGUMENT;
        }
        uint32_t adjusted = (uint32_t)(requested_ticks - SWIM_PIO_SEGMENT_OVERHEAD_TICKS);
        commands[i] = (adjusted << 1) | (uint32_t)segments[i].level;
        total += segments[i].duration_us;
    }
    if (total > UINT32_MAX) {
        return RPSW_ERR_BAD_ARGUMENT;
    }
    *total_us = (uint32_t)total;
    return RPSW_OK;
}

rpsw_status_t swim_pio_emit_segments(const swim_segment_t *segments, size_t count) {
    if (!g_pio.initialized) {
        set_error("PIO waveform engine not initialized");
        return RPSW_ERR_INTERNAL;
    }

    uint32_t commands[SWIM_PIO_MAX_SEGMENTS];
    uint32_t total_us = 0;
    rpsw_status_t st = encode_segments(segments, count, commands, &total_us);
    if (st != RPSW_OK) {
        set_error("bad waveform segment list");
        return st;
    }

    pio_sm_set_enabled(g_pio.pio, g_pio.sm, false);
    pio_sm_clear_fifos(g_pio.pio, g_pio.sm);
    pio_sm_restart(g_pio.pio, g_pio.sm);
    pio_sm_exec(g_pio.pio, g_pio.sm, pio_encode_set(pio_pindirs, 0));

    dma_channel_config dma_config = dma_channel_get_default_config(g_pio.dma_channel);
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_config, true);
    channel_config_set_write_increment(&dma_config, false);
    channel_config_set_dreq(&dma_config, pio_get_dreq(g_pio.pio, g_pio.sm, true));
    dma_channel_configure(g_pio.dma_channel, &dma_config,
                          &g_pio.pio->txf[g_pio.sm], commands, count, false);

    dma_channel_start(g_pio.dma_channel);
    pio_sm_set_enabled(g_pio.pio, g_pio.sm, true);
    dma_channel_wait_for_finish_blocking(g_pio.dma_channel);

    absolute_time_t deadline = make_timeout_time_us(total_us + 1000u);
    while (!pio_sm_is_tx_fifo_empty(g_pio.pio, g_pio.sm)) {
        if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
            pio_sm_exec(g_pio.pio, g_pio.sm, pio_encode_set(pio_pindirs, 0));
            set_error("PIO waveform timeout");
            return RPSW_ERR_SWIM_TIMEOUT;
        }
        tight_loop_contents();
    }

    busy_wait_us_32(total_us + 10u);
    pio_sm_exec(g_pio.pio, g_pio.sm, pio_encode_set(pio_pindirs, 0));
    release_pin(g_pio.swim_pin, g_pio.internal_pullup);
    set_error("ok");
    return RPSW_OK;
}
