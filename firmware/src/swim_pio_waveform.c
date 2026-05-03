#include "swim_pio_waveform.h"

#include <stdio.h>
#include <string.h>

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "hardware/structs/sio.h"
#include "pico/stdlib.h"
#include "swim_rx_width.pio.h"
#include "swim_waveform.pio.h"
#include "swim_sync_limits.h"

#define SWIM_PIO_TICKS_PER_US 10u
#define SWIM_PIO_MAX_SEGMENTS 32u
#define SWIM_PIO_COMPLETION_IRQ 0u

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

static void pio_release_swim_pin(void) {
    uint32_t mask = 1u << g_pio.swim_pin;
    pio_sm_set_pins_with_mask(g_pio.pio, g_pio.sm, 0u, mask);
    pio_sm_set_pindirs_with_mask(g_pio.pio, g_pio.sm, 0u, mask);
}

static inline bool sample_swim_pin_fast(void) {
    return (sio_hw->gpio_in & (1u << g_pio.swim_pin)) != 0u;
}

static void set_pio_tick_hz(uint32_t tick_hz) {
    float div = (float)clock_get_hz(clk_sys) / (float)tick_hz;
    pio_sm_set_clkdiv(g_pio.pio, g_pio.sm, div);
}

static void restart_tx_sm_at_program_start(void) {
    pio_sm_restart(g_pio.pio, g_pio.sm);
    pio_sm_exec(g_pio.pio, g_pio.sm, pio_encode_jmp(g_pio.offset));
}

static bool try_claim_pio(PIO pio) {
    /*
     * Production SWIM PIO backend needs all of this on the same PIO block:
     * - 1 SM for TX waveform
     * - 1 SM for single low-width RX
     * - 1 SM for burst/frame low-width RX
     * - instruction memory for swim_waveform_program
     * - instruction memory for swim_rx_width_program
     *
     * TX and RX must share the same PIO because TX wakes RX via PIO IRQ0.
     */
    int tx_sm = pio_claim_unused_sm(pio, false);
    if (tx_sm < 0) {
        return false;
    }

    int rx_sm = pio_claim_unused_sm(pio, false);
    if (rx_sm < 0) {
        pio_sm_unclaim(pio, (uint)tx_sm);
        return false;
    }

    int frame_sm = pio_claim_unused_sm(pio, false);
    if (frame_sm < 0) {
        pio_sm_unclaim(pio, (uint)rx_sm);
        pio_sm_unclaim(pio, (uint)tx_sm);
        return false;
    }

    /*
     * We only keep TX SM here. RX SMs will be claimed by swim_pio_rx_init().
     * This pre-claim is only a capacity test.
     */
    pio_sm_unclaim(pio, (uint)frame_sm);
    pio_sm_unclaim(pio, (uint)rx_sm);

    if (!pio_can_add_program(pio, &swim_waveform_program)) {
        pio_sm_unclaim(pio, (uint)tx_sm);
        return false;
    }

    uint tx_offset = pio_add_program(pio, &swim_waveform_program);

    if (!pio_can_add_program(pio, &swim_rx_width_program)) {
        pio_remove_program(pio, &swim_waveform_program, tx_offset);
        pio_sm_unclaim(pio, (uint)tx_sm);
        return false;
    }

    g_pio.pio = pio;
    g_pio.sm = (uint)tx_sm;
    g_pio.offset = tx_offset;
    g_pio.program_loaded = true;
    return true;
}

static void release_claims(void) {
    if (g_pio.dma_channel >= 0) {
        dma_channel_unclaim((uint)g_pio.dma_channel);
        g_pio.dma_channel = -1;
    }
    if (g_pio.program_loaded) {
        pio_remove_program(g_pio.pio, &swim_waveform_program, g_pio.offset);
        g_pio.program_loaded = false;
    }
    pio_sm_unclaim(g_pio.pio, g_pio.sm);
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
            release_claims();
            set_error("no DMA channel");
            release_pin(swim_pin, enable_internal_pullup);
            return RPSW_ERR_INTERNAL;
        }
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
    set_pio_tick_hz(SWIM_PIO_TICKS_PER_US * 1000000u);
    pio_release_swim_pin();

    /*
     * RX must be on the same PIO instance as TX.
     * TX signals "SWIM released, target may respond" with irq set 0.
     * PIO IRQ flags are per-PIO block, so RX on pio1 would not see TX irq from pio0.
     */
    rpsw_status_t rx_st = swim_pio_rx_init(g_pio.pio, swim_pin);
    if (rx_st != RPSW_OK) {
        pio_sm_set_enabled(g_pio.pio, g_pio.sm, false);
        if (g_pio.dma_channel >= 0) {
            dma_channel_unclaim(g_pio.dma_channel);
            g_pio.dma_channel = -1;
        }
        release_claims();
        release_pin(swim_pin, enable_internal_pullup);
        set_error("PIO RX init failed");
        return rx_st;
    }

    pio_sm_set_enabled(g_pio.pio, g_pio.sm, true);

    g_pio.initialized = true;
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

static rpsw_status_t encode_tick_segments(const swim_pio_tick_segment_t *segments, size_t count,
                                          uint32_t *commands, uint32_t *total_ticks) {
    if (segments == NULL || commands == NULL || total_ticks == NULL || count == 0 ||
        count > SWIM_PIO_MAX_SEGMENTS) {
        return RPSW_ERR_BAD_ARGUMENT;
    }

    uint64_t total = 0;
    for (size_t i = 0; i < count; i++) {
        if (segments[i].level != SWIM_SEG_RELEASE && segments[i].level != SWIM_SEG_LOW) {
            return RPSW_ERR_BAD_ARGUMENT;
        }
        if (segments[i].duration_ticks <= SWIM_PIO_SEGMENT_OVERHEAD_TICKS ||
            segments[i].duration_ticks > 0x7ffffffful) {
            return RPSW_ERR_BAD_ARGUMENT;
        }
        uint32_t adjusted = segments[i].duration_ticks - SWIM_PIO_SEGMENT_OVERHEAD_TICKS;
        commands[i] = (adjusted << 1) | (uint32_t)segments[i].level;
        total += segments[i].duration_ticks;
    }
    if (total > UINT32_MAX) {
        return RPSW_ERR_BAD_ARGUMENT;
    }
    *total_ticks = (uint32_t)total;
    return RPSW_OK;
}

rpsw_status_t swim_pio_emit_segments(const swim_segment_t *segments, size_t count) {
    if (!g_pio.initialized) {
        set_error("PIO waveform engine not initialized");
        return RPSW_ERR_INTERNAL;
    }

    uint32_t commands[SWIM_PIO_MAX_SEGMENTS + 1u];
    uint32_t total_us = 0;
    rpsw_status_t st = encode_segments(segments, count, commands, &total_us);
    if (st != RPSW_OK) {
        set_error("bad waveform segment list");
        return st;
    }
    commands[count] = 0u;

    pio_sm_set_enabled(g_pio.pio, g_pio.sm, false);
    pio_sm_clear_fifos(g_pio.pio, g_pio.sm);
    restart_tx_sm_at_program_start();
    pio_interrupt_clear(g_pio.pio, SWIM_PIO_COMPLETION_IRQ);
    pio_gpio_init(g_pio.pio, g_pio.swim_pin);
    set_pio_tick_hz(SWIM_PIO_TICKS_PER_US * 1000000u);
    pio_release_swim_pin();

    dma_channel_config dma_config = dma_channel_get_default_config(g_pio.dma_channel);
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_config, true);
    channel_config_set_write_increment(&dma_config, false);
    channel_config_set_dreq(&dma_config, pio_get_dreq(g_pio.pio, g_pio.sm, true));
    dma_channel_configure(g_pio.dma_channel, &dma_config,
                          &g_pio.pio->txf[g_pio.sm], commands, count + 1u, false);

    dma_channel_start(g_pio.dma_channel);
    absolute_time_t waveform_done = make_timeout_time_us(total_us);
    pio_sm_set_enabled(g_pio.pio, g_pio.sm, true);
    dma_channel_wait_for_finish_blocking(g_pio.dma_channel);

    absolute_time_t deadline = make_timeout_time_us(total_us + 1000u);
    while (!pio_interrupt_get(g_pio.pio, SWIM_PIO_COMPLETION_IRQ)) {
        if (absolute_time_diff_us(get_absolute_time(), waveform_done) <= 0 &&
            pio_sm_is_tx_fifo_empty(g_pio.pio, g_pio.sm)) {
            break;
        }
        if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
            pio_sm_set_enabled(g_pio.pio, g_pio.sm, false);
            pio_release_swim_pin();
            set_error("PIO waveform timeout");
            return RPSW_ERR_SWIM_TIMEOUT;
        }
        tight_loop_contents();
    }

    pio_sm_set_enabled(g_pio.pio, g_pio.sm, false);
    pio_interrupt_clear(g_pio.pio, SWIM_PIO_COMPLETION_IRQ);
    pio_release_swim_pin();
    /*
     * Do not call gpio_init()/pull reconfiguration here. Real STM8 targets can
     * begin the synchronization low pulse only about 30 us after the final
     * entry release; the pad is already high-Z under PIO ownership and gpio_get
     * can sample the input level directly. Later SWIM bit operations explicitly
     * switch the pin back to SIO before driving.
     */
    set_error("ok");
    return RPSW_OK;
}

rpsw_status_t swim_pio_emit_segments_capture_response(
    const swim_segment_t *segments,
    size_t count,
    uint32_t rx_max_loops,
    uint32_t timeout_us,
    swim_pio_rx_width_t *width
) {
    if (width == NULL) {
        return RPSW_ERR_BAD_ARGUMENT;
    }
    if (!g_pio.initialized) {
        set_error("PIO waveform engine not initialized");
        return RPSW_ERR_INTERNAL;
    }

    uint32_t commands[SWIM_PIO_MAX_SEGMENTS + 1u];
    uint32_t total_us = 0;
    rpsw_status_t st = encode_segments(segments, count, commands, &total_us);
    if (st != RPSW_OK) {
        set_error("bad waveform segment list");
        return st;
    }
    commands[count] = 0u;

    /*
     * Critical ordering for production RX:
     * 1. Clear TX->RX PIO IRQ.
     * 2. Arm RX before TX starts.
     * 3. Start TX waveform.
     * 4. TX releases SWIM and sets IRQ0.
     * 5. RX wakes in PIO and captures target low pulse width.
     */
    // pio_interrupt_clear(g_pio.pio, SWIM_PIO_COMPLETION_IRQ);

    st = swim_pio_rx_arm_after_tx_done(rx_max_loops);
    if (st != RPSW_OK) {
        set_error("PIO RX arm failed");
        return st;
    }

    pio_sm_set_enabled(g_pio.pio, g_pio.sm, false);
    pio_sm_clear_fifos(g_pio.pio, g_pio.sm);
    restart_tx_sm_at_program_start();
    pio_gpio_init(g_pio.pio, g_pio.swim_pin);
    set_pio_tick_hz(SWIM_PIO_TICKS_PER_US * 1000000u);
    pio_release_swim_pin();

    dma_channel_config dma_config = dma_channel_get_default_config(g_pio.dma_channel);
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_config, true);
    channel_config_set_write_increment(&dma_config, false);
    channel_config_set_dreq(&dma_config, pio_get_dreq(g_pio.pio, g_pio.sm, true));
    dma_channel_configure(g_pio.dma_channel, &dma_config,
                          &g_pio.pio->txf[g_pio.sm], commands, count + 1u, false);

    dma_channel_start(g_pio.dma_channel);
    pio_sm_set_enabled(g_pio.pio, g_pio.sm, true);
    dma_channel_wait_for_finish_blocking(g_pio.dma_channel);

    st = swim_pio_rx_get_width(width, timeout_us);

    pio_sm_set_enabled(g_pio.pio, g_pio.sm, false);
    pio_release_swim_pin();

    if (dma_channel_is_busy(g_pio.dma_channel)) {
        dma_channel_abort(g_pio.dma_channel);
    }

    if (st != RPSW_OK) {
        set_error("PIO RX response timeout");
        return st;
    }

    set_error("ok");
    return RPSW_OK;
}

rpsw_status_t swim_pio_emit_tick_segments(const swim_pio_tick_segment_t *segments, size_t count,
                                          uint32_t tick_hz) {
    if (!g_pio.initialized || tick_hz == 0) {
        set_error("PIO waveform engine not initialized");
        return RPSW_ERR_INTERNAL;
    }

    uint32_t commands[SWIM_PIO_MAX_SEGMENTS + 1u];
    uint32_t total_ticks = 0;
    rpsw_status_t st = encode_tick_segments(segments, count, commands, &total_ticks);
    if (st != RPSW_OK) {
        set_error("bad waveform tick segment list");
        return st;
    }
    commands[count] = 0u;

    pio_sm_set_enabled(g_pio.pio, g_pio.sm, false);
    pio_sm_clear_fifos(g_pio.pio, g_pio.sm);
    restart_tx_sm_at_program_start();
    pio_interrupt_clear(g_pio.pio, SWIM_PIO_COMPLETION_IRQ);
    pio_gpio_init(g_pio.pio, g_pio.swim_pin);
    set_pio_tick_hz(tick_hz);
    pio_release_swim_pin();

    dma_channel_config dma_config = dma_channel_get_default_config(g_pio.dma_channel);
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_config, true);
    channel_config_set_write_increment(&dma_config, false);
    channel_config_set_dreq(&dma_config, pio_get_dreq(g_pio.pio, g_pio.sm, true));
    dma_channel_configure(g_pio.dma_channel, &dma_config,
                          &g_pio.pio->txf[g_pio.sm], commands, count + 1u, false);

    uint64_t timeout_us64 = ((uint64_t)total_ticks * 1000000ull + tick_hz - 1u) / tick_hz + 1000u;
    dma_channel_start(g_pio.dma_channel);
    pio_sm_set_enabled(g_pio.pio, g_pio.sm, true);
    dma_channel_wait_for_finish_blocking(g_pio.dma_channel);

    absolute_time_t deadline = make_timeout_time_us((uint32_t)timeout_us64);
    while (!pio_interrupt_get(g_pio.pio, SWIM_PIO_COMPLETION_IRQ)) {
        if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
            pio_sm_set_enabled(g_pio.pio, g_pio.sm, false);
            pio_release_swim_pin();
            set_error("PIO tick waveform timeout");
            return RPSW_ERR_SWIM_TIMEOUT;
        }
        tight_loop_contents();
    }

    pio_sm_set_enabled(g_pio.pio, g_pio.sm, false);
    pio_interrupt_clear(g_pio.pio, SWIM_PIO_COMPLETION_IRQ);
    pio_release_swim_pin();
    set_error("ok");
    return RPSW_OK;
}

rpsw_status_t swim_pio_emit_tick_segments_wait_response(const swim_pio_tick_segment_t *segments,
                                                        size_t count, uint32_t tick_hz,
                                                        uint32_t timeout_us,
                                                        swim_pio_sync_measurement_t *measurement) {
    if (!g_pio.initialized || tick_hz == 0 || measurement == NULL) {
        set_error("PIO waveform engine not initialized");
        return RPSW_ERR_INTERNAL;
    }

    uint32_t commands[SWIM_PIO_MAX_SEGMENTS + 1u];
    uint32_t total_ticks = 0;
    rpsw_status_t st = encode_tick_segments(segments, count, commands, &total_ticks);
    if (st != RPSW_OK) {
        set_error("bad waveform tick segment list");
        return st;
    }
    commands[count] = 0u;
    measurement->low_us = 0;
    measurement->low_ns = 0;
    measurement->low_loop_count = 0;

    pio_sm_set_enabled(g_pio.pio, g_pio.sm, false);
    pio_sm_clear_fifos(g_pio.pio, g_pio.sm);
    restart_tx_sm_at_program_start();
    pio_interrupt_clear(g_pio.pio, SWIM_PIO_COMPLETION_IRQ);
    pio_gpio_init(g_pio.pio, g_pio.swim_pin);
    set_pio_tick_hz(tick_hz);
    pio_release_swim_pin();

    dma_channel_config dma_config = dma_channel_get_default_config(g_pio.dma_channel);
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_config, true);
    channel_config_set_write_increment(&dma_config, false);
    channel_config_set_dreq(&dma_config, pio_get_dreq(g_pio.pio, g_pio.sm, true));
    dma_channel_configure(g_pio.dma_channel, &dma_config,
                          &g_pio.pio->txf[g_pio.sm], commands, count + 1u, false);

    uint64_t frame_timeout_us = ((uint64_t)total_ticks * 1000000ull + tick_hz - 1u) / tick_hz + 1000u;
    dma_channel_start(g_pio.dma_channel);
    pio_sm_set_enabled(g_pio.pio, g_pio.sm, true);
    dma_channel_wait_for_finish_blocking(g_pio.dma_channel);

    absolute_time_t frame_deadline = make_timeout_time_us((uint32_t)frame_timeout_us);
    while (!pio_interrupt_get(g_pio.pio, SWIM_PIO_COMPLETION_IRQ)) {
        if (absolute_time_diff_us(get_absolute_time(), frame_deadline) <= 0) {
            pio_sm_set_enabled(g_pio.pio, g_pio.sm, false);
            pio_release_swim_pin();
            set_error("PIO tick waveform timeout");
            return RPSW_ERR_SWIM_TIMEOUT;
        }
        tight_loop_contents();
    }

    uint32_t irq_state = save_and_disable_interrupts();
    absolute_time_t response_deadline = make_timeout_time_us(timeout_us);
    while (sample_swim_pin_fast()) {
        if (absolute_time_diff_us(get_absolute_time(), response_deadline) <= 0) {
            restore_interrupts(irq_state);
            pio_sm_set_enabled(g_pio.pio, g_pio.sm, false);
            pio_interrupt_clear(g_pio.pio, SWIM_PIO_COMPLETION_IRQ);
            pio_release_swim_pin();
            set_error("response timeout high");
            return RPSW_ERR_SWIM_TIMEOUT;
        }
        tight_loop_contents();
    }

    uint64_t start_us = time_us_64();
    uint32_t low_count = 0;
    while (!sample_swim_pin_fast()) {
        low_count++;
        tight_loop_contents();
        if (absolute_time_diff_us(get_absolute_time(), response_deadline) <= 0) {
            restore_interrupts(irq_state);
            pio_sm_set_enabled(g_pio.pio, g_pio.sm, false);
            pio_interrupt_clear(g_pio.pio, SWIM_PIO_COMPLETION_IRQ);
            pio_release_swim_pin();
            set_error("response timeout low");
            return RPSW_ERR_SWIM_TIMEOUT;
        }
    }

    uint64_t elapsed_us64 = time_us_64() - start_us;
    pio_sm_set_enabled(g_pio.pio, g_pio.sm, false);
    pio_interrupt_clear(g_pio.pio, SWIM_PIO_COMPLETION_IRQ);
    pio_release_swim_pin();
    restore_interrupts(irq_state);

    measurement->low_us = elapsed_us64 > UINT32_MAX ? UINT32_MAX : (uint32_t)elapsed_us64;
    measurement->low_ns = measurement->low_us * 1000u;
    measurement->low_loop_count = low_count;
    set_error("ok");
    return low_count == 0 ? RPSW_ERR_SWIM_TIMEOUT : RPSW_OK;
}

rpsw_status_t swim_pio_emit_tick_segments_capture_response(
    const swim_pio_tick_segment_t *segments,
    size_t count,
    uint32_t tick_hz,
    uint32_t rx_max_loops,
    uint32_t timeout_us,
    swim_pio_rx_width_t *width
) {
    if (width == NULL) {
        return RPSW_ERR_BAD_ARGUMENT;
    }
    if (!g_pio.initialized) {
        set_error("PIO waveform engine not initialized");
        return RPSW_ERR_INTERNAL;
    }

    uint32_t commands[SWIM_PIO_MAX_SEGMENTS + 1u];
    uint32_t total_ticks = 0;
    rpsw_status_t st = encode_tick_segments(segments, count, commands, &total_ticks);
    if (st != RPSW_OK) {
        set_error("bad tick segment list");
        return st;
    }
    commands[count] = 0u;

    st = swim_pio_rx_arm_after_tx_done(rx_max_loops);
    if (st != RPSW_OK) {
        set_error("PIO RX arm failed");
        return st;
    }

    pio_sm_set_enabled(g_pio.pio, g_pio.sm, false);
    pio_sm_clear_fifos(g_pio.pio, g_pio.sm);
    restart_tx_sm_at_program_start();
    pio_gpio_init(g_pio.pio, g_pio.swim_pin);

    set_pio_tick_hz(tick_hz);
    pio_release_swim_pin();

    dma_channel_config dma_config = dma_channel_get_default_config(g_pio.dma_channel);
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_config, true);
    channel_config_set_write_increment(&dma_config, false);
    channel_config_set_dreq(&dma_config, pio_get_dreq(g_pio.pio, g_pio.sm, true));

    dma_channel_configure(
        g_pio.dma_channel,
        &dma_config,
        &g_pio.pio->txf[g_pio.sm],
        commands,
        count + 1u,
        false
    );

    dma_channel_start(g_pio.dma_channel);
    pio_sm_set_enabled(g_pio.pio, g_pio.sm, true);
    dma_channel_wait_for_finish_blocking(g_pio.dma_channel);

    st = swim_pio_rx_get_width(width, timeout_us);

    pio_sm_set_enabled(g_pio.pio, g_pio.sm, false);
    pio_release_swim_pin();

    if (dma_channel_is_busy(g_pio.dma_channel)) {
        dma_channel_abort(g_pio.dma_channel);
    }

    if (st != RPSW_OK) {
        set_error("PIO RX response timeout");
        return st;
    }

    set_error("ok");
    return RPSW_OK;
}

static rpsw_status_t emit_tick_segments_capture_width_burst_common(
    const swim_pio_tick_segment_t *segments,
    size_t count,
    uint32_t tick_hz,
    uint32_t rx_max_loops,
    uint32_t timeout_us,
    swim_pio_rx_width_t *widths,
    uint32_t width_count,
    uint32_t *captured_count,
    bool partial
) {
    if (widths == NULL || width_count == 0u) {
        return RPSW_ERR_BAD_ARGUMENT;
    }
    if (partial && captured_count == NULL) {
        return RPSW_ERR_BAD_ARGUMENT;
    }
    if (!g_pio.initialized || tick_hz == 0u) {
        set_error("PIO waveform engine not initialized");
        return RPSW_ERR_INTERNAL;
    }

    if (captured_count != NULL) {
        *captured_count = 0u;
    }

    uint32_t commands[SWIM_PIO_MAX_SEGMENTS + 1u];
    uint32_t total_ticks = 0;
    rpsw_status_t st = encode_tick_segments(segments, count, commands, &total_ticks);
    if (st != RPSW_OK) {
        set_error("bad tick segment list");
        return st;
    }
    commands[count] = 0u;

    st = swim_pio_rx_arm_width_burst_after_tx_done(width_count, rx_max_loops);
    if (st != RPSW_OK) {
        set_error("PIO RX width-burst arm failed");
        return st;
    }

    pio_sm_set_enabled(g_pio.pio, g_pio.sm, false);
    pio_sm_clear_fifos(g_pio.pio, g_pio.sm);
    restart_tx_sm_at_program_start();
    pio_gpio_init(g_pio.pio, g_pio.swim_pin);
    set_pio_tick_hz(tick_hz);
    pio_release_swim_pin();

    dma_channel_config dma_config = dma_channel_get_default_config(g_pio.dma_channel);
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_config, true);
    channel_config_set_write_increment(&dma_config, false);
    channel_config_set_dreq(&dma_config, pio_get_dreq(g_pio.pio, g_pio.sm, true));
    dma_channel_configure(g_pio.dma_channel, &dma_config,
                          &g_pio.pio->txf[g_pio.sm], commands, count + 1u, false);

    dma_channel_start(g_pio.dma_channel);
    pio_sm_set_enabled(g_pio.pio, g_pio.sm, true);

    /*
     * Critical for UM0470 target response:
     * RX FIFO must be drained while the response is happening. If we wait for TX
     * DMA completion first, the RX FIFO can fill and the RX SM blocks on push.
     */
    if (partial) {
        st = swim_pio_rx_get_width_burst_partial(widths, width_count,
                                                 timeout_us, captured_count);
    } else {
        st = swim_pio_rx_get_width_burst(widths, width_count, timeout_us);
        if (st == RPSW_OK && captured_count != NULL) {
            *captured_count = width_count;
        }
    }

    dma_channel_wait_for_finish_blocking(g_pio.dma_channel);

    pio_sm_set_enabled(g_pio.pio, g_pio.sm, false);
    pio_release_swim_pin();

    if (dma_channel_is_busy(g_pio.dma_channel)) {
        dma_channel_abort(g_pio.dma_channel);
    }

    if (st != RPSW_OK) {
        set_error(partial ? "PIO RX width-burst partial timeout"
        : "PIO RX width-burst timeout");
        return st;
    }

    set_error("ok");
    return RPSW_OK;
}

rpsw_status_t swim_pio_emit_tick_segments_capture_width_burst(
    const swim_pio_tick_segment_t *segments,
    size_t count,
    uint32_t tick_hz,
    uint32_t rx_max_loops,
    uint32_t timeout_us,
    swim_pio_rx_width_t *widths,
    uint32_t width_count
) {
    return emit_tick_segments_capture_width_burst_common(
        segments,
        count,
        tick_hz,
        rx_max_loops,
        timeout_us,
        widths,
        width_count,
        NULL,
        false
    );
}

rpsw_status_t swim_pio_emit_tick_segments_capture_width_burst_partial(
    const swim_pio_tick_segment_t *segments,
    size_t count,
    uint32_t tick_hz,
    uint32_t rx_max_loops,
    uint32_t timeout_us,
    swim_pio_rx_width_t *widths,
    uint32_t max_width_count,
    uint32_t *captured_count
) {
    return emit_tick_segments_capture_width_burst_common(
        segments,
        count,
        tick_hz,
        rx_max_loops,
        timeout_us,
        widths,
        max_width_count,
        captured_count,
        true
    );
}

rpsw_status_t swim_pio_emit_tick_segments_decode_bits(
    const swim_pio_tick_segment_t *segments,
    size_t count,
    uint32_t tick_hz,
    uint32_t threshold_loops,
    uint32_t timeout_us,
    uint32_t bit_count,
    uint32_t *bits
) {
    if (bits == NULL || bit_count == 0u || bit_count > 31u) {
        return RPSW_ERR_BAD_ARGUMENT;
    }
    if (!g_pio.initialized || tick_hz == 0u) {
        set_error("PIO waveform engine not initialized");
        return RPSW_ERR_INTERNAL;
    }

    uint32_t commands[SWIM_PIO_MAX_SEGMENTS + 1u];
    uint32_t total_ticks = 0;
    rpsw_status_t st = encode_tick_segments(segments, count, commands, &total_ticks);
    if (st != RPSW_OK) {
        set_error("bad tick segment list");
        return st;
    }
    commands[count] = 0u;

    st = swim_pio_rx_arm_decode_bits_after_tx_done(bit_count, threshold_loops);
    if (st != RPSW_OK) {
        set_error("PIO RX decode arm failed");
        return st;
    }

    pio_sm_set_enabled(g_pio.pio, g_pio.sm, false);
    pio_sm_clear_fifos(g_pio.pio, g_pio.sm);
    restart_tx_sm_at_program_start();
    pio_gpio_init(g_pio.pio, g_pio.swim_pin);
    set_pio_tick_hz(tick_hz);
    pio_release_swim_pin();

    dma_channel_config dma_config = dma_channel_get_default_config(g_pio.dma_channel);
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_config, true);
    channel_config_set_write_increment(&dma_config, false);
    channel_config_set_dreq(&dma_config, pio_get_dreq(g_pio.pio, g_pio.sm, true));
    dma_channel_configure(g_pio.dma_channel, &dma_config,
                          &g_pio.pio->txf[g_pio.sm], commands, count + 1u, false);

    dma_channel_start(g_pio.dma_channel);
    pio_sm_set_enabled(g_pio.pio, g_pio.sm, true);

    /*
     * RX decode is armed before TX starts. It wakes on TX sentinel IRQ0 after
     * SWIM is released and then decodes the target response without per-bit
     * FIFO traffic.
     */
    st = swim_pio_rx_get_decoded_bits(timeout_us, bit_count, bits);

    dma_channel_wait_for_finish_blocking(g_pio.dma_channel);

    pio_sm_set_enabled(g_pio.pio, g_pio.sm, false);
    pio_release_swim_pin();

    if (dma_channel_is_busy(g_pio.dma_channel)) {
        dma_channel_abort(g_pio.dma_channel);
    }

    if (st != RPSW_OK) {
        set_error("PIO RX decode timeout");
        return st;
    }

    set_error("ok");
    return RPSW_OK;
}

rpsw_status_t swim_pio_emit_segments_wait_sync(const swim_segment_t *segments, size_t count,
                                               uint32_t timeout_us,
                                               swim_pio_sync_measurement_t *measurement) {
    if (!g_pio.initialized || measurement == NULL) {
        set_error("PIO waveform engine not initialized");
        return RPSW_ERR_INTERNAL;
    }

    uint32_t commands[SWIM_PIO_MAX_SEGMENTS + 1u];
    uint32_t total_us = 0;
    rpsw_status_t st = encode_segments(segments, count, commands, &total_us);
    if (st != RPSW_OK) {
        set_error("bad waveform segment list");
        return st;
    }
    commands[count] = 0u;
    measurement->low_us = 0;
    measurement->low_ns = 0;
    measurement->low_loop_count = 0;

    pio_sm_set_enabled(g_pio.pio, g_pio.sm, false);
    pio_sm_clear_fifos(g_pio.pio, g_pio.sm);
    restart_tx_sm_at_program_start();
    pio_interrupt_clear(g_pio.pio, SWIM_PIO_COMPLETION_IRQ);
    pio_gpio_init(g_pio.pio, g_pio.swim_pin);
    pio_release_swim_pin();

    dma_channel_config dma_config = dma_channel_get_default_config(g_pio.dma_channel);
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_config, true);
    channel_config_set_write_increment(&dma_config, false);
    channel_config_set_dreq(&dma_config, pio_get_dreq(g_pio.pio, g_pio.sm, true));
    dma_channel_configure(g_pio.dma_channel, &dma_config,
                          &g_pio.pio->txf[g_pio.sm], commands, count + 1u, false);

    dma_channel_start(g_pio.dma_channel);
    pio_sm_set_enabled(g_pio.pio, g_pio.sm, true);

    absolute_time_t waveform_deadline = make_timeout_time_us(total_us + 1000u);
    while (!pio_interrupt_get(g_pio.pio, SWIM_PIO_COMPLETION_IRQ)) {
        if (absolute_time_diff_us(get_absolute_time(), waveform_deadline) <= 0) {
            pio_sm_set_enabled(g_pio.pio, g_pio.sm, false);
            pio_release_swim_pin();
            set_error("PIO waveform timeout");
            return RPSW_ERR_SWIM_TIMEOUT;
        }
        tight_loop_contents();
    }

    absolute_time_t sync_deadline = make_timeout_time_us(timeout_us);
    uint32_t irq_state = save_and_disable_interrupts();
    while (absolute_time_diff_us(get_absolute_time(), sync_deadline) > 0) {
        while (sample_swim_pin_fast()) {
            if (absolute_time_diff_us(get_absolute_time(), sync_deadline) <= 0) {
                restore_interrupts(irq_state);
                pio_sm_set_enabled(g_pio.pio, g_pio.sm, false);
                pio_interrupt_clear(g_pio.pio, SWIM_PIO_COMPLETION_IRQ);
                pio_release_swim_pin();
                set_error("sync timeout high");
                return RPSW_ERR_SWIM_TIMEOUT;
            }
            tight_loop_contents();
        }

        uint64_t start_us = time_us_64();
        uint32_t low_count = 0;
        while (!sample_swim_pin_fast()) {
            low_count++;
            tight_loop_contents();
            if (absolute_time_diff_us(get_absolute_time(), sync_deadline) <= 0) {
                restore_interrupts(irq_state);
                pio_sm_set_enabled(g_pio.pio, g_pio.sm, false);
                pio_interrupt_clear(g_pio.pio, SWIM_PIO_COMPLETION_IRQ);
                pio_release_swim_pin();
                set_error("sync timeout low");
                return RPSW_ERR_SWIM_TIMEOUT;
            }
        }

        uint64_t elapsed_us64 = time_us_64() - start_us;
        if (elapsed_us64 == 0 || elapsed_us64 > UINT32_MAX || low_count == 0) {
            continue;
        }

        uint32_t elapsed_us = (uint32_t)elapsed_us64;
        if (!swim_sync_low_width_is_plausible_us(elapsed_us)) {
            continue;
        }

        pio_sm_set_enabled(g_pio.pio, g_pio.sm, false);
        pio_interrupt_clear(g_pio.pio, SWIM_PIO_COMPLETION_IRQ);
        pio_release_swim_pin();
        if (dma_channel_is_busy(g_pio.dma_channel)) {
            dma_channel_abort(g_pio.dma_channel);
        }

        measurement->low_us = elapsed_us;
        measurement->low_ns = measurement->low_us * 1000u;
        measurement->low_loop_count = low_count;
        restore_interrupts(irq_state);
        set_error("ok");
        return RPSW_OK;
    }

    restore_interrupts(irq_state);
    pio_sm_set_enabled(g_pio.pio, g_pio.sm, false);
    pio_interrupt_clear(g_pio.pio, SWIM_PIO_COMPLETION_IRQ);
    pio_release_swim_pin();
    if (dma_channel_is_busy(g_pio.dma_channel)) {
        dma_channel_abort(g_pio.dma_channel);
    }
    set_error("sync timeout");
    return RPSW_ERR_SWIM_TIMEOUT;
}
