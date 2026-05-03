#include "swim_phy.h"

#include "board_rp2040_zero.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "hardware/structs/sio.h"
#include "pico/platform.h"
#include "pico/stdlib.h"
#include "swim_pio_waveform.h"
#include "swim_sync_limits.h"
#include "swim_bits.h"
#include <stdio.h>
#include <string.h>

#define SWIM_LOW_SPEED_ONE_LOW_CLOCKS 2u
#define SWIM_LOW_SPEED_ONE_HIGH_CLOCKS 20u
#define SWIM_LOW_SPEED_ZERO_LOW_CLOCKS 20u
#define SWIM_LOW_SPEED_ZERO_HIGH_CLOCKS 2u
#define SWIM_LOW_SPEED_ONE_MAX_LOW_CLOCKS 8u
#define SWIM_LOW_SPEED_ZERO_MIN_LOW_CLOCKS 9u
#define SWIM_LOW_SPEED_INTERFRAME_GAP_CLOCKS 22u
#define SWIM_PHY_BACKEND_PIO 0u
#define SWIM_PHY_BACKEND_BITBANG_FALLBACK 1u
#define SWIM_ENTRY_PROTOCOL_US 6016u
#define SWIM_ENTRY_SLOW_PULSES 4u
#define SWIM_ENTRY_FAST_PULSES 4u
#define SWIM_PIO_FRAME_TICKS_PER_SWIM_CLOCK 8u


static swim_phy_config_t g_phy = {
    .swim_pin = RP2040_SWIM_DEFAULT_SWIM_PIN,
    .nrst_pin = RP2040_SWIM_DEFAULT_NRST_PIN,
    .internal_pullup = false,
    .active_drive_high = false,
    .speed = SWIM_SPEED_LOW,
};

static swim_phy_debug_t g_debug = {
    .synced = false,
    .speed = SWIM_SPEED_LOW,
    .phy_backend = SWIM_PHY_BACKEND_BITBANG_FALLBACK,
    .entry_protocol_us = SWIM_ENTRY_PROTOCOL_US,
    .entry_slow_pulses = SWIM_ENTRY_SLOW_PULSES,
    .entry_fast_pulses = SWIM_ENTRY_FAST_PULSES,
    .pio_init_ok = false,
    .pio_error = "not initialized",
};

static uint32_t g_low_speed_one_low_cycles = 1u;
static uint32_t g_low_speed_one_high_cycles = 1u;
static uint32_t g_low_speed_zero_low_cycles = 1u;
static uint32_t g_low_speed_zero_high_cycles = 1u;
static char g_tx_context[24] = "";

static const swim_segment_t g_entry_segments[] = {
    /*
     * UM0470 Figure 5:
     * - sequence starts with SWIM released/high,
     * - force SWIM low for 16 us,
     * - four pulses at 1 kHz,
     * - four pulses at 2 kHz,
     * - sequence ends with SWIM released/high.
     */
    {SWIM_SEG_RELEASE, 10},   // pre-settle before entry, not protocol timing
    {SWIM_SEG_LOW, 16},

    {SWIM_SEG_RELEASE, 500}, {SWIM_SEG_LOW, 500},
    {SWIM_SEG_RELEASE, 500}, {SWIM_SEG_LOW, 500},
    {SWIM_SEG_RELEASE, 500}, {SWIM_SEG_LOW, 500},
    {SWIM_SEG_RELEASE, 500}, {SWIM_SEG_LOW, 500},

    {SWIM_SEG_RELEASE, 250}, {SWIM_SEG_LOW, 250},
    {SWIM_SEG_RELEASE, 250}, {SWIM_SEG_LOW, 250},
    {SWIM_SEG_RELEASE, 250}, {SWIM_SEG_LOW, 250},
    {SWIM_SEG_RELEASE, 250}, {SWIM_SEG_LOW, 250},

    {SWIM_SEG_RELEASE, 10},  // end released/high
};

static size_t entry_segment_count(void) {
    return sizeof(g_entry_segments) / sizeof(g_entry_segments[0]);
}

void swim_phy_set_enter_stage(swim_enter_stage_t stage) {
    g_debug.enter_stage = stage;
}

void swim_phy_mark_enter_fail(void) {
    g_debug.enter_stage = SWIM_ENTER_STAGE_FAIL;
}

void swim_phy_mark_second_sync_seen(void) {
    g_debug.second_sync_seen = true;
}

void swim_phy_set_tx_context(const char *context) {
    snprintf(g_tx_context, sizeof(g_tx_context), "%s", context != NULL ? context : "");
}

static void set_pio_debug(bool ok, const char *message) {
    g_debug.pio_init_ok = ok;
    snprintf(g_debug.pio_error, sizeof(g_debug.pio_error), "%s",
             message != NULL ? message : "");
}

static void set_pio_fallback_warning(const char *message) {
    g_debug.pio_init_ok = false;
    snprintf(g_debug.pio_error, sizeof(g_debug.pio_error), "warning: bitbang fallback; %s",
             message != NULL && message[0] != '\0' ? message : "PIO unavailable");
}

static void swim_release_pin(uint pin, bool pullup) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    if (pullup) {
        gpio_pull_up(pin);
    } else {
        gpio_disable_pulls(pin);
    }
}

void swim_phy_init(const swim_phy_config_t *config) {
    if (config != NULL) {
        g_phy = *config;
    }
    if (g_phy.speed == SWIM_SPEED_HIGH) {
        g_phy.speed = SWIM_SPEED_LOW;
    }
    g_debug.speed = g_phy.speed;
    swim_release_pin(g_phy.swim_pin, g_phy.internal_pullup);
    swim_release_pin(g_phy.nrst_pin, true);
    rpsw_status_t st = swim_pio_waveform_init(g_phy.swim_pin, g_phy.internal_pullup);
    if (st == RPSW_OK) {
        g_debug.phy_backend = SWIM_PHY_BACKEND_PIO;
        set_pio_debug(true, "ok");
    } else {
        g_debug.phy_backend = SWIM_PHY_BACKEND_BITBANG_FALLBACK;
        set_pio_fallback_warning(swim_pio_waveform_error());
    }
}

void swim_phy_set_pins(uint swim_pin, uint nrst_pin, bool internal_pullup) {
    g_phy.swim_pin = swim_pin;
    g_phy.nrst_pin = nrst_pin;
    g_phy.internal_pullup = internal_pullup;
    swim_phy_init(&g_phy);
}

bool swim_phy_set_speed(swim_speed_t speed) {
    /*
     * High-speed SWIM requires HSIT/HS handling and tighter sampling than this
     * first-pass PHY provides. Keep the link explicitly low-speed only.
     */
    if (speed != SWIM_SPEED_LOW) {
        return false;
    }
    g_phy.speed = speed;
    g_debug.speed = speed;
    return true;
}

void swim_phy_release(void) {
    swim_release_pin(g_phy.swim_pin, g_phy.internal_pullup);
}

void swim_phy_drive_low(void) {
    gpio_init(g_phy.swim_pin);
    gpio_put(g_phy.swim_pin, 0);
    gpio_set_dir(g_phy.swim_pin, GPIO_OUT);
}

static void swim_phy_sio_input(void) {
    gpio_set_function(g_phy.swim_pin, GPIO_FUNC_SIO);
    gpio_put(g_phy.swim_pin, 0);
    gpio_set_dir(g_phy.swim_pin, GPIO_IN);
    if (g_phy.internal_pullup) {
        gpio_pull_up(g_phy.swim_pin);
    } else {
        gpio_disable_pulls(g_phy.swim_pin);
    }
}

static void swim_phy_sio_drive_low_fast(void) {
    uint32_t mask = 1u << g_phy.swim_pin;
    sio_hw->gpio_clr = mask;
    sio_hw->gpio_oe_set = mask;
}

static void swim_phy_sio_release_fast(void) {
    sio_hw->gpio_oe_clr = 1u << g_phy.swim_pin;
}

static inline bool swim_phy_sample_fast(void) {
    return (sio_hw->gpio_in & (1u << g_phy.swim_pin)) != 0u;
}

bool swim_phy_sample(void) {
    return gpio_get(g_phy.swim_pin);
}

void swim_phy_nrst_assert(void) {
    gpio_init(g_phy.nrst_pin);
    gpio_put(g_phy.nrst_pin, 0);
    gpio_set_dir(g_phy.nrst_pin, GPIO_OUT);
}

void swim_phy_nrst_release(void) {
    swim_release_pin(g_phy.nrst_pin, true);
}

void swim_phy_delay_us(uint32_t us) {
    sleep_us(us);
}

static void pulse_low(uint32_t low_us, uint32_t release_us) {
    swim_phy_drive_low();
    sleep_us(low_us);
    swim_phy_release();
    sleep_us(release_us);
}

static void bitbang_emit_segments(const swim_segment_t *segments, size_t count) {
    for (size_t i = 0; i < count; i++) {
        if (segments[i].level == SWIM_SEG_LOW) {
            swim_phy_drive_low();
        } else {
            swim_phy_release();
        }
        sleep_us(segments[i].duration_us);
    }
    swim_phy_release();
}

static rpsw_status_t emit_entry_segments(size_t count) {
    if (swim_pio_waveform_available()) {
        rpsw_status_t st = swim_pio_emit_segments(g_entry_segments, count);
        if (st == RPSW_OK) {
            g_debug.phy_backend = SWIM_PHY_BACKEND_PIO;
            set_pio_debug(true, "ok");
        } else {
            set_pio_debug(true, swim_pio_waveform_error());
        }
        return st;
    }
    swim_phy_release();
    g_debug.phy_backend = SWIM_PHY_BACKEND_BITBANG_FALLBACK;
    set_pio_fallback_warning(swim_pio_waveform_error());
    bitbang_emit_segments(g_entry_segments, count);
    swim_phy_release();
    return RPSW_OK;
}

static void busy_wait_ns(uint32_t ns) {
    if (ns == 0) {
        return;
    }
    uint64_t cycles = ((uint64_t)clock_get_hz(clk_sys) * (uint64_t)ns + 999999999ull) / 1000000000ull;
    if (cycles == 0) {
        cycles = 1;
    }
    busy_wait_at_least_cycles((uint32_t)cycles);
}

static uint32_t clocks_to_ns(uint32_t clocks) {
    if (!g_debug.synced || g_debug.derived_tswim_ns == 0) {
        return 0;
    }
    return g_debug.derived_tswim_ns * clocks;
}

static uint32_t swim_clocks_to_cycles_from_tswim(uint32_t tswim_ns, uint32_t clocks) {
    uint64_t ns = (uint64_t)tswim_ns * (uint64_t)clocks;
    uint64_t cycles = ((uint64_t)clock_get_hz(clk_sys) * ns + 999999999ull) / 1000000000ull;
    return cycles > 0 ? (uint32_t)cycles : 1u;
}

static void update_low_speed_cycle_cache(void) {
    if (g_debug.derived_tswim_ns == 0) {
        return;
    }
    g_low_speed_one_low_cycles =
        swim_clocks_to_cycles_from_tswim(g_debug.derived_tswim_ns, SWIM_LOW_SPEED_ONE_LOW_CLOCKS);
    g_low_speed_one_high_cycles =
        swim_clocks_to_cycles_from_tswim(g_debug.derived_tswim_ns, SWIM_LOW_SPEED_ONE_HIGH_CLOCKS);
    g_low_speed_zero_low_cycles =
        swim_clocks_to_cycles_from_tswim(g_debug.derived_tswim_ns, SWIM_LOW_SPEED_ZERO_LOW_CLOCKS);
    g_low_speed_zero_high_cycles =
        swim_clocks_to_cycles_from_tswim(g_debug.derived_tswim_ns, SWIM_LOW_SPEED_ZERO_HIGH_CLOCKS);
}

static void record_sync_measurement_ns(uint32_t elapsed_ns, uint32_t low_count) {
    uint32_t elapsed_us = (elapsed_ns + 999u) / 1000u;
    uint32_t tswim_ns = (elapsed_ns + (SWIM_SYNC_CLOCKS / 2u)) / SWIM_SYNC_CLOCKS;

    g_debug.synced = true;
    g_debug.speed = SWIM_SPEED_LOW;
    g_debug.last_sync_low_us = elapsed_us;
    g_debug.last_sync_low_ns = elapsed_ns;
    g_debug.derived_tswim_ns = tswim_ns;
    g_debug.sync_low_loop_count = low_count;
    g_phy.speed = SWIM_SPEED_LOW;
    update_low_speed_cycle_cache();
}

rpsw_status_t swim_phy_entry_sequence_um0470(void) {
    /*
     * UM0470 Rev 4 section 3.2:
     * - force SWIM low for 16 us,
     * - four pulses at 1 kHz,
     * - four pulses at 2 kHz,
     * sequence starts and ends released/high.
     */
    /*
     * The last table entry is a capture/debug settle segment. For real entry,
     * stop immediately after the final fast low and let the PIO sentinel release
     * SWIM high-Z. Real STM8S003 hardware can start the sync frame about 1 us
     * after that release, so spending an extra 10 us in PIO can miss the sync.
     */
    return emit_entry_segments(entry_segment_count() - 1u);
}

bool swim_phy_entry_sequence_um0470_wait_sync(uint32_t timeout_us) {
    swim_pio_rx_width_t width = {0};

    /*
     * Real STM8S003 currently answers with a short sync-like low around
     * 16..17 us. Give the RX state machine enough budget for up to 300 us.
     */
    uint32_t max_loops = swim_pio_rx_ns_to_max_loops(300000u);

    rpsw_status_t st = swim_pio_emit_segments_capture_response(
        g_entry_segments,
        entry_segment_count() - 1u,
        max_loops,
        timeout_us,
        &width
    );

    if (st != RPSW_OK) {
        set_pio_debug(true, swim_pio_waveform_error());
        return false;
    }

    if (width.timeout || width.low_us < 8u || width.low_us > 300u) {
        set_pio_debug(true, "PIO RX sync width out of range");
        return false;
    }

    record_sync_measurement_ns(width.low_ns, width.loops_used);
    set_pio_debug(true, "ok");
    return true;
}

void swim_phy_entry_waveform(void) {
    (void)emit_entry_segments(entry_segment_count());
}

void swim_phy_comm_reset(void) {
    /*
     * UM0470 section 3.6: reset SWIM communication by holding SWIM low for
     * 128 SWIM clock periods. After the first synchronization frame this uses
     * the measured SWIM clock; before calibration it falls back to the nominal
     * low-speed 16 us reset frame only for debug waveform generation.
     */
    if (swim_phy_timing_ready()) {
        uint32_t low_ns = clocks_to_ns(SWIM_SYNC_CLOCKS);
        uint32_t low_us = (low_ns + 999u) / 1000u;

        g_debug.comm_reset_sent = true;
        g_debug.comm_reset_low_ns = low_ns;
        g_debug.comm_reset_low_us = low_us;

        swim_phy_sio_input();
        swim_phy_sio_drive_low_fast();
        busy_wait_ns(low_ns);
        swim_phy_sio_release_fast();
    } else {
        g_debug.comm_reset_sent = true;
        g_debug.comm_reset_low_ns = 16000u;
        g_debug.comm_reset_low_us = 16u;

        pulse_low(16, 16);
    }
}

static bool wait_sync_fast(uint32_t timeout_us) {
    absolute_time_t deadline = make_timeout_time_us(timeout_us);

    while (absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
        while (swim_phy_sample_fast()) {
            if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
                return false;
            }
            tight_loop_contents();
        }

        uint64_t start_us = time_us_64();
        uint32_t low_count = 0;
        while (!swim_phy_sample_fast()) {
            low_count++;
            tight_loop_contents();
            if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
                return false;
            }
        }

        uint64_t elapsed_us64 = time_us_64() - start_us;
        if (elapsed_us64 < 4u || elapsed_us64 > UINT32_MAX || low_count == 0) {
            continue;
        }

        uint32_t elapsed_us = (uint32_t)elapsed_us64;
        if (!swim_sync_low_width_is_plausible_us(elapsed_us)) {
            continue;
        }

        record_sync_measurement_ns((uint32_t)(elapsed_us * 1000u), low_count);
        return true;
    }
    return false;
}

bool swim_phy_comm_reset_wait_sync(uint32_t timeout_us) {
    if (!swim_phy_timing_ready()) {
        swim_phy_comm_reset();
        return wait_sync_fast(timeout_us);
    }

    uint32_t low_ns = clocks_to_ns(SWIM_SYNC_CLOCKS);
    uint32_t low_us = (low_ns + 999u) / 1000u;

    g_debug.comm_reset_sent = true;
    g_debug.comm_reset_low_ns = low_ns;
    g_debug.comm_reset_low_us = low_us;

    if (g_debug.derived_tswim_ns == 0u) {
        set_pio_debug(true, "missing Tswim for comm reset");
        return false;
    }

    uint32_t tick_hz = 1000000000u / g_debug.derived_tswim_ns;
    if (tick_hz == 0u) {
        set_pio_debug(true, "bad Tswim for comm reset");
        return false;
    }

    /*
     * Communication reset:
     *   host LOW for 128 SWIM clocks, then release.
     *
     * TX PIO releases SWIM and raises IRQ0.
     * RX PIO is already armed and captures the target's second sync pulse
     * immediately after release, avoiding C polling latency.
     */
    swim_pio_tick_segment_t segments[] = {
        { .level = SWIM_SEG_LOW, .duration_ticks = SWIM_SYNC_CLOCKS },
    };

    swim_pio_rx_width_t width = {0};
    uint32_t max_loops = swim_pio_rx_ns_to_max_loops(300000u);

    rpsw_status_t st = swim_pio_emit_tick_segments_capture_response(
        segments,
        1u,
        tick_hz,
        max_loops,
        timeout_us,
        &width
    );

    if (st != RPSW_OK) {
        set_pio_debug(true, swim_pio_waveform_error());
        return false;
    }

    if (width.timeout || width.low_us < 8u || width.low_us > 300u) {
        set_pio_debug(true, "PIO RX second sync width out of range");
        return false;
    }

    record_sync_measurement_ns(width.low_ns, width.loops_used);
    set_pio_debug(true, "ok");
    return true;
}

bool swim_phy_write_bit(bool bit) {
    /*
     * UM0470 section 3.3:
     * Low speed = 22 SWIM clocks, 2/20 for one, 20/2 for zero. Timing is
     * derived from the target synchronization frame; do not guess if no target
     * has been measured.
     */
    if (g_phy.speed != SWIM_SPEED_LOW || !swim_phy_timing_ready()) {
        return false;
    }
    uint32_t low_cycles = bit ? g_low_speed_one_low_cycles : g_low_speed_zero_low_cycles;
    uint32_t high_cycles = bit ? g_low_speed_one_high_cycles : g_low_speed_zero_high_cycles;
    swim_phy_sio_drive_low_fast();
    busy_wait_at_least_cycles(low_cycles);
    swim_phy_sio_release_fast();
    busy_wait_at_least_cycles(high_cycles);
    return true;
}

bool swim_phy_write_frame_bits(uint32_t bits_msb_first, unsigned bit_count) {
    if (g_phy.speed != SWIM_SPEED_LOW || !swim_phy_timing_ready() ||
        bit_count == 0 || bit_count > 16 || !swim_pio_waveform_available()) {
        return false;
    }

    swim_pio_tick_segment_t segments[34];
    size_t segment_count = 0;
    segments[segment_count++] = (swim_pio_tick_segment_t){
        .level = SWIM_SEG_RELEASE,
        .duration_ticks = SWIM_LOW_SPEED_INTERFRAME_GAP_CLOCKS * SWIM_PIO_FRAME_TICKS_PER_SWIM_CLOCK,
    };
    for (unsigned i = 0; i < bit_count; i++) {
        bool bit = ((bits_msb_first >> (bit_count - 1u - i)) & 1u) != 0u;
        uint32_t low_clocks = bit ? SWIM_LOW_SPEED_ONE_LOW_CLOCKS : SWIM_LOW_SPEED_ZERO_LOW_CLOCKS;
        uint32_t high_clocks = bit ? SWIM_LOW_SPEED_ONE_HIGH_CLOCKS : SWIM_LOW_SPEED_ZERO_HIGH_CLOCKS;
        segments[segment_count++] = (swim_pio_tick_segment_t){
            .level = SWIM_SEG_LOW,
            .duration_ticks = low_clocks * SWIM_PIO_FRAME_TICKS_PER_SWIM_CLOCK,
        };
        segments[segment_count++] = (swim_pio_tick_segment_t){
            .level = SWIM_SEG_RELEASE,
            .duration_ticks = high_clocks * SWIM_PIO_FRAME_TICKS_PER_SWIM_CLOCK,
        };
    }

    uint64_t tick_hz = (uint64_t)SWIM_PIO_FRAME_TICKS_PER_SWIM_CLOCK * 1000000000ull /
                       (uint64_t)g_debug.derived_tswim_ns;
    rpsw_status_t st = swim_pio_emit_tick_segments(segments, segment_count, (uint32_t)tick_hz);
    if (st == RPSW_OK) {
        swim_phy_sio_input();
        return true;
    }
    set_pio_debug(true, swim_pio_waveform_error());
    swim_phy_sio_input();
    return false;
}

bool swim_phy_write_frame_bits_read_ack(uint32_t bits_msb_first, unsigned bit_count,
                                        uint32_t timeout_us, bool *ack) {
    if (ack == NULL || g_phy.speed != SWIM_SPEED_LOW || !swim_phy_timing_ready() ||
        bit_count == 0 || bit_count > 16 || !swim_pio_waveform_available()) {
        return false;
    }

    swim_pio_tick_segment_t segments[34];
    size_t segment_count = 0;
    segments[segment_count++] = (swim_pio_tick_segment_t){
        .level = SWIM_SEG_RELEASE,
        .duration_ticks = SWIM_LOW_SPEED_INTERFRAME_GAP_CLOCKS * SWIM_PIO_FRAME_TICKS_PER_SWIM_CLOCK,
    };
    for (unsigned i = 0; i < bit_count; i++) {
        bool bit = ((bits_msb_first >> (bit_count - 1u - i)) & 1u) != 0u;
        uint32_t low_clocks = bit ? SWIM_LOW_SPEED_ONE_LOW_CLOCKS : SWIM_LOW_SPEED_ZERO_LOW_CLOCKS;
        uint32_t high_clocks = bit ? SWIM_LOW_SPEED_ONE_HIGH_CLOCKS : SWIM_LOW_SPEED_ZERO_HIGH_CLOCKS;
        segments[segment_count++] = (swim_pio_tick_segment_t){
            .level = SWIM_SEG_LOW,
            .duration_ticks = low_clocks * SWIM_PIO_FRAME_TICKS_PER_SWIM_CLOCK,
        };
        segments[segment_count++] = (swim_pio_tick_segment_t){
            .level = SWIM_SEG_RELEASE,
            .duration_ticks = high_clocks * SWIM_PIO_FRAME_TICKS_PER_SWIM_CLOCK,
        };
    }

    uint64_t tick_hz = (uint64_t)SWIM_PIO_FRAME_TICKS_PER_SWIM_CLOCK * 1000000000ull /
                       (uint64_t)g_debug.derived_tswim_ns;
    swim_pio_rx_width_t response = {0};
    uint32_t max_loops = swim_pio_rx_ns_to_max_loops(clocks_to_ns(64u));
    rpsw_status_t st = swim_pio_emit_tick_segments_capture_response(
        segments, segment_count, (uint32_t)tick_hz, max_loops, timeout_us, &response);
    swim_phy_sio_input();
    if (st != RPSW_OK) {
        set_pio_debug(true, swim_pio_waveform_error());
        return false;
    }
    if (response.timeout || response.loops_used == 0) {
        char message[64];
        snprintf(message, sizeof(message), "%s ACK width timeout",
                 g_tx_context[0] != '\0' ? g_tx_context : "tx");
        set_pio_debug(true, message);
        return false;
    }

    uint32_t low_clocks = (response.low_ns + (g_debug.derived_tswim_ns / 2u)) /
                           g_debug.derived_tswim_ns;
    *ack = low_clocks <= SWIM_LOW_SPEED_ONE_MAX_LOW_CLOCKS;
    return true;
}

static bool classify_low_width_ns(uint32_t low_ns, bool *bit) {
    if (bit == NULL || low_ns == 0u || g_debug.derived_tswim_ns == 0u) {
        return false;
    }

    uint32_t low_clocks =
    (uint32_t)(((uint64_t)low_ns + (g_debug.derived_tswim_ns / 2u)) /
    g_debug.derived_tswim_ns);

    if (low_clocks <= SWIM_LOW_SPEED_ONE_MAX_LOW_CLOCKS) {
        *bit = true;
        return true;
    }

    if (low_clocks >= SWIM_LOW_SPEED_ZERO_MIN_LOW_CLOCKS) {
        *bit = false;
        return true;
    }

    return false;
}

#define SWIM_PHY_MAX_TX_FRAME_SEGMENTS 96u

static bool build_tx_frame_segments(uint32_t bits,
                                    uint bit_count,
                                    swim_pio_tick_segment_t *segments,
                                    size_t *segment_count) {
    if (segments == NULL || segment_count == NULL || bit_count == 0u || bit_count > 32u) {
        return false;
    }

    size_t out = 0;

    if (out + 1u > SWIM_PHY_MAX_TX_FRAME_SEGMENTS) {
        return false;
    }
    segments[out++] = (swim_pio_tick_segment_t){
        .level = SWIM_SEG_RELEASE,
        .duration_ticks = SWIM_LOW_SPEED_INTERFRAME_GAP_CLOCKS * SWIM_PIO_FRAME_TICKS_PER_SWIM_CLOCK,
    };

    /*
     * Low-speed SWIM bit encoding:
     *   bit 1: LOW  2*Tswim, HIGH 20*Tswim
     *   bit 0: LOW 20*Tswim, HIGH  2*Tswim
     *
     * tick_hz for frame TX is:
     *   SWIM_PIO_FRAME_TICKS_PER_SWIM_CLOCK / Tswim
     *
     * Therefore duration_ticks must be SWIM clocks multiplied by
     * SWIM_PIO_FRAME_TICKS_PER_SWIM_CLOCK.
     *
     * Do NOT add SWIM_PIO_SEGMENT_OVERHEAD_TICKS here. encode_tick_segments()
     * subtracts that internally from the requested duration.
     */
    const uint32_t short_ticks = 2u * SWIM_PIO_FRAME_TICKS_PER_SWIM_CLOCK;
    const uint32_t long_ticks  = 20u * SWIM_PIO_FRAME_TICKS_PER_SWIM_CLOCK;

    for (int i = (int)bit_count - 1; i >= 0; --i) {
        bool bit = ((bits >> (uint)i) & 1u) != 0u;

        if (out + 2u > SWIM_PHY_MAX_TX_FRAME_SEGMENTS) {
            return false;
        }

        segments[out++] = (swim_pio_tick_segment_t){
            .level = SWIM_SEG_LOW,
            .duration_ticks = bit ? short_ticks : long_ticks,
        };

        segments[out++] = (swim_pio_tick_segment_t){
            .level = SWIM_SEG_RELEASE,
            .duration_ticks = bit ? long_ticks : short_ticks,
        };
    }

    *segment_count = out;
    return true;
}

#define SWIM_PHY_ACK_FRAME_SCAN_WIDTHS 48u
#define SWIM_PHY_RESPONSE_GAP_TIMEOUT_US 5000u

static bool decode_target_frame_from_widths(const swim_pio_rx_width_t *widths,
                                            uint32_t start_index,
                                            uint32_t *frame_out) {
    if (widths == NULL || frame_out == NULL) {
        return false;
    }

    uint32_t frame = 0;

    for (uint32_t i = 0; i < 10u; ++i) {
        bool bit = false;
        if (!classify_low_width_ns(widths[start_index + i].low_ns, &bit)) {
            return false;
        }

        frame = (frame << 1u) | (bit ? 1u : 0u);
    }

    /*
     * Expected target data frame layout:
     *   header/start bit = 1
     *   8 data bits
     *   parity bit
     */
    bool header = ((frame >> 9u) & 1u) != 0u;
    if (!header) {
        return false;
    }

    uint8_t value = (uint8_t)((frame >> 1u) & 0xffu);
    bool parity = (frame & 1u) != 0u;

    if (parity != swim_parity8(value)) {
        return false;
    }

    *frame_out = frame;
    return true;
}

static uint32_t reverse_low_bits(uint32_t value, uint32_t bit_count) {
    uint32_t out = 0;
    for (uint32_t i = 0; i < bit_count; ++i) {
        out = (out << 1u) | (value & 1u);
        value >>= 1u;
    }
    return out;
}

static bool decode_ack_frame_raw(uint32_t raw, uint32_t *frame_out) {
    if (frame_out == NULL) {
        return false;
    }

    bool ack = ((raw >> 10u) & 1u) != 0u;
    if (!ack) {
        return false;
    }

    uint32_t frame = raw & 0x3ffu;
    bool header = ((frame >> 9u) & 1u) != 0u;
    if (!header) {
        return false;
    }

    uint8_t value = (uint8_t)((frame >> 1u) & 0xffu);
    bool parity = (frame & 1u) != 0u;
    if (parity != swim_parity8(value)) {
        return false;
    }

    *frame_out = frame;
    return true;
}

bool swim_phy_write_frame_bits_read_ack_and_frame(uint32_t bits,
                                                  uint bit_count,
                                                  uint32_t timeout_us,
                                                  uint32_t *rx_frame) {
    if (rx_frame == NULL || bit_count == 0u || bit_count > 32u ||
        g_phy.speed != SWIM_SPEED_LOW || !swim_phy_timing_ready() ||
        g_debug.derived_tswim_ns == 0u) {
        return false;
    }

    swim_pio_tick_segment_t segments[SWIM_PHY_MAX_TX_FRAME_SEGMENTS];
    size_t segment_count = 0;
    if (!build_tx_frame_segments(bits, bit_count, segments, &segment_count)) {
        set_pio_debug(true, "bad tx frame for ack+frame");
        return false;
    }

    uint64_t tick_hz64 =
        (uint64_t)SWIM_PIO_FRAME_TICKS_PER_SWIM_CLOCK * 1000000000ull /
        (uint64_t)g_debug.derived_tswim_ns;
    if (tick_hz64 == 0u || tick_hz64 > UINT32_MAX) {
        set_pio_debug(true, "bad tick_hz for ack+frame");
        return false;
    }

    /*
     * UM0470 receive after the last ROTF address byte:                    *
     *   target ACK bit + target 10-bit data frame.
     *
     * Decode this in PIO as one state-machine transaction. The previous
     * low-width stream mode pushed after every bit and could lose sync on the
     * short 2*Tswim HIGH gaps. This path pushes only one decoded raw word.
     */

    uint32_t threshold_ns = g_debug.derived_tswim_ns * SWIM_LOW_SPEED_ONE_MAX_LOW_CLOCKS;
    uint32_t threshold_loops = swim_pio_rx_ns_to_max_loops(threshold_ns);
    if (threshold_loops == 0u) {
        threshold_loops = 1u;
    }

    uint32_t raw = 0;
    uint32_t rx_timeout_us = timeout_us;
    if (rx_timeout_us < 2000u) {
        rx_timeout_us = 2000u;
    }

    rpsw_status_t st = swim_pio_emit_tick_segments_decode_bits(
        segments,
        segment_count,
        (uint32_t)tick_hz64,
        threshold_loops,
        rx_timeout_us,
        11u,
        &raw
    );

    swim_phy_sio_input();

    if (st != RPSW_OK) {
        static char err[96];
        snprintf(err, sizeof(err), "ackfrm timeout %s raw=0x%03lx",
                 g_tx_context[0] != '\0' ? g_tx_context : "tx",
                 (unsigned long)raw);
        set_pio_debug(true, err);
        return false;
    }

    uint32_t frame = 0;
    if (decode_ack_frame_raw(raw, &frame)) {
        *rx_frame = frame;
        set_pio_debug(true, "ok");
        return true;
    }

    uint32_t rev = reverse_low_bits(raw, 11u);
    if (decode_ack_frame_raw(rev, &frame)) {
        *rx_frame = frame;
        set_pio_debug(true, "ok rev");
        return true;
    }

    static char err[96];
    snprintf(err, sizeof(err), "bad ackfrm %s raw=0x%03lx rev=0x%03lx",
             g_tx_context[0] != '\0' ? g_tx_context : "tx",
             (unsigned long)(raw & 0x7ffu),
             (unsigned long)(rev & 0x7ffu));
    set_pio_debug(true, err);
    return false;
}

bool swim_phy_read_frame_bits(uint32_t timeout_us, uint32_t *bits) {
    if (bits == NULL || g_phy.speed != SWIM_SPEED_LOW || !swim_phy_timing_ready() ||
        g_debug.derived_tswim_ns == 0u) {
        return false;
    }

    uint32_t frame = 0;

    /*
     * SWIM data frame is read MSB-first:
     *   header bit, 8 data bits, parity bit
     *
     * swim_phy_read_bit() must use PIO low-width capture internally.
     */
    for (uint32_t i = 0; i < 10u; ++i) {
        bool ok = false;
        bool bit = swim_phy_read_bit(timeout_us, &ok);
        if (!ok) {
            set_pio_debug(true, "PIO RX frame bit timeout");
            return false;
        }

        frame = (frame << 1u) | (bit ? 1u : 0u);
    }

    *bits = frame;
    set_pio_debug(true, "ok");
    return true;
}

static bool swim_phy_read_bit_pio(uint32_t timeout_us, bool *ok) {
    if (ok == NULL) {
        return false;
    }

    *ok = false;

    if (g_phy.speed != SWIM_SPEED_LOW || !swim_phy_timing_ready() ||
        g_debug.derived_tswim_ns == 0u) {
        return false;
    }

    /*
    * Target bit low is either:
    *   1:  2*Tswim
    *   0: 20*Tswim
    * Give enough budget for a long zero plus margin.
    */
    uint32_t max_ns = g_debug.derived_tswim_ns * 80u;
    if (max_ns < 10000u) {
        max_ns = 10000u;
    }

    uint32_t max_loops = swim_pio_rx_ns_to_max_loops(max_ns);
    swim_pio_rx_width_t width = {0};

    rpsw_status_t st = swim_pio_rx_arm_now(max_loops);
    if (st != RPSW_OK) {
        set_pio_debug(true, "PIO RX bit arm failed");
        return false;
    }

    st = swim_pio_rx_get_width(&width, timeout_us);
    if (st != RPSW_OK || width.timeout || width.low_ns == 0u) {
        set_pio_debug(true, "PIO RX bit timeout");
        return false;
    }

    bool bit = false;
    if (!classify_low_width_ns(width.low_ns, &bit)) {
        *ok = false;
        set_pio_debug(true, "PIO RX bit ambiguous");
        return false;
    }

    *ok = true;
    return bit;
}

bool swim_phy_read_bit(uint32_t timeout_us, bool *ok) {
    return swim_phy_read_bit_pio(timeout_us, ok);
}

bool swim_phy_wait_sync(uint32_t timeout_us) {
    absolute_time_t deadline = make_timeout_time_us(timeout_us);
    while (absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
        while (swim_phy_sample()) {
            if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
                return false;
            }
            tight_loop_contents();
        }

        uint64_t start_us = time_us_64();
        uint32_t low_count = 0;
        while (!swim_phy_sample()) {
            low_count++;
            tight_loop_contents();
            if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
                return false;
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

        record_sync_measurement_ns((uint32_t)(elapsed_us * 1000u), low_count);
        return true;
    }

    return false;
}

bool swim_phy_timing_ready(void) {
    return g_debug.synced && g_debug.derived_tswim_ns != 0 &&
           g_debug.sync_low_loop_count != 0 && g_phy.speed == SWIM_SPEED_LOW;
}

void swim_phy_reset_timing(void) {
    g_debug.synced = false;
    g_debug.last_sync_low_us = 0;
    g_debug.last_sync_low_ns = 0;
    g_debug.derived_tswim_ns = 0;
    g_debug.sync_low_loop_count = 0;

    g_debug.enter_stage = SWIM_ENTER_STAGE_IDLE;
    g_debug.comm_reset_sent = false;
    g_debug.second_sync_seen = false;
    g_debug.comm_reset_low_us = 0;
    g_debug.comm_reset_low_ns = 0;

    g_debug.swim_csr = 0;
    g_debug.swim_csr_valid = false;
    g_debug.speed = g_phy.speed;
    g_debug.entry_protocol_us = SWIM_ENTRY_PROTOCOL_US;
    g_debug.entry_slow_pulses = SWIM_ENTRY_SLOW_PULSES;
    g_debug.entry_fast_pulses = SWIM_ENTRY_FAST_PULSES;
    g_low_speed_one_low_cycles = 1u;
    g_low_speed_one_high_cycles = 1u;
    g_low_speed_zero_low_cycles = 1u;
    g_low_speed_zero_high_cycles = 1u;
}

void swim_phy_set_swim_csr_debug(uint8_t value, bool valid) {
    g_debug.swim_csr = value;
    g_debug.swim_csr_valid = valid;
}

swim_phy_debug_t swim_phy_get_debug(void) {
    return g_debug;
}

void swim_phy_debug_waveform(void) {
    swim_phy_reset_timing();
    swim_phy_nrst_assert();
    sleep_ms(2);
    swim_phy_entry_waveform();
    g_debug.synced = true;
    g_debug.last_sync_low_us = 16;
    g_debug.last_sync_low_ns = 16000;
    g_debug.derived_tswim_ns = 125;
    g_debug.sync_low_loop_count = 128;
    swim_phy_comm_reset();
    for (unsigned i = 0; i < 8; i++) {
        (void)swim_phy_write_bit((i & 1u) != 0u);
    }
    swim_phy_nrst_release();
    swim_phy_release();
}
