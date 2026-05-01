#include "swim_phy.h"

#include "board_rp2040_zero.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "pico/platform.h"
#include "pico/stdlib.h"
#include "swim_pio_waveform.h"
#include <stdio.h>
#include <string.h>

#define SWIM_SYNC_CLOCKS 128u
#define SWIM_LOW_SPEED_ONE_LOW_CLOCKS 2u
#define SWIM_LOW_SPEED_ONE_HIGH_CLOCKS 20u
#define SWIM_LOW_SPEED_ZERO_LOW_CLOCKS 20u
#define SWIM_LOW_SPEED_ZERO_HIGH_CLOCKS 2u
#define SWIM_LOW_SPEED_ONE_MAX_LOW_CLOCKS 8u
#define SWIM_LOW_SPEED_ZERO_MIN_LOW_CLOCKS 9u
#define SWIM_PHY_BACKEND_PIO 0u
#define SWIM_PHY_BACKEND_BITBANG_FALLBACK 1u
#define SWIM_ENTRY_PROTOCOL_US 6016u
#define SWIM_ENTRY_SLOW_PULSES 4u
#define SWIM_ENTRY_FAST_PULSES 4u

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

static const swim_segment_t g_entry_segments[] = {
    {SWIM_SEG_RELEASE, 10},
    {SWIM_SEG_LOW, 16},
    {SWIM_SEG_RELEASE, 500}, {SWIM_SEG_LOW, 500},
    {SWIM_SEG_RELEASE, 500}, {SWIM_SEG_LOW, 500},
    {SWIM_SEG_RELEASE, 500}, {SWIM_SEG_LOW, 500},
    {SWIM_SEG_RELEASE, 500}, {SWIM_SEG_LOW, 500},
    {SWIM_SEG_RELEASE, 250}, {SWIM_SEG_LOW, 250},
    {SWIM_SEG_RELEASE, 250}, {SWIM_SEG_LOW, 250},
    {SWIM_SEG_RELEASE, 250}, {SWIM_SEG_LOW, 250},
    {SWIM_SEG_RELEASE, 250}, {SWIM_SEG_LOW, 250},
    {SWIM_SEG_RELEASE, 10},
};

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

rpsw_status_t swim_phy_entry_sequence_um0470(void) {
    /*
     * UM0470 Rev 4 section 3.2:
     * - force SWIM low for 16 us,
     * - four pulses at 1 kHz,
     * - four pulses at 2 kHz,
     * sequence starts and ends released/high.
     */
    if (swim_pio_waveform_available()) {
        rpsw_status_t st = swim_pio_emit_segments(g_entry_segments,
                                                  sizeof(g_entry_segments) / sizeof(g_entry_segments[0]));
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
    bitbang_emit_segments(g_entry_segments, sizeof(g_entry_segments) / sizeof(g_entry_segments[0]));
    swim_phy_release();
    return RPSW_OK;
}

void swim_phy_entry_waveform(void) {
    (void)swim_phy_entry_sequence_um0470();
}

void swim_phy_comm_reset(void) {
    /*
     * UM0470 section 3.6: reset SWIM communication by holding SWIM low for
     * 128 SWIM clock periods. After the first synchronization frame this uses
     * the measured SWIM clock; before calibration it falls back to the nominal
     * low-speed 16 us reset frame only for debug waveform generation.
     */
    if (swim_phy_timing_ready()) {
        swim_phy_drive_low();
        busy_wait_ns(clocks_to_ns(SWIM_SYNC_CLOCKS));
        swim_phy_release();
        busy_wait_ns(clocks_to_ns(SWIM_SYNC_CLOCKS));
    } else {
        pulse_low(16, 16);
    }
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
    uint32_t low_clocks = bit ? SWIM_LOW_SPEED_ONE_LOW_CLOCKS : SWIM_LOW_SPEED_ZERO_LOW_CLOCKS;
    uint32_t high_clocks = bit ? SWIM_LOW_SPEED_ONE_HIGH_CLOCKS : SWIM_LOW_SPEED_ZERO_HIGH_CLOCKS;
    swim_phy_drive_low();
    busy_wait_ns(clocks_to_ns(low_clocks));
    swim_phy_release();
    busy_wait_ns(clocks_to_ns(high_clocks));
    return true;
}

bool swim_phy_read_bit(uint32_t timeout_us, bool *ok) {
    if (g_phy.speed != SWIM_SPEED_LOW || !swim_phy_timing_ready() ||
        g_debug.sync_low_loop_count == 0) {
        *ok = false;
        return false;
    }

    absolute_time_t deadline = make_timeout_time_us(timeout_us);
    while (swim_phy_sample()) {
        if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
            *ok = false;
            return false;
        }
    }

    uint32_t low_count = 0;
    while (!swim_phy_sample()) {
        low_count++;
        tight_loop_contents();
        if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
            *ok = false;
            return false;
        }
    }

    *ok = true;
    uint32_t low_clocks = (uint32_t)(((uint64_t)low_count * SWIM_SYNC_CLOCKS +
                                      (g_debug.sync_low_loop_count / 2u)) /
                                     g_debug.sync_low_loop_count);
    if (low_clocks <= SWIM_LOW_SPEED_ONE_MAX_LOW_CLOCKS) {
        return true;
    }
    if (low_clocks >= SWIM_LOW_SPEED_ZERO_MIN_LOW_CLOCKS) {
        return false;
    }
    *ok = false;
    return false;
}

bool swim_phy_wait_sync(uint32_t timeout_us) {
    absolute_time_t deadline = make_timeout_time_us(timeout_us);
    while (swim_phy_sample()) {
        if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
            return false;
        }
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
        return false;
    }

    uint32_t elapsed_us = (uint32_t)elapsed_us64;
    uint32_t elapsed_ns = elapsed_us * 1000u;
    uint32_t tswim_ns = (elapsed_ns + (SWIM_SYNC_CLOCKS / 2u)) / SWIM_SYNC_CLOCKS;
    if (tswim_ns == 0) {
        return false;
    }

    g_debug.synced = true;
    g_debug.speed = SWIM_SPEED_LOW;
    g_debug.last_sync_low_us = elapsed_us;
    g_debug.last_sync_low_ns = elapsed_ns;
    g_debug.derived_tswim_ns = tswim_ns;
    g_debug.sync_low_loop_count = low_count;
    g_phy.speed = SWIM_SPEED_LOW;
    return true;
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
    g_debug.swim_csr = 0;
    g_debug.swim_csr_valid = false;
    g_debug.speed = g_phy.speed;
    g_debug.entry_protocol_us = SWIM_ENTRY_PROTOCOL_US;
    g_debug.entry_slow_pulses = SWIM_ENTRY_SLOW_PULSES;
    g_debug.entry_fast_pulses = SWIM_ENTRY_FAST_PULSES;
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
