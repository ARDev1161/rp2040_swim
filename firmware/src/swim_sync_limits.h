#ifndef SWIM_SYNC_LIMITS_H
#define SWIM_SYNC_LIMITS_H

#include <stdbool.h>
#include <stdint.h>

#define SWIM_SYNC_CLOCKS 128u

/*
 * Real STM8S003 observations:
 * - short post-entry lows around 16..17 us must be ignored;
 * - plausible sync candidates may be around 33 us, 62 us, or 128 us
 *   depending on target timing / measurement path.
 */
#define SWIM_SYNC_IGNORE_SHORT_US 8u
#define SWIM_SYNC_MIN_US 12u
#define SWIM_SYNC_MAX_US 300u

static inline bool swim_sync_low_width_is_plausible_us(uint32_t elapsed_us) {
    if (elapsed_us < SWIM_SYNC_IGNORE_SHORT_US) {
        return false;
    }
    return elapsed_us >= SWIM_SYNC_MIN_US && elapsed_us <= SWIM_SYNC_MAX_US;
}

#endif
