#ifndef SWIM_SYNC_LIMITS_H
#define SWIM_SYNC_LIMITS_H

#include <stdbool.h>
#include <stdint.h>

#define SWIM_SYNC_CLOCKS 128u

/*
 * Real STM8S003 observations:
 * - first sync may be around 16..17 us when SWIM clock is near 8 MHz;
 * - slower sync candidates around 33 us, 62 us, or 128 us are also plausible
 *   depending on target timing and measurement path.
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
