#ifndef SWIM_PIO_WAVEFORM_H
#define SWIM_PIO_WAVEFORM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "pico/types.h"
#include "usb_protocol.h"

typedef enum {
    SWIM_SEG_RELEASE = 0,
    SWIM_SEG_LOW = 1,
} swim_segment_level_t;

typedef struct {
    swim_segment_level_t level;
    uint32_t duration_us;
} swim_segment_t;

typedef struct {
    uint32_t low_us;
    uint32_t low_ns;
    uint32_t low_loop_count;
} swim_pio_sync_measurement_t;

typedef struct {
    swim_segment_level_t level;
    uint32_t duration_ticks;
} swim_pio_tick_segment_t;

rpsw_status_t swim_pio_waveform_init(uint swim_pin, bool enable_internal_pullup);
rpsw_status_t swim_pio_emit_segments(const swim_segment_t *segments, size_t count);
rpsw_status_t swim_pio_emit_tick_segments(const swim_pio_tick_segment_t *segments, size_t count,
                                          uint32_t tick_hz);
rpsw_status_t swim_pio_emit_tick_segments_wait_response(const swim_pio_tick_segment_t *segments,
                                                        size_t count, uint32_t tick_hz,
                                                        uint32_t timeout_us,
                                                        swim_pio_sync_measurement_t *measurement);
rpsw_status_t swim_pio_emit_segments_wait_sync(const swim_segment_t *segments, size_t count,
                                               uint32_t timeout_us,
                                               swim_pio_sync_measurement_t *measurement);
bool swim_pio_waveform_available(void);
const char *swim_pio_waveform_error(void);

#endif
