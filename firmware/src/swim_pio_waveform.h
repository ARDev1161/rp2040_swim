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

rpsw_status_t swim_pio_waveform_init(uint swim_pin, bool enable_internal_pullup);
rpsw_status_t swim_pio_emit_segments(const swim_segment_t *segments, size_t count);
bool swim_pio_waveform_available(void);
const char *swim_pio_waveform_error(void);

#endif
