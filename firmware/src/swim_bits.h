#pragma once

#include <stdbool.h>
#include <stdint.h>

static inline bool swim_parity8(uint8_t v) {
    v ^= (uint8_t)(v >> 4u);
    v ^= (uint8_t)(v >> 2u);
    v ^= (uint8_t)(v >> 1u);
    return (v & 1u) != 0u;
}
