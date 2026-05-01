#include "stm8_dm.h"

#include "swim_link.h"

/*
 * The SWIM debug module is memory mapped. For MVP probing, avoid writes to DM
 * control registers and simply attempt a benign read from the reset vector area.
 */
rpsw_status_t stm8_dm_probe(uint8_t *status, uint16_t *status_len) {
    uint8_t reset_vector[4] = {0};
    rpsw_status_t st = swim_link_read(0x008000u, reset_vector, sizeof(reset_vector));
    if (st != RPSW_OK) {
        return st;
    }
    for (unsigned i = 0; i < sizeof(reset_vector); i++) {
        status[i] = reset_vector[i];
    }
    *status_len = sizeof(reset_vector);
    return RPSW_OK;
}

rpsw_status_t stm8_dm_memory_read(uint32_t address, uint8_t *data, size_t len) {
    size_t done = 0;
    while (done < len) {
        size_t chunk = len - done;
        if (chunk > 255u) {
            chunk = 255u;
        }
        rpsw_status_t st = swim_link_read(address + (uint32_t)done, data + done, chunk);
        if (st != RPSW_OK) {
            return st;
        }
        done += chunk;
    }
    return RPSW_OK;
}

rpsw_status_t stm8_dm_memory_write(uint32_t address, const uint8_t *data, size_t len) {
    size_t done = 0;
    while (done < len) {
        size_t chunk = len - done;
        if (chunk > 255u) {
            chunk = 255u;
        }
        rpsw_status_t st = swim_link_write(address + (uint32_t)done, data + done, chunk);
        if (st != RPSW_OK) {
            return st;
        }
        done += chunk;
    }
    return RPSW_OK;
}
