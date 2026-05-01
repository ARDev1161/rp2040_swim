#ifndef STM8_DM_H
#define STM8_DM_H

#include <stddef.h>
#include <stdint.h>
#include "usb_protocol.h"

rpsw_status_t stm8_dm_probe(uint8_t *status, uint16_t *status_len);
rpsw_status_t stm8_dm_memory_read(uint32_t address, uint8_t *data, size_t len);
rpsw_status_t stm8_dm_memory_write(uint32_t address, const uint8_t *data, size_t len);

#endif
