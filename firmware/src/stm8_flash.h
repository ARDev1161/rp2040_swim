#ifndef STM8_FLASH_H
#define STM8_FLASH_H

#include <stddef.h>
#include <stdint.h>
#include "usb_protocol.h"

const char *stm8_flash_last_error(void);
rpsw_status_t stm8_flash_unlock_program(void);
rpsw_status_t stm8_flash_unlock_eeprom(void);
rpsw_status_t stm8_flash_write_option_byte(uint32_t address, uint8_t value);
rpsw_status_t stm8_flash_erase_range(uint32_t address, uint32_t length);
rpsw_status_t stm8_flash_write_block(uint32_t address, const uint8_t *data, size_t len);

#endif
