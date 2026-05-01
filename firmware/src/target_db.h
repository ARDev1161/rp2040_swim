#ifndef TARGET_DB_H
#define TARGET_DB_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
    const char *name;
    uint32_t flash_start;
    uint32_t flash_size;
    uint32_t ram_start;
    uint32_t ram_size;
    uint32_t eeprom_start;
    uint32_t eeprom_size;
    uint32_t option_start;
    uint32_t option_size;
    uint16_t block_size;
} stm8_target_t;

const stm8_target_t *target_db_find(const char *name);
const stm8_target_t *target_db_all(size_t *count);

#endif
