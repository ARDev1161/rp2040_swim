#include "stm8_flash.h"

#include "stm8_dm.h"
#include "swim_phy.h"

#define STM8_FLASH_IAPSR 0x00505Fu
#define STM8_FLASH_PUKR  0x005062u
#define STM8_FLASH_DUKR  0x005064u
#define STM8_PUKR_KEY1   0x56u
#define STM8_PUKR_KEY2   0xAEu
#define STM8_DUKR_KEY1   0xAEu
#define STM8_DUKR_KEY2   0x56u
#define STM8_IAPSR_PUL   0x02u
#define STM8_IAPSR_DUL   0x08u

static rpsw_status_t wait_iapsr_mask(uint8_t mask) {
    for (unsigned attempt = 0; attempt < 100; attempt++) {
        uint8_t v = 0;
        rpsw_status_t st = stm8_dm_memory_read(STM8_FLASH_IAPSR, &v, 1);
        if (st != RPSW_OK) {
            return st;
        }
        if ((v & mask) == mask) {
            return RPSW_OK;
        }
        swim_phy_delay_us(1000);
    }
    return RPSW_ERR_TARGET;
}

rpsw_status_t stm8_flash_unlock_program(void) {
    const uint8_t keys[] = {STM8_PUKR_KEY1, STM8_PUKR_KEY2};
    rpsw_status_t st = stm8_dm_memory_write(STM8_FLASH_PUKR, keys, sizeof(keys));
    if (st != RPSW_OK) {
        return st;
    }
    return wait_iapsr_mask(STM8_IAPSR_PUL);
}

rpsw_status_t stm8_flash_unlock_eeprom(void) {
    const uint8_t keys[] = {STM8_DUKR_KEY1, STM8_DUKR_KEY2};
    rpsw_status_t st = stm8_dm_memory_write(STM8_FLASH_DUKR, keys, sizeof(keys));
    if (st != RPSW_OK) {
        return st;
    }
    return wait_iapsr_mask(STM8_IAPSR_DUL);
}

rpsw_status_t stm8_flash_erase_range(uint32_t address, uint32_t length) {
    (void)address;
    (void)length;
    /*
     * TODO(hardware-validation): implement block erase through FLASH_CR2/NCR2
     * once verified against STM8S003F3/STM8S103F3 hardware. Returning a guard
     * error prevents accidental mass changes based on unverified sequencing.
     */
    return RPSW_ERR_FLASH_GUARD;
}

rpsw_status_t stm8_flash_write_block(uint32_t address, const uint8_t *data, size_t len) {
    (void)address;
    (void)data;
    (void)len;
    /*
     * TODO(hardware-validation): implement standard block programming using
     * PRG/NPRG and WR_PG_DIS timing after erase behavior is checked on target.
     */
    return RPSW_ERR_FLASH_GUARD;
}
