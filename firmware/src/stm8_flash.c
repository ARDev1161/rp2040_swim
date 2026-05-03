#include "stm8_flash.h"
#include <stdio.h>
#include "stm8_dm.h"
#include "swim_phy.h"

#define STM8_FLASH_CR2   0x00505Bu
#define STM8_FLASH_NCR2  0x00505Cu
#define STM8_FLASH_IAPSR 0x00505Fu
#define STM8_FLASH_PUKR  0x005062u
#define STM8_FLASH_DUKR  0x005064u

#define STM8_PUKR_KEY1   0x56u
#define STM8_PUKR_KEY2   0xAEu
#define STM8_DUKR_KEY1   0xAEu
#define STM8_DUKR_KEY2   0x56u

#define STM8_FLASH_BLOCK_SIZE   64u
#define STM8_FLASH_ERASED_VALUE 0x00u

#define STM8_OPTION_START 0x004800u
#define STM8_OPTION_END   0x004840u

#define STM8_CR2_ERASE 0x20u
#define STM8_CR2_OPT   0x80u

#define STM8_IAPSR_WR_PG_DIS 0x01u
#define STM8_IAPSR_PUL       0x02u
#define STM8_IAPSR_EOP       0x04u
#define STM8_IAPSR_DUL       0x08u
#define STM8_IAPSR_HVOFF     0x40u

static char g_flash_last_error[128];

const char *stm8_flash_last_error(void) {
    return g_flash_last_error;
}

static void flash_diag_clear(void) {
    g_flash_last_error[0] = '\0';
}

static void flash_diag_set(const char *stage, uint32_t address, rpsw_status_t status) {
    snprintf(g_flash_last_error, sizeof(g_flash_last_error),
             "%s addr=0x%06lx status=%s", stage, (unsigned long)address,
             rpsw_status_text(status));
}

static rpsw_status_t read_u8(uint32_t address, uint8_t *value) {
    return stm8_dm_memory_read(address, value, 1);
}

static rpsw_status_t write_u8(uint32_t address, uint8_t value) {
    return stm8_dm_memory_write(address, &value, 1);
}

static rpsw_status_t wait_iapsr_mask(uint8_t mask) {
    for (unsigned attempt = 0; attempt < 100; attempt++) {
        uint8_t v = 0;
        rpsw_status_t st = read_u8(STM8_FLASH_IAPSR, &v);
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

static rpsw_status_t flash_busy_delay_and_resync(unsigned delay_us) {
    if (delay_us != 0u) {
        swim_phy_delay_us(delay_us);
    }

    /*
     * During FLASH high-voltage operations the target may not answer the final
     * ACK/turnaround of the write that triggered erase/programming. Treat that
     * as a busy condition, then restore the SWIM link before polling IAPSR.
     */
    if (!swim_phy_comm_reset_wait_sync(100000u)) {
        return RPSW_ERR_SWIM_TIMEOUT;
    }
    return RPSW_OK;
}

static rpsw_status_t wait_flash_complete(unsigned initial_delay_us) {
    uint8_t last_iapsr = 0;

    if (initial_delay_us != 0u) {
        swim_phy_delay_us(initial_delay_us);
    }

    for (unsigned attempt = 0; attempt < 2000; attempt++) {
        uint8_t v = 0;
        rpsw_status_t st = read_u8(STM8_FLASH_IAPSR, &v);
        if (st != RPSW_OK) {
            snprintf(g_flash_last_error, sizeof(g_flash_last_error),
                     "wait_complete:read_iapsr failed attempt=%u last_iapsr=0x%02x status=%s",
                     attempt, last_iapsr, rpsw_status_text(st));
            return st;
        }

        last_iapsr = v;

        if ((v & STM8_IAPSR_WR_PG_DIS) != 0u) {
            snprintf(g_flash_last_error, sizeof(g_flash_last_error),
                     "wait_complete:wr_pg_dis IAPSR=0x%02x", v);
            return RPSW_ERR_TARGET;
        }

        /*
         * For byte programming EOP is the important completion flag.
         * For block erase/programming HVOFF may also indicate high-voltage
         * completion. Accept either, but still reject WR_PG_DIS above.
         */
        if ((v & (STM8_IAPSR_EOP | STM8_IAPSR_HVOFF)) != 0u) {
            return RPSW_OK;
        }

        swim_phy_delay_us(100);
    }

    snprintf(g_flash_last_error, sizeof(g_flash_last_error),
             "wait_complete:timeout last_iapsr=0x%02x", last_iapsr);
    return RPSW_ERR_TARGET;
}

static rpsw_status_t set_flash_cr2(uint8_t value) {
    rpsw_status_t st = write_u8(STM8_FLASH_CR2, value);
    if (st != RPSW_OK) {
        return st;
    }
    st = write_u8(STM8_FLASH_NCR2, (uint8_t)~value);
    if (st != RPSW_OK) {
        return st;
    }

    uint8_t cr2 = 0;
    uint8_t ncr2 = 0;
    st = read_u8(STM8_FLASH_CR2, &cr2);
    if (st != RPSW_OK) {
        return st;
    }
    st = read_u8(STM8_FLASH_NCR2, &ncr2);
    if (st != RPSW_OK) {
        return st;
    }
    if (cr2 != value || ncr2 != (uint8_t)~value) {
        snprintf(g_flash_last_error, sizeof(g_flash_last_error),
                 "set_cr2 verify failed CR2=0x%02x NCR2=0x%02x expected=0x%02x/0x%02x",
                 cr2, ncr2, value, (uint8_t)~value);
        return RPSW_ERR_TARGET;
    }
    return RPSW_OK;
}

static rpsw_status_t clear_flash_cr2(void) {
    return set_flash_cr2(0x00u);
}

static rpsw_status_t require_program_unlocked(void) {
    uint8_t iapsr = 0;
    rpsw_status_t st = read_u8(STM8_FLASH_IAPSR, &iapsr);
    if (st != RPSW_OK) {
        return st;
    }
    if ((iapsr & STM8_IAPSR_PUL) != 0u) {
        return RPSW_OK;
    }

    /*
     * Host-side unlock is intentionally used for now. Hardware validation
     * showed that the firmware-side back-to-back unlock sequence can time out
     * on this target, while two host MEMORY_WRITE transactions are stable.
     */
    return RPSW_ERR_TARGET;
}

static rpsw_status_t write_key_pair_same_register(uint32_t key_reg, uint8_t key1, uint8_t key2) {
    /*
     * STM8 FLASH_PUKR/DUKR are key registers, not a two-byte memory window.
     * The two key bytes must be written as two separate byte writes to the
     * same register address. A multi-byte WOTF to key_reg would write the
     * second byte to key_reg + 1 and the unlock sequence would be ignored.
     */
    rpsw_status_t st = write_u8(key_reg, key1);
    if (st != RPSW_OK) {
        return st;
    }
    return write_u8(key_reg, key2);
}

rpsw_status_t stm8_flash_unlock_eeprom(void) {
    uint8_t iapsr = 0;
    rpsw_status_t st = read_u8(STM8_FLASH_IAPSR, &iapsr);
    if (st != RPSW_OK) {
        return st;
    }
    if ((iapsr & STM8_IAPSR_DUL) != 0u) {
        return RPSW_OK;
    }

    /*
     * Keep this as a fallback only. Hardware validation showed that host-side
     * split MEMORY_WRITE transactions are more reliable for the DUKR sequence
     * than firmware-side back-to-back key writes on some targets.
     */
    st = write_key_pair_same_register(STM8_FLASH_DUKR,
                                      STM8_DUKR_KEY1,
                                      STM8_DUKR_KEY2);
    if (st != RPSW_OK) {
        return st;
    }
    return wait_iapsr_mask(STM8_IAPSR_DUL);
}

rpsw_status_t stm8_flash_write_option_byte(uint32_t address, uint8_t value) {
    flash_diag_clear();

    if (address < STM8_OPTION_START || address >= STM8_OPTION_END) {
        flash_diag_set("option:bad_address", address, RPSW_ERR_BAD_ARGUMENT);
        return RPSW_ERR_BAD_ARGUMENT;
    }

    rpsw_status_t st = stm8_flash_unlock_eeprom();
    if (st != RPSW_OK) {
        flash_diag_set("option:unlock_eeprom", address, st);
        return st;
    }

    st = set_flash_cr2(STM8_CR2_OPT);
    if (st != RPSW_OK) {
        (void)clear_flash_cr2();
        flash_diag_set("option:set_cr2_opt", address, st);
        return st;
    }

    st = write_u8(address, value);
    rpsw_status_t trigger_status = st;

    /*
     * Option-byte programming can make the target temporarily unavailable.
     * Treat NACK/timeout after the triggering write as a busy condition and
     * restore SWIM before polling FLASH_IAPSR.
     */
    if (st == RPSW_ERR_SWIM_TIMEOUT || st == RPSW_ERR_SWIM_NACK) {
        st = flash_busy_delay_and_resync(10000u);
        if (st != RPSW_OK) {
            snprintf(g_flash_last_error, sizeof(g_flash_last_error),
                     "option:post_trigger_resync_after_timeout addr=0x%06lx trigger=%s resync=%s",
                     (unsigned long)address, rpsw_status_text(trigger_status), rpsw_status_text(st));
        }
    } else if (st == RPSW_OK) {
        st = flash_busy_delay_and_resync(10000u);
        if (st != RPSW_OK) {
            snprintf(g_flash_last_error, sizeof(g_flash_last_error),
                     "option:post_trigger_resync addr=0x%06lx resync=%s",
                     (unsigned long)address, rpsw_status_text(st));
        }
    } else {
        flash_diag_set("option:trigger_write", address, st);
    }

    if (st != RPSW_OK) {
        (void)clear_flash_cr2();
        return st;
    }

    st = wait_flash_complete(0u);
    if (st != RPSW_OK) {
        (void)clear_flash_cr2();
        flash_diag_set("option:wait_complete", address, st);
        return st;
    }

    st = clear_flash_cr2();
    if (st != RPSW_OK) {
        flash_diag_set("option:clear_cr2", address, st);
        return st;
    }

    return RPSW_OK;
}


rpsw_status_t stm8_flash_erase_range(uint32_t address, uint32_t length) {
    flash_diag_clear();
    if (length == 0u) {
        return RPSW_OK;
    }

    rpsw_status_t st = require_program_unlocked();
    if (st != RPSW_OK) {
        flash_diag_set("erase:require_unlocked", address, st);
        return st;
    }

    uint32_t start = address & ~(uint32_t)(STM8_FLASH_BLOCK_SIZE - 1u);
    uint32_t end = address + length;
    if (end < address) {
        return RPSW_ERR_BAD_ARGUMENT;
    }
    end = (end + STM8_FLASH_BLOCK_SIZE - 1u) & ~(uint32_t)(STM8_FLASH_BLOCK_SIZE - 1u);

    uint8_t erase_block[STM8_FLASH_BLOCK_SIZE];
    for (unsigned i = 0; i < STM8_FLASH_BLOCK_SIZE; i++) {
        erase_block[i] = STM8_FLASH_ERASED_VALUE;
    }

    for (uint32_t block = start; block < end; block += STM8_FLASH_BLOCK_SIZE) {
        st = set_flash_cr2(STM8_CR2_ERASE);
        if (st != RPSW_OK) {
            (void)clear_flash_cr2();
            flash_diag_set("erase:set_cr2", block, st);
            return st;
        }

        /*
         * STM8 block erase is not a single-byte trigger. With ERASE/NERASE set,
         * the target expects the complete block erase sequence: one write to
         * each byte location in the 64-byte block. If fewer locations are sent,
         * the flash controller/debug interface can remain waiting for the rest
         * of the sequence and the next SWIM transaction may NACK/timeout.
         */
        st = stm8_dm_memory_write(block, erase_block, STM8_FLASH_BLOCK_SIZE);
        rpsw_status_t trigger_status = st;
        if (st == RPSW_ERR_SWIM_TIMEOUT || st == RPSW_ERR_SWIM_NACK) {
            st = flash_busy_delay_and_resync(30000u);
            if (st != RPSW_OK) {
                snprintf(g_flash_last_error, sizeof(g_flash_last_error),
                         "erase:post_trigger_resync_after_timeout addr=0x%06lx trigger=%s resync=%s",
                         (unsigned long)block, rpsw_status_text(trigger_status), rpsw_status_text(st));
            }
        } else if (st == RPSW_OK) {
            st = flash_busy_delay_and_resync(30000u);
            if (st != RPSW_OK) {
                snprintf(g_flash_last_error, sizeof(g_flash_last_error),
                         "erase:post_trigger_resync addr=0x%06lx resync=%s",
                         (unsigned long)block, rpsw_status_text(st));
            }
        } else {
            flash_diag_set("erase:trigger_write", block, st);
        }
        if (st != RPSW_OK) {
            (void)clear_flash_cr2();
            return st;
        }

        st = wait_flash_complete(0u);
        if (st != RPSW_OK) {
            (void)clear_flash_cr2();
            flash_diag_set("erase:wait_complete", block, st);
            return st;
        }

        /*
         * Return CR2/NCR2 to normal mode after the block operation has completed.
         * If this ever NACKs again, the diagnostic will distinguish it from the
         * actual block sequence above.
         */
        st = clear_flash_cr2();
        if (st != RPSW_OK) {
            flash_diag_set("erase:clear_cr2", block, st);
            return st;
        }
    }

    return RPSW_OK;
}

rpsw_status_t stm8_flash_write_block(uint32_t address, const uint8_t *data, size_t len) {
    flash_diag_clear();
    if (data == NULL && len != 0u) {
        return RPSW_ERR_BAD_ARGUMENT;
    }
    if (len == 0u) {
        return RPSW_OK;
    }

    rpsw_status_t st = require_program_unlocked();
    if (st != RPSW_OK) {
        flash_diag_set("erase:require_unlocked", address, st);
        return st;
    }

    /*
     * Use conservative byte programming for the MVP. Do not set PRG/FPRG here:
     * those bits are for block programming modes and require a different write
     * sequence. Byte programming only needs PUL plus a direct byte write.
     */
    st = clear_flash_cr2();
    if (st != RPSW_OK) {
        flash_diag_set("write:clear_cr2", address, st);
        return st;
    }

    for (size_t i = 0; i < len; i++) {
        if (data[i] == STM8_FLASH_ERASED_VALUE) {
            continue;
        }

        st = write_u8(address + (uint32_t)i, data[i]);
        rpsw_status_t trigger_status = st;
        if (st == RPSW_ERR_SWIM_TIMEOUT || st == RPSW_ERR_SWIM_NACK) {
            st = flash_busy_delay_and_resync(5000u);
            if (st != RPSW_OK) {
                snprintf(g_flash_last_error, sizeof(g_flash_last_error),
                         "write:post_trigger_resync_after_timeout addr=0x%06lx trigger=%s resync=%s",
                         (unsigned long)(address + (uint32_t)i), rpsw_status_text(trigger_status), rpsw_status_text(st));
            }
        } else if (st == RPSW_OK) {
            st = flash_busy_delay_and_resync(5000u);
            if (st != RPSW_OK) {
                snprintf(g_flash_last_error, sizeof(g_flash_last_error),
                         "write:post_trigger_resync addr=0x%06lx resync=%s",
                         (unsigned long)(address + (uint32_t)i), rpsw_status_text(st));
            }
        } else {
            flash_diag_set("write:trigger_write", address + (uint32_t)i, st);
        }
        if (st != RPSW_OK) {
            (void)clear_flash_cr2();
            return st;
        }
        st = wait_flash_complete(0u);
        if (st != RPSW_OK) {
            (void)clear_flash_cr2();
            flash_diag_set("write:wait_complete", address + (uint32_t)i, st);
            return st;
        }
    }

    return clear_flash_cr2();
}
