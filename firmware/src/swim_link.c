#include "swim_link.h"

#include "hardware/sync.h"
#include "swim_phy.h"
#include "swim_bits.h"
#include "pico/stdlib.h"

#define SWIM_CMD_SRST 0x0u
#define SWIM_CMD_ROTF 0x1u
#define SWIM_CMD_WOTF 0x2u
#define SWIM_ACK_TIMEOUT_US 2000u
#define SWIM_READ_TIMEOUT_US 5000u
#define SWIM_SYNC_TIMEOUT_US 20000u
#define SWIM_CSR_ADDR 0x007F80u
#define DM_CSR2_ADDR 0x007F99u
#define DM_CSR2_STALL 0x08u
#define SWIM_CSR_SAFE_MASK 0x80u
#define SWIM_CSR_NO_ACCESS 0x40u
#define SWIM_CSR_SWIM_DM 0x20u
#define SWIM_CSR_HS 0x10u
#define SWIM_CSR_HSIT 0x02u
#define SWIM_CSR_WRITABLE_MASK 0xbdU
#define SWIM_CSR_INIT_VALUE (SWIM_CSR_SAFE_MASK | SWIM_CSR_SWIM_DM)

static bool parity3(uint8_t v) {
    return ((v ^ (v >> 1) ^ (v >> 2)) & 1u) != 0u;
}

static rpsw_status_t read_ack(void) {
    bool ok = false;
    bool ack = swim_phy_read_bit(SWIM_ACK_TIMEOUT_US, &ok);
    if (!ok) {
        return RPSW_ERR_SWIM_TIMEOUT;
    }
    return ack ? RPSW_OK : RPSW_ERR_SWIM_NACK;
}

static const char *command_name(uint8_t command) {
    switch (command) {
    case SWIM_CMD_SRST: return "SRST";
    case SWIM_CMD_ROTF: return "ROTF";
    case SWIM_CMD_WOTF: return "WOTF";
    default: return "CMD";
    }
}

static rpsw_status_t send_command(uint8_t command) {
    if (!swim_phy_timing_ready()) {
        return RPSW_ERR_SWIM_TIMEOUT;
    }
    swim_phy_set_tx_context(command_name(command));
    uint32_t frame = ((uint32_t)(command & 0x7u) << 1u) | (parity3(command) ? 1u : 0u);
    bool ack = false;
    if (!swim_phy_write_frame_bits_read_ack(frame, 5, SWIM_ACK_TIMEOUT_US, &ack)) {
        return RPSW_ERR_SWIM_TIMEOUT;
    }
    return ack ? RPSW_OK : RPSW_ERR_SWIM_NACK;
}

static rpsw_status_t send_byte_labeled(uint8_t byte, const char *label) {
    if (!swim_phy_timing_ready()) {
        return RPSW_ERR_SWIM_TIMEOUT;
    }
    swim_phy_set_tx_context(label);
    uint32_t frame = ((uint32_t)byte << 1u) | (swim_parity8(byte) ? 1u : 0u);
    bool ack = false;
    if (!swim_phy_write_frame_bits_read_ack(frame, 10, SWIM_ACK_TIMEOUT_US, &ack)) {
        return RPSW_ERR_SWIM_TIMEOUT;
    }
    return ack ? RPSW_OK : RPSW_ERR_SWIM_NACK;
}

static rpsw_status_t decode_data_frame_no_ack(uint32_t frame, uint8_t *byte) {
    bool header = ((frame >> 9u) & 1u) != 0u;
    if (!header) {
        return RPSW_ERR_TARGET;
    }

    uint8_t value = (uint8_t)((frame >> 1u) & 0xffu);
    bool parity = (frame & 1u) != 0u;
    if (parity != swim_parity8(value)) {
        return RPSW_ERR_TARGET;
    }

    *byte = value;
    return RPSW_OK;
}

static rpsw_status_t send_target_frame_ack(bool ack) {
    if (!swim_phy_write_bit(ack)) {
        return RPSW_ERR_SWIM_TIMEOUT;
    }
    return RPSW_OK;
}

static rpsw_status_t recv_byte_after_ack(uint8_t *byte) {
    uint32_t frame = 0;
    if (!swim_phy_write_bit_read_target_frame(true, SWIM_READ_TIMEOUT_US, &frame)) {
        return RPSW_ERR_SWIM_TIMEOUT;
    }

    rpsw_status_t st = decode_data_frame_no_ack(frame, byte);
    if (st != RPSW_OK) {
        (void)send_target_frame_ack(false);
    }
    return st;
}

static rpsw_status_t send_byte_read_ack_frame_labeled(uint8_t byte, uint8_t *target_byte,
                                                      const char *label) {
    if (!swim_phy_timing_ready()) {
        return RPSW_ERR_SWIM_TIMEOUT;
    }

    swim_phy_set_tx_context(label);

    uint32_t tx_frame = ((uint32_t)byte << 1u) | (swim_parity8(byte) ? 1u : 0u);
    uint32_t rx_frame = 0;

    if (!swim_phy_write_frame_bits_read_ack_and_frame(tx_frame, 10u,
        SWIM_READ_TIMEOUT_US,
        &rx_frame)) {
        return RPSW_ERR_SWIM_TIMEOUT;
    }

    rpsw_status_t st = decode_data_frame_no_ack(rx_frame, target_byte);
    if (st != RPSW_OK) {
        (void)send_target_frame_ack(false);
    }
    return st;
}

static rpsw_status_t swim_link_stall_target(void) {
    for (unsigned attempt = 0; attempt < 3; ++attempt) {
        uint8_t csr2 = 0;

        if (attempt != 0) {
            sleep_us(500);
        }

        rpsw_status_t st = swim_link_read(DM_CSR2_ADDR, &csr2, 1u);
        if (st != RPSW_OK) {
            continue;
        }

        csr2 |= DM_CSR2_STALL;
        st = swim_link_write(DM_CSR2_ADDR, &csr2, 1u);
        if (st == RPSW_OK) {
            return RPSW_OK;
        }
    }

    return RPSW_ERR_SWIM_TIMEOUT;
}

static rpsw_status_t swim_link_handle_swim_csr(swim_csr_action_t action) {
    if (action == SWIM_CSR_WRITE_INIT_AND_VERIFY) {
        uint8_t csr = SWIM_CSR_INIT_VALUE;

        swim_phy_set_enter_stage(SWIM_ENTER_STAGE_SWIM_CSR_WRITE_START);

        rpsw_status_t st = swim_link_write(SWIM_CSR_ADDR, &csr, 1u);
        if (st != RPSW_OK) {
            return st;
        }

        swim_phy_set_enter_stage(SWIM_ENTER_STAGE_SWIM_CSR_WRITE_OK);
    }

    uint8_t csr_readback = 0;

    swim_phy_set_enter_stage(SWIM_ENTER_STAGE_SWIM_CSR_READ_START);

    rpsw_status_t st = swim_link_read(SWIM_CSR_ADDR, &csr_readback, 1u);
    if (st != RPSW_OK) {
        swim_phy_set_swim_csr_debug(0u, false);
        return st;
    }

    swim_phy_set_swim_csr_debug(csr_readback, true);
    swim_phy_set_enter_stage(SWIM_ENTER_STAGE_SWIM_CSR_READ_OK);

    if ((csr_readback & SWIM_CSR_SWIM_DM) == 0u) {
        return RPSW_ERR_TARGET;
    }

    return RPSW_OK;
}

static rpsw_status_t swim_link_release_reset_and_resync(void) {
    swim_phy_nrst_release();
    sleep_ms(1);

    swim_phy_set_enter_stage(SWIM_ENTER_STAGE_COMM_RESET_SENT);

    if (!swim_phy_comm_reset_wait_sync(SWIM_SYNC_TIMEOUT_US)) {
        return RPSW_ERR_SWIM_TIMEOUT;
    }

    swim_phy_mark_second_sync_seen();
    swim_phy_set_enter_stage(SWIM_ENTER_STAGE_SYNC2_OK);
    return RPSW_OK;
}

rpsw_status_t swim_link_enter(void) {
    rpsw_status_t st = RPSW_OK;

    swim_phy_reset_timing();
    if (!swim_phy_set_speed(SWIM_SPEED_LOW)) {
        swim_phy_mark_enter_fail();
        return RPSW_ERR_UNSUPPORTED;
    }

    swim_phy_release();
    swim_phy_nrst_assert();
    swim_phy_set_enter_stage(SWIM_ENTER_STAGE_RESET_ASSERTED);
    sleep_ms(2);

    if (!swim_phy_entry_sequence_um0470_wait_sync(SWIM_SYNC_TIMEOUT_US)) {
        swim_phy_mark_enter_fail();
        swim_phy_release();
        swim_phy_nrst_release();
        return RPSW_ERR_SWIM_TIMEOUT;
    }
    swim_phy_set_enter_stage(SWIM_ENTER_STAGE_ENTRY_SENT);
    swim_phy_set_enter_stage(SWIM_ENTER_STAGE_SYNC1_OK);

    /*
     * UM0470 sequence:
     * - after entry sync, write 0xA0 to SWIM_CSR while reset is still held;
     * - release reset and wait 1 ms for option-byte loading / stabilization;
     * - then issue communication reset to get a more reliable calibrated sync.
     */
    st = swim_link_handle_swim_csr(SWIM_CSR_WRITE_INIT_AND_VERIFY);
    if (st != RPSW_OK) {
        goto fail;
    }

    st = swim_link_release_reset_and_resync();
    if (st != RPSW_OK) {
        goto fail;
    }

    /*
     * Do one post-stabilization DM access before reporting ENTER_SWIM as OK.
     * Without this, the first external MEMORY_READ/MEMORY_WRITE sometimes becomes
     * the sacrificial transaction and times out.
     */
    st = swim_link_handle_swim_csr(SWIM_CSR_VERIFY_ONLY);
    if (st != RPSW_OK) {
        goto fail;
    }

    /*
     * Make ENTER_SWIM complete only after the debug module accepts STALL.
     * Otherwise the first host-side DM_CSR2 write becomes the sacrificial
     * transaction and sometimes times out.
     */
    st = swim_link_stall_target();
    if (st != RPSW_OK) {
        goto fail;
    }

    swim_phy_set_enter_stage(SWIM_ENTER_STAGE_DONE);
    return RPSW_OK;

fail:
    swim_phy_mark_enter_fail();
    swim_phy_release();
    swim_phy_nrst_release();
    return st;
}

rpsw_status_t swim_link_set_speed(swim_speed_t speed) {
	if (speed == SWIM_SPEED_LOW) {
		return swim_phy_set_speed(SWIM_SPEED_LOW) ? RPSW_OK : RPSW_ERR_UNSUPPORTED;
	}
	if (speed != SWIM_SPEED_HIGH) {
		return RPSW_ERR_BAD_ARGUMENT;
	}

	/*
	 * UM0470: SWIM is activated in low-speed.  High-speed becomes valid only
	 * after option-byte loading has finished, indicated by SWIM_CSR.HSIT.  Do
	 * the SWIM_CSR write while the PHY is still low-speed; switch the local
	 * encoder/decoder only after that write is ACKed.
	 */
	if (!swim_phy_set_speed(SWIM_SPEED_LOW)) {
		return RPSW_ERR_UNSUPPORTED;
	}

	uint8_t csr = 0;
	rpsw_status_t st = swim_link_read(SWIM_CSR_ADDR, &csr, 1u);
	if (st != RPSW_OK) {
		return st;
	}
	if ((csr & SWIM_CSR_HSIT) == 0u) {
		swim_phy_set_swim_csr_debug(csr, true);
		return RPSW_ERR_UNSUPPORTED;
	}

	uint8_t hs_csr = (uint8_t)((csr |
                            SWIM_CSR_SAFE_MASK |
                            SWIM_CSR_SWIM_DM |
                            SWIM_CSR_HS) &
                           (uint8_t)~SWIM_CSR_NO_ACCESS);
	st = swim_link_write(SWIM_CSR_ADDR, &hs_csr, 1u);
	if (st != RPSW_OK) {
		return st;
	}
	sleep_us(20);
	swim_phy_set_swim_csr_debug(hs_csr, true);
	return swim_phy_set_speed(SWIM_SPEED_HIGH) ? RPSW_OK : RPSW_ERR_UNSUPPORTED;
}

rpsw_status_t swim_link_srst(void) {
    uint32_t irq_state = save_and_disable_interrupts();
    rpsw_status_t st = send_command(SWIM_CMD_SRST);
    restore_interrupts(irq_state);
    return st;
}

rpsw_status_t swim_link_read(uint32_t address, uint8_t *data, size_t len) {
    if (len == 0 || len > 255u) {
        return RPSW_ERR_BAD_ARGUMENT;
    }
    uint32_t irq_state = save_and_disable_interrupts();
    rpsw_status_t st = send_command(SWIM_CMD_ROTF);
    if (st != RPSW_OK) goto out;
    st = send_byte_labeled((uint8_t)len, "ROTF len");
    if (st != RPSW_OK) goto out;
    st = send_byte_labeled((uint8_t)((address >> 16) & 0xffu), "ROTF AE");
    if (st != RPSW_OK) goto out;
    st = send_byte_labeled((uint8_t)((address >> 8) & 0xffu), "ROTF AH");
    if (st != RPSW_OK) goto out;

    st = send_byte_read_ack_frame_labeled((uint8_t)(address & 0xffu), &data[0], "ROTF AL");
    if (st != RPSW_OK) goto out;

    /*
     * The first target byte arrives immediately after the target ACKs AL. For
     * multi-byte ROTF, each following byte is released by a host ACK to the
     * previous data frame. That ACK must be a single immediate SWIM bit; using
     * a normal TX frame with an interframe gap can make real STM8S targets stop
     * after the first byte.
     */
    for (size_t i = 1; i < len; i++) {
        st = recv_byte_after_ack(&data[i]);
        if (st != RPSW_OK) goto out;
    }

    st = send_target_frame_ack(true);
    if (st != RPSW_OK) goto out;
out:
    restore_interrupts(irq_state);
    return st;
}

rpsw_status_t swim_link_write(uint32_t address, const uint8_t *data, size_t len) {
    if (len == 0 || len > 255u) {
        return RPSW_ERR_BAD_ARGUMENT;
    }
    uint32_t irq_state = save_and_disable_interrupts();
    rpsw_status_t st = send_command(SWIM_CMD_WOTF);
    if (st != RPSW_OK) goto out;
    st = send_byte_labeled((uint8_t)len, "WOTF len");
    if (st != RPSW_OK) goto out;
    st = send_byte_labeled((uint8_t)((address >> 16) & 0xffu), "WOTF AE");
    if (st != RPSW_OK) goto out;
    st = send_byte_labeled((uint8_t)((address >> 8) & 0xffu), "WOTF AH");
    if (st != RPSW_OK) goto out;
    st = send_byte_labeled((uint8_t)(address & 0xffu), "WOTF AL");
    if (st != RPSW_OK) goto out;

    for (size_t i = 0; i < len; i++) {
        st = send_byte_labeled(data[i], "WOTF data");
        if (st != RPSW_OK) goto out;
    }
out:
    restore_interrupts(irq_state);
    return st;
}
