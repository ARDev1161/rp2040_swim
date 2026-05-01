#include "swim_link.h"

#include "hardware/sync.h"
#include "swim_phy.h"
#include "pico/stdlib.h"

#define SWIM_CMD_SRST 0x0u
#define SWIM_CMD_ROTF 0x1u
#define SWIM_CMD_WOTF 0x2u
#define SWIM_ACK_TIMEOUT_US 2000u
#define SWIM_READ_TIMEOUT_US 5000u
#define SWIM_SYNC_TIMEOUT_US 20000u
#define SWIM_CSR_ADDR 0x007F80u
#define SWIM_CSR_SAFE_MASK 0x80u
#define SWIM_CSR_SWIM_DM 0x20u
#define SWIM_CSR_INIT_VALUE (SWIM_CSR_SAFE_MASK | SWIM_CSR_SWIM_DM)

static bool parity3(uint8_t v) {
    return ((v ^ (v >> 1) ^ (v >> 2)) & 1u) != 0u;
}

static bool parity8(uint8_t v) {
    v ^= (uint8_t)(v >> 4);
    v ^= (uint8_t)(v >> 2);
    v ^= (uint8_t)(v >> 1);
    return (v & 1u) != 0u;
}

static rpsw_status_t read_ack(void) {
    bool ok = false;
    bool ack = swim_phy_read_bit(SWIM_ACK_TIMEOUT_US, &ok);
    if (!ok) {
        return RPSW_ERR_SWIM_TIMEOUT;
    }
    return ack ? RPSW_OK : RPSW_ERR_SWIM_NACK;
}

static rpsw_status_t send_command(uint8_t command) {
    if (!swim_phy_timing_ready()) {
        return RPSW_ERR_SWIM_TIMEOUT;
    }
    uint32_t frame = ((uint32_t)(command & 0x7u) << 1u) | (parity3(command) ? 1u : 0u);
    bool ack = false;
    if (!swim_phy_write_frame_bits_read_ack(frame, 5, SWIM_ACK_TIMEOUT_US, &ack)) {
        return RPSW_ERR_SWIM_TIMEOUT;
    }
    return ack ? RPSW_OK : RPSW_ERR_SWIM_NACK;
}

static rpsw_status_t send_byte(uint8_t byte) {
    if (!swim_phy_timing_ready()) {
        return RPSW_ERR_SWIM_TIMEOUT;
    }
    uint32_t frame = ((uint32_t)byte << 1u) | (parity8(byte) ? 1u : 0u);
    bool ack = false;
    if (!swim_phy_write_frame_bits_read_ack(frame, 10, SWIM_ACK_TIMEOUT_US, &ack)) {
        return RPSW_ERR_SWIM_TIMEOUT;
    }
    return ack ? RPSW_OK : RPSW_ERR_SWIM_NACK;
}

static rpsw_status_t recv_byte(uint8_t *byte) {
    bool ok = false;
    bool header = swim_phy_read_bit(SWIM_READ_TIMEOUT_US, &ok);
    if (!ok) {
        return RPSW_ERR_SWIM_TIMEOUT;
    }
    if (!header) {
        return RPSW_ERR_TARGET;
    }

    uint8_t value = 0;
    for (unsigned bit = 0; bit < 8; bit++) {
        bool b = swim_phy_read_bit(SWIM_READ_TIMEOUT_US, &ok);
        if (!ok) {
            return RPSW_ERR_SWIM_TIMEOUT;
        }
        value = (uint8_t)((value << 1) | (b ? 1u : 0u));
    }

    bool parity = swim_phy_read_bit(SWIM_READ_TIMEOUT_US, &ok);
    if (!ok) {
        return RPSW_ERR_SWIM_TIMEOUT;
    }
    if (parity != parity8(value)) {
        (void)swim_phy_write_frame_bits(0u, 1);
        return RPSW_ERR_TARGET;
    }

    if (!swim_phy_write_frame_bits(1u, 1)) {
        return RPSW_ERR_SWIM_TIMEOUT;
    }
    *byte = value;
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

    swim_phy_set_enter_stage(SWIM_ENTER_STAGE_COMM_RESET_SENT);
    if (!swim_phy_comm_reset_wait_sync(SWIM_SYNC_TIMEOUT_US)) {
        st = RPSW_ERR_SWIM_TIMEOUT;
        goto fail;
    }
    swim_phy_mark_second_sync_seen();
    swim_phy_set_enter_stage(SWIM_ENTER_STAGE_SYNC2_OK);

    uint8_t csr = SWIM_CSR_INIT_VALUE;

    swim_phy_set_enter_stage(SWIM_ENTER_STAGE_SWIM_CSR_WRITE_START);
    st = swim_link_write(SWIM_CSR_ADDR, &csr, 1);
    if (st != RPSW_OK) {
        goto fail;
    }
    swim_phy_set_enter_stage(SWIM_ENTER_STAGE_SWIM_CSR_WRITE_OK);

    uint8_t csr_readback = 0;

    swim_phy_set_enter_stage(SWIM_ENTER_STAGE_SWIM_CSR_READ_START);
    st = swim_link_read(SWIM_CSR_ADDR, &csr_readback, 1);
    if (st != RPSW_OK) {
        swim_phy_set_swim_csr_debug(0, false);
        goto fail;
    }

    swim_phy_set_swim_csr_debug(csr_readback, true);
    swim_phy_set_enter_stage(SWIM_ENTER_STAGE_SWIM_CSR_READ_OK);

    if ((csr_readback & SWIM_CSR_SWIM_DM) == 0u) {
        st = RPSW_ERR_TARGET;
        goto fail;
    }

    swim_phy_nrst_release();
    sleep_ms(1);

    swim_phy_set_enter_stage(SWIM_ENTER_STAGE_DONE);
    return RPSW_OK;

fail:
    swim_phy_mark_enter_fail();
    swim_phy_release();
    swim_phy_nrst_release();
    return st;
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
    st = send_byte((uint8_t)len);
    if (st != RPSW_OK) goto out;
    st = send_byte((uint8_t)((address >> 16) & 0xffu));
    if (st != RPSW_OK) goto out;
    st = send_byte((uint8_t)((address >> 8) & 0xffu));
    if (st != RPSW_OK) goto out;
    st = send_byte((uint8_t)(address & 0xffu));
    if (st != RPSW_OK) goto out;

    for (size_t i = 0; i < len; i++) {
        st = recv_byte(&data[i]);
        if (st != RPSW_OK) goto out;
    }
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
    st = send_byte((uint8_t)len);
    if (st != RPSW_OK) goto out;
    st = send_byte((uint8_t)((address >> 16) & 0xffu));
    if (st != RPSW_OK) goto out;
    st = send_byte((uint8_t)((address >> 8) & 0xffu));
    if (st != RPSW_OK) goto out;
    st = send_byte((uint8_t)(address & 0xffu));
    if (st != RPSW_OK) goto out;

    for (size_t i = 0; i < len; i++) {
        st = send_byte(data[i]);
        if (st != RPSW_OK) goto out;
    }
out:
    restore_interrupts(irq_state);
    return st;
}
