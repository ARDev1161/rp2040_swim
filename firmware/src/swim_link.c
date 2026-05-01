#include "swim_link.h"

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
    if (!swim_phy_write_bit(false) ||
        !swim_phy_write_bit((command & 0x4u) != 0u) ||
        !swim_phy_write_bit((command & 0x2u) != 0u) ||
        !swim_phy_write_bit((command & 0x1u) != 0u) ||
        !swim_phy_write_bit(parity3(command))) {
        return RPSW_ERR_SWIM_TIMEOUT;
    }
    swim_phy_release();
    return read_ack();
}

static rpsw_status_t send_byte(uint8_t byte) {
    if (!swim_phy_timing_ready()) {
        return RPSW_ERR_SWIM_TIMEOUT;
    }
    if (!swim_phy_write_bit(false)) {
        return RPSW_ERR_SWIM_TIMEOUT;
    }
    for (int bit = 7; bit >= 0; bit--) {
        if (!swim_phy_write_bit(((byte >> bit) & 1u) != 0u)) {
            return RPSW_ERR_SWIM_TIMEOUT;
        }
    }
    if (!swim_phy_write_bit(parity8(byte))) {
        return RPSW_ERR_SWIM_TIMEOUT;
    }
    swim_phy_release();
    return read_ack();
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
        (void)swim_phy_write_bit(false);
        return RPSW_ERR_TARGET;
    }

    if (!swim_phy_write_bit(true)) {
        return RPSW_ERR_SWIM_TIMEOUT;
    }
    *byte = value;
    return RPSW_OK;
}

rpsw_status_t swim_link_enter(void) {
    swim_phy_reset_timing();
    if (!swim_phy_set_speed(SWIM_SPEED_LOW)) {
        return RPSW_ERR_UNSUPPORTED;
    }

    swim_phy_release();
    swim_phy_nrst_assert();
    sleep_ms(2);

    rpsw_status_t st = swim_phy_entry_sequence_um0470();
    if (st != RPSW_OK) {
        swim_phy_release();
        swim_phy_nrst_release();
        return st;
    }
    if (!swim_phy_wait_sync(SWIM_SYNC_TIMEOUT_US)) {
        swim_phy_release();
        swim_phy_nrst_release();
        return RPSW_ERR_SWIM_TIMEOUT;
    }

    swim_phy_comm_reset();
    if (!swim_phy_wait_sync(SWIM_SYNC_TIMEOUT_US)) {
        swim_phy_release();
        swim_phy_nrst_release();
        return RPSW_ERR_SWIM_TIMEOUT;
    }

    uint8_t csr = SWIM_CSR_INIT_VALUE;
    st = swim_link_write(SWIM_CSR_ADDR, &csr, 1);
    if (st != RPSW_OK) {
        swim_phy_release();
        swim_phy_nrst_release();
        return st;
    }

    uint8_t csr_readback = 0;
    st = swim_link_read(SWIM_CSR_ADDR, &csr_readback, 1);
    if (st != RPSW_OK) {
        swim_phy_set_swim_csr_debug(0, false);
        swim_phy_release();
        swim_phy_nrst_release();
        return st;
    }
    swim_phy_set_swim_csr_debug(csr_readback, true);
    if ((csr_readback & SWIM_CSR_SWIM_DM) == 0u) {
        swim_phy_release();
        swim_phy_nrst_release();
        return RPSW_ERR_TARGET;
    }

    swim_phy_nrst_release();
    sleep_ms(1);
    return RPSW_OK;
}

rpsw_status_t swim_link_srst(void) {
    return send_command(SWIM_CMD_SRST);
}

rpsw_status_t swim_link_read(uint32_t address, uint8_t *data, size_t len) {
    if (len == 0 || len > 255u) {
        return RPSW_ERR_BAD_ARGUMENT;
    }
    rpsw_status_t st = send_command(SWIM_CMD_ROTF);
    if (st != RPSW_OK) return st;
    st = send_byte((uint8_t)len);
    if (st != RPSW_OK) return st;
    st = send_byte((uint8_t)((address >> 16) & 0xffu));
    if (st != RPSW_OK) return st;
    st = send_byte((uint8_t)((address >> 8) & 0xffu));
    if (st != RPSW_OK) return st;
    st = send_byte((uint8_t)(address & 0xffu));
    if (st != RPSW_OK) return st;

    for (size_t i = 0; i < len; i++) {
        st = recv_byte(&data[i]);
        if (st != RPSW_OK) return st;
    }
    return RPSW_OK;
}

rpsw_status_t swim_link_write(uint32_t address, const uint8_t *data, size_t len) {
    if (len == 0 || len > 255u) {
        return RPSW_ERR_BAD_ARGUMENT;
    }
    rpsw_status_t st = send_command(SWIM_CMD_WOTF);
    if (st != RPSW_OK) return st;
    st = send_byte((uint8_t)len);
    if (st != RPSW_OK) return st;
    st = send_byte((uint8_t)((address >> 16) & 0xffu));
    if (st != RPSW_OK) return st;
    st = send_byte((uint8_t)((address >> 8) & 0xffu));
    if (st != RPSW_OK) return st;
    st = send_byte((uint8_t)(address & 0xffu));
    if (st != RPSW_OK) return st;

    for (size_t i = 0; i < len; i++) {
        st = send_byte(data[i]);
        if (st != RPSW_OK) return st;
    }
    return RPSW_OK;
}
