#ifndef SWIM_PHY_H
#define SWIM_PHY_H

#include <stdbool.h>
#include <stdint.h>
#include "pico/types.h"
#include "usb_protocol.h"

typedef enum {
    SWIM_SPEED_LOW = 0,
    SWIM_SPEED_HIGH = 1,
} swim_speed_t;

typedef struct {
    uint swim_pin;
    uint nrst_pin;
    bool internal_pullup;
    bool active_drive_high;
    swim_speed_t speed;
} swim_phy_config_t;

typedef enum {
    SWIM_ENTER_STAGE_IDLE = 0,
    SWIM_ENTER_STAGE_RESET_ASSERTED,
    SWIM_ENTER_STAGE_ENTRY_SENT,
    SWIM_ENTER_STAGE_SYNC1_OK,
    SWIM_ENTER_STAGE_COMM_RESET_SENT,
    SWIM_ENTER_STAGE_SYNC2_OK,
    SWIM_ENTER_STAGE_SWIM_CSR_WRITE_START,
    SWIM_ENTER_STAGE_SWIM_CSR_WRITE_OK,
    SWIM_ENTER_STAGE_SWIM_CSR_READ_START,
    SWIM_ENTER_STAGE_SWIM_CSR_READ_OK,
    SWIM_ENTER_STAGE_DONE,
    SWIM_ENTER_STAGE_FAIL,
} swim_enter_stage_t;

typedef struct {
    bool synced;
    swim_speed_t speed;
    uint32_t last_sync_low_us;
    uint32_t last_sync_low_ns;
    uint32_t derived_tswim_ns;
    uint32_t sync_low_loop_count;

    swim_enter_stage_t enter_stage;
    bool comm_reset_sent;
    bool second_sync_seen;
    uint32_t comm_reset_low_us;
    uint32_t comm_reset_low_ns;

    uint8_t swim_csr;
    bool swim_csr_valid;
    uint8_t phy_backend;
    uint32_t entry_protocol_us;
    uint8_t entry_slow_pulses;
    uint8_t entry_fast_pulses;
    bool pio_init_ok;
    char pio_error[64];
} swim_phy_debug_t;

void swim_phy_init(const swim_phy_config_t *config);
void swim_phy_release_target(void);
void swim_phy_set_pins(uint swim_pin, uint nrst_pin, bool internal_pullup);
bool swim_phy_set_speed(swim_speed_t speed);
void swim_phy_release(void);
void swim_phy_drive_low(void);
bool swim_phy_sample(void);
void swim_phy_nrst_assert(void);
void swim_phy_nrst_release(void);
void swim_phy_delay_us(uint32_t us);
rpsw_status_t swim_phy_entry_sequence_um0470(void);
bool swim_phy_entry_sequence_um0470_wait_sync(uint32_t timeout_us);
void swim_phy_entry_waveform(void);
void swim_phy_comm_reset(void);
bool swim_phy_comm_reset_wait_sync(uint32_t timeout_us);
bool swim_phy_write_bit(bool bit);
bool swim_phy_write_frame_bits(uint32_t bits_msb_first, unsigned bit_count);
bool swim_phy_write_frame_bits_read_ack(uint32_t bits_msb_first, unsigned bit_count, uint32_t timeout_us,
                                        bool *ack);
bool swim_phy_write_frame_bits_read_ack_and_frame(uint32_t bits, uint bit_count,
                                                  uint32_t timeout_us, uint32_t *rx_frame);
bool swim_phy_write_bit_read_target_frame(bool ack, uint32_t timeout_us, uint32_t *rx_frame);
bool swim_phy_read_target_frame(uint32_t timeout_us, uint32_t *rx_frame);
bool swim_phy_read_frame_bits(uint32_t timeout_us, uint32_t *bits);
bool swim_phy_read_bit(uint32_t timeout_us, bool *ok);
bool swim_phy_wait_sync(uint32_t timeout_us);
bool swim_phy_timing_ready(void);
void swim_phy_reset_timing(void);
void swim_phy_set_swim_csr_debug(uint8_t value, bool valid);
swim_phy_debug_t swim_phy_get_debug(void);
void swim_phy_debug_waveform(void);

void swim_phy_set_enter_stage(swim_enter_stage_t stage);
void swim_phy_mark_enter_fail(void);
void swim_phy_mark_second_sync_seen(void);
void swim_phy_set_tx_context(const char *context);
#endif
