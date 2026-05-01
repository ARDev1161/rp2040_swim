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

typedef struct {
    bool synced;
    swim_speed_t speed;
    uint32_t last_sync_low_us;
    uint32_t last_sync_low_ns;
    uint32_t derived_tswim_ns;
    uint32_t sync_low_loop_count;
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
void swim_phy_set_pins(uint swim_pin, uint nrst_pin, bool internal_pullup);
bool swim_phy_set_speed(swim_speed_t speed);
void swim_phy_release(void);
void swim_phy_drive_low(void);
bool swim_phy_sample(void);
void swim_phy_nrst_assert(void);
void swim_phy_nrst_release(void);
void swim_phy_delay_us(uint32_t us);
rpsw_status_t swim_phy_entry_sequence_um0470(void);
void swim_phy_entry_waveform(void);
void swim_phy_comm_reset(void);
bool swim_phy_write_bit(bool bit);
bool swim_phy_read_bit(uint32_t timeout_us, bool *ok);
bool swim_phy_wait_sync(uint32_t timeout_us);
bool swim_phy_timing_ready(void);
void swim_phy_reset_timing(void);
void swim_phy_set_swim_csr_debug(uint8_t value, bool valid);
swim_phy_debug_t swim_phy_get_debug(void);
void swim_phy_debug_waveform(void);

#endif
