#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "hardware/pio.h"
#include "pico/types.h"
#include "usb_protocol.h"

#define SWIM_PIO_RX_ACK_FRAME_WIDTHS 11u

typedef struct {
    uint32_t low_ticks;
    uint32_t low_ns;
    uint32_t low_us;
    uint32_t loops_used;
    bool timeout;
} swim_pio_rx_width_t;

rpsw_status_t swim_pio_rx_init(PIO pio, uint swim_pin);
rpsw_status_t swim_pio_rx_arm_after_tx_done(uint32_t max_loop_count);

rpsw_status_t swim_pio_rx_arm_now(uint32_t max_loop_count);
rpsw_status_t swim_pio_rx_arm_width_burst_after_tx_done(uint32_t count, uint32_t max_loop_count);
rpsw_status_t swim_pio_rx_arm_decode_bits_after_tx_done(uint32_t bit_count,
                                                        uint32_t threshold_loop_count);
rpsw_status_t swim_pio_rx_arm_decode_bits_now(uint32_t bit_count,
                                              uint32_t threshold_loop_count);
rpsw_status_t swim_pio_rx_get_decoded_bits(uint32_t timeout_us, uint32_t bit_count, uint32_t *bits);

rpsw_status_t swim_pio_rx_get_width(swim_pio_rx_width_t *out, uint32_t timeout_us);
rpsw_status_t swim_pio_rx_get_width_burst(swim_pio_rx_width_t *out,
                                          uint32_t count,
                                          uint32_t timeout_us);
rpsw_status_t swim_pio_rx_get_width_burst_partial(swim_pio_rx_width_t *out,
                                                  uint32_t max_count,
                                                  uint32_t timeout_us,
                                                  uint32_t *captured_count);

uint32_t swim_pio_rx_ns_to_max_loops(uint32_t max_ns);
uint32_t swim_pio_rx_loops_to_ns(uint32_t loops);

uint32_t swim_pio_rx_burst_requested_count(void);
uint32_t swim_pio_rx_burst_captured_count(void);
uint32_t swim_pio_rx_burst_timeout_index(void);
uint32_t swim_pio_rx_burst_first_low_us(uint32_t index);
uint32_t swim_pio_rx_burst_first_low_ns(uint32_t index);
