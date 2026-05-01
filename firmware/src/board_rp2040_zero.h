#ifndef BOARD_RP2040_ZERO_H
#define BOARD_RP2040_ZERO_H

#include <stdint.h>

#define RP2040_SWIM_DEFAULT_SWIM_PIN 2u
#define RP2040_SWIM_DEFAULT_NRST_PIN 3u
#define RP2040_SWIM_UART_TX_PIN 4u
#define RP2040_SWIM_UART_RX_PIN 5u

#define RP2040_SWIM_FW_NAME "rp2040-swim"
#define RP2040_SWIM_FW_VERSION_MAJOR 0u
#define RP2040_SWIM_FW_VERSION_MINOR 1u
#define RP2040_SWIM_FW_VERSION_PATCH 0u

/*
 * Electrical safety:
 * RP2040 GPIOs are not 5 V tolerant. Connect SWIM only to 3.3 V targets or
 * through level shifting/protection. The SWIM pin is open-drain by default:
 * drive low as output-low, release high as input/high-Z.
 */

#endif
