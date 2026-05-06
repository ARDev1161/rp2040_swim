#ifndef SWIM_LINK_H
#define SWIM_LINK_H

#include <stddef.h>
#include <stdint.h>
#include "usb_protocol.h"
#include "swim_phy.h"

typedef enum {
    SWIM_CSR_VERIFY_ONLY = 0,
    SWIM_CSR_WRITE_INIT_AND_VERIFY,
} swim_csr_action_t;

rpsw_status_t swim_link_enter(void);
rpsw_status_t swim_link_set_speed(swim_speed_t speed);
rpsw_status_t swim_link_srst(void);
rpsw_status_t swim_link_read(uint32_t address, uint8_t *data, size_t len);
rpsw_status_t swim_link_write(uint32_t address, const uint8_t *data, size_t len);

#endif
