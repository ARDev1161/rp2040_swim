#ifndef SWIM_LINK_H
#define SWIM_LINK_H

#include <stddef.h>
#include <stdint.h>
#include "usb_protocol.h"

rpsw_status_t swim_link_enter(void);
rpsw_status_t swim_link_srst(void);
rpsw_status_t swim_link_read(uint32_t address, uint8_t *data, size_t len);
rpsw_status_t swim_link_write(uint32_t address, const uint8_t *data, size_t len);

#endif
