# USB Protocol

The probe exposes a USB CDC serial interface. Timing-critical SWIM activity runs
inside the RP2040 firmware; the host sends high-level commands.

All multi-byte integers are little-endian.

Request frame:

| Field | Size | Meaning |
| --- | ---: | --- |
| magic | 4 | `0x53575052` |
| version | 1 | protocol version, currently `1` |
| command | 1 | command ID |
| sequence | 2 | host sequence number |
| payload_length | 2 | payload bytes |
| payload | N | command payload |
| crc32 | 4 | IEEE CRC32 over all previous fields |

Response frame:

| Field | Size | Meaning |
| --- | ---: | --- |
| magic | 4 | `0x53575052` |
| version | 1 | protocol version |
| command | 1 | request command ORed with `0x80` |
| sequence | 2 | matching request sequence |
| payload_length | 2 | status plus payload bytes |
| status | 2 | status code |
| payload | N | response payload |
| crc32 | 4 | IEEE CRC32 over all previous fields |

Commands:

| ID | Name | Payload |
| ---: | --- | --- |
| 0x01 | GET_VERSION | none |
| 0x02 | SET_PINS | `swim_gpio:u8 nrst_gpio:u8 internal_pullup:u8` |
| 0x03 | SET_SPEED | `0` low speed; high speed is currently rejected |
| 0x04 | ENTER_SWIM | none |
| 0x05 | RESET_TARGET | none |
| 0x06 | SWIM_READ | `address:u32 length:u16` |
| 0x07 | SWIM_WRITE | `address:u32 length:u16 data...` |
| 0x08 | MEMORY_READ | `address:u32 length:u16` |
| 0x09 | MEMORY_WRITE | `address:u32 length:u16 data...` |
| 0x0A | FLASH_ERASE | `address:u32 length:u32` |
| 0x0B | FLASH_WRITE_BLOCK | `address:u32 length:u16 data...` |
| 0x0C | FLASH_VERIFY | reserved |
| 0x0D | GET_LAST_ERROR | none |
| 0x0E | DEBUG_WAVEFORM | emit SWIM entry/test bits |
| 0x0F | GET_SWIM_DEBUG | none; returns timing/debug state including sync timing, PHY backend, entry waveform metadata, and PIO status |
| 0x10 | ENTRY_WAVEFORM | `delay_ms:u32`; waits, holds NRST low, emits only the UM0470 entry waveform, releases NRST |

Flash commands currently return a guarded error until the erase/program control
sequence is validated on real STM8S003F3/STM8S103F3 hardware.
