# Limitations

- Experimental project. Validate waveforms with a logic analyzer before
  connecting valuable targets.
- RP2040 GPIO is not 5 V tolerant. Use 3.3 V targets or level shifting.
- High-speed SWIM is disabled. The firmware only attempts low-speed SWIM after
  measuring the target synchronization frame.
- UM0470 entry waveform uses PIO by default with open-drain pin-direction
  control. C bit-bang entry is fallback/debug only if PIO initialization fails.
- Post-entry SWIM TX/RX is currently C bit-banged and calibrated from the target
  synchronization frame.
- Program flash erase/write is guarded and returns an error until verified on
  STM8S003F3/STM8S103F3 hardware.
- Flash programming remains disabled; the host command shape exists, but the
  firmware refuses erase/write instead of faking success.
- Option byte writing is intentionally not implemented.
- Device auto-detection is limited to target responsiveness/reset-vector reads.
- USB transport is CDC serial, not a custom TinyUSB vendor interface.

Logic analyzer test points:

- Run `entry-waveform`, then inspect GPIO2 for the 16 us low activation pulse,
  four 1 kHz pulses, and four 2 kHz pulses. This command does not wait for STM8
  sync and does not access target memory.
- With a second Pico analyzer, capture at 10 MS/s for 10 ms while running:
  `python3 host/rp2040_swim.py entry-waveform --port /dev/ttyACM0 --pullup --delay-ms 1000`.
- Run firmware command `DEBUG_WAVEFORM` from a host script to emit a repeatable
  pattern of alternating SWIM bits.
- Decode low-speed bits by pulse width: short-low/long-release is `1`,
  long-low/short-release is `0`.
