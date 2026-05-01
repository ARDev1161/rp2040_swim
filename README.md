# RP2040 SWIM

Experimental open-source STM8 SWIM programmer/debug-access firmware for the
Waveshare RP2040-Zero. The goal is to turn an RP2040-Zero into a direct USB to
SWIM probe for STM8 devices such as STM8S003F3P6 and STM8S103F3P6, without
requiring an ST-Link.

Status: experimental low-speed SWIM access is implemented. USB CDC framing,
host CLI, GPIO control, deterministic PIO UM0470 entry waveform,
synchronization-frame calibration, `SWIM_CSR=0xA0` setup, low-level SWIM
read/write plumbing, Intel HEX parsing, docs, and tests are present. Flash
erase/program commands are guarded until validated on real STM8 hardware.

## Hardware Wiring

```text
RP2040-Zero GPIO2 -> STM8 SWIM through 220-1000 ohm resistor
RP2040-Zero GPIO3 -> STM8 NRST
RP2040-Zero GND   -> STM8 GND
```

Optional target VDD sense is not implemented initially.

Warning: RP2040 GPIO is not 5 V tolerant. Use a 3.3 V STM8 target or a level
shifter/protection circuit for 5 V targets.

## Build Firmware

Install the Pico SDK and set `PICO_SDK_PATH`, for example:

```sh
export PICO_SDK_PATH=/path/to/pico-sdk
cmake -B build -S firmware -DPICO_BOARD=pico
cmake --build build -j
```

If your Pico SDK provides a Waveshare RP2040-Zero board file, use:

```sh
cmake -B build -S firmware -DPICO_BOARD=waveshare_rp2040_zero
cmake --build build -j
```

Flash `build/rp2040_swim.uf2` to the RP2040-Zero using BOOTSEL.

## Host Setup

```sh
python3 -m pip install -r host/requirements.txt
```

Example commands:

```sh
python3 host/rp2040_swim.py probe
python3 host/rp2040_swim.py enter-debug
python3 host/rp2040_swim.py entry-waveform --port /dev/ttyACM0 --pullup --delay-ms 1000
python3 host/rp2040_swim.py read --addr 0x8000 --len 256 --out dump.bin
python3 host/rp2040_swim.py flash --device stm8s003f3 --file firmware.ihx --verify --reset
```

`flash` currently reaches the firmware flash API and then fails safely with a
guarded error until the hardware flash-control sequence is completed.

## Troubleshooting

- No probe detected: check USB CDC serial permissions and pass `--port`.
- No SWIM response: check SWIM/NRST wiring and common GND.
- Target held in reset: check GPIO3 to NRST and target reset pull-up.
- Wrong voltage: RP2040 GPIO must not see 5 V.
- Missing common GND: connect RP2040 and target grounds.
- Too long wires: keep SWIM wiring short and add the series resistor.
- SWIM pin disabled or locked: try reset entry and confirm target option bytes.
- Timing issues: try default low speed first; validate GPIO2 with a logic
  analyzer.

## Implementation Report

Implemented:

- Pico SDK firmware layout and CMake build.
- USB CDC binary protocol with CRC32 and structured status codes.
- Open-drain style SWIM/NRST GPIO handling for RP2040-Zero default pins.
- PIO-generated UM0470 SWIM activation waveform using open-drain pin direction
  control. C bit-bang entry is fallback/debug only when PIO init fails.
- Low-speed bit primitives for post-entry communication.
- Target synchronization-frame measurement and low-speed timing derivation.
- `SWIM_CSR=0xA0` initialization before memory access.
- Host Python 3.10 CLI with `list`, `probe`, `reset`, `read`, `write-ram`,
  `enter-debug`, `entry-waveform`, `flash`, `erase`, and `unlock` command
  surfaces.
- Device database entries for `stm8s003f3` and `stm8s103f3`.
- Intel HEX/IHX parser and protocol unit tests.

Remaining TODO:

- Validate calibrated post-entry bit-banged SWIM timing on real STM8 hardware.
- Add high-speed SWIM only after HSIT/HS handling and sampling are validated.
- Complete and validate STM8 flash erase/program sequences.
- Add robust target ID/device detection through debug module registers.
- Add optional UART passthrough on GPIO4/GPIO5.

Logic analyzer test:

```sh
python3 host/rp2040_swim.py entry-waveform --port /dev/ttyACM0 --pullup --delay-ms 1000
```

Inspect GPIO2 for the activation sequence: 16 us low, four 1 kHz pulses, and
four 2 kHz pulses. `entry-waveform` holds NRST low only during the waveform,
does not wait for STM8 sync, and does not access target memory.

Two-Pico capture:

Terminal 1:

```sh
cd /mnt/raid/projects/Keyboard/rp2040_swim_test
python3 host/validation_runner.py passive-capture \
  --analyzer-port /dev/ttyACM1 \
  --rate 10m \
  --duration 10ms
```

Terminal 2:

```sh
cd /mnt/raid/projects/Keyboard/rp2040_swim
python3 host/rp2040_swim.py entry-waveform \
  --port /dev/ttyACM0 \
  --pullup \
  --delay-ms 1000
```

Expected analyzer result: NRST low during the waveform, SWIM starts high, initial
low around 16 us, four slow pulses with about 1000 us period, four fast pulses
with about 500 us period, and SWIM ends high/released.
