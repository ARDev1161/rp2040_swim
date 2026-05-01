# Wiring

Default Waveshare RP2040-Zero pins:

| RP2040-Zero | STM8 target |
| --- | --- |
| GPIO2 | SWIM through 220-1000 ohm series resistor |
| GPIO3 | NRST |
| GND | GND |

RP2040 GPIO is not 5 V tolerant. Use a 3.3 V STM8 target or add a proper level
shifter/protection circuit when the target board runs at 5 V.

Recommended SWIM line:

```text
RP2040 GPIO2 -- 220..1000 ohm -- STM8 SWIM
                         |
                       4.7k
                         |
                       3.3 V
```

The firmware drives SWIM low by switching GPIO2 to output-low. It releases SWIM
high by switching GPIO2 to input/high-Z. Internal pull-up can be enabled with
`--pullup`, but an external pull-up is recommended.

NRST is currently driven output-low for reset and released to input with pull-up.
If your target reset circuit is not 3.3 V safe, add an open-drain transistor or
level shifter.
