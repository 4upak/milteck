# Homework 11: UART + GPIO autopilot skeleton

`homework_11` is the next step after `homework_10`:
- keep the mission/autopilot idea,
- replace internal simulation wiring with UART protocol I/O,
- drive two GPIO outputs: `START` and `DROP`.

This folder is a **skeleton** for the DZ11 checker loop, not a finished hit solver.

## Main ideas

- parse frames from `protocol/drone_link.h`
- open UART in raw 115200 mode
- send `PKT_CONTROL` continuously
- raise `START` once on launch
- emit `DROP` pulse when release condition is met

## CLI

```bash
./homework_11_cli [uart_device] [gpio_chip] [start_line] [drop_line]
```

Defaults:
- uart: `/tmp/ttyA`
- gpio chip: `gpiochip0`
- start line: `24`
- drop line: `23`

## Real Raspberry Pi notes

Typical UART pins:
- TX: GPIO14, physical pin 8
- RX: GPIO15, physical pin 10
- GND: any ground pin

For a simple UART loopback self-test on one Pi:
- connect **pin 8 (TX)** to **pin 10 (RX)**

For GPIO line observation with LEDs:
- `START` on **GPIO24** = physical pin **18**
- `DROP` on **GPIO23** = physical pin **16**
- connect each through ~330Ω resistor to GND

If checker and student app both run on the same Pi and inspect the same GPIO lines in software,
no physical jumper is needed for `START/DROP`; only UART transport may still need wiring or a simulated endpoint.
