# Homework 11: UART + GPIO autopilot

`homework_11` continues `homework_10`, but now the program talks to the checker over UART and drives two GPIO outputs through `libgpiod`.

The implementation is intentionally aligned with the homework text:
- UART via `termios`
- GPIO via `libgpiod`
- the same code path for simulation and for the Raspberry Pi

## Main behavior

- open UART in raw `115200 8N1`
- parse frames from `protocol/drone_link.h`
- raise `START` to `1` on launch and keep it high
- continuously send `PKT_CONTROL`
- emit a short `DROP` pulse when release conditions are met

## CLI

```bash
./homework_11_cli [uart_device] [gpio_chip] [start_line] [drop_line]
```

Defaults:
- uart: `/tmp/ttyA`
- gpio chip: `gpiochip0`
- start line: `24`
- drop line: `23`

Use examples:

```bash
# simulation / socat
./homework_11_cli /tmp/ttyA gpiochip0 24 23

# real Raspberry Pi UART
./homework_11_cli /dev/ttyAMA1 gpiochip0 24 23
```

## Raspberry Pi notes

Typical UART pins:
- TX: GPIO14, physical pin 8
- RX: GPIO15, physical pin 10
- GND: any ground pin

GPIO lines used by default:
- `START` -> GPIO24 -> physical pin 18
- `DROP` -> GPIO23 -> physical pin 16

## Pi smoke run with checker

```bash
sudo socat -d -d \
  pty,raw,echo=0,mode=666,link=/tmp/ttyA \
  pty,raw,echo=0,mode=666,link=/tmp/ttyB

sudo ./build/debug/homework_11/homework_11_cli /tmp/ttyA gpiochip0 24 23
sudo ~/checker_pi_arm64 1 --uart /tmp/ttyB --gpiochip gpiochip0 --start-line 24 --drop-line 23
```

On shutdown the program resets `START` and `DROP` back to inactive so the lines do not stay latched after exit.
