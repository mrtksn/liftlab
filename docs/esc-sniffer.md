# ESC signal sniffer

`runner/escsniff` is ESP32-S3 firmware that only listens. It reads up to four motor-signal lines and reports what each one carries: servo-style PWM, OneShot, duty PWM, DShot (decoded and CRC-checked), or UART serial (with the decoded bytes). Each guess is printed with the timing numbers it is based on. It was written for the salvaged AirVolt A13 toy drone: its flight board drives four small brushless ESCs, and we don't know what signal it sends them. Until we know, an ESP32 can't drive those ESCs.

Nothing here drives a motor. The only output is an optional test signal on GPIO8.

## Get it on the board

CI (`.github/workflows/boards.yml`, job `escsniff`) builds one image, `escsniff-esp32s3.bin`, from ESP-IDF v5.3.2. Download it from the run's artifacts and flash it at offset `0x0`, either with the [esptool-js web flasher](https://espressif.github.io/esptool-js/) or with:

```bash
esptool.py --chip esp32s3 write_flash 0x0 escsniff-esp32s3.bin
```

To build it locally: `cd runner/escsniff && idf.py -D IDF_TARGET=esp32s3 build flash monitor`.

Open a serial terminal at 115200 baud on either USB port, then type commands and press Enter.

| Command | What it does |
|---|---|
| Enter (or `s`) | One capture of all four lines (60 ms by default), each line described |
| `w` | Watch: a capture every 300 ms until you press Enter |
| `raw <1-4> [n]` | The first n high/low stretches on one line, in µs |
| `csv [n]` | Every change as `time_us,ch4ch3ch2ch1`, for pasting elsewhere |
| `win <10-200>` | Capture window in ms. A 50 Hz servo signal needs at least 45 |
| `gen <hz> <high_us>` / `gen off` | Test signal on GPIO8 |

## Check the sniffer first

Put a jumper wire from GPIO8 to GPIO4, then run:

- `gen 50 1500`, then Enter: CH1 should read about 50 Hz, high 1500 µs, servo-style PWM.
- `gen 8000 30`: CH1 should read about 8000 Hz, 24% duty PWM.

The first report line also shows the sampling interval (about 0.1 µs). If both of those read correctly, the capture path works.

## Tap the A13

1. **Props off.** Leave the A13's wiring as it is and power it from its own battery.
2. **Find the pins.** Each arm plugs into the flight board on a 4-pin `LMGV` connector. Switch the drone on and measure every pin against `B−` with a multimeter. The pin at battery voltage (3.7–4.2 V) is V, and the pin at 0 V is G. L and M are the other two; the LED line probably blinks with the arm LED.
3. **Wire it.**
   - Solder or clip a thin wire to each M pad. Leave the connector plugged in, so the ESC still gets its signal.
   - Run each wire through a **4.7 kΩ resistor** to CH1–CH4 (GPIO4–7). Note which arm goes to which channel.
   - Join the drone's `B−` to the S3's GND.
   - Never connect the V line. Power the S3 from the laptop's USB only.
   - The resistor keeps the current tiny even if the signal swings to the full 4.2 V of a 1S battery.
4. **Watch.** Type `w`. Then:
   - Note what each line shows while the drone is switched on but not armed.
   - Arm it with the remote and raise the throttle slowly. Note low, middle and full throttle.
   - Touch a stick so the flight board speeds up one motor and slows another, and note how each line changes.
5. **Save the evidence.** Run `csv` once at idle and once at mid throttle, and keep the output. It's enough to work out an unusual protocol later.

## What the result means for LiftLab

| Reading | Next step |
|---|---|
| Servo-style PWM (about 1000–2000 µs) | The flight firmware's **PWM ESC** motor mode can drive these ESCs once its pulse range and frequency are set to match. |
| Duty PWM at a fixed frequency, with duty that follows throttle | The **brushed MOSFET** mode produces this kind of signal (active-high duty, 1–30 kHz). Check the signal's voltage level before connecting a GPIO directly to the ESC's input. |
| OneShot or DShot | Not supported by the flight firmware yet. DShot output would need a new motor driver (RMT). |
| Serial bytes | A proprietary bus. We'd need captures across the whole throttle range to decode it. |
| Steady level that follows throttle on a multimeter | An analog (filtered) command, not pulses. Measure it with the multimeter instead. |

The analysis is plain C (`runner/escsniff/main/sniff_core.c`). `tools/test_escsniff.c` runs it on the host against synthetic captures of each signal kind, and CI runs those tests. The capture loop and the S3's dedicated-GPIO path are only compile-checked in CI. Nothing has been verified on a physical board yet.
