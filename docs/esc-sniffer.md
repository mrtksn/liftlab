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
LF, CR and CRLF line endings are accepted; CRLF triggers just one command, including when split across USB reads.

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

### Input voltage protection

**The S3 GPIOs take 3.3 V logic. A 4.7 kΩ series resistor alone does not make a 4.2 V signal safe.**
Espressif specifies an input-high maximum of VDD + 0.3 V (3.6 V with a 3.3 V supply):
[ESP32-S3 module DC characteristics](https://documentation.espressif.com/esp32-s3-wroom-2_datasheet_en.html).

For a signal that may swing to the A13's full 1S battery voltage, use a non-inverting,
overvoltage-tolerant buffer such as the **SN74LVC125A**, powered from the S3's **3V3** pin.
TI specifies input voltages up to 5.5 V and 3.3 V operation for this exact part:
[SN74LVC125A datasheet](https://www.ti.com/lit/ds/symlink/sn74lvc125a.pdf).
Choose the LVC part; a generic HC125 is not an equivalent input-protection substitute.

- Join buffer GND, S3 GND, and drone `B−`.
- Add a 100 nF ceramic capacitor between buffer VCC and GND near the chip.
- Tie each used buffer channel's active-low output-enable (`/OE`) to GND.
- Connect the drone M signal to buffer input `A`; connect its corresponding output `Y` to the S3 CH input.
- A 4.7 kΩ series resistor can be retained **between Y and the S3**, where the signal is already 3.3 V.
- Tie unused buffer inputs to GND. Follow the exact package pinout in the datasheet.

This is a wiring correction, not protection that firmware can supply. The buffer circuit has not been bench-validated here.
A multimeter reading on a pulsing line is an average; use an oscilloscope to establish its peak voltage before considering a direct 3.3 V tap.

### Capture sequence

1. **Props off.** Leave the A13's original ESC wiring connected. Disconnect its battery while attaching probes.
2. **Find the pins.** Each arm plugs into the flight board on a 4-pin `LMGV` connector. Confirm G from its trace or direct continuity to `B−` with power off; a powered 0 V reading alone does not distinguish ground from an idle-low command. Use the printed labels and verify V against the battery supply. L and M are the other two; the LED line probably blinks with the arm LED.
3. **Wire the protected inputs above.**
   - Solder or clip a thin wire to each M pad. Leave the connector plugged in, so the ESC still gets its signal.
   - Run each M wire to a buffer input A, and the corresponding Y to CH1–CH4 (GPIO4–7). Note which arm goes to which channel.
   - Join the drone's `B−` to the S3's GND.
   - Never connect the drone's V line to a GPIO or the buffer's VCC. Power the S3 from USB and the buffer from S3 3V3.
   - Reconnect the drone's original 1S battery after checking the wiring.
4. **Watch.** Type `w`. Then:
   - Note what each line shows while the drone is switched on but not armed.
   - Arm it with the remote and raise the throttle slowly. Note low, middle and full throttle.
   - Touch a stick so the flight board speeds up one motor and slows another, and note how each line changes.
5. **Save the evidence.** At each throttle setting, stop watch, run `s`, then `raw 1 100` and `csv 12000`, and keep the output with the setting noted. Raw/csv export the last capture; they do not take a fresh one. These captures are a starting point for decoding an unusual protocol.

## What the result means for LiftLab

| Reading | Next step |
|---|---|
| A PWM reading ending in `[also fits UART ~N baud, repeated xx]` | The pulse train also decodes as one serial byte sent over and over. PWM is the likelier answer, so treat it as PWM. To check, change the throttle: PWM keeps its period and moves its duty smoothly, while UART keeps its edges on whole bit times. |
| Servo-style PWM (about 1000–2000 µs) | The flight firmware's **PWM ESC** motor mode can drive these ESCs once its pulse range and frequency are set to match. |
| Duty PWM at a fixed frequency, with duty that follows throttle | The **brushed MOSFET** mode produces this kind of signal (active-high duty, 1–30 kHz). Check the signal's voltage level before connecting a GPIO directly to the ESC's input. |
| OneShot or DShot | Not supported by the flight firmware yet. DShot output would need a new motor driver (RMT). |
| Serial bytes | A proprietary bus. We'd need captures across the whole throttle range to decode it. |
| Steady level that follows throttle on a multimeter | An analog (filtered) command, not pulses. Measure it with the multimeter instead. |

The analysis is plain C (`runner/escsniff/main/sniff_core.c`). `tools/test_escsniff.c` runs it on the host against synthetic captures of each signal kind, including repeated `00`/`FF`/`33` UART bytes, which read as pulse trains with a UART note. It also exercises the shared console parser with LF, CR, CRLF, consecutive Enter, and separate reader state. CI runs this test file.

UART detection is heuristic: it checks framing and edge alignment, and considers standard baud rates when the shortest stretch spans multiple bits. Candidate bytes are evidence to compare, not proof of the command protocol. The original `6bcdeab` firmware compiled in CI; the fixes need a new firmware build before flashing. Sampling timing and the input buffer still need physical loopback and A13 validation.
