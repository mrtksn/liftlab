# ESP32 Signal Workbench

The dedicated ESP32-WROOM-32 is a USB signal generator, digital logic analyzer, and analog input logger. Its browser UI is `tools/esc-control/`; the toy-drone motor test is now one editable preset. The original MicroPython panel is preserved at `tools/esc-control/legacy/`. The dedicated firmware is `runner/signalbench`, built with ESP-IDF v5.3.2 for **esp32**, not the old ESP32-S3 sniffer image.

## Open the UI

From the repository root:

```sh
python3 -m http.server 8765 --bind 127.0.0.1 --directory tools/esc-control
```

Open <http://127.0.0.1:8765/> in Chrome or Edge, click **Connect ESP32**, and choose its USB serial port. The workbench uses **921600 baud** and checks the firmware protocol before enabling the controls. Opening this board's USB serial port can reset it; the connection handshake retries while it boots. MicroPython cannot run the new workbench protocol.

**Try demo** explicitly simulates the controls and captures without driving GPIOs. Hardware and simulated captures are labelled separately. Presets are saved in this browser's local storage; use Export presets / Import JSON to back up or move them. Capture history holds the latest ten captures for the current page session; export JSON for metadata and edges or CSV for timestamped channel levels. Logs retain the latest 500 events, with search, level filtering, and JSON export.

## Graphical wiring diagrams and configuration

The top-right **Wiring diagram** button (also available from the sidebar) opens the selected preset's wiring document. Choose a channel to highlight its configured GPIO on the board profile. Opening this view sends no device commands. **Edit wiring** opens the preset editor at its wiring section, where you can edit power notes, defaults, diagram titles, voltage limits, components, connection notes and drawing labels; add a blank diagram or copy a library configuration; duplicate or remove diagrams. Advanced sections expose parameters, calculations, drawing elements/connections, sources and the board profile as JSON.

Device facts and examples are configuration data, loaded before the application:

| Configuration | Contents |
| --- | --- |
| `tools/esc-control/config/presets.json` | Shipped example presets, including the ESC's timing, initialization sequence, supply observations and complete wiring document |
| `tools/esc-control/config/board.json` | Board name, physical pin positions/labels, capabilities, voltage/ADC metadata, presentation and power notes |
| `tools/esc-control/config/wiring-templates.json` | Library configurations copied when adding diagrams; component values, pinouts, limits, text, drawing elements and calculations |
| `tools/esc-control/config/defaults.json` | Generic new-preset/control defaults and declarative legacy migration data |
| `tools/esc-control/config/manifest.json` | Configuration resources, script load order and cache revision |

The renderer does not recognize a toy-ESC ID, mode, device name, resistor value, supply boundary, or part number. It filters configured diagrams by channel direction, expands data bindings, draws allowlisted SVG primitives and evaluates a small declarative expression language. The photographed 30-pin board layout is configuration too. Firmware still validates actual supported GPIO roles and signal formats; editing documentation cannot change the installed hardware's electrical properties.

Every preset carries a complete `wiring` object with its own board profile, power notes, default diagram IDs and diagram list. Duplication and JSON export/import preserve it, even after the preset ID changes. Existing saved presets receive missing wiring configuration once; saved controls and GPIOs remain intact. Deleted buttons/diagrams are not restored. Editing shipped configuration files changes future seeds; edit an existing saved preset in the UI or import your edited JSON to change that copy. Export format is `signal-workbench/presets/3`, while older presets remain importable.

A diagram contains `id`, `label`, `title`, allowed `directions`, optional channel indices, `viewBox`, SVG `elements`, `parameters`, `calculations`, `limits`, `components`, `notes` and `sources`. Each element is data such as `{ "tag": "text", "attrs": { "x": 20, "y": 40 }, "text": "Device on {{channel.pinLabel}}" }`. Allowed drawing primitives are groups, rectangles, circles, text, polylines, paths and lines. Script elements, event attributes and external drawing resources are rejected. Explicit SVG stroke/fill values configure wire colors.

Bindings resolve `channel`, `board`, `adc`, `params` and `calc` properties. Calculations use arrays such as `["divide", {"ref":"params.sourceV"}, {"ref":"calc.ratio"}]`; they support arithmetic, comparisons, boolean operations and `if`, without JavaScript execution. Missing references, invalid parameter ranges, division by zero and cyclic calculations report errors. Limits can include a declarative `when` condition and `info`, `warning` or `danger` severity.

The shipped ADC divider example uses **20 kΩ / 10 kΩ, 1%, ¼ W** and an optional **100 nF ceramic capacitor rated at least 10 V** from the ADC node to GND. Its input bounds, component values, range calculation and warning conditions are all editable configuration. With those values the ideal RC corner is near 239 Hz; omit the capacitor for faster analog signals. The wiring-view inputs are temporary calculation previews; change parameter defaults in Edit wiring to save them. Software scale is still separate from physical components.

The shipped buffer configuration follows [TI's SN74LVC125A datasheet](https://www.ti.com/lit/ds/symlink/sn74lvc125a.pdf), ADC capacitor guidance follows [Espressif](https://docs.espressif.com/projects/esp-idf/en/v5.3.2/esp32/api-reference/peripherals/adc_calibration.html#minimize-noise), and the example I²C pull-up notes refer to [NXP UM10204, section 7.1](https://www.nxp.com/docs/en/user-guide/UM10204.pdf). Sources travel with their configured diagrams.

## Channels and signals

A preset configures **up to eight simultaneous logical channels total**. Each channel selects its own GPIO and digital input, analog input, or output direction. Any mixture works within that total. GPIOs must be unique; inputs never drive a target. Eight outputs have independent PWM timers. They are not phase-synchronized.

| Function | Implementation / scope |
| --- | --- |
| PWM / servo-style PWM | Hardware LEDC; integer frequency 1–1,000,000 Hz, subject to hardware divider quantization. Value is high time in µs. High time must fit the period. |
| Duty PWM | Same hardware; value is 0–100%. The hardware maximum is one timer count below a full-period high. Use a digital HIGH for an exact steady high. |
| Digital level / pulse | Repeating mode holds HIGH or LOW. Once mode sends that level for the configured duration, then returns to the opposite level. |
| Raw pulse sequences | Hardware RMT; `[level, duration_us]` segments at 1 µs resolution. Once or repeated. |
| UART output | Hardware-timed RMT encoding of 8N1, idle high; 1–12 editable bytes, baud 300–1,000,000. Actual baud is quantized to RMT clock ticks. No UART receive peripheral or flow control is used. |
| DShot output | Standard unidirectional DShot150/300/600; 16-bit frames including checksum, optional telemetry request bit. Values 1–47 are ESC commands; 0 is stop. No bidirectional telemetry generation. |
| Analog inputs | 12-bit ADC1 / ADC2, selectable attenuation, factory-calibrated voltage estimates when available, raw counts and scaled units. See the ADC workflow below. |
| Automatic input analysis | Existing native sniffer core: pulse statistics, PWM/OneShot candidates, CRC-checked DShot and heuristic UART candidates, when sampling resolves the signal. A repeated UART byte can also look like PWM; the description retains that ambiguity. |
| Explicit UART decoder | Passive 8N1 decoding from captured edges at the selected baud; framing errors are shown. |
| Explicit I²C decoder | Passive SDA/SCL decoding: START, STOP, 7-bit address + direction, data and ACK/NACK. No 10-bit address interpretation or active bus-master generation. |
| Explicit SPI decoder | Passive clock/data/optional active-low CS, modes 0–3, eight-bit MSB-first words. Select MOSI or MISO as the data probe; no active SPI master. |

RMT transmission stores at most **126 half-segments / 63 symbols per channel**, reserving a terminator. Durations longer than 32767 hardware ticks consume extra segments. Payload length, bitrate, and idle gap therefore interact: a nominally valid 12-byte payload or long repeat gap can still exceed memory. The firmware rejects it with an error; shorten the waveform or repeat interval. For UART and raw pulses, repeat interval includes the waveform duration. DShot repeat mode uses a fixed minimum 50 µs frame gap; PWM repeats at its configured frequency. A one-shot PWM signal is one high/low period, not a sustained ESC arming sequence.

## Controls and presets

Create, edit, duplicate, delete, import, and export presets in the sidebar. Each preset has channel definitions and up to 24 controls, plus a wiring configuration. A control targets one output channel and defines its on and off signals. Direct signal fields cover PWM frequency/width, duty, digital level/duration, UART baud/payload, DShot value/rate, or pulse segments. A slider plus numeric input can supply PWM width, duty, digital level, or DShot value. The editor validates the whole slider range against the signal's limits.

| Button behavior | Press / enable | Release / disable / next press |
| --- | --- | --- |
| On/off · repeat while on | Start hardware repeating output; slider updates live | Send off signal once, or apply channel idle |
| On/off · one signal on each edge | Send on signal once | Send off signal once |
| Hold · repeat until released | Start repeating output; slider updates live | Send off signal once, or apply channel idle |
| Hold · signal on press and release | Send on signal once | Send off signal once |
| Once · one signal per press | Send on signal once on every press | No off action |
| Alternate · on/off signal each press | First press sends on signal once | Next press sends off signal once, then alternate |
| Sequence · signals and delays | Execute authored signal, wait, channel-idle and final global-stop steps in order | Press again to cancel and idle its owned channels |
| Stop · idle all outputs | Cancel pending actions, apply idle to driven channels, disable the internal gate | No off action |

Every generator button is a preset control. Choose a large card or compact toolbar placement, idle/active colors, and idle/active labels. Toolbar controls can expose their slider, payload, or steps through **Values / steps**. The editor can rename, move, or delete the example arming and stop buttons; deleted controls do not reappear on reload. Existing presets migrate once to schema version 2 without losing their controls.

Sequences contain 1–32 steps: send a signal once or continuously, wait 1–60,000 ms, apply a channel's idle, or stop all outputs as the last step. Combined waits are limited to 120 seconds. Each signal uses direct fields or the parent control's slider. Hardware times each waveform; **the browser times waits**, so a sequence is not a precise hardware sequencer. A signal step starts transmission; add a wait if it must finish before the next step. Cancellation prevents later queued signals. Taking a channel from a running sequence cancels that sequence and idles its owned channels.

Two controls targeting the same output take ownership from one another. Independent channels can run together. The **Toy drone ESC · 1S** preset contains large on/off and hold controls targeting GPIO18 and an editable **Arm ESC** toolbar sequence. That sequence explicitly starts the 490 Hz / 550 µs stop waveform and waits 1 second; this is an example initialization sequence, not an independently established ESC requirement. GPIO19 is available for a monitor jumper. The motor slider retains 1070–2000 µs as its initial/provisional running range, with 550 µs available at the slider minimum. The [ESC bench notes](../tools/esc-control/README.md#bench-notes--89-november-2026) retain timing observations and this specimen's voltage-test results.

Boot, connect, and initial preset configuration leave output GPIOs high impedance. The private `enable` command only allows subsequent explicit actions; it sends no idle waveform and has no separate UI button. A preset's **Stop** action restores idle only on previously driven channels, then disables the gate. A motor preset's idle is a continuing stop pulse; a generic preset normally idles LOW. A preset may choose high impedance. Switching or editing presets stops outputs before reconfiguration. Outputs never restart automatically after stop, focus loss, timeout, or reconnect.

### UART payload editing and readable values

Edit UART payloads directly on generator cards or in the preset editor. The same editor supports on/off signals and sequence steps. Choose hex, escaped UTF-8 text, signed/unsigned 8-, 16-, or 32-bit integers, float32, or float64; multi-byte values have little/big endian selection. The wire-byte preview remains visible. Leaving the field commits a valid edit; pressing Send also commits the current edit. Changing an active repeat payload updates transmission. Invalid drafts show an error and cannot be sent; the OFF action remains usable.

**Text** `123\n` sends ASCII digits plus newline: `31 32 33 0A`. **Unsigned 16-bit**, little endian, `1234` sends binary `D2 04`. Text accepts `\n`, `\r`, `\t`, `\0`, `\\`, and `\xNN`; non-text bytes are preserved using escapes. Each waveform still has the native 12-byte/RMT-memory limit.

The UART decoder offers the same readable representations, byte order, and a starting byte offset. It groups consecutive successfully decoded bytes, reports incomplete numeric groups, and keeps the raw byte table and framing errors visible. It cannot infer application message boundaries or the sender's numeric format; select the layout used by the other board.

The browser sends a host heartbeat every 180 ms. A separate firmware task checks the lease on core 0 and applies idle/disarms after **900 ms without a heartbeat**, including while a capture is running on core 1. Tab hiding, window blur, disconnect, and Stop reset all control states. Software stop signals do not disconnect motor power. Resetting the board leaves outputs high impedance; external equipment's signal-loss behavior is separate.

## Capture workflow

1. Select a preset and connect. Capturing inputs does not require enabling outputs.
2. Choose **Input probes**, or **Inputs + output readback** to include the ESP32's own output pads. Readback verifies its generated logic timing without an external jumper; it does not establish voltage level or signal integrity at a remote device.
3. Choose a 5–100 ms window and an immediate/rising/falling/either-edge trigger. A trigger waits up to 100 ms; timeout is reported explicitly with an empty capture.
4. Capture once, or start live view. Each live-view capture begins after the previous one is transferred; this is **not gap-free streaming**.
5. Zoom and pan the waveform. Click twice for time cursors, delta time and the corresponding frequency. Inspect per-channel frequency, high time, duty and analysis evidence; choose explicit UART/I²C/SPI roles below.
6. Export evidence. **Use waveform as a pulse preset** extracts the selected decoder data channel in the visible time range, excludes the cut leading/trailing segments, and rounds durations to 1 µs. Zoom to at most 126 complete segments first, then review the generated preset before driving a device.

Capture stores up to 12,000 changes shared across channels. A full buffer cuts the window short and is marked truncated. Core 1 reads the GPIO banks in a tight loop with interrupts masked, timestamping changes with the CPU cycle counter. The capture reports its **measured average polling interval**. This is not a fixed guaranteed sampling rate, a worst-case latency bound, or a voltage measurement. The raw GPIO banks are captured first and channels are packed afterwards, so the tight polling loop does not grow with channel count; very short pulses may still be missed or aliased. Fast DShot/UART/SPI decoding needs sufficient timing resolution, so check the raw evidence and validate externally before treating a candidate decode as established.

## ADC workflow

Choose **Analog inputs · ADC**, or add channels with direction **Analog input · ADC** in any preset. Select attenuation and optional display scale, offset, and unit for each channel. In **Analog / ADC**, read once or start live readings; select rate/window, graph raw counts or voltage/scaled values, inspect min/max/mean and the last reading, and export timestamped JSON/CSV. Live view consists of separate acquisitions and transfer gaps.

The firmware allows 1–1000 requested samples/s, 10–10,000 ms windows, and at most **1024 samples per channel** per acquisition. UI combinations exceeding that budget are rejected. Up to eight channels share this budget with digital inputs/outputs. Channels are converted sequentially, not simultaneously; timestamps describe the start of each scan. Actual timing is recorded. This is a slow signal logger, not a high-speed analog oscilloscope. Acquisition can be cancelled without stopping unrelated outputs.

| Attenuation | Approximate characterized pin-voltage range |
| --- | --- |
| 0 dB | 0.10–0.95 V |
| 2.5 dB | 0.10–1.25 V |
| 6 dB | 0.15–1.75 V |
| 12 dB | 0.15–2.45 V |

The ADC returns 0–4095 raw counts. Voltage uses ESP-IDF line-fitting calibration with factory eFuse data when available; otherwise it uses a nominal 1100 mV reference and is explicitly labelled an estimate. Saturation, noise, source impedance, and offsets near zero still matter; compare against a meter/reference before relying on absolute accuracy. The physical pin must remain between GND and 3.3 V even when the useful measurement range is narrower.

**Analog wiring:** target GND → ESP32 GND; a suitable positive analog voltage → GPIO34 or GPIO35. ADC1 also exposes GPIO32, GPIO33, GPIO36/**VP**, and GPIO39/**VN**. VN is an ordinary single-ended ADC input here, not a negative-voltage input. ADC2 exposes GPIO4, GPIO13, GPIO14, GPIO25, GPIO26, and GPIO27; it is usable because this firmware keeps Wi-Fi off. Reserved boot/flash/USB GPIOs remain unavailable even if the chip has an ADC channel there.

For battery measurements, use a **physical voltage divider**. For example: source + → 20 kΩ → GPIO34 → 10 kΩ → GND, with source − sharing GND. This divides voltage by 3 (5.1 V becomes approximately 1.7 V). Set display scale to 3 and offset to 0 to show source volts; resistor tolerance and ADC accuracy still apply. Software scale does not protect a pin. Do not use the digital logic buffer described below for analog measurements—it discards amplitude information.

Implementation and ranges follow [Espressif's ADC oneshot driver](https://docs.espressif.com/projects/esp-idf/en/v5.3.2/esp32/api-reference/peripherals/adc_oneshot.html), [ADC calibration](https://docs.espressif.com/projects/esp-idf/en/v5.3.2/esp32/api-reference/peripherals/adc_calibration.html), and [WROOM-32 datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32_datasheet_en.pdf).

## Wiring for this photographed board

This is the 30-pin ESP32-WROOM board with the USB socket at the top and the metal module/antenna below it. Use the printed **GPIO names**, not physical header position numbers. GPIO16/17 are labelled **RX2/TX2**; GPIO36/39 are **VP/VN**.

| Role | Available pins |
| --- | --- |
| Input or output | GPIO4, 13, 14, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33 |
| Extra inputs only | GPIO34, 35, 36/VP, 39/VN; no internal pull-up/down |
| Reserved by the workbench | GPIO0, 2, 5, 12, 15 (boot strapping); GPIO6–11 (module flash); GPIO1/TX0 and GPIO3/RX0 (USB serial) |
| Not GPIOs | 3V3, VIN, GND, EN |

For the photo's orientation, **GPIO18/D18 and GPIO19/D19 are consecutive on the left header**, below D5 and above D21. The left header also has GND near 3V3; any board GND can be used. Disconnect the old Pi UART wiring before assigning this board's pins to other work. This workbench needs no Pi connection.

**First external loopback:** USB powers the ESP32; jumper **D18 → D19**. Select PWM / pulse bench, connect, press its signal button, then capture. UART and DShot loopback presets use the same pair. Alternatively choose output readback for a check requiring no jumper.

**Probe another board:** target ground → ESP32 GND; target signal → chosen input GPIO. Inputs are **3.3 V logic**, not 5 V tolerant. A series resistor alone does not shift a 1S/5 V signal. For up to 5.5 V input, the documented option is an **SN74LVC125A** powered at ESP32 3V3: common GND, 100 nF decoupling, used `/OE` low, target signal → A, Y → probe GPIO; unused A inputs grounded. Follow the exact package pinout. Do not substitute a generic HC125. See [TI's part datasheet](https://www.ti.com/lit/ds/symlink/sn74lvc125a.pdf) and the [existing buffer wiring notes](esc-sniffer.md#input-voltage-protection).

**Drive the toy ESC:** GPIO18 → blue M input; ESP32 GND → ESC supply negative; ESC red → external supply positive. Motor remains on the ESC and ESP32 stays USB-powered. GPIO19 can monitor M in parallel only if the signal remains within 3.3 V. The ESC's observed 5.1 V operating boundary applies to its **power input**, not GPIO signal voltage. Do not connect that supply positive to a GPIO or 3V3.

The allowable pin roles are based on [Espressif's ESP32 GPIO documentation](https://docs.espressif.com/projects/esp-idf/en/v5.3.2/esp32/api-reference/peripherals/gpio.html) and the [WROOM-32 module datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32_datasheet_en.pdf). GPIO16/17 are exposed on this WROOM board; WROVER modules with PSRAM can reserve them. This firmware intentionally targets this dedicated WROOM, not every ESP32 module.

## Build and tests

With ESP-IDF v5.3.2 exported:

```sh
idf.py -C runner/signalbench -B /tmp/signal-workbench-build build
idf.py -C runner/signalbench -B /tmp/signal-workbench-build -p /dev/cu.usbserial-0001 flash
node --test tools/esc-control/test_app.cjs
cc -std=c99 -O2 -Irunner/escsniff/main tools/test_escsniff.c runner/escsniff/main/sniff_core.c -lm -o /tmp/test-escsniff
/tmp/test-escsniff
```

The native sniffer analysis is shared with the original S3 firmware. The new WROOM adapter does not use the S3-only dedicated-GPIO peripheral or its GPIO4–7 input mapping.

Protocol **2**: newline-delimited JSON at 921600 baud; replies are prefixed `@SB `. Commands: `hello`, `configure` (full channel array), `enable` (silent output gate), `stop`, `ping`, `signal` (channel index, signal object, once/continuous mode and repeatMs), `idle` (one channel), `capture` (ms, triggerCh/triggerEdge, includeOutputs), `analog` (hz/ms), and `cancelCapture` (analog acquisition only). `id` correlates requests and replies. Digital captures stream `capture`, `edges` chunks of `[time_us, channel_bitmask]`, then `captureEnd`. Analog captures stream `analog` metadata, `analogPoints` chunks of `[time_us, raw_CH1, mV_CH1, ...]` in metadata channel order, then `analogEnd`. Metadata includes requested rate, calibration source, and cancellation status. Timeouts and boots are unsolicited events.

Pin-map validation occurs before replacing a valid configuration. Output-pin checks and the gate also apply to direct serial commands. Stop, enable, signal and idle are handled in the UART reader task so output actions stay responsive during a long analog acquisition. Stop invalidates older queued output actions. Sequences remain browser-side preset data; firmware does not contain ESC-specific arming buttons or sequences.

## Verified on the dedicated board — 18 November 2026

The initial protocol-1 firmware was built and flashed to ESP32-D0WDQ6 revision 1.0. With the user confirming USB power only and the ESC supply / Pi UART disconnected, unloaded output-pad readback verified:

- 490 Hz PWM at 1500 µs, live update to 1200 µs, and the 550 µs idle signal after Stop.
- Eight independently timed PWM outputs at 50, 100, 200, 300, 400, 490, 1000 and 8000 Hz. Average capture polling was approximately **0.275 µs**, including the eight-channel test; this is a measured average from this run.
- DShot150, DShot300 and DShot600, each decoded with value 100 and zero reported CRC errors in the captured frames.
- UART 115200 baud, repeating `55 AA`, decoded as 24 bytes with zero framing errors in a 60 ms capture.
- Raw pulse-train generation, finite one-shot completion, invalid pin-map rejection without changing existing outputs, and the lost-heartbeat stop/disarm behavior.
- An explicit Stop sent approximately 20 ms into a 100 ms capture changed a HIGH output to its LOW idle **during that capture**, rather than waiting for the capture/transfer to finish.

These checks observe the ESP32's own pads; they do not validate a remote load, an external protection buffer, or voltage accuracy. Evidence is under `captures/wroom-signal-workbench-2026-11-18/`. The shared C analysis tests and browser model/decoder/serial tests passed. Chrome UI checks covered simulated on/off control, capture/waveform rendering, preset creation and deletion. A live hardware check moved the PWM slider to 1290 µs while running; the browser capture measured 1290.64 µs at 489.78 Hz. Stop all outputs was then acknowledged and the workbench was left connected and disarmed. See `browser-verification.json` in the evidence directory.

Protocol 2 was subsequently built, flashed, and verified on the same unloaded board:

- Connecting/configuring, silent enable, and stopping an untouched output produced no detected edges; an explicit PWM action produced approximately 489.8 Hz / 1500.4 µs.
- UART text bytes `31 32 33 0A` decoded with zero framing errors.
- Eight analog channels (ADC1: 32/33/34/35/36/39; ADC2: 4/13) returned 50 complete timestamped 12-bit samples each at requested 100 samples/s. The board reported **eFuse line fitting**. Inputs were floating; this checks conversion/transport, not voltage accuracy.
- Invalid ADC GPIO/rate-window requests were rejected. Cancelling a 5-second ADC acquisition returned a partial capture marked cancelled, and a direct output action was acknowledged during acquisition.
- Browser checks confirmed editable text/numeric UART bytes in the activity log, preset-authored sequence/toolbar editing, and analog demo plots. The JS suite contains 18 passing tests covering codecs, migrations, sequence order/cancellation, serial cancellation, ADC validation, and decoders.

Protocol-2 evidence is under `captures/wroom-signal-workbench-2026-11-18/v2/`; the full reusable hardware suite is in its `full/` subdirectory. Earlier evidence above describes the initial firmware/UI.

A merged image is provided at `firmware/signal-workbench-esp32.bin`, ready to flash at offset `0x0` with esptool for **esp32**. The reproducible hardware check requires pyserial and deliberately drives otherwise unloaded pins:

```sh
python3 tools/esc-control/hardware_check.py --port /dev/cu.usbserial-0001 --unloaded --output-dir /tmp/signal-workbench-check
```

### Configuration refactor verification

The browser verified a copied preset with a new ID still selected its configured wiring default. Editing a diagram heading, device label, component list and supply-limit note changed the rendered diagram and survived reload. Removing that diagram stayed removed after reload. Twenty-five JS tests cover existing generator/decoder behavior plus configured calculations, arbitrary preset IDs, round-trip wiring data, migration without restoring deleted controls, and rejection of executable drawing content. These are UI/configuration changes; no firmware update or powered target test was needed.
