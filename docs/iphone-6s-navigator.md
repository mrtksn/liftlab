# iPhone 6s board as a drone navigator — boards, power, wiring

Nov 25, 2026 · Mert · exported from the Claude Doc <https://claude.ai/artifact/AxG9M91KdeYuLWX36w1NnC> (rev 27)

It's doable as a navigator, with three limits:

- **Power:** no iPhone battery; the battery's gauge board, with its cell removed, fed 4.0 V from a converter. It's unproven, so bench-test it first.
- **Link:** talk to the ESP32-S3 over BLE.
- **Setup:** do it with the screen on, using Xcode 26.

Nobody has published a finished build of this, so the bench tests at the end come before any flight.

## Boards you need

In flight you need only the logic board, a power source in its battery socket, its antennas, and the Lightning flex if power or data comes in through Lightning. The screen and home button are only needed for setup.

| Part | In flight | For setup | Why |
| --- | --- | --- | --- |
| Logic board | Yes | Yes | A9 SoC with 2 GB RAM, InvenSense MP67B gyro/accel plus a Bosch accelerometer, barometer (likely BMP280), compass, GPS, USI Wi-Fi/BT module ([iFixit 6s teardown](https://www.ifixit.com/Teardown/iPhone+6s+Teardown/48170)) |
| Battery gauge board, cell removed | Yes | Yes | Repairers disagree on whether an iPhone boots from Lightning with no battery at all ([iFixit](https://www.ifixit.com/Answers/View/577000/What+parts+do+I+NEED+to+boot+iPhone+6)). The phone needs a gas gauge in the battery socket even with no cell (see Powering) |
| Lightning / headphone flex | Yes | Yes | Carries USB data and charging power (restore, Xcode, screen recording). On the 6s it also carries the cellular antenna cables |
| Antennas: the antenna unit and the rear-case antenna flexes | Yes | Yes | Without them the BLE/Wi-Fi range drops and GPS may not get a fix. Keep the rear case as the mount at first |
| Power / volume flex | Optional | Yes | Plugging in power turns the phone on. The power button is still needed to force a restart (power + home) |
| Rear camera (12 MP) | Only for vision | No | Plugs into its own connector |
| Display assembly (LCD, digitizer, 3D Touch, front camera, earpiece) | No | Yes | Activation, settings, pairing the Mac. At 60 g it is the heaviest part you can drop (the whole phone is 143 g) |
| Home button / Touch ID | No | Helpful | AssistiveTouch can stand in for it. Touch ID is paired to this board, so leave the passcode and Touch ID off |
| Taptic Engine, loudspeaker, SIM tray | No | No | Not needed |

Reports that the board boots with the display unplugged are anecdotal ([iFixit, iPhone 5s](https://www.ifixit.com/Answers/View/203539/Will+iPhone+turn+on+with+no+display-home+cables+attached)). Test a headless boot on the bench before you fly. Weigh your own stripped stack, since no source gives the board's weight.

## Powering the board without an iPhone battery

Keep the battery's own gauge board, throw away the cell, and feed the board 4.0 V from a converter on the drone LiPo. That way the phone still sees a real gas gauge. I found no published build that runs an iPhone long-term without a cell, so this is a bench project before it is a flight one.

**Why the phone needs the gauge.** The 6s battery has a TI gas gauge (sn27546). It talks to the phone over one data line, which iOS calls SWI ([ripitapart](https://ripitapart.com/2014/09/15/so-phone-me-maybe-a-list-of-iphone-batteries-with-gas-gauge-functionality/)). On Lightning power with no pack in the socket, a 6s boot-loops: the logo shows for a few seconds, then goes off ([iFixit, 6s](https://www.ifixit.com/Answers/View/316746/Will+this+phone+turn+on+without+a+battery+plugged+in)). If the board can't see the gauge chip, "the phone immediately trips off" ([iFixit, 4S](https://www.ifixit.com/Answers/View/205839/Bypass+battery+instructions+Use+wall+charger+only+w-o+battery...)). One repair thread is titled [iPhones keep restarting when using DC power supply](https://forum.ipadrehab.com/forum/iphone-logic-board-repair/3314-iphones-keep-restarting-when-using-dc-power-supply-cables) (the page itself wouldn't open).

| Option | How | What is known | Verdict |
| --- | --- | --- | --- |
| **1. Cell-less pack** | Cut the cell off an iPhone 6s battery, keeping its PCB and flex. Feed the PCB's cell pads 4.0 V from a converter | The phone gets real gauge data. Reported working on other phones. Not confirmed on a 6s | **Try first.** A few grams |
| 2. Repair boot tool (iPower MAX type) | Bench supply at 3.7–4.2 V with a 1.5–3 A limit. The tool supplies "battery power, gas gauge data" and a boot button. Lists the 6s ([Microsoldering Supply](https://microsolderingsupply.com/ipower-max-iphone-power-prompt-boot-tool)) | Built to boot boards for diagnosis. Whether it keeps iOS running for hours is not stated. Out of stock | Bench test only |
| 3. Plain DC into the battery connector, a resistor for the thermistor | Supply on the power pins, no gauge | Restarts reported. The gauge is missing | No |
| 4. Lightning 5 V, nothing in the socket | — | Boot loop on a 6s | No |
| 5. ESP32-S3 answers for the gauge on the SWI line | The S3 replies to the phone's gauge reads | The gauge chip supports SHA-1/HMAC authentication ([TI bq27546](https://www.ti.com/product/BQ27546-G1)). Unknown whether iOS 15 checks it on a 6s. No public example | Research project. Lightest if it works |
| 6. Small drone cell on the gauge board (fallback) | Swap the iPhone cell for a 1S LiPo of a few hundred mAh | Looks like a normal battery to the phone and buffers motor sag | Fallback if option 1 reboots under load. Not an iPhone battery, but still a cell |

Building option 1:

1. **Take the cell off safely.** Discharge the battery first. Cut one tab at a time and tape each cut end straight away. Never bend or puncture the pouch, and do it next to a fire-safe container.
2. **Pick the converter for your pack.** On 1S, use a buck-boost, so motor sag can't pull the phone under voltage. On 2S or more, a buck converter is enough. Set it to 4.0 V, inside the 3.7–4.2 V range repair tools use, and rate it for 3 A or more.
3. **Put bulk capacitance at the PCB pads**, 1000 µF or more of low-ESR capacitors. Without a cell, nothing else rides out a dip, and a dip means a reboot.
4. **Add an ideal diode** (for example TI LM66100) between the converter and the pads. Then, when Lightning is plugged in for Xcode, the phone's charger can't push current back into the converter.
5. **Bench-test in stages.** First a bench supply at 4.0 V with a 3 A limit, screen attached, for two hours with GPS, BLE and the camera running. Then the converter on its own. Then the converter with the motors spinning. Log battery %, temperature and any restarts through the app.

Watch the battery % the gauge reports. It counts charge as it leaves, so over hours it may decide the "cell" is empty and shut the phone down even at a steady 4.0 V. Odd gauge values may also trigger iOS's performance throttling on the 6s.

## Wiring to the ESP32-S3

Link them by radio, not by wire. Without Apple's MFi program, iOS gives apps no plain serial port on Lightning, but BLE and Wi-Fi work from any app.

```
Drone LiPo ──► Phone converter (4.0 V, ≥3 A) ──► ideal diode + ≥1000 µF ──► gauge board (no cell) ──► iPhone 6s board
     │                                                                                        (GPS, baro, compass, camera)
     │                                                                                                  ▲
     │                                                                          BLE / Wi-Fi UDP: setpoints down, telemetry up
     │                                                                                                  ▼
     └──────────────────────────────────────────────────────────────────────────────────────► ESP32-S3 (fast loop + own IMU) ──► ESCs
```

The only wires to the phone carry power. The ESP32-S3 keeps the fast loop and its own IMU. The phone sends roll, pitch, yaw-rate and thrust setpoints at 20–50 Hz.

| Link | iPhone side | ESP32-S3 side | Notes |
| --- | --- | --- | --- |
| **BLE (recommended)** | CoreBluetooth central | NimBLE GATT peripheral: a write-without-response characteristic for setpoints, a notify characteristic for telemetry | No network needed, low power. iOS sets the connection interval, roughly 15–30 ms (approximate, not confirmed in Apple's guidelines) |
| Wi-Fi UDP | Network framework or BSD sockets | softAP; the phone joins it | Lower latency, more power. iOS warns that the network has no internet |
| Wired USB (experimental) | Lightning to USB 3 Camera Adapter, read with CoreMIDI | TinyUSB MIDI device on the S3's native USB pins (GPIO19/20) | Class-compliant USB MIDI is widely reported to work on iOS. Untested here |
| Lightning UART | Not possible | — | Needs Apple MFi hardware |

Failsafe rule for the ESP32-S3: if no setpoint arrives for 200 ms, drop to angle mode, level the drone and descend slowly. Never let the phone's link decide whether the motors stay armed.

## Setup with the screen attached

Do all setup as a whole phone, before you strip it. Xcode 26 is the newest version that can install to and debug an iOS 15 device; Xcode 27 needs iOS 17 or later ([Apple Xcode support table](https://developer.apple.com/support/xcode/)).

1. **Clear Activation Lock.** Sign out of iCloud and turn off Find My while the old owner's account is still on the phone, then erase it.
2. **Update to iOS 15.8.8**, the last release for iPhone 6s (11 May 2026, [Apple security releases](https://support.apple.com/en-us/100100)). The 6s gets no iOS 16.
3. **Run the setup assistant** on Wi-Fi. Skip Touch ID, choose no passcode, and skip the Apple ID. An empty passcode means the phone is usable right after boot.
4. **Change these settings:**
   - Display & Brightness → Auto-Lock → Never
   - General → Software Update → Automatic Updates off
   - Accessibility → Touch → AssistiveTouch on (stands in for the home button)
   - Privacy → Location Services on
   - Battery → Low Power Mode off
   - Bluetooth on; join the Wi-Fi you will use
   - Airplane mode, then turn Wi-Fi and Bluetooth back on, to cut cellular power. Check that GPS still gets a fix
5. **Install your app.** Xcode 26 runs on macOS Sequoia 15.6 through Tahoe 26 ([Apple](https://developer.apple.com/support/xcode/)). If your Mac is on macOS 27, you have two options:
   - Use Xcode 26 on a Tahoe Mac, or in a Tahoe virtual machine with USB access to the phone.
   - Build in Xcode 27 with an iOS 15 deployment target, then install the signed app with Apple Configurator or `ideviceinstaller` (untested).

   A free Apple ID signs the app for 7 days only. A paid developer account ($99 a year) lasts a year.
6. **Build the app for headless use:**
   - set `isIdleTimerDisabled = true`
   - add the `bluetooth-central` and `location` background modes
   - send all status over the link (GPS fix, battery %, temperature), so you never need the screen to know what it's doing
7. **Lock it in.** Turn on Guided Access (Accessibility) and start it in your app, so a stray touch or notification can't leave the app.
8. **Strip and test.** Unplug the display, power the phone through the cell-less pack, and check that telemetry arrives at the ESP32-S3.

Without a jailbreak, iOS will not start your app by itself after a reboot. If the phone reboots in the field, you need the screen back, or the jailbreak route below, to start the app again. With no cell there is nothing to buffer a brownout, so any dip reboots the phone. Size the converter and capacitors with margin.

## Accessing the iPhone afterwards

While it flies, your own app is the way in: everything it knows goes over the link to the ESP32-S3 and on to LiftLab. On the bench, a Lightning cable to the Mac gives you the screen and the logs.

| Need | Without jailbreak | With palera1n |
| --- | --- | --- |
| See the screen headless | QuickTime Player → New Movie Recording → choose the iPhone (USB, view only) | Same, or a VNC server tweak |
| Logs | Xcode Devices window or Console.app over USB | `ssh` and read syslog |
| Send commands | Through your app's own link (start, stop, calibrate, a status request) | `ssh` over USB (`iproxy 2222 22`) or Wi-Fi |
| Start the app after a reboot | Needs the screen | A command over `ssh` |
| Install an update | Xcode 26 over USB | Copy and install over `ssh`, no 7-day signing |

[palera1n](https://palera.in) is the jailbreak for iOS 15 and later on A8 to A11 devices, which includes the 6s. It runs from a Linux or macOS computer and has rootless and rootful modes ([GitHub](https://github.com/palera1n/palera1n)). checkra1n covers the 6s only up to iOS 14. Both rely on the same bootrom bug, so expect to re-run palera1n from the computer after every reboot, even though its pages don't say so outright. That makes it a bench tool, not something to depend on in the field.

**Downgrading: possible, not recommended.** Turdus Merula can put an A9 phone such as the 6s on an older iOS without saved SHSH blobs ([tethered guide](https://ios.cfw.guide/turdusmerula-tethered/), [sep.lol](https://sep.lol/)). It's a poor fit for a drone, for four reasons:

- **Every boot needs the Mac.** Each power-on means DFU mode, which needs the power and home buttons, then a Terminal boot command over a USB-A cable. The boot can take several tries.
- **The Secure Enclave is exploited**, so the phone should hold no Apple ID or personal data.
- **Older iOS makes the tooling worse.** Below iOS 15, Xcode 26 can no longer install or debug the app.
- **It adds nothing the drone needs.** palera1n already jailbreaks iOS 15 on the 6s.

Stay on iOS 15.8.8 unless a feature you need exists only on an older iOS. My notes on the downgrade video are in [This video shows how to downgrade an iPhone iOS_....pdf](This%20video%20shows%20how%20to%20downgrade%20an%20iPhone%20iOS_....pdf).

Keep the power/volume flex fitted. Plugging in power turns the phone on, and holding power + home for about 10 seconds forces a restart. With no home button, restart it from the Mac over USB with `idevicediagnostics restart` (libimobiledevice).

## Risks and open questions

- **No finished build to copy.** I found no public project that flies a stripped iPhone board. The closest is your own Hacker News thread from December 2024 ([HN](https://news.ycombinator.com/item?id=42501335)), and nobody there reports trying it.
- **Power without a battery is unproven.** No public build runs an iPhone long-term with no cell. Option 1 is the best bet, not a known recipe.
- **Headless boot is anecdotal.** Test it on your board before you remove the screen for good.
- **Antennas.** The teardown doesn't say where the GPS antenna connects. Check that GPS gets a fix with the parts you keep.
- **Toolchain.** You need Xcode 26, which runs on macOS Sequoia 15.6 to Tahoe 26, not on macOS 27, plus re-signing every 7 days on a free account.
- **Heat.** A phone kept at 100% charge, with GPS and the camera on, gets warm. Leave it some airflow.
- **Weight.** The whole phone is 143 g, and the display alone is 60 g. A small frame can't carry this.

- [ ] Bench test: does it boot to the home screen with the display unplugged?
- [ ] Bench test: run on the cell-less pack from the converter, with the motors spinning, for a full flight's length
- [ ] Check: GPS fix with the stripped stack
- [ ] Check: BLE setpoint rate and latency to the ESP32-S3

## Sources

- [iFixit: What parts do I need to boot iPhone 6](https://www.ifixit.com/Answers/View/577000/What+parts+do+I+NEED+to+boot+iPhone+6)
- [iFixit: Will iPhone turn on with no display/home cables (iPhone 5s)](https://www.ifixit.com/Answers/View/203539/Will+iPhone+turn+on+with+no+display-home+cables+attached)
- [iFixit: iPhone 6s teardown](https://www.ifixit.com/Teardown/iPhone+6s+Teardown/48170)
- [Hacker News: turning an old iPhone into a drone](https://news.ycombinator.com/item?id=42501335)
- [palera1n](https://palera.in) and its [GitHub page](https://github.com/palera1n/palera1n)
- [checkra1n](https://checkra.in)
- [Apple: Xcode support (macOS, device and deployment ranges)](https://developer.apple.com/support/xcode/)
- [Apple: security releases (iOS 15.8.8)](https://support.apple.com/en-us/100100)
- [ripitapart: iPhone batteries with gas gauges](https://ripitapart.com/2014/09/15/so-phone-me-maybe-a-list-of-iphone-batteries-with-gas-gauge-functionality/)
- [iFixit: Will this 6s turn on without a battery](https://www.ifixit.com/Answers/View/316746/Will+this+phone+turn+on+without+a+battery+plugged+in)
- [iFixit: Bypass battery, wall charger only (4S)](https://www.ifixit.com/Answers/View/205839/Bypass+battery+instructions+Use+wall+charger+only+w-o+battery...)
- [Microsoldering Supply: iPower MAX boot tool](https://microsolderingsupply.com/ipower-max-iphone-power-prompt-boot-tool)
- [TI bq27546-G1 gas gauge](https://www.ti.com/product/BQ27546-G1)
