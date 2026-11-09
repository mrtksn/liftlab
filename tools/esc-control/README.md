# ESC Bench Control

Local Web Serial interface for the classic ESP32 running MicroPython. From the repository root:

```sh
python3 -m http.server 8765 --bind 127.0.0.1 --directory tools/esc-control
```

Open http://127.0.0.1:8765/ in Chrome or Edge. Disconnect the ESC battery before connecting the panel to USB. The panel starts `device.py` in the MicroPython REPL; it does not install or auto-start the program on the board. The serial port must not be open in another application.

Choose a frequency with the battery disconnected and before connecting the panel: 490 Hz (up to 2,000 µs), 400 Hz (up to 2,450 µs), or 300 Hz (up to 3,000 µs). Operation and release-to-stop were observed in the limited 490 Hz and 300 Hz tests below; 400 Hz remains untested. A 550 µs stop-like pulse changes duty cycle with frequency, and stopping has not been established under every condition. Start with a short low-speed test and disconnect the battery if the motor does not stop. The maximum pulse values are timing limits, not motor speed ratings.

Arm sends a 550 µs stop-like signal at the selected frequency. Hold Run to send the selected pulse; release, Stop, or Disarm restores the stop-like pulse. Disarm also prevents Run until rearmed; it does not remove electrical power. While Run is held, the browser renews a short lease. If the device stops receiving run commands for 650 ms while its program is running, it returns to the stop-like pulse and disarms. Browser disconnect or a device reset can instead leave the signal undriven; ESC behavior on signal loss is unknown. Neither software stop nor the watchdog is a substitute for a physical battery disconnect.

## Bench notes — 8–9 November 2026

The setup uses a classic ESP32-D0WDQ6 revision 1.0 with 4 MB flash and MicroPython 1.29.0. GPIO18 is connected to the separate original ESC's blue M input; ESP32 ground and the ESC's battery negative are shared. The ESC powers the connected BLDC motor from its original 1S battery. Tests were reported with the propeller removed. The exact ESC model, signal voltage thresholds, and complete wiring details have not been independently verified.

- Input-only capture of the original flight controller showed approximately 550 µs high / 1,492 µs low at stop; about 745 µs / 1,292 µs at one running setting; and roughly 1,470–1,570 µs high / 465–560 µs low at a higher setting. High and low were sampled separately; initial outliers occurred. These are approximate, not a complete protocol specification or preserved raw capture.
- The user reported the motor remained running at reduced throttle and stopped using the original remote's stop button; after that button, captured timing returned to the initial range.
- With the ESP32 driving the ESC at 490 Hz, 750 µs and 1,000 µs short tests produced sound but no observed movement; 1,500 µs spun the motor, and 550 µs stopped it. A GPIO18-to-GPIO19 loopback confirmed approximately the intended timing, not signal voltage at the ESC.
- In the control panel, the user reported 1,070 µs as the lowest tried setting that started the motor, and release-to-stop worked. This was before the frequency selector was added, at the panel's 490 Hz setting.
- At 300 Hz, the user reported the motor spun at 2,960 µs, did not start at 2,970 µs, and stopped on release at 2,960 µs. The cause of the boundary is unknown.
- The user measured current with the propeller off and reported no increase above 2,000 µs at 300 Hz. Individual current readings, measurement setup, RPM, and thrust were not recorded. This observation alone does not establish maximum motor speed or thrust.

**Working proposal:** Treat 2,000 µs as a provisional upper test command and use 490 Hz as the baseline, since it is close to the original controller's measured frequency and operation and release-to-stop were observed there. The current plateau was observed at **300 Hz**, not 490 Hz, so it does not prove that 2,000 µs is the maximum effective command at 490 Hz or establish a safe motor/ESC limit. At 490 Hz, 2,000 µs is approximately 98% duty cycle. Recheck motor response and current under the intended supply and load before treating this as a calibrated limit.

These are limited bench observations, not flight validation. Use a propeller-free setup with the battery connector accessible, and verify stopping at each frequency before further experiments. The panel's conservative 300 Hz warning predates the limited test; the behavior is observed but not fully characterized.

Run the simulated watchdog test with `python3 tools/esc-control/test_device.py`.
