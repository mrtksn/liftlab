# A13 camera board (KL-908) Wi-Fi protocol

Decoded on 2026-10-21 from the stock iPhone app talking to the A13's
`KL15-D-V6` camera board. Capture: `captures/a13_app_1.pcapng` (iPhone mirrored
with `rvictl`, ~2 min, the bench rig had no motors connected).

## Network

| What            | Value                                           |
|-----------------|-------------------------------------------------|
| Board address   | `192.168.1.1`, open 2.4 GHz access point        |
| DHCP            | clients get `192.168.1.100`+, **no default route** |
| Video           | `rtsp://192.168.1.1:7070/webcam`, MJPEG 640×352, RTP over UDP |
| Control         | UDP to `192.168.1.1:7099`                       |

The app also sends `ef 00 04 00` to `192.168.169.1:8800` once every ~100 ms.
That address isn't on this network; it's the app probing for a different drone
family and can be ignored.

## Control packet (phone → 7099), 21 bytes, ~18 Hz

```
idx  0  1  2  3    4     5     6    7     8   9..18  19   20
     03 66 14 ROLL PITCH THROT YAW  FLAGS 02  00…00  CHK  99
```

- `03` is the packet type, `66` … `99` frame the payload, `14` (= 20) is its length.
- Sticks are unsigned bytes, centre `0x80`, range `0x00`–`0xFF`:
  - ROLL: left < `80` < right (seen `5b` left, `a7` right)
  - PITCH: back < `80` < forward (seen `5c` back, `a8` forward)
  - THROT: down < `80` < up (seen `00` bottom, `f8` top); the app's throttle stick
    self-centres, so `80` means "hold altitude", not "motors off"
  - YAW: left < `80` < right (seen `0c` left, `f1` right)
- FLAGS: `01` = take off, `02` = land. The app holds the flag for ~1 s, then
  returns to `00`. Emergency stop was not captured (likely another bit, e.g. `04`).
- Byte 8 is always `02` (speed or mode setting, unconfirmed).
- CHK = XOR of bytes 3–18.

Neutral packet: `03 66 14 80 80 80 80 00 02 00 00 00 00 00 00 00 00 00 00 02 99`

## Other phone → 7099 packets (2 bytes)

| Bytes   | Meaning                          |
|---------|----------------------------------|
| `01 01` | heartbeat, once per second       |
| `06 02` | switch to camera 2               |
| `06 01` | switch to camera 1               |
| `08 01` | sent once when the app leaves the flight screen or quits (goodbye) |

After a camera switch, the app re-opens the RTSP stream (new RTP ports).

## Board → phone (7099)

A constant 11-byte status packet, `65 02 31 46 31 46 00 00 00 00 00`, about
once per control packet. Its meaning is unknown; it never changed during the capture.

## Not yet known

- What the flight controller does when control packets stop (failsafe).
- Emergency stop, flip, headless and calibration flags. A second capture
  (`captures/a13_app_2.pcapng`) of the app's other buttons showed no new bytes:
  only sticks, take-off/land and `08 01`. These features may be missing from this
  app, or handled inside the app without telling the board.
- Whether the status packet reports battery or anything else.

## Flying it from a Mac

`tools/a13_pilot.py` shows the video and sends these packets from the keyboard or a gamepad
(`.venv/bin/python tools/a13_pilot.py`, on the drone's Wi-Fi). `tools/test_a13_pilot.py` checks its
packets byte for byte against the captured ones.
