# The data bus: topics every program reads and writes

Status: steps 1 and 2 implemented in the C library and the simulator; on real boards from step 2's wiring on (see
[Migration](#migration)).

## Why

Today every connection between programs is written by hand for its one purpose. The flight core and the navigation
pass `RN_LINK_NAV` and guided `RN_LINK_CMD` frames; the learning and the supervisor have `RN_LINK_LTEL`, `EXC`,
`MODEL`, `SET`; the radio has `RN_LINK_RC` and `RN_LINK_TLM`; the fleet has `RN_LINK_PEER`. Each new piece of data
that has to reach another program means a new frame type, packing code on one end, unpacking on the other, and the
simulator's copy of both (`js/boards.js` `sendFrame`/`deliverFrames`). A program of your own has no way in at all.

The bus replaces that with one idea: **every board keeps a table of named, typed values (topics); programs publish
what they compute and read what they need; boards copy each other's topics over their link when asked.** This is the
pattern PX4 uses inside its flight controller (uORB) and ROS 2 uses on companion computers.

## What a topic is

A topic is a named record of up to 32 floats, with:

- **one writer** (a program on one board) — nobody else may write it;
- a **sequence number** that goes up by one at each publish, so a reader can tell "new" from "same as before";
- the **time** it was published (the board's clock), so a reader can tell how old it is;
- on another board, a **mirror** of it: the same values, sequence and age, read-only there.

Names are dotted paths: `fc.attitude`, `nav.estimate`, `cmd.pilot`, `user.camera.photo`. A board's bus holds at most
48 topics and 1024 floats (4 KB): small enough for an ESP32, and fixed, so nothing allocates in flight.

**Watching** a topic means remembering the sequence number last seen and asking whether it changed
(`bus_changed`). There are no callbacks: a program checks at its own rate, in its own loop. That is what keeps the
flight loop deterministic.

## Rules

These are what make it safe to fly on, not just convenient.

1. **The flight loop never waits on the bus.** Inside the flight core the chain from attitude estimate to throttles
   stays as it is: direct and on time. The bus is read at the start of a step (latest values) and written at the end.
   Publishing is a copy into a fixed slot; nothing blocks, allocates or queries.
2. **One writer per topic; the flight core arbitrates commands.** Several programs may *want* to steer (the pilot's
   radio, the navigation, a mission, a script on the Pi), but each writes its own command topic, and the flight core
   decides with fixed priorities: failsafe > the pilot's sticks > the navigation > a mission or companion program. A
   program never writes another program's topic. (Arbitration is step 4; today the flight core still takes the
   commands it took before, and records them on the bus.)
3. **Every value has an age, and stale is missing.** A reader decides how old is too old for its use; the safety logic
   treats a stale command as no command. The Pi can't hold the flight core hostage: if its topics stop, the existing
   failsafes apply.
4. **The link has a budget.** The ESP32–Pi link runs at 921600 baud, about 90 KB/s. A board copies a topic to another
   only when that board subscribes to it, at the period it asks for (or on change). Nothing is mirrored by default.
5. **Names and sizes are checked.** A mirror is made from the subscriber's own declaration of the topic (name and
   size); data whose size doesn't match is refused and counted, so a Pi and an ESP32 built from different versions
   notice instead of misreading each other.

## Topics the flight code publishes

Implemented in step 1 (`runner/fc/bus.h`). Index order is the order of the floats.

| Topic | Writer | Floats | When |
|---|---|---|---|
| `fc.state` | flight core | state (0 disarmed, 1 armed, 2 failsafe, 3 crashed, 4 motor test), attitude settled, has height, guided, supervisor mode, on the learned model, last formula error | every step |
| `fc.attitude` | flight core | q (4, body → world), body rates (3) [rad/s] | every step |
| `fc.imu` | flight core | specific force (3) [m/s²], gyro (3) [rad/s], body axes | every step with an IMU sample |
| `fc.height` | flight core | height [m], vertical speed [m/s], has height | every step |
| `fc.output` | flight core | motor throttles 0–1 (12), servo angles [rad] (8) | every step |
| `fc.torque` | flight core | torque the attitude control asked for, body [N·m] (3) | every flying step |
| `cmd.pilot` | flight core (what it accepted) | arm, roll, pitch, yaw, throttle, guided, acceleration (3), heading | each command |
| `nav.estimate` | navigation | position from home (3) [m], velocity (3) [m/s], has home, ready, landed | every step |
| `nav.setpoint` | navigation | target from home (3), target velocity (3), heading [rad], fly | every step |
| `nav.command` | navigation | acceleration wanted (3) [m/s², gravity not included], heading, fly | every step |

`cmd.pilot` is written by the flight core in this first step because that is where commands arrive today; in step 4
the radio and the navigation each write their own command topic and the flight core reads them.

Programs of your own use names under `user.`; that is all they may publish (the flight code's topics keep their one
writer).

## Copying topics between boards

Two frame types on the board link (`runner/rn_link.h`), both lists of little-endian floats like the other frames:

- **`RN_LINK_BUS_SUB`** (subscriber → publisher, every 0.5 s while it wants them):
  `version (1), count, then per topic: name hash high 16 bits, low 16 bits, period [ms] (0: on change), floats`.
  A subscription that isn't renewed for 2 s lapses.
- **`RN_LINK_BUS`** (publisher → subscriber, as subscriptions fall due):
  `version (1), count, then per topic: name hash high, low, sequence (mod 2²⁴), age [ms] when sent, floats n, values`.

A topic travels by the FNV-1a hash of its name (two 16-bit halves: a float holds integers exactly to 2²⁴). The
subscriber declared the name, so it knows which mirror the hash belongs to. A topic with a period is sent when it has been
published since and its period has passed. **On change** (period 0) means its *values* changed — the flight core
publishes `fc.state` every step, but the copy travels only when the state does — and at least every 0.5 s regardless
(a heartbeat), so the copy's age keeps saying whether its writer is still there. Old firmware ignores both frame types (unknown types are dropped), so adding them
breaks nothing.

The receiving board sets the mirror's time to *now − age*: its own clock, less how old the value was when it left.
The link's own delay (6 ms in the simulator's model) is not added: the boards' clocks aren't synchronized, so the
receiver can't measure it. A copy's age is therefore "at least this old".

## Programs on the bus

- **The flight code** (C: `fc_core`, `nav_core`) publishes the topics above when it has a bus (`F->bus`, `N->bus`);
  without one (the ESP32 and Pi builds until they're wired up) nothing changes.
- **The simulator** gives each simulated board its own bus, copies topics between boards with the same frames and
  delay as a real link, and shows every board's topics live: **Computers → Live data…** (a card per board; each topic
  with where it comes from, its age, how many updates a second arrive *on that board*, and its values, labelled on
  hover). The board linked to the flight core asks for `fc.state` (on change), `fc.attitude` (every 20 ms),
  `fc.height` (50 ms) and `cmd.pilot` (on change); the flight core asks for that board's `nav.estimate` and
  `nav.command` (50 ms). A program's own topic is published with `bus_put` (names under `user.` only) and asked for
  with `bus_want`.
- **Programs on the Pi** (step 3): a local socket on `dfb_pi`, the same calls (publish, read, subscribe), and a small
  Python client, so a script can watch `cmd.pilot` and publish `user.camera.photo`. The simulator offers the same
  calls through a local bridge, so one script runs against both.
- **Formulas** (step 4): a formula's signature may name topics it reads and the topic it writes, and is run at a rate
  or when an input changes. A mission is then a formula writing `cmd.mission`, which the flight core arbitrates.

## Migration

Each step ships something useful and keeps flying as it did.

1. **The store** — done. `runner/fc/bus.c`: topics, publish, read, watch, ages; the flight core and the navigation
   publish what they already compute; the simulator's boards have one each and the Live data view shows them.
   Behaviour is unchanged (the formulas, outputs and frames are the same).
2. **Copying between boards** — done in the simulator and the C library (`bus_sub_pack`, `bus_pack`, `bus_unpack`,
   tested natively); the simulated Pi subscribes to the flight core's topics over the simulated link. On real boards,
   `dfb_pi` and the ESP32 firmware still need to send and answer the two frames (the code is the same library calls;
   the firmware then needs a rebuild with ESP-IDF 5.3.2).
3. **Programs on the Pi**: the socket API, a Python client, the simulator bridge.
4. **Commands as topics, with arbitration**: `cmd.radio`, `cmd.nav`, `cmd.mission`; the flight core picks by priority
   and age; formulas with topic inputs and outputs.
5. **Retire the special frames** one by one (`RN_LINK_NAV` first), once the bus carries the same data at the same rate
   and the link tests pass on it.

## Checks

`runner/fc/test_bus.c` (run by `runner/test.sh` and the boards CI job, with the address and undefined-behaviour
sanitizers): publishing, sequence and age, watching, the limits (names, sizes, the pool), one writer, subscriptions
lapsing, two boards asking for the same topic, sizes that don't match, malformed frames, and two buses copying topics
through their frames with periods, on change and the heartbeat. `tools/test_bus_browser.cjs`: the simulated boards
publish while flying, the copies come at the rates asked for (the state on change), a program's own topic travels and
another program can't overwrite the flight core's, and Live data shows and refreshes it all.

A bus is 8.6 KB per board. Publishing adds a copy of about 60 floats to each 1 ms flight-core step.
