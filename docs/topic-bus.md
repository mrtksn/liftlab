# The data bus: topics every program reads and writes

Status: the store, copying between boards, layouts, relaying and programs are implemented in the C library and the
simulator; real boards, external apps and command arbitration are the next steps (see [Migration](#migration)).

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

A topic is a named record of up to 32 floats, laid out as named fields (its **layout**: `q[4] w[3]` is a 4-float
field `q` then a 3-float field `w`; a field without a count is one float), with:

- **one writer** (a program on one board) — nobody else may write it;
- a **sequence number** that goes up by one at each publish, so a reader can tell "new" from "same as before";
- the **time** it was published (the board's clock), so a reader can tell how old it is;
- on another board, a **mirror** of it: the same values, sequence and age, read-only there.

Names are dotted paths: `fc.attitude`, `nav.estimate`, `cmd.pilot`, `sensor.baro`, `user.camera.photo`. A board's
bus holds at most 48 topics and 1024 floats (11.7 KB in all): small enough for an ESP32, and fixed, so nothing
allocates in flight. Who may write what: the flight code its own (`fc.`, `nav.`, `cmd.`), the sensor drivers `sensor.`,
programs and external apps only `user.`.

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

Each sensor publishes its raw readings (its own axes, as its driver gives them) on the board it is wired to:
`sensor.imu` (`gyro[3] accel[3]`), `sensor.mag` (`field[3]`), `sensor.baro` (`height`), `sensor.fix` (`p[3] v[3]`),
`sensor.flow` (`flow[2] range quality`); a second sensor of a kind gets a number (`sensor.imu2`).

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

**Relaying.** The boards are linked in a star around the flight controller. A board may serve a copy it holds itself,
so a topic written on one board reaches a board it isn't linked to through the flight controller, which asks for it
too; the copy keeps its age. Who asks for what is worked out from the design: the built-in reads (the navigation's
board follows `fc.state` and `cmd.pilot` on change, `fc.attitude` every 20 ms and `fc.height` every 50 ms; the flight
controller follows `nav.estimate` and `nav.command` every 50 ms), and every program's reads.

**In the simulator** each simulated board has its own bus and copies topics with the same frames and the link's
delay. **Computers → Live data…** shows every board's topics live: a card per board; each topic with where it comes
from, its age, how many updates a second arrive *on that board*, and its values (named from its layout on hover).

The receiving board sets the mirror's time to *now − age*: its own clock, less how old the value was when it left.
The link's own delay (6 ms in the simulator's model) is not added: the boards' clocks aren't synchronized, so the
receiver can't measure it. A copy's age is therefore "at least this old".

## Programs

A **program** is a formula of your own on a board of your choice. In the Formula editor, **+ New program** gives a
template and a header:

- **Name** (a word: it is the formula's name) and **Board** (any board: the flight controller, the Pi, an ESP32 that
  only has sensors);
- **Runs**: when a topic it reads changes, or every so many milliseconds;
- **Reads**: up to 8 topics, picked from every topic the design has (grouped by the board that writes them);
- **Writes**: one topic under `user.`, and its fields (`range rate ok`, `v[3]`).

```js
function altFilter(st, inp, dt) {     // runs on Sensor board when sensor.baro changes
  if (st.h == null) st.h = inp.baro.height;
  const h = st.h + 0.2 * (inp.baro.height - st.h), rate = (h - st.h) / dt;
  st.h = h;
  return { height: h, rate };          // → user.alt (height rate)
}
```

`st` is its own memory between runs, `inp` the topics it reads (each by the last part of its name, `inp.baro`; two
alike by the whole name with `_`), their fields by their layouts, and `dt` the time since its last run. The editor
shows exactly what `inp` holds and what to return. It's the same JavaScript subset as the formulas, compiled into its
board's flight program and run there by `runner/fc/prog_core.c`: sandboxed (the step runner checks every address and
step), self-tested when loaded, and the same code in the simulator as on a board. A program waits until every topic it
reads has a value; a run that traps or returns a number that isn't finite publishes nothing and is counted.

Programs are part of the design (saved, shared, undone with it). The board cards list each board's programs; Live data
shows what they publish. **Editing a program's code reloads it in flight**, checked and flown in the background first
like any formula. **Changing its header** (what it reads or writes, when it runs, its board) **restarts the flight**,
as wiring changes do: the board's loader matches every program it loads against the one it started with, function by
function.

That loader rule is also why programs don't run on real boards yet: a board's built-in program is compiled into its
firmware, and a program it doesn't have can't be matched. The loader needs to accept functions the built-in program
lacks (step 4).

## Apps outside LiftLab (native C, Python)

Step 3. A board running Linux (the Pi) offers its bus on a local socket (`/run/liftlab/bus.sock`), with the same
frames the boards use plus *publish* and *list* (each topic's name, size and layout, so an app adapts to the design
loaded). A one-header C client (`liftlab_bus.h`) and a Python client speak it:

```c
lb_conn *c = lb_connect("/run/liftlab/bus.sock");
lb_subscribe(c, "user.alt", 0);                       /* 0: on change */
for (lb_msg m; lb_read(c, &m) == 0;)
  if (lb_is(&m, "user.alt") && m.v[1] > 2.0f) { float why[1] = { m.v[1] }; lb_publish(c, "user.alert", why, 1); }
```

Such an app is built and installed separately (a binary and a systemd service, say). If it crashes or hangs, nothing
that flies waits for it, and its topics just go stale. It may publish only under `user.` (or its own `app.<name>.`).
The socket is a file with permissions, not an open network port. A bridge on your computer offers the same socket for
the simulator, so the same app runs against both.

On an ESP32 there is no operating system to install a second program on: native C there is a **user component**
compiled into the firmware (ESP-IDF), with the same calls in-process, in a task below the flight loop's priority. It has
no memory protection, so keep native code off the flight controller: use programs there, and put native code on a
separate sensor board or the Pi.

## Migration

Each step ships something useful and keeps flying as it did.

1. **The store and copying between boards** — done in the C library and the simulator: topics, publish, read, watch,
   ages; the `RN_LINK_BUS_SUB` and `RN_LINK_BUS` frames; the flight core and the navigation publish what they compute;
   Live data.
2. **Layouts, relaying, sensor topics** — done in the C library and the simulator.
3. **Programs** — done in the simulator: the header, the editor, `prog_core.c` (shared with the boards), routing from
   the reads, reload in flight.
4. **Real boards.** `dfb_pi` runs the bus, the two frames and programs; the ESP32 firmware answers the frames
   (ESP-IDF rebuild); a sensor/program-board role for a second ESP32 and its serial link to the flight controller
   (simulated only today); the loader accepts programs the built-in program doesn't have.
5. **Apps outside LiftLab**: the socket, `liftlab_bus.h`, the Python client, the simulator bridge, user components.
6. **Commands as topics, with arbitration**: `cmd.radio`, `cmd.nav`, `cmd.mission`; the flight core picks by priority
   and age; a mission is a program writing `cmd.mission`.
7. **Retire the special frames** one by one (`RN_LINK_NAV` first), once the bus carries the same data at the same rate
   and the link tests pass on it.

## Checks

`runner/fc/test_bus.c` (run by `runner/test.sh` and the boards CI job, with the address and undefined-behaviour
sanitizers): layouts, relaying through a third board, publishing, sequence and age, watching, the limits (names, sizes, the pool), one writer, subscriptions
lapsing, two boards asking for the same topic, sizes that don't match, malformed frames, and two buses copying topics
through their frames with periods, on change and the heartbeat. `tools/test_bus_browser.cjs`: the simulated boards
publish while flying, the copies come at the rates asked for (the state on change), a program's own topic travels and
another program can't overwrite the flight core's, and Live data shows and refreshes it all.

A bus is 8.6 KB per board. Publishing adds a copy of about 60 floats to each 1 ms flight-core step.

`runner/fc/test_prog.c` (with `tools/prog_test_data.js`, in `runner/test.sh` and CI): headers checked against the
compiled formulas, running on a change and by period, waiting for inputs, memory kept, results published, failures
counted. `tools/test_programs_browser.cjs`: two programs written in the editor on two boards (a sensor board and the
Pi, through the flight controller), their results as their code says, what's refused and why, code edits reloading in
flight and header edits restarting, undo, delete, design files and reload.
