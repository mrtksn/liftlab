# Apps

Code of your own in C or Python, on the boards that run apps, talking to everything else through the
[data bus](topic-bus.md). Formulas stay what flies the drone; apps are for what formulas can't do: protocols, drivers,
logic that wants C, libraries on the Pi.

Status: the board settings, the app manager, compiling C to WebAssembly in the browser and running WebAssembly apps on
the simulated boards are done. On the drone, the boards' app hosts (the ESP32 app-board firmware with a WebAssembly
runtime, and on the Pi the WebAssembly runtime, the native and Python app host) are the next steps; until then apps
run in the simulator, and the install dialog gives their files.

## What a board runs

Each board's settings (Computers → a board) say what it runs:

| | What | Boards |
|---|---|---|
| **Formulas** | Its duties (flight core, navigation, learning, health supervisor) and formula programs, on the step runner | Any |
| **WebAssembly apps** | C apps compiled here, on the board's WebAssembly runtime | Any |
| **Native apps** | C built on the Pi (so it can use Linux's libraries), and Python | Linux computers |

A microcontroller runs formulas *or* WebAssembly apps (it decides the firmware it gets). A Linux computer runs any of
them together, so a Pi can keep its duties and run apps beside them. A board that runs formulas takes duties and
formula programs as before; one that runs apps takes apps, in its settings (**Apps on this board**: put one on it, take
one off, or **+ New app here**). An app is on one board at a time. A board can't stop running formulas while it has
duties or programs, nor stop running apps while it has them: it says which to move first.

Sensors and other devices can be wired to any board, an app board too: their readings are published on its bus
(`sensor.baro`, …), where its apps read them.

## The app manager

Computers → Tools → **App manager…**. An app has:

- a **name** and what it's **written in**: C · WebAssembly, C · native (Pi), or Python (Pi);
- a header like a program's: **when it runs** (when a topic it reads changes, or every so many milliseconds), the topics
  it **reads** (up to 8, from every topic the design has), the topic it **writes** (one under `user.`) and its
  **fields**;
- its **code**.

**Apply** checks the header and, for a WebAssembly app, compiles it: clang and lld built as WebAssembly
([YoWASP](https://yowasp.github.io/)), fetched from the CDN the first time (about 25 MB, then cached) and run in a
worker, so compiling takes about a second and the page keeps flying. What the compiler says, with the line, shows under
the code. The compiled module is saved with the design, so a reload or a design file doesn't compile again.

## Writing a C app

The header becomes `liftlab.h` (shown under the code, and downloaded with it): `struct inputs`, the topics it reads,
each by the last part of its name (two alike: the whole name, `.` as `_`), their fields by their layouts; and
`struct output`, the fields of the topic it writes.

```c
#include "liftlab.h"

static float h0; static int n;

void setup(void) {                  /* once, when its board starts (optional) */
  printf("altFilter up\n");
}

/* when it's due: read in->…, fill out->…; return 1 to publish out, 0 not to */
int step(const struct inputs *in, struct output *out, float dt) {
  if (!n++) h0 = in->baro.height;
  float h = h0 + 0.2f * (in->baro.height - h0);
  out->rate = (h - h0) / dt; out->height = h0 = h;
  return 1;
}
```

`static` variables are its memory between runs. The C library is there (`math.h`, `string.h`, `printf`/`snprintf`,
`malloc`); `printf` goes to the board's log (the app manager shows it); `board_time()` is the board's clock in seconds.
Nothing else from the board: no files, no network. Hardware calls (UART, I²C, SPI, GPIO, ADC, PWM) come with the
boards' app hosts.

The board calls `step` as it calls a formula program (`runner/fc/prog_core.c`, the same code on the boards): when the
trigger topic changes or every period, once every topic it reads has a value, with `dt` the time since its last run.

## Safe by construction

A WebAssembly app can only touch its own memory (128 KB, up to 256 KB). Its loops are limited: before it's saved, the
module is rewritten (`js/wasm-meter.js`) to count down a budget at the top of every loop, which the board sets before
each call; a loop that runs out traps. So a bad pointer, a stack overflow or a loop that never ends stops that run
instead of the board, or the page. A run that stops publishes nothing, is counted, and the app starts again (`setup`,
its memory fresh), while the drone flies on. The module may ask its board only for its clock and what `printf` needs;
one that asks for more (`fopen`, say) is refused when it's compiled. The same metered module runs in the simulator and
will run on the boards.

**Editing the code** of a WebAssembly app compiles it and loads it in flight (it starts fresh). **Changing its header**
(what it reads or writes, when it runs, what it's written in) or the board it's on **restarts the flight**, as wiring
changes do.

## Python and native C on the Pi

They have the same header and the same shape: native C uses the same `liftlab.h`, compiled on the Pi itself (so it can
include Linux's libraries), and a Python app has

```python
def setup():
    print("camPy started")

def step(inp, out, dt):      # inp.alt.height, …; out.v = …
    out.v = 0
    return True              # publish out
```

The Pi's app host will run them beside `dfb_pi`, each in its own process, so one that crashes or hangs only lets its
topic go stale. They aren't simulated (a Python runtime in the page is a later option); their topics show in the
catalog, marked so, and readers wait for them.

## On the drone (next)

- **The Pi:** WAMR (the Bytecode Alliance's WebAssembly runtime, which runs on 32- and 64-bit ARM) in an app process
  beside `dfb_pi`, connected to its bus; the native and Python app host; apps sent with the board's install, no rebuild.
- **The ESP32 app board:** firmware with WAMR, the bus frames, its serial link to the flight controller, and apps stored
  in flash and loaded at start. WAMR's interpreter is about 60 KB of flash, an app about 100 KB of memory, and it runs
  roughly ten times slower than native C, plenty for filters, parsers and state machines.

## Checks

- `tools/test_apps_browser.cjs`: an ESP32 set to run apps with the barometer on it, a C app on it and one on the Pi
  (beside its duties) written in the app manager, compiled in the browser, run through `prog_core.c`, the first's topic
  relayed to the second through the flight controller; `printf` in the log; a loop that never ends stopped, counted and
  started again while the drone flies on; code reloads in flight, header changes restart; what's refused (names,
  topics, C words, compile errors with their lines, modules asking for what boards lack, programs on app boards);
  a Python app not simulated; the install dialog; undo, delete, design files and reload (from the saved modules).
  `APP_CLANG_DIR=<the @yowasp/clang package>` serves the compiler from disk.
- `tools/test_wasm_meter.js` (CI): the step limit on hand-made modules.
- `runner/fc/test_prog.c` (CI, sanitizers): apps through `prog_core.c` with a stand-in app host: triggers, inputs and
  `dt`, publishing only when the app says so, traps counted, inputs checked against what the app was compiled for.
