# Map navigation

The drone flies on what it measures itself (the flight core's attitude, the downward camera's flow, the rangefinder,
the barometer) and on what a **mapper** tells it: a **map** of the space around it and **position fixes** in that
map. The mapper is whatever can afford the heavy work: a computer on the ground running Room Survey, or the
simulator. The drone only ever gets two small things from it, in one format, by any route.

Nothing on the drone depends on the mapper being there. Without a map it flies as before. Without fixes its position
drifts as before (flow only). A map that stops changing stays the last one it had.

## On the drone: what it is given

Both arrive as text commands (the radio's text, the Pi's command port, or the simulator's link), so the same lines
work over every link:

- **The map** (`map_core.h`): a grid of the space in the map's own axes. Each cell is *unknown*, *free* or
  *occupied*. It comes as a file (`file NAME SIZE CRC` / `fd` / `file end`, `file_rx.h`) named `map.dmp`; the
  navigation takes it when it is whole (`dfb_pi` also loads `map.dmp` from its folder when it starts). Format: magic
  `DFMP`, version, resolution [m], origin [m] (the corner of cell 0,0,0), size in cells nx ny nz, then runs of cells
  (a kind and a length: unknown, free or occupied cells, or cells as the row or the layer before, since walls and
  floors repeat), then a CRC-32. A room or the parkour city at 11 cm is a few hundred bytes; the file link takes 32 KB.
- **A fix**: `mapfix T X Y Z PSI [SIGMA]`: where the drone's hub was at time T [s], in the map's axes [m]. T is on
  the flight core's clock (`dfb_pi`: the clock its NAV frames carry; the simulator: the board's), or, at zero or
  below, that many seconds ago. PSI [rad] is the turn from the map's axes to the navigation's (the flight core's
  heading has no north: it starts where the drone points and drifts, and the mapper sees both). SIGMA [m], how sure.
  The first fix anchors the map to the navigation's estimate (nothing jumps); after it each fix corrects the
  estimate, as a GPS fix does (the estimator's fix without a velocity, or with the GPS's velocity when there is a GPS),
  so the flow's drift goes. `map fuse off`: fixes only place the drone in the map.
- `mapguard RADIUS [V_UNKNOWN]`: the drone's radius with its margin (the occupied cells are grown by it) and its speed
  near the unknown; `map`: how the map and the fixes stand; `map clear`.

The navigation keeps the map in the map's axes and turns points by PSI to look them up, so a new PSI needs no new
map.

## On the drone: what it does with them

- **The guard** (in `nav_step`, for every pilot: sticks, text commands, return home, the fleet program): the target is
  cut short so the drone stops before an occupied (grown) cell, and goes at most half a metre into unknown space;
  toward what's mapped it flies no faster than it can stop in the room left (1 m/s² of braking), and near the unknown
  slowly. A pilot's own target (the radio's, a text command's) waits where the guard stopped it. Not while it lands
  (the floor is in the map too), and not without a fix for 10 s (where it is in the map is no longer known).
- Later: **routes** for go-to and return home around what the map shows (a search on the grid).

## Mappers

- **The simulator, from the World**: the World's boxes (the city, imported objects, a survey brought in as a model)
  made into a map, all at once ("the map is known") or as its cameras see it (each camera part's view, ray by ray,
  marks cells free and occupied: "it explores"). Fixes from where the drone truly is, with the noise, rate, delay and
  dropouts set. Both go up the simulated radio when it lands on the navigation's board (a lost link loses them),
  otherwise straight to that board (as Room Survey would reach the Pi over its Wi-Fi).
- **Room Survey**: its reconstruction made into the same map file, its localisation of the downward camera's frames
  (stamped on the drone's clock) as fixes, sent through the Wi-Fi bridge or straight to the Pi's command port.

Which cameras a mapper uses, and where they are, is the design's: any camera part on the frame, any computer that
reads it. The drone's side doesn't know or care.

## Status

| Piece | State |
| --- | --- |
| Map format, load, look-ups (`map_core.c`, `test_map.c`) | built |
| The guard in the navigation | built |
| Fixes anchor the map and correct the position estimate | built |
| `dfb_pi`: `map.dmp` and the map commands | built |
| Simulator: map from the World, sent up the radio or to the board, fixes with noise, delay and dropouts (World settings → The navigation's map) | built |
| Simulator: map from what the cameras see (unknown until seen) | next |
| Room Survey: map file and fixes | later |
| Routes around obstacles | later |

Checked in the simulator (open World; fixes 5 a second, 3 cm noise, 0.2 s delay):
- A wall 3 m ahead, told to go 6 m: the toy drone on its flow camera stops at 2.59 m (its radius 0.39 m and the
  margin; without the map it flies into the wall); a GPS quad at 2.77 m.
- A minute's hover: the toy drone's estimate within 0.08 m of where it is (0.17 m without fixes), held within 0.24 m.
  A GPS quad's hold in the simulator is poor with or without the map (it circles, a few metres over the minute; the
  nav test's GPS hold fails the same way): the fixes, at the estimator's gains (0.8/s on position), only take some of
  it out.
- The link cut while hovering: the fixes stop reaching it, it flies home and lands (the guard stands down after 10 s
  without a fix).
