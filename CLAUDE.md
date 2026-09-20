# brew-utils

Control software for a Knight-Rider-themed brewery dashboard. Two Raspberry Pis
drive real KITT dash replica hardware (bought from Paolo at ideegeniali) showing
mash/HLT temperatures, fermenter temps and Tilt specific gravities, serving
keezer probe temps, lager keg temps, and keg capacity from RaspberryPints.

The author's own write-up, with photos of what every gauge means, is the best
orientation for anything user-facing:
https://www.homebrewtalk.com/threads/brewery-dashboard.726917/
(WebFetch gets a 403 from that host; the in-app browser loads it fine.)

## Layout

| Path | Status | What it is |
| --- | --- | --- |
| `kitt/panp/` | **live** | The production system. `panp.py`, its systemd units, and a README on setting up the Hue bench light. |
| `kitt/switchpod/` | live | Arduino Uno firmware for the resistive keypads. |
| `brewpitosmith` | utility | BrewPi Remix beer log → BeerSmith 3 CSV. |
| `images/` | reference | The `dash-*.jpg` photographs of the lit dash used by `kitt/DASHBOARD.md`, switch pod photographs, the season 2 dash scan, and the `hw-*` photos of the bench, supply box and control panel used by `HARDWARE.md`. |
| `HARDWARE.md` | reference | The physical system, walked with the owner: power, fusing, every relay and barrier strip, pin tables for both Pis, all six Arduinos, the fermenter and glycol side. Its "Still to confirm" list is the only part not yet checked. |
| `schematics/` | reference | KiCad schematics of the wiring, **generated** by `schematics/ksheets.py` via `schematics/kicadgen.py` — edit the Python, not the `.kicad_sch`. Rendered SVGs are `images/schematic-*.svg`. Needs `kicad-cli` (KiCad 10 is installed in /Applications) to export and ERC-check. |

Only `kitt/` is worth changing unless asked otherwise.

Two dead efforts were archived out of the tree and live on tags only: an ESP32
+ TLC5940 rebuild of the dash electronics (`karr-prototype`) and the 7-segment
display that preceded the KITT dash (`pi-temp-display-final`). They were removed
because they described hardware that never ran, and kept being mistaken for
documentation of the live system. **Do not restore either one into the working
tree to answer a question** — read it at the tag instead:

```bash
git show karr-prototype:karr/tlc5940/src/main.c
```

## The important thing about panp.py

One ~1000-line script runs on **both** Pis and branches on
`socket.gethostname()`. Read any change twice, once as each host:

- **`rpints`** — owns the PANP buttons/lamps, dash power relays, and the GPIO
  lines that signal mode to the other Pi. Reads 6 keezer + 3 lager DS18B20s and
  the RaspberryPints MySQL database. Drives the tacho and the two dummy
  displays via `RPintsLoopHandler`. Also switches the **Hue light strip over
  the workbench** on in Pursuit and off in Norm and Auto (`HueLight`, over
  the Hue bridge's v2 API from a worker thread). That is configured by
  `/usr/local/etc/panp-hue.json`, which is not in the repo because it holds
  the bridge key; with no file the class does nothing, which is how `brewpi`
  runs. Setup is in `kitt/panp/README.md`.
- **`brewpi`** — reads mash/HLT probes, polls three BrewPi Remix instances over
  their `KITTSOCKET` unix sockets for temperature and Tilt SG, toggles the five
  flowmeter relays. Drives the speedo and message center via
  `BrewPiLoopHandler`. Receives mode changes from `rpints` as GPIO input.

Anything hostname-independent (`Service`, `Bus`, `BrightnessHandler`,
`read_probe_f`, `get_temps`, the pin and cadence constants at the top) is
shared by both. `main()` builds the per-host objects in `setup_rpints` and
`setup_brewpi`; there are no module-level globals to chase.

## The KITT serial protocol

Paolo Sancono's "KITT GLU" protocol. The spec, the board firmware and the
schematics live alongside this repo at `~/projects/ideegeniali`, which is not
public.

**Start with `~/projects/ideegeniali/README.md`.** It is the reference for the
protocol, the boards and their firmware, and it opens with what may and may not
be repeated outside that repo. Read it before answering any question about how a
board behaves, rather than guessing or reverse-engineering from `panp.py` — and
before writing anything here that describes the firmware.

**The PDF does not document the tacho and speedo registers this code uses.**
Those tables are blank in v8 because the feature did not exist — the stock
firmware could not set those displays at all, and the registers were added to it
in 2023 so the host could. For anything on boards `A` or `B`, the board firmware
in the private repo is the only authority; the PDF covers `C`, `E`, `F` and `G`.

Displays are slaves on two 57600-baud buses (`/dev/ttyAMA0`, `/dev/ttyAMA1`),
polled by the Pi as master. A master packet is five parts:

```
>  A  B  p  3C  ?
│  │  │  │  │   └─ end of message
│  │  │  │  └───── payload, hex byte pairs
│  │  │  └──────── register  (which value on that board)
│  │  └─────────── command   (B = write byte, H = write hex seq, S = write string)
│  └────────────── destination board
└───────────────── start of message
```

The third character is the **register, not part of the command** — `B` and `H`
mean the same thing on every board, and the register selects what gets set.
Slaves reply `<`…`!`, though `panp.py` never reads replies.

| Letter | Display | On host |
| --- | --- | --- |
| `A` | tacho (RPM digits + 7 bars, the 7th being the RPM circle) | rpints, `tacho_tx` |
| `B` | speedo (MPH digits, lower multifunction) | brewpi, `speedo_tx` |
| `C` | message center (text) | brewpi, `msgctr_tx` |
| `E` | red dummy3 (kegs 3–5) | rpints, `dummy_tx` |
| `F` | red/green dummy3 (fermenter temps) | brewpi, `speedo_tx` |
| `G` | dummy6 (lager temps, capacity) | rpints, `dummy_tx` |

Every string `panp.py` sends, and what the firmware does with it:

| Sent | Meaning |
| --- | --- |
| `>ABp{v}?` | tacho 7-seg value, and switches it to user mode |
| `>ABo01?` | tacho 7-seg mode 01 (show user value) |
| `>AHh{7 bytes}?` | user values for the tacho's **7** bars (6 keezer probes + RPM circle) |
| `>AHa{7 bytes}?` | modes for those 7 bars — `01` = show user value |
| `>BBc{v}?` / `>BBb{v}?` | speedo upper (speed) and lower (fuel) LED bargraphs |
| `>BHd{4 bytes}?` | speedo upper 4 digits |
| `>BHe{5 bytes}?` | speedo lower 5 digits — 5th is decimal-point position |
| `>E/F/GHm{n bytes}?` | dummy bar user values (3 bytes for E/F, 6 for G) |
| `>E/F/GHa{n bytes}?` | dummy bar modes — `00` light-play, `01` user value, `02` voltmeter, `03` fill-and-empty light-play |
| `>{board}BD{v}?` | master brightness, `00` full dim to `FF` full bright |
| `>CSc{text}?` | message center: flash text now, not stored |
| `>CSb{a\|b\|c}~?` | message center: overwrite the user message list (in EEPROM) |
| `>CBa{v}?` | message center: PC-override timeout, in **tenths of a second** |

Three things that are easy to get wrong:

- **`>CBa00?` does not mean "off".** It sets the override timeout to zero,
  i.e. *never expires*, so text sent with `Sc` stays up forever. `>CBa01?` sets
  a 0.1 s timeout, so the override lapses immediately and the board falls back
  to cycling its own message list. That is how `panp.py` switches the message
  center between a static caption and the auto rotation.
- **Hex writes must have an exact payload length** or the firmware silently
  ignores them: 4 byte-pairs for `BHd`, 5 for `BHe`, and at most one per bar for
  the bar registers. Adding a digit does nothing rather than erroring.
- **Writing bar values by `H` does not set the mode**, so the `Ha` mode write is
  genuinely required alongside `Hm`/`Hh`. (Per-bar *byte* writes do set mode
  automatically, which is why the two paths look inconsistent.)

Payload bytes are always two hex digits, hence the `"{:0>2X}"` formatting.

**Every serial write is preceded by a `SERIAL_GAP` pause, and only one packet
is ever in flight on a Pi.** The displays drop messages sent closer together
than the gap on one bus. Worse, a Pi's two buses are adjacent channels of one
TXS0108E level shifter on long cable runs, and a packet on one bus is
corrupted when the other bus is transmitting at the same instant. All writes
go through `Bus.write`, which takes a turn on the `Wire` shared by both buses,
sleeps `SERIAL_GAP`, then writes. Turns are first come first served (a ticket
queue, not a plain lock), so a burst from the GPIO callback thread cannot
hold the main loop off the bus. No call site needs its own sleep. Do not
write to a port any other way. The remaining explicit sleeps in the display
code are the deliberate one-second pauses: the flow-meter flash, and settling
the boards after the dash powers up.

This was learnt the hard way. A version with one lock per port, sleeping
inside it, lined the two threads up so that they wrote to both buses within a
millisecond of each other at every thread handover, which is exactly when the
one-shot tacho mode packet goes out after Auto. The tacho bars came up frozen
or still in play mode on every Auto exit. Never sleep inside a per-port lock.

### Every board writes its settings to EEPROM

This is the sharpest edge in the whole system, because the damage is silent and
cumulative. The boards save their settings to EEPROM, and some of them do it on
every write that changes a displayed value. Bytes that do not change are not
re-written, which is the only reason any of this is survivable — so the rule
is: **do not repeatedly send a message whose payload changes.**

- **Message centre.** Auto mode re-sends the whole `>CSb` list every iteration.
  Safe only because the text is constant. If you ever make it vary — a clock, a
  countdown, live temperatures — you will write EEPROM a few times a second and
  wear the board out. Use `>CSc` for anything that changes.
- **Dummy3 / dummy6.** Bar *values* are free; a bar *mode* write (`>?Ha`) is
  not, and it persists the current values as a side effect. So mode writes must
  be occasional — see `dummy3_user_mode` in `BrewPiLoopHandler`, which
  re-asserts the mode once per dash power-up instead of once per pass.
- **Tacho and speedo.** Every value write used to save, including the speedo's
  lower display, whose tenths digit changes on nearly every update — roughly one
  EEPROM write per second with the dash lit. Fixed in the board firmware in
  September 2026 by holding displayed values in RAM; nothing on the Pi side
  could have avoided it, because those values genuinely change. **The fix only
  applies to a board once its ATmega has been reflashed**, so check that before
  assuming a board is safe.

The memory being worn is **inside the ATmega328**, which sits in a socket, and
the boards carry no external EEPROM. So a worn-out board is repaired by swapping
a DIP chip, not by reworking the board, and the soldered parts store nothing and
are never at risk.

There is a second-order cost. An EEPROM write blocks the AVR for about 3.3 ms,
and at 57600 baud the 64-byte UART buffer overflows in 11.1 ms, so several
changed bytes in one message can drop characters mid-packet. The
`time.sleep(0.1)` before every serial write is very likely paying for this.

Specifics — which call saves and which does not — are documented in the private
repo's README. **Keep them there:** that source is confidential to Paolo, and
the commitment covers fragments of it as well as the whole.

## Conventions to preserve

- **Broad `except Exception: pass` is deliberate.** This is an unattended
  daemon behind `Restart=no`; a probe read failing or the database being down
  must not kill the loop. Don't convert them to narrow handlers without asking,
  and don't widen them back to a bare `except:`, which would also swallow the
  `SystemExit` that shutdown relies on.
- `PortNotOpenError` is re-raised only while `service.running` is true, so
  that shutdown races stay quiet. Keep that pattern in new display code.
- Temperature-to-bar mapping is done with parallel threshold/value lists at
  module level (`TACHO_BAR_TEMPS`, `LAGER_BAR_TEMPS`, `KEG_BAR_VOLS`,
  `TEMPERATURE_BAR_TEMPS`, `RPM_CIRCLE_TEMPS`) walked by one `step()` helper:
  `for i, t in enumerate(...): if value < t: break`. Falling off the end
  clamps to the top value on purpose. Tuning the dash means editing those
  lists — most recent commits are exactly that. The lookup functions keep
  their old names (`tacho_bar`, `lager_bar`, `keg_bar`, `temperature_bar`,
  `rpm_circle`).
- systemd integration is `Type=notify` via sdnotify. New long-running work
  should report through `service.status(...)`, which prints and notifies.
- Background work is a daemon `Thread` feeding a `Queue` of `(index, value)`
  tuples, drained non-destructively at the top of `loop()` with `drain(q)`,
  which is the `while q.qsize() > 0` idiom in one place.
- Button and mode dispatch is table-driven (`RIGHT_POD`, `LEFT_POD_*`,
  `LOWER_DISPLAY`, `PANP_OUTPUTS`). The tables mirror the button tables in
  kitt/DASHBOARD.md; change both together.

## Verifying changes

There is no build, no test suite, and no dependency manifest. The runtime
imports (`RPi.GPIO`, `serial`, `sdnotify`, `mysql.connector`) only exist on
the Pis, so **the code cannot be imported or run on a dev machine.** (`pigpio`
belonged to the archived 7-segment display and is not imported any more.)

What you can do locally:

```bash
python3 -m py_compile kitt/panp/panp.py
```

Beyond that, changes are verified by deploying and watching the dash. Be
conservative: a bad edit means physically walking to the brewery.

Deployment is manual — `panp.py` is copied to `/usr/local/sbin/panp.py` and the
units to `/usr/local/lib/systemd/system/` on each Pi. Most of the
`.service`/`.path`/`.sh` files carry a header comment recording where they
belong; the two `power-relay-*.service` files do not.

## Known rough edges

- `power-relay-brewpi.service` is described as "Open RPints Power Relay" —
  copy-paste from the rpints unit, cosmetic only.
- The fermenter-state message-center feature (`IDLE`/`COOL`/`HEAT` in the
  rotation) was commented out for years and its never-drained queue was
  removed in the cleanup. The old code is in git history:
  `git show 9f49075:kitt/panp/panp.py`, around the `brewpi_rmx_state_q`
  block.
- `panp.path` fires when **any** of its three `PathExists=` conditions holds,
  not all of them. The switch pod thread copes with the adapter being absent,
  so this is cosmetic, but do not describe the unit as waiting for all three.
- `KITTSOCKET` exists only on the `kitt` branch of the brewpi-script-rmx fork,
  which the Pi runs and which is ten commits behind that fork's `main`. It lacks
  main's 2022 tilt-colour-expiry fix; unitank-2 has that fix hand-patched and
  uncommitted on the Pi. See "The BrewPi side" in `kitt/README.md`.

## Git

Single `master` branch, pushed to `git@github.com:duncan-brown/brew-utils`.
Commit messages are short and lowercase ("tweak red/green fermenter temps",
"restart switchpod on error"). No PR workflow.
