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
| `kitt/panp/` | **live** | The production system. `panp.py` + systemd units. |
| `kitt/switchpod/` | live | Arduino Uno firmware for the resistive keypads. |
| `karr/` | dormant prototype | Abandoned ESP32 + TLC5940 reimplementation. |
| `pi-temp-display/` | superseded | The 7-segment predecessor to the KITT dash. |
| `stattosmith`, `brewpitosmith` | legacy | One-shot CSV converters, **Python 2**. |

Only `kitt/` is worth changing unless asked otherwise.

## The important thing about panp.py

One ~1000-line script runs on **both** Pis and branches on
`socket.gethostname()`. Read any change twice, once as each host:

- **`rpints`** — owns the PANP buttons/lamps, dash power relays, and the GPIO
  lines that signal mode to the other Pi. Reads 6 keezer + 3 lager DS18B20s and
  the RaspberryPints MySQL database. Drives the tacho and the two dummy
  displays via `RPintsLoopHandler`.
- **`brewpi`** — reads mash/HLT probes, polls three BrewPi Remix instances over
  their `KITTSOCKET` unix sockets for temperature and Tilt SG, toggles the five
  flowmeter relays. Drives the speedo and message center via
  `BrewPiLoopHandler`. Receives mode changes from `rpints` as GPIO input.

Anything hostname-independent (`BrightnessHandler`, `TempProbe`, `get_temps`,
the GPIO pin constants at the top) is shared by both.

## The KITT serial protocol

Displays sit on two 57600-baud serial buses (`/dev/ttyAMA0`, `/dev/ttyAMA1`),
addressed by a letter. Messages are `>` + device letter + command + hex payload
+ `?`.

| Letter | Display | On host |
| --- | --- | --- |
| `A` | tacho (RPM digits, 6 bars, RPM circle) | rpints, `tacho_tx` |
| `B` | speedo (MPH digits, lower multifunction) | brewpi, `speedo_tx` |
| `C` | message center (text) | brewpi, `msgctr_tx` |
| `E` | red dummy3 (kegs 3–5) | rpints, `dummy_tx` |
| `F` | red/green dummy3 (fermenter temps) | brewpi, `speedo_tx` |
| `G` | dummy6 (lager temps, capacity) | rpints, `dummy_tx` |

Observed commands: `Ha`/`Hm`/`Hh` set bar-graph modes and values, `Bp`/`Bb`/`Bc`
and `Hd`/`He` set digits and LED counts, `Sb`/`Sc` set scrolling and static
message-center text, `Ba00`/`Ba01` switch the message center between user and
auto text, `BD` sets brightness. Payload bytes are two-hex-digit values, so
formatting is always `"{:0>2X}"`.

**Every serial write is preceded by `time.sleep(0.1)`.** The displays drop
messages without it. Do not "clean up" those sleeps.

## Conventions to preserve

- **Bare `except: pass` is deliberate.** This is an unattended daemon behind
  `Restart=no`; a probe read failing or the database being down must not kill
  the loop. Don't convert them to narrow handlers without asking.
- `PortNotOpenError` is re-raised only when `run_loop` is still true, so that
  shutdown races stay quiet. Keep that pattern in new display code.
- Temperature-to-bar mapping is done with parallel threshold/value lists and
  `for i, t in enumerate(...): if temp < t: break`. Falling off the end clamps
  to the top value on purpose. Tuning the dash means editing those lists
  (`tacho_bar`, `lager_bar`, `keg_bar`, `temperature_bar`, `rpm_circle`) — most
  recent commits are exactly that.
- systemd integration is `Type=notify` via sdnotify. New long-running work
  should `n.notify("STATUS=...")` alongside its `print`.
- Background work is a daemon `Thread` feeding a `Queue`, drained
  non-destructively at the top of `loop()` with `while q.qsize() > 0`.

## Verifying changes

There is no build, no test suite, and no dependency manifest. The runtime
imports (`RPi.GPIO`, `serial`, `sdnotify`, `mysql.connector`, `pigpio`) only
exist on the Pis, so **the code cannot be imported or run on a dev machine.**

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

- `stattosmith` and `brewpitosmith` are Python 2 (`print` statement) with a
  `#!/usr/bin/python` shebang; they will not run as-is on a current system.
- `power-relay-brewpi.service` is described as "Open RPints Power Relay" —
  copy-paste from the rpints unit, cosmetic only.
- The fermenter-state message-center code in `BrewPiLoopHandler.loop` is
  commented out (see the `brewpi_rmx_state_q` block); the queue is still fed.
- `karr/tlc5940` creates a FreeRTOS task directly from a timer ISR, which is not
  ISR-safe. It is prototype code and does not run in production.

## Git

Single `master` branch, pushed to `git@github.com:duncan-brown/brew-utils`.
Commit messages are short and lowercase ("tweak red/green fermenter temps",
"restart switchpod on error"). No PR workflow.
