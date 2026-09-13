# kitt

The software that drives the KITT dashboard. Two pieces:

- [`panp/panp.py`](panp/panp.py) — the daemon that runs on both Raspberry Pis
  and puts brewery data onto the dash gauges.
- [`switchpod/`](switchpod/) — Arduino firmware that reads the dash keypads.
  How a pod reaches the Pi is in [switchpod/README.md](switchpod/README.md).

PANP stands for Power / Auto / Norm / Pursuit, the four buttons on the dash.

This file is about how the software works. For what each gauge on the dash
means — KITT's labels against the brewery values behind them — see
[DASHBOARD.md](DASHBOARD.md).

## One script, two Pis

`panp.py` is installed identically on both machines and decides what it is at
startup from `socket.gethostname()`. Anything else exits with an error.

| | `rpints` | `brewpi` |
| --- | --- | --- |
| Loop class | `RPintsLoopHandler` | `BrewPiLoopHandler` |
| Reads | 6 keezer + 3 lager probes, RaspberryPints MySQL | mash + HLT probes, 3 BrewPi Remix instances |
| Drives | tacho, red dummy3, dummy6 | speedo, message center, red/green dummy3 |
| Owns | PANP buttons and lamps, dash power relays | flow meter relays |
| Mode GPIO | output (tells `brewpi` the mode) | input (follows `rpints`) |

Shared between them: `TempProbe` and the `get_temps` thread, `BrightnessHandler`,
the pin constants at the top of the file, and the systemd lifecycle.

## Structure

Everything slow runs in a daemon thread and communicates with the main loop
through a `Queue`. The main loop never blocks on I/O.

```
  ┌────────────────────┐
  │ get_switchpod()    │  reads /dev/switchpod, one digit per keypress
  ├────────────────────┤
  │ get_temps()        │  polls one-wire probes once a second
  ├────────────────────┤        ┌───────────┐      ┌──────────────┐
  │ get_brewpi_rmx_... │───────▶│  Queues   │─────▶│ loop_handler │────▶ serial
  └────────────────────┘        └───────────┘      │  .loop()     │      to dash
                                                   └──────────────┘
                                                    every 0.25 s
```

Each `loop()` call drains its queues with `while q.qsize() > 0`, recomputes what
should be on screen, and writes it out. Queue entries are `"index,value"`
strings, split and cast on arrival.

The database is polled less often than everything else: `RPintsLoopHandler`
keeps a `database_clicks` counter and only queries RaspberryPints on every tenth
pass.

## The PANP modes

Pressing a PANP button on `rpints` moves power relays and flips the two GPIO
lines that `brewpi` watches, so both Pis change behaviour together.

| Mode | Dash | Message center |
| --- | --- | --- |
| **Auto** | off (relays cut power to upper and lower dash) | stays on, showing `BREWPI UP` and flow meter status |
| **Norm** | on, dimmed | captions the lower display |
| **Pursuit** | on, full brightness | captions the lower display |

Leaving Auto clears every display and re-sends brightness, because the boards
were unpowered and have lost their state.

The Power button is not software at all: it is a relay held closed by each Pi
while it is up, so the dash cannot be switched off without halting both Pis
first. That is `power-relay-*.service` (closes the relay at boot) and
`power-off-*.sh` (drops it during shutdown).

## Switch pod buttons

Each switch pod is a resistive keypad read by an Arduino, which prints a
position 0–9 over serial; `panp.py` reads them from `/dev/switchpod` in
`get_switchpod()` and queues them for the loop handler.

Even positions are the pod's left column, odd positions the right — which is
why each handler treats `sp_val` 0/2/4/6/8 as one family and 1/3/5/7/9 as
another. The **right** pod (`rpints`) selects which temperature the tacho digits
show; the **left** pod (`brewpi`) selects what the lower speedo display shows
and toggles the flow meter relays. Toggling a flow meter flashes `FLOW n on` on
the message center for a second, then restores the normal caption.

Full button-by-button tables, with photographs of both pods, are in
[DASHBOARD.md](DASHBOARD.md).

## Talking to the dash

The boards are serial slaves at 57600 baud on `/dev/ttyAMA0` and `/dev/ttyAMA1`,
addressed by a letter — `A` tacho, `B` speedo, `C` message center, `E`/`F`
dummy3, `G` dummy6. A command looks like:

```
>ABp3C?
 │││ └── payload: hex byte pairs
 ││└──── register: which value on that board
 │└───── command: B write byte, H write hex sequence, S write string
 └────── destination board
```

so `>ABp3C?` means "tacho, write byte, register p (the seven-segment value),
0x3C". Values are scaled to bytes before sending — `tacho_bar`, `lager_bar`,
`keg_bar` and `temperature_bar` map a temperature or a percentage onto the
bargraph steps by walking a list of thresholds, which is where the dash gets
tuned.

The protocol is Paolo Sancono's and the full specification comes with the
boards — but note that the tacho and speedo registers this code writes are
**not in that specification**. The shipped firmware could only set those
displays from the car's own inputs, so in 2023 they were added to Paolo's
parser: user-value registers for the tacho's bars and seven-segment, and for
the speedo's two bargraphs and both digit groups. Paolo has the modified
firmware and is free to ship it. When one of these commands needs checking, the
board's `parser.ino` is the authority, not the protocol PDF.

Two things worth knowing before editing the serial code:

- **Every write is preceded by `time.sleep(0.1)`.** Without the gap the boards
  drop messages.
- **Hex payloads must be exactly the right length** — four byte pairs for the
  speedo's upper digits, five for the lower, one per bar for the bargraphs.
  A payload of the wrong length is ignored silently rather than rejected.

## Installation

There is nothing to build. On each Pi:

```bash
sudo install -m 755 panp/panp.py /usr/local/sbin/panp.py
sudo install -m 644 panp/panp.service panp/panp.path /usr/local/lib/systemd/system/
sudo install -m 644 panp/power-relay-<host>.service /usr/local/lib/systemd/system/
sudo install -m 755 panp/power-off-<host>.sh /lib/systemd/system-shutdown/
sudo systemctl enable panp.path power-relay-<host>.service
```

`panp.path` starts the service only once `/dev/switchpod`, `/dev/ttyAMA0` and
`/dev/ttyAMA1` all exist, so the daemon never comes up before its hardware. The
service is `Type=notify` and reports progress through sdnotify, so
`systemctl status panp` shows what it is doing.

Requires Python 3 with `pyserial`, `sdnotify`, `RPi.GPIO` and
`mysql-connector-python`. The switch pod's serial adapter needs to appear as
`/dev/switchpod`, which takes a udev rule — not included here.

## Notes

- `RPintsLoopHandler` computes `keezer_min`, `keezer_max` and `keezer_median`
  every pass, but no switch pod button currently selects them — only the
  individual probes, the lagers and the mean are reachable.
- The code that cycled fermenter status (`IDLE`/`COOL`/`HEAT`) on the message
  center is commented out, though the queue feeding it is still filled. In Auto
  the message center shows `BREWPI UP` instead.
- Exception handling is deliberately broad. This is an unattended daemon with
  `Restart=no`; a probe that fails to read or a database that is down must not
  take the dashboard down with it.
