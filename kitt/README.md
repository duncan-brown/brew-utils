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
keeps a `database_clicks` counter and only queries RaspberryPints once every
eleven passes (the counter runs 0–10, and only 0 queries).

## The PANP modes

Pressing a PANP button on `rpints` moves power relays and flips the two GPIO
lines that `brewpi` watches, so both Pis change behaviour together.

| Mode | Dash | Message center |
| --- | --- | --- |
| **Auto** | off (relays cut power to upper and lower dash) | stays on, showing `BREWPI UP` and flow meter status |
| **Norm** | on, dimmed | captions the lower display |
| **Pursuit** | on, full brightness, switch pod lamps lit | captions the lower display |

Leaving Auto clears every display and re-sends brightness, because the boards
were unpowered and have lost their state.

The Power button is not software at all. It triggers a latching relay that
switches the whole 12 V supply, and each Pi holds an interlock relay *open*
while it is up, taking the button out of the circuit — so nothing can be
switched off without halting both Pis first. That is `power-relay-*.service`
(opens the interlock at boot) and `power-off-*.sh` (releases it during
shutdown). The wiring is in [../HARDWARE.md](../HARDWARE.md#the-power-button).

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
**not in that specification**. The shipped firmware could not set those displays
from a host at all, so in 2023 the commands were added to it: user-value
registers for the tacho's bars and seven-segment, and for the speedo's two
bargraphs and both digit groups. Paolo holds that modified firmware and is free
to ship it. When one of these commands needs checking, the board firmware is the
authority, not the protocol PDF.

That firmware is confidential to Paolo and is not in this repo. Keep it that
way: describing what a command *does* is fine, quoting or paraphrasing his
source is not.

Two things worth knowing before editing the serial code:

- **Every write is preceded by `time.sleep(0.1)`.** Without the gap the boards
  drop messages.
- **Hex payloads must be exactly the right length** — four byte pairs for the
  speedo's upper digits, five for the lower, one per bar for the bargraphs.
  A payload of the wrong length is ignored silently rather than rejected.

## The BrewPi side

The fermenter temperatures and Tilt gravities come from three BrewPi Remix
instances on `brewpi`, in `/home/brewpi/{unitank-1,unitank-2,chronical}/`.
`get_brewpi_rmx_data()` asks each one `lcd` and `statusText` every 60 seconds
over a Unix socket called `KITTSOCKET` in the instance directory.

Stock BrewPi Remix has no such socket. It listens on one, `BEERSOCKET`, for its
own PHP front end. `KITTSOCKET` comes from the **`kitt` branch** of
[duncan-brown/brewpi-script-rmx](https://github.com/duncan-brown/brewpi-script-rmx),
commit `7e0e500` "open a second socket for kitt" (April 2023) and two
follow-ups, and that branch is what the Pi runs. The change:

- pulls the socket setup into a helper and calls it twice, so `KITTSOCKET` is
  created next to `BEERSOCKET` with the same `brewpi:www-data` ownership and
  `0660` mode;
- makes the main loop **alternate** between accepting on the PHP socket and the
  KITT socket, one per pass, so neither client can starve the other. Whichever
  connects is served by the same command handler, so the dash could ask for
  anything the web UI can;
- halves the socket timeout from 0.5 s to 0.25 s so the controller is still
  polled as often with two accepts sharing the loop, and replaces the
  `raise socket.timeout` trick for "go and do serial now" with an explicit
  `SerialExpected` exception.

A separate socket rather than sharing `BEERSOCKET` keeps the dash's polling
from ever queueing behind the web UI's, and vice versa.

**Known state, September 2026.** The `kitt` branch forked from `main` in 2021
and is ten commits behind it. In particular it lacks `9aabb93` "expire the
correct tilt color" (2022), which sets the colour from the config before
clearing a stale TiltBridge reading; without it that code path uses an
undefined name. On the Pi, `unitank-2`'s copy of `brewpi.py` carries an
uncommitted `color = 'Green'` at that point — the same fix done by hand for its
own Tilt — and `unitank-1` has a commented-out `# color = 'Blue'`. The clean fix
is to merge `main` into `kitt`, or cherry-pick `9aabb93`, redeploy all three
instances and drop the local edits. Until then, anyone reinstalling from either
branch as-is gets different behaviour from what is running.

## Installation

There is nothing to build. On each Pi:

```bash
sudo install -m 755 panp/panp.py /usr/local/sbin/panp.py
sudo install -m 644 panp/panp.service panp/panp.path /usr/local/lib/systemd/system/
sudo install -m 644 panp/power-relay-<host>.service /usr/local/lib/systemd/system/
sudo install -m 755 panp/power-off-<host>.sh /lib/systemd/system-shutdown/
sudo systemctl enable panp.path power-relay-<host>.service
```

`panp.path` lists `/dev/switchpod`, `/dev/ttyAMA0` and `/dev/ttyAMA1`. Note
that systemd fires a path unit when **any** of its conditions is met, not all
of them, and the two UARTs exist from early boot — so in practice the service
starts as soon as the UARTs appear, whether or not the switch pod adapter is
there yet. That is harmless: `get_switchpod()` waits for `/dev/switchpod` to
appear and reopens it if it goes away. The service is `Type=notify` and
reports progress through sdnotify, so `systemctl status panp` shows what it is
doing.

Requires Python 3 with `pyserial`, `sdnotify`, `RPi.GPIO` and
`mysql-connector-python`. The switch pod's serial adapter needs to appear as
`/dev/switchpod`, which takes a udev rule — not included here.

## Notes

- `RPintsLoopHandler` computes `keezer_min`, `keezer_max` and `keezer_median`
  every pass, but no switch pod button currently selects them — only the
  individual probes, the lagers and the mean are reachable.
- The code that cycled fermenter status (`IDLE`/`COOL`/`HEAT`) on the message
  center is commented out. The queue that fed it, `brewpi_rmx_state_q`, is
  still written to — only with `DOWN`, once a minute per fermenter whose
  `KITTSOCKET` cannot be reached — and nothing drains it, so it grows for as
  long as a fermenter is offline. In Auto the message center shows
  `BREWPI UP` instead.
- After leaving Auto, `BrewPiLoopHandler` keeps `msgctr_mode` at `BREWPI_UP`
  (set on entering Auto) but re-sends the caption that was selected before
  Auto. Until a left-pod button is pressed, the lower display shows mash
  temperature on the `BREWPI_UP` scale while the caption may say something
  else, for example `SG UTK1`.
- Exception handling is deliberately broad. This is an unattended daemon with
  `Restart=no`; a probe that fails to read or a database that is down must not
  take the dashboard down with it.
