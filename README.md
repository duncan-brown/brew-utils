# brew-utils

Software for a homebrewery dashboard built out of Knight Rider (KITT) dash
replica hardware.

Two Raspberry Pis read temperature probes, flow meters, fermentation
controllers and a keg-monitoring database, and render all of it onto the gauges
of a replica KITT dashboard: the MPH display shows hot liquor tank temperature,
the RPM gauges show serving keezer probes, the LED bargraphs show fermenter
temperatures and how much beer is left in each keg, and the message center
captions whatever the multi-function display is currently showing.

There is a full build write-up, with photos of what every gauge means, at
[Brewery Dashboard on HomebrewTalk](https://www.homebrewtalk.com/threads/brewery-dashboard.726917/).
The physical side — power supply, relays, probes, which wire goes to which pin
on which Pi — is in [HARDWARE.md](HARDWARE.md).

## How it fits together

```
    probes, flow meters, kegs                two Raspberry Pis            KITT dash
    ─────────────────────────                ─────────────────            ─────────
    DS18B20 one-wire probes  ──────────────▶  rpints  ──── serial ────▶  tacho
    RaspberryPints (MySQL)   ──────────────▶          ──── serial ────▶  dummy3, dummy6
                                                 │
                                            GPIO │ mode
                                                 ▼
    DS18B20 one-wire probes  ──────────────▶  brewpi  ──── serial ────▶  speedo
    BrewPi Remix ◀── TiltBridge ◀── Tilt ──▶          ──── serial ────▶  message center
    flow meter relays        ◀──────────────
```

- **`rpints`** runs [RaspberryPints](https://github.com/RaspberryPints/RaspberryPints)
  for keg volume tracking, and owns the dashboard's Power/Auto/Norm/Pursuit
  buttons and the power relays for the whole dash.
- **`brewpi`** runs three instances of [BrewPi Remix](https://github.com/brewpi-remix/brewpi-tools-rmx)
  for fermentation control, which get specific-gravity readings from Tilt
  hydrometers via [TiltBridge](https://github.com/thorrak/tiltbridge).
- The two Pis coordinate over GPIO lines so that pressing Auto, Norm or Pursuit
  on one changes what the other displays and how brightly.
- Both drive the dash boards over serial using Paolo Sancono's KITT protocol.
  The dash electronics themselves are from [ideegeniali](https://www.ideegeniali.it/).

## What's in here

| Path | Status | Description |
| --- | --- | --- |
| [`kitt/`](kitt/) | **in service** | The current system. `panp/panp.py` drives the whole dashboard; `switchpod/` is the Arduino firmware for the dash keypads. See [kitt/README.md](kitt/README.md) for how it works, and [kitt/DASHBOARD.md](kitt/DASHBOARD.md) for what each gauge means. |
| `brewpitosmith` | utility | Converts a BrewPi Remix beer log into a CSV that BeerSmith 3 will import, thinned to one row an hour. |
| `images/` | reference | Photographs of the switch pods, the season 2 dash scan, and the `hw-*` photographs of the bench, supply box and control panel that [HARDWARE.md](HARDWARE.md) refers to. |

Two earlier efforts have been archived out of the tree. Each is tagged, so the
whole thing comes back with one command:

| Tag | What it was |
| --- | --- |
| `karr-prototype` | An attempt to rebuild the dash electronics from scratch — four ESP32s driving daisy-chained TLC5940 drivers, with a KiCad board and gerbers. Shelved in 2023 when the ideegeniali boards arrived; never ran the brewery. |
| `pi-temp-display-final` | The predecessor to the KITT dash: seven-segment LEDs multiplexed with pigpio, showing mash temperature and keezer min/max. |

```bash
git checkout karr-prototype -- karr/
```

## Credits

The parts of this system that do the actual work of running the brewery are
other people's: BrewPi Remix by [@LBussy](https://github.com/lbussy), TiltBridge
by [@Thorrak](https://github.com/thorrak), RaspberryPints by
[@RandR+](https://github.com/RandRPints), and the KITT dash boards and serial
protocol by Paolo Sancono at ideegeniali. Thanks also to @day_trippr for help
along the way.

## License

MIT — see [LICENSE](LICENSE).
