# Hue lights over the bridge

`rpints` drives two things on the house's Hue bridge, which is in another
part of the building, over WiFi:

- **The bench light.** Pursuit switches the Hue lightstrip over the brewery
  workbench on at full brightness and a chosen white, so there is enough
  light to read small print; Norm and Auto switch it off, as does halting the
  Pis. On the bridge it is named "Brewery Bench Light". Do not confuse it with
  "Brewery Lightstrip", a second strip round the back of the workbench that
  HomeKit and a Hue motion sensor look after and `panp.py` never touches.
- **The room lights.** `P IND` on the right switch pod turns the brewery's
  room lights on at a set brightness and `EJECT R` turns them off. Which
  lights are in that set is a list in the config file; the bench light and
  the motion-sensor lightstrip are deliberately not in it. See "Room lights"
  below.

It is optional. `panp.py` looks for `/usr/local/etc/panp-hue.json` and does
nothing at all if it is absent, which is the case on `brewpi`. Nothing needs
installing — it uses `urllib.request` and `ssl` from the standard library, and
talks to the bridge's v2 API directly rather than going through HomeKit. The
bridge is what Home talks to anyway, so Home follows along.

The config file is **not** in this repository, because the key it holds grants
full control of every light on that bridge. See
[panp-hue.json.example](panp-hue.json.example) for the shape.

## Getting the bridge address

The discovery service returns the bridges on your network:

```bash
curl -s https://discovery.meethue.com
```

Confirm which is which — the `name` field is whatever you called it in the Hue
app, and `modelid` must be `BSB002` for HomeKit to be involved at all:

```bash
curl -s http://BRIDGE_IP/api/0/config
```

Run this from the Pi that will drive the light, not from a laptop. It doubles as
proof the Pi can reach the bridge, which is the link that actually has to work.

## Getting an application key

The bridge grants a key only to someone who can physically press its button.
Start this first, then go and press the round button on top of the bridge — it
polls for two minutes so there is no 30 second window to hit:

```bash
for i in $(seq 1 60); do r=$(curl -s -X POST http://BRIDGE_IP/api -d '{"devicetype":"panp#rpints"}'); case "$r" in *username*) echo "$r"; break;; esac; sleep 2; done
```

Before the button is pressed each attempt returns `link button not pressed`.
After it, you get `{"success":{"username":"..."}}`. That 40-character string is
the key. Treat it as a password: it is permanent until deleted and it controls
every light on the bridge.

## Finding the light id

The v2 API addresses lights by UUID:

```bash
curl -sk -H "hue-application-key: YOUR_KEY" https://BRIDGE_IP/clip/v2/resource/light
```

`-k` is needed because the bridge serves a self-signed certificate. Find the
light you want by `metadata.name` and take its `id`.

## Installing

```bash
sudo install -m 600 -o root -g root /dev/null /usr/local/etc/panp-hue.json
```

Fill it in following the example, then:

```bash
sudo systemctl restart panp
```

`systemctl status panp` then reports either `has a bench light` or `has no bench
light configured`, so a typo in the json shows up immediately rather than
silently doing nothing.

## Tuning

`brightness` is a percentage. `mirek` is colour temperature, and it is the
reciprocal of Kelvin — **lower is colder**:

| mirek | ≈ Kelvin | |
| --- | --- | --- |
| 153 | 6500K | coldest most Hue fixtures manage |
| 200 | 5000K | daylight, can read clinical |
| 250 | 4000K | neutral |
| 300 | 3300K | warm neutral |
| 366 | 2730K | the Hue default warm white |
| 500 | 2000K | candle-ish |

Check the fixture's own range first — `mirek_schema` in the light resource above
gives its minimum and maximum.

Both values are in the config file rather than in the code so the light can be
tuned without editing a public repository. **The config is read once at
startup**, so restart the service after changing it. And note that `panp.py`
sets brightness and colour temperature explicitly on every Pursuit press, so
adjusting that light by hand in the Hue app will not survive the next press —
the config wins.

## Room lights

The `room` block lists the lights the two right-pod keys switch, by v2 id
from the same listing as above, and the brightness to switch them on at:

```json
"room": {
  "brightness": 100,
  "lights": ["0dd1d1b9-...", "23edba6e-...", "..."]
}
```

Each key press sends one request per light from the worker thread, so a dozen
lights take a second or so to all change and the dash never waits. Only the
on/off state and brightness are sent; colour temperature is left as the Hue
app or a scene last set it. If several presses pile up while the bridge is
slow, only the last one is sent. Leaving the block out disables the two keys
and they do nothing.

Rooms and zones would let one request switch a whole group, but the brewery
room on the bridge also contains the bench light and the lightstrip, which
must not follow these keys, so the lights are listed individually.
