# switchpod

Firmware for the Arduinos that read the dash switch pods.

Each Pi has its own pod and its own Arduino running this same sketch: the right
pod on `rpints`, the left pod on `brewpi`. Which buttons do what is in
[../DASHBOARD.md](../DASHBOARD.md).

## How a pod is read

A pod is ten buttons wired as a **resistive ladder** on a single analog pin
(`A0`). Each button taps a different point in the chain, so pressing one
produces a distinct voltage and the whole pod costs one ADC channel instead of
ten digital inputs.

`readSwitchpod()` reads the ADC and bins the result:

| ADC reading | Position |
| --- | --- |
| ≤ 421 | 0 |
| 422–463 | 1 |
| 464–507 | 2 |
| 508–549 | 3 |
| 550–595 | 4 |
| 596–640 | 5 |
| 641–680 | 6 |
| 681–717 | 7 |
| 718–751 | 8 |
| 752–895 | 9 |
| > 895 | nothing pressed |

Resting state is the top band — the line idles pulled high, and pressing a
button drags it down. Position 0 sits at the bottom of the ladder and gets the
whole range below 421.

**Positions are physical, not logical**: even positions run down the left column
of the pod, odd positions down the right. So position 0 is top left, 1 is top
right, 2 is second left, and so on. That is why `panp.py` handles `sp_val` 0, 2,
4, 6, 8 as one family of actions and 1, 3, 5, 7, 9 as another — the two columns
do different jobs.

If a button reads as its neighbour, the fix is to re-measure and adjust the
thresholds in `values[]` rather than to change anything on the Pi.

## Debounce and repeat

`readDebouncedSwitchpod()` requires the reading to stay put for **20 ms** before
accepting it, and gives up after **150 ms** if it never settles, returning "no
key". Contact bounce and the slew as your finger lands are both rejected.

`loop()` holds a `stoProcessando` flag so a key is reported **once per press**.
It will not repeat until the pod reads empty again, so holding a button down
does nothing more than tapping it.

## Wire format

9600 baud, 8N1, over the USB serial adapter. On a press the Arduino sends the
position as a single ASCII digit followed by a newline:

```
4\n
```

`panp.py` opens the adapter as `/dev/switchpod` (a udev rule maps it; not
included in this repo), reads **two bytes at a time**, strips the newline, and
pushes the integer onto a queue for the loop handler to act on.

## The every-twentieth-press hiccup

`processaTasto()` counts presses and emits an extra blank line on each
twentieth:

```cpp
if ((aCapo % 20) == 0) Serial.println();
```

Arduino's bare `Serial.println()` writes `\r\n`, so that adds two more bytes to
the stream. The reader on the Pi takes a fixed two bytes per press, so the pair
stays aligned — but every twentieth press it consumes `"\r\n"` on its own,
`strip()` reduces that to an empty string, and `int("")` raises.

`get_switchpod()` catches it, closes the port, waits, reopens, and logs
`restarted switchpod i/o`. So roughly every twenty presses the pod goes deaf for
about three seconds and then carries on. This is the mechanism behind the
`restart switchpod on error` commit: that error handler is not covering for
flaky hardware, it is covering for this line of firmware.

Two ways to be rid of it, neither yet applied:

- drop the `aCapo % 20` line and reflash, or
- read line-wise on the Pi (`rx.readline()`) instead of `rx.read(2)`, which
  tolerates stray newlines whatever their length.

The second is the safer of the two, because it does not depend on every Arduino
in the brewery carrying the same firmware build.

## Building

PlatformIO, targeting an Arduino Uno:

```bash
pio run -t upload
```

The pods came from ideegeniali with the dash; this sketch is Paolo's key-reading
code adapted to report over serial.
