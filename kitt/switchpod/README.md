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

Arduino's bare `Serial.println()` writes `\r\n`, so every twentieth press puts
an extra empty line on the wire. It is there to keep a serial monitor readable,
and for a long time it cost nothing: the original reader on the Pi was

```python
data = state.decode().strip()
if data:
    sp_q.put(data)
```

which quietly dropped the empty line.

The `restart switchpod on error` commit (March 2024) replaced that with a parse
and a reopen, so that unreadable input would re-establish the port instead of
being ignored. That was aimed at genuine flakiness on the wire — but it also
removed the `if data:` guard, and from then on the routine twentieth-press
newline parsed as an empty string, raised, and triggered a full close-and-reopen
cycle. The pod went deaf for about three seconds every twenty presses.

Both behaviours are wanted, so `get_switchpod()` now does both: it skips empty
lines and still reopens the port on anything it genuinely cannot parse.

It also reads with `readline()` rather than a fixed two bytes. That matters for
the flaky case specifically. With fixed-size reads, one stray byte shifts the
framing permanently — every later read straddles two presses and yields
nonsense until something triggers a reopen. Reading line-wise resynchronises at
the very next newline, so a glitch costs one keypress instead of the whole
stream. If you ever see garbage that does *not* recover, suspect a burst with no
newline in it at all, which will sit in `readline()` until one arrives.

## Building

PlatformIO, targeting an Arduino Uno:

```bash
pio run -t upload
```

The pods came from ideegeniali with the dash; this sketch is Paolo's key-reading
code adapted to report over serial.
