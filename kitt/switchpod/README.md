# switchpod

Firmware for the Arduinos that read the dash switch pods.

Each Pi has its own pod and its own Arduino running this same sketch: the right
pod on `rpints`, the left pod on `brewpi`. Which buttons do what is in
[../DASHBOARD.md](../DASHBOARD.md).

## How a pod is read

A pod is ten buttons wired as a **resistive ladder** brought out on a single
signal wire. Each button switches a different resistor — 1k, 1k2, 1k5, 1k8,
2k2, 2k7, 3k3, 3k9, 4k7, 5k6 — so pressing one produces a distinct voltage and
the whole pod costs one ADC channel instead of ten digital inputs.

Wiring is three conductors: the pod's `SGN` to an analog pin, ground common
between pod and Arduino, and a **pull-up resistor from `SGN` to +5V** (to 3V3
instead on a 3.3V board). Paolo's recommendation is 3k3, sitting in the middle
of the ladder's span for the best noise margin. With nothing pressed the pin
floats up to the rail and reads about 1023; each button pulls it down to its own
level.

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
thresholds in `values[]` rather than to change anything on the Pi. Paolo's
calibration method: print `analogRead()` continuously, press each button in
turn, note the ten values, sort them, and set each threshold at the midpoint
between neighbours. The values in the sketch came from that exercise on these
pods and are not universal — a different pull-up moves all ten.

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
included in this repo), reads a line at a time with `readline()`, strips the
newline, and pushes the integer onto a queue for the loop handler to act on.
(It used to read a fixed two bytes; the section below explains why that
changed.)

## The every-twentieth-press dropout, and why it took three changes

Worth reading before touching either side, because no one change here was wrong
on its own.

Paolo's example sketch separates keys with a **space** and wraps the line every
twentieth key, so a serial monitor stays readable:

```cpp
Serial.print(key);
Serial.print(" ");                        // "0 3 7 2 ..."
if ((aCapo % 20) == 0) Serial.println();  // wrap every 20 keys
```

**Change one:** this sketch made the separator `"\n"` so each key arrives on its
own line for parsing. At that moment the wrap stopped being a wrap and became a
stray empty line — a bare `Serial.println()` writes `\r\n`.

It still cost nothing, because the original reader on the Pi was

```python
data = state.decode().strip()
if data:
    sp_q.put(data)
```

which quietly dropped the empty line.

**Change two:** the `restart switchpod on error` commit (March 2024) replaced
that with a parse and a reopen, so unreadable input would re-establish the port
instead of being ignored. That was aimed at genuine flakiness on the wire — but
it also removed the `if data:` guard. From then on the routine twentieth-press
newline parsed as an empty string, raised, and triggered a full
close-and-reopen. The pod went deaf for about three seconds every twenty
presses, for two years.

**Change three**, the fix, is on both sides:

- The firmware no longer emits the wrap. One key, one line, nothing else.
- `get_switchpod()` skips empty lines *and* still reopens the port on anything
  it genuinely cannot parse, so the flaky-wire recovery survives.

The Pi also reads with `readline()` rather than a fixed two bytes, which matters
for the flaky case specifically. With fixed-size reads one stray byte shifts the
framing permanently — every later read straddles two presses and yields nonsense
until something triggers a reopen, so a single glitch looks like total failure.
Reading line-wise resynchronises at the next newline, so a glitch costs one
keypress. If you ever see garbage that does *not* recover, suspect a burst with
no newline in it at all, which will sit in `readline()` until one arrives.

Keeping the host-side skip after fixing the firmware is deliberate: an Arduino
that has not been reflashed, or one flashed from an older checkout, still works.

## Building

PlatformIO, targeting an Arduino Uno:

```bash
pio run -t upload
```

The pods came from ideegeniali with the dash, and this sketch is Paolo's
key-reading code with two changes. His original reads **two** pods from one
Arduino — `A0` and `A1`, reporting 0–19 with the right-hand pod offset by ten.
Here each pod has its own Arduino reading `A0` alone and reporting 0–9, because
the two pods are wired to different Raspberry Pis. The other change is the
separator, described above.
