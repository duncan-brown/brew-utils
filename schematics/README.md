# Schematics

KiCad schematics of the brewery control electronics, generated from the wiring
recorded in [../HARDWARE.md](../HARDWARE.md). Four sheets, A3:

| Sheet | File | What is on it |
| --- | --- | --- |
| 1 | `sheet1-power.kicad_sch` | the supply box: 12 V supply, the three inline fuses, the latching relay and the two Pi interlocks behind the `POWER` button, both D-1012 contactors, the D-1384 fuse block, the 12→24 V converter and the buffer capacitor |
| 2 | `sheet2-rpints.kicad_sch` | rpints: PANP buttons and lamps, the 8-way relay module (flow meters and lamps), the 4-channel dash power module, the serving keezer strip and RaspberryPints Uno, the 12-way serial/one-wire strip, the right switch pod |
| 3 | `sheet3-brewpi.kicad_sch` | brewpi: relay drives, mode lines, hot-side probes, serial to the message centre and speedo, the left switch pod, and the three BrewPi Remix controllers with their heaters and glycol pumps |
| 4 | `sheet4-dash.kicad_sch` | strip 1 to the Pi buck converters, the DASH POWER block to the six dash boards, and the four PANP switches on their two 8-way strips |

Rendered copies are in [`../images/`](../images/) as `schematic-sheet*.svg`.

## How they are made

The sheets are **generated**, not drawn by hand. `ksheets.py` describes every
component and wire in Python; `kicadgen.py` turns that into KiCad's S-expression
format (version `20250114`, KiCad 9 and 10). Stock symbols such as fuses,
resistors and screw terminals are copied out of the installed KiCad libraries at
generation time; the relay modules, Pis, Unos and dash boards are simple boxes
with named terminals defined in `kicadgen.box_symbol`, which is how they present
physically.

Nets that cross sheets are global labels with the same name on each sheet, for
example `MASTER_RELAY`, `P1_+12V`, `RPINTS_GND`, `BREWPI_IO16`.

To regenerate after editing `ksheets.py`:

```bash
python3 schematics/ksheets.py schematics
```

then export and check with KiCad's command line:

```bash
K=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli
for f in schematics/sheet*.kicad_sch; do
  $K sch export svg -o images -e "$f"
  $K sch erc --severity-all --format report -o "${f%.kicad_sch}.erc" "$f"
done
```

All four sheets pass ERC with no errors. The remaining warnings are expected:
global labels that appear on only one sheet (they are for the reader, and for
the sheet they pair with), and the note that the home-made `brew:` symbols are
not in a configured library, which is by design since they are embedded in each
file.

## Editing in KiCad

The files open directly in KiCad's schematic editor. Edits made there are fine
for a one-off print, but they will be overwritten by the next run of
`ksheets.py`, so a lasting change belongs in the Python. The reference
designators are deliberate and match [../HARDWARE.md](../HARDWARE.md): `K1` is
the latching relay, `K2`/`K3` the interlocks, `K4`/`K5` the contactors, `K6` the
8-way module, `K7` the 4-channel dash power module, `K8` the 6-channel fermenter
module.
