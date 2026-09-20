"""KiCad schematics for the brew-utils hardware, generated from the wiring
recorded in HARDWARE.md.  Run:  python3 ksheets.py OUTDIR

Coordinates are in grid units of 1.27 mm via g(); the sheets are A3.
Nets that cross sheets use global labels with the same name on each sheet.
"""
import os
import sys
from kicadgen import Schematic, stock, box_symbol

OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.abspath(__file__))
os.makedirs(OUT, exist_ok=True)


def g(n):
    return round(n * 1.27, 4)


def G(x, y):
    return (g(x), g(y))


def snapx(v):
    return round(round(v / 1.27) * 1.27, 4)


# ---------------------------------------------------------------- symbols
FUSE = stock('Device', 'Fuse')
CAP = stock('Device', 'C_Polarized')
RES = stock('Device', 'R')
SW = stock('Switch', 'SW_Push')
LAMP = stock('Device', 'Lamp')
TB = {n: stock('Connector', f'Screw_Terminal_01x{n:02d}') for n in (4, 6, 8, 12)}

PSU = box_symbol('PSU_12V_30A', left=['L', 'N'], right=['+12V', '0V'], ref='PS')
LATCH = box_symbol('HiLetgo_Latching_Relay', left=['+12', '0V', 'TRIG', 'TRIG_RTN'], right=['COM', 'NC', 'NO'], ref='K')
OPTO1 = box_symbol('Relay_Module_1ch_5V', left=['IN', 'VCC', 'GND', 'COM'], right=['NC', 'NO'], ref='K')
D1012 = box_symbol('CZH_D-1012_30A', left=['IN+', 'IN-', 'COM'], right=['NO'], ref='K')
D1384 = box_symbol('CZH_D-1384_Fuse_Block', left=['V+', 'V-'],
                   right=['V1:7.5A', 'V2:5A', 'V3:7.5A', 'V4:7.5A', 'V5:7.5A', 'V6:5A'], ref='FB')
MGGI = box_symbol('MGGi_12-24V_15A', left=['IN+', 'IN-'], right=['OUT+', 'OUT-'], ref='U')
PI_R = box_symbol('RPi4_rpints_HAT', right=['IO5', 'IO6', 'IO16', 'IO19', 'IO20', 'IO12', 'IO17', 'IO18', 'IO10',
                                            'IO23', 'IO7', 'IO22', 'IO4', 'IO26', 'TXD', 'CE0', 'IO25', '5V', '3V3', 'GND'], ref='PI')
PI_B = box_symbol('RPi4_brewpi_HAT', right=['IO2', 'IO3', 'IO27', 'IO21', 'IO13', 'IO16', 'IO5', 'IO7', 'IO22',
                                            'IO4', 'TXD', 'CE0', 'IO25', '5V', '3V3', 'GND'], ref='PI')
RELAY8 = box_symbol('Relay_Module_8ch_5V', left=[f'IN{i}' for i in range(1, 9)] + ['VCC', 'GND'],
                    right=sum([[f'COM{i}', f'NO{i}'] for i in range(1, 9)], []), ref='K')
RELAY6 = box_symbol('Relay_Module_6ch_5V', left=[f'IN{i}' for i in range(1, 7)] + ['VCC', 'GND'],
                    right=sum([[f'COM{i}', f'NO{i}'] for i in range(1, 7)], []), ref='K')
RELAY4 = box_symbol('Relay_Module_4ch_5V_LowTrigger', left=[f'IN{i}' for i in range(1, 5)] + ['VCC', 'GND'],
                    right=sum([[f'COM{i}', f'NO{i}'] for i in range(1, 5)], []), ref='K')
UNO_RP = box_symbol('Uno_RaspberryPints', left=['D6', 'D7', 'D8', 'D9', 'D10'], right=['5V', '3V3', 'GND'], ref='A')
UNO_SP = box_symbol('Uno_Switchpod', left=['A0'], right=['5V', 'GND'], ref='A')
UNO_BP = box_symbol('Uno_BrewPi', left=['A0', '5V', 'GND'], right=['D5', 'D6'], ref='A')
POD = box_symbol('Switch_Pod_Ladder', right=['SGN', 'GND', '+12V', '0V'], ref='SP')
SHIFT = box_symbol('TXS0108E', left=['VA', 'A6', 'A7', 'OE', 'GND'], right=['VB', 'B6', 'B7'], ref='U')
PROBES = box_symbol('DS18B20_bus', right=['VDD', 'DQ', 'GND'], ref='T')
BUCK = box_symbol('SSLHONG_12-5V_USB-C', left=['IN+', 'IN-'], right=['USB-C'], ref='U')
BOARD = box_symbol('KITT_dash_board', left=['+12V', 'GND', 'COM'], ref='DB')
HEATER = box_symbol('FTSs_heating_pad_12V', left=['+', '-'], ref='H')
PUMP = box_symbol('Penguin_XL_pump_24V', left=['+', '-'], ref='M')
PANPSW = box_symbol('MICRO_SWITCH_4A13BAA31', left=['C', 'NO', 'NC'], right=['LAMP5', 'LAMP2'], ref='SW')


# ---------------------------------------------------------------- helpers
def gl(s, pin, name, side='right', stub=4):
    """Stub wire from a pin and a global label at its end."""
    dx = g(stub) if side == 'right' else -g(stub)
    end = (pin[0] + dx, pin[1])
    s.wire(pin, end)
    s.glabel(end, name, rot=0 if side == 'right' else 180)
    return end


def down_to(s, pin, y, dx=0, junction=True):
    """Pin → short horizontal → vertical to row y; junction at the end."""
    x = pin[0] + g(dx)
    if dx:
        s.wire(pin, (x, pin[1]))
    s.wire((x, pin[1]), (x, g(y)))
    if junction:
        s.junction((x, g(y)))
    return (x, g(y))


def bend(s, a, b, xm):
    """a → (xm, a.y) → (xm, b.y) → b : a three-segment orthogonal wire."""
    s.wire(a, (xm, a[1])); s.wire((xm, a[1]), (xm, b[1])); s.wire((xm, b[1]), b)


def bendy(s, a, b, ym):
    """a → (a.x, ym) → (b.x, ym) → b."""
    s.wire(a, (a[0], ym)); s.wire((a[0], ym), (b[0], ym)); s.wire((b[0], ym), b)


def vres(s, res, x, ybot, ytop):
    """Return (bottom_pin, top_pin) of a vertical resistor placed at x between rows."""
    a, b = res['1'], res['2']
    return (a, b) if a[1] > b[1] else (b, a)


def header(s, title, sub=None):
    s.text(G(8, 8), title, size=1.6)
    if sub:
        s.text(G(8, 12), sub, size=1.27)


# ================================================================= sheet 1
def sheet1():
    s = Schematic('brew-utils sheet 1 — supply box and the POWER button', paper='A3')
    header(s, 'Sheet 1 — the supply box and the POWER button circuit.  The latch is unlatched at power-up and its output is taken from NC, so the system comes up on mains.',
           'Interlocks: each Pi drives its relay IN high while it is up (power-relay-*.service), opening the NC contact and taking the POWER button out of the trigger loop.  Both Pis must halt before the button does anything.')
    RAIL, GND = 30, 200
    s.wire(G(20, RAIL), G(320, RAIL)); s.glabel(G(20, RAIL), '+12V_UNSWITCHED', rot=180); s.glabel(G(320, RAIL), '+12V_UNSWITCHED')
    s.wire(G(20, GND), G(320, GND)); s.glabel(G(20, GND), '0V', rot=180); s.glabel(G(320, GND), '0V')

    psu = s.place(PSU, *G(40, 100), ref='PS1', value='12 V 30 A switching supply')
    s.route(psu['+12V'], G(58, RAIL), 'h'); s.junction(G(58, RAIL))
    s.route(psu['0V'], G(58, GND), 'h'); s.junction(G(58, GND))
    gl(s, psu['L'], 'MAINS_L', 'left'); gl(s, psu['N'], 'MAINS_N', 'left')

    f1 = s.place(FUSE, *G(72, 42), ref='F1', value='3 A')
    s.wire(G(72, RAIL), f1['1']); s.junction(G(72, RAIL))
    latch = s.place(LATCH, *G(96, 62), ref='K1', value='HiLetgo bistable latching relay')
    node = (f1['2'][0], latch['+12'][1])
    s.wire(f1['2'], node); s.junction(node); s.wire(node, latch['+12'])
    top = latch['+12'][1] - g(8)
    xr = latch['COM'][0] + g(4)
    s.wire(node, (node[0], top)); s.wire((node[0], top), (xr, top)); s.wire((xr, top), (xr, latch['COM'][1])); s.wire((xr, latch['COM'][1]), latch['COM'])
    down_to(s, latch['0V'], GND, dx=-4)
    s.nc(latch['NO'])
    mr_x = latch['NC'][0] + g(14)
    s.wire(latch['NC'], (mr_x, latch['NC'][1])); s.junction((mr_x, latch['NC'][1]))
    s.label((latch['NC'][0] + g(2), latch['NC'][1]), 'MASTER_RELAY')
    lamp = s.place(LAMP, mr_x, g(120), ref='LA1', value='POWER button lamp 12 V')
    a, b = lamp['1'], lamp['2']
    top_pin, bot_pin = (a, b) if a[1] < b[1] else (b, a)
    s.wire((mr_x, latch['NC'][1]), top_pin)
    s.wire(bot_pin, (bot_pin[0], g(GND))); s.junction((bot_pin[0], g(GND)))

    sw = s.place(SW, *G(40, 150), ref='SW1', value='POWER button (C · NO)')
    k2 = s.place(OPTO1, *G(66, 150), ref='K2', value='rpints interlock')
    k3 = s.place(OPTO1, *G(106, 150), ref='K3', value='brewpi interlock')
    s.wire(sw['2'], k2['COM'])
    xl = sw['1'][0] - g(4)
    s.wire(sw['1'], (xl, sw['1'][1])); s.wire((xl, sw['1'][1]), (xl, g(176)))
    xt = latch['TRIG_RTN'][0] - g(6)
    s.wire((xl, g(176)), (xt, g(176))); s.wire((xt, g(176)), (xt, latch['TRIG_RTN'][1])); s.wire((xt, latch['TRIG_RTN'][1]), latch['TRIG_RTN'])
    for k, names in ((k2, ('RPINTS_IO23', 'RPINTS_5V')), (k3, ('BREWPI_IO5', 'BREWPI_5V'))):
        for pn, net in zip(('IN', 'VCC'), names):
            gl(s, k[pn], net, 'left', stub=6)
        down_to(s, k['GND'], GND, dx=-2)
        s.nc(k['NO'])
    s.wire(k2['NC'], k3['COM'])
    xk = k3['NC'][0] + g(4)
    s.wire(k3['NC'], (xk, k3['NC'][1])); s.wire((xk, k3['NC'][1]), (xk, g(126)))
    xq = latch['TRIG'][0] - g(3)
    s.wire((xk, g(126)), (xq, g(126))); s.wire((xq, g(126)), (xq, latch['TRIG'][1])); s.wire((xq, latch['TRIG'][1]), latch['TRIG'])
    s.text(G(56, 168), 'button NO → K2 COM/NC → K3 COM/NC → TRIG;  button C → TRIG_RTN', size=1.1)

    k4 = s.place(D1012, *G(168, 70), ref='K4', value='D-1012 lower contactor')
    k5 = s.place(D1012, *G(168, 130), ref='K5', value='D-1012 upper contactor')
    bus = k4['IN+'][0] - g(8)
    s.wire((mr_x, latch['NC'][1]), (bus, latch['NC'][1])); s.wire((bus, latch['NC'][1]), (bus, k5['IN+'][1]))
    s.junction((bus, k4['IN+'][1])); s.wire((bus, k4['IN+'][1]), k4['IN+']); s.wire((bus, k5['IN+'][1]), k5['IN+'])
    for k in (k4, k5):
        down_to(s, k['IN-'], GND, dx=-4, junction=(k is k4))
    comx = k4['COM'][0] - g(6)
    s.wire((comx, g(RAIL)), (comx, k4['COM'][1])); s.junction((comx, g(RAIL)))
    s.junction((comx, k4['COM'][1])); s.wire((comx, k4['COM'][1]), k4['COM'])
    s.wire((comx, k4['COM'][1]), (comx, k5['COM'][1])); s.wire((comx, k5['COM'][1]), k5['COM'])

    fb = s.place(D1384, *G(236, 70), ref='FB1', value='D-1384 fuse block, 6 × ATO')
    xv = fb['V+'][0] - g(6)
    s.wire(k4['NO'], (xv, k4['NO'][1])); s.wire((xv, k4['NO'][1]), (xv, fb['V+'][1])); s.wire((xv, fb['V+'][1]), fb['V+'])
    s.label((k4['NO'][0] + g(2), k4['NO'][1]), '+12V_SWITCHED')
    down_to(s, fb['V-'], GND, dx=-3)
    outs = [('V1:7.5A', 'P1_+12V', '"P1" → strip 1: Pi buck converters and PANP lamps  (sheet 4)'),
            ('V2:5A', 'DASH_POWER_+12V', '"DASH POWER" → 4-way block → dash power relays  (sheet 2, 4)'),
            ('V3:7.5A', 'H1_FEED', 'strip 2 → relay 1 → heater H1, unitank-1  (sheet 3)'),
            ('V4:7.5A', 'H2_FEED', 'strip 2 → relay 2 → heater H2, unitank-2  (sheet 3)'),
            ('V5:7.5A', 'H3_FEED', 'strip 2 → relay 3 → heater H3, chronical  (sheet 3)')]
    for pin, net, txt in outs:
        p = gl(s, fb[pin], net)
        s.text((p[0] + g(22), p[1] - g(.8)), txt, size=1.1)
    s.nc(fb['V6:5A']); s.text((fb['V6:5A'][0] + g(4), fb['V6:5A'][1] - g(.8)), 'not connected', size=1.1)

    f2 = s.place(FUSE, *G(210, 42), ref='F2', value='25 A')
    s.wire(G(210, RAIL), f2['1']); s.junction(G(210, RAIL))
    conv = s.place(MGGI, *G(236, 140), ref='U1', value='MGGi 12→24 V boost, 15 A 360 W')
    s.wire(f2['2'], (f2['2'][0], conv['IN+'][1])); s.wire((f2['2'][0], conv['IN+'][1]), conv['IN+'])
    s.text(G(212, 52), 'F2 sits hidden behind the contactors', size=1.1)
    down_to(s, conv['IN-'], GND, dx=-3)
    p = gl(s, conv['OUT+'], '+24V'); s.text((p[0] + g(12), p[1] - g(.8)), '→ strip 3 → relays 4–6 → glycol pumps  (sheet 3)', size=1.1)
    gl(s, conv['OUT-'], '24V_RTN')

    f3 = s.place(FUSE, k5['NO'][0] + g(8), k5['NO'][1], rot=90, ref='F3', value='25 A')
    s.wire(k5['NO'], f3['1'])
    cap = s.place(CAP, f3['2'][0] + g(6), g(160), ref='C1', value='Belva BB1D 1 F')
    s.wire(f3['2'], (cap['1'][0], f3['2'][1])); s.wire((cap['1'][0], f3['2'][1]), cap['1'])
    s.wire(cap['2'], (cap['2'][0], g(GND))); s.junction((cap['2'][0], g(GND)))
    s.text(G(180, 150), 'C1 is effectively in parallel with the converter input while the upper contactor is closed;\nit rides out the supply dip when a 24 V glycol pump starts.', size=1.1)
    return s.write(os.path.join(OUT, 'sheet1-power.kicad_sch'))


# ================================================================= sheet 2
def sheet2():
    s = Schematic('brew-utils sheet 2 — rpints', paper='A3')
    header(s, 'Sheet 2 — rpints: PANP buttons and lamps, dash power relays, the flow-meter relays, the serving and lagering keezer sensors, '
              'the RaspberryPints Uno, the right switch pod, and the serial buses to the tacho and dummy boards.',
           'Cross-sheet nets are global labels.  RPINTS_* nets are the rpints HAT; BREWPI_* nets come from sheet 3; P1_+12V and DASH_POWER_+12V from sheet 1.')
    pi = s.place(PI_R, *G(30, 110), ref='PI1', value='rpints · Raspberry Pi 4 · 10.0.1.31 · GeeekPi HAT')
    for pn, net in (('5V', 'RPINTS_5V'), ('3V3', 'RPINTS_3V3'), ('GND', 'RPINTS_GND'), ('IO23', 'RPINTS_IO23'),
                    ('IO7', 'MODE_AUTO'), ('IO22', 'MODE_NORM'), ('IO25', 'RPINTS_OE')):
        gl(s, pi[pn], net, stub=6)
    s.text((pi['IO23'][0] + g(20), pi['IO23'][1] - g(.8)), 'interlock relay K2 IN (sheet 1); driven by power-relay-rpints.service, not panp.py', size=1.1)
    s.text((pi['IO7'][0] + g(18), pi['IO7'][1] - g(.8)), 'auto_mode_comm → brewpi IO7', size=1.1)
    s.text((pi['IO22'][0] + g(18), pi['IO22'][1] - g(.8)), 'normal_mode_comm → brewpi IO22', size=1.1)

    # --- PANP buttons: NO to the GPIO, C commoned to GND ------------------
    xc = 92
    for io, name, ref in (('IO5', 'AUTO', 'SW2'), ('IO6', 'NORM', 'SW3'), ('IO16', 'PURSUIT', 'SW4')):
        p = pi[io]
        sw = s.place(SW, g(74), p[1], ref=ref, value=f'{name} (NO · C)', hide_value=True)
        s.wire(p, sw['1']); s.wire(sw['2'], (g(xc), p[1]))
        s.label((p[0] + g(2), p[1]), f'{name}_NO')
        s.text((g(66), p[1] - g(2.2)), name, size=1.1)
    s.wire((g(xc), pi['IO5'][1]), (g(xc), pi['IO16'][1])); s.junction((g(xc), pi['IO6'][1]))
    s.wire((g(xc), pi['IO16'][1]), (g(xc), pi['IO16'][1] + g(6))); s.glabel((g(xc), pi['IO16'][1] + g(6)), 'RPINTS_GND', rot=270)
    s.text(G(62, 92), 'PANP buttons: NO contacts on the lower 8-way strip 5·6·7; C contacts commoned via lower strip 8 → middle strip 1 → 2 → GND (sheet 4).', size=1.1)

    # --- 8-way relay module: ch 1–5 flow meters (inputs from brewpi), ch 6–8 PANP lamps
    r8 = s.place(RELAY8, *G(130, 80), ref='K6', value='HiLetgo 8-ch relay module, high-level trigger')
    for i, net in enumerate(['BREWPI_IO2', 'BREWPI_IO3', 'BREWPI_IO27', 'BREWPI_IO21', 'BREWPI_IO13'], 1):
        gl(s, r8[f'IN{i}'], net, 'left', stub=6)
    for io, inp in (('IO12', 'IN6'), ('IO20', 'IN7'), ('IO19', 'IN8')):
        bend(s, pi[io], r8[inp], g(104 + 2 * (['IN6', 'IN7', 'IN8'].index(inp))))
    gl(s, r8['VCC'], 'RPINTS_5V', 'left', stub=6); gl(s, r8['GND'], 'RPINTS_GND', 'left', stub=6)
    xcom = r8['COM6'][0] + g(4)
    s.wire(r8['COM6'], (xcom, r8['COM6'][1])); s.wire((xcom, r8['COM6'][1]), (xcom, r8['COM8'][1]))
    s.junction((xcom, r8['COM7'][1])); s.wire((xcom, r8['COM7'][1]), r8['COM7']); s.wire((xcom, r8['COM8'][1]), r8['COM8'])
    s.wire((xcom, r8['COM6'][1]), (xcom, r8['COM6'][1] - g(4))); s.glabel((xcom, r8['COM6'][1] - g(4)), 'P1_+12V', rot=90)
    s.text((xcom + g(2), r8['COM6'][1] - g(6)), 'strip 1 terminal 6', size=1.1)
    xl = xcom + g(26)
    for no, name, ref in (('NO6', 'PURSUIT', 'LA4'), ('NO7', 'NORM', 'LA3'), ('NO8', 'AUTO', 'LA2')):
        p = r8[no]
        lamp = s.place(LAMP, p[0] + g(14), p[1], rot=90, ref=ref, value=f'{name} lamp 12 V', hide_value=True)
        la, lb = lamp['1'], lamp['2']
        left, right = (la, lb) if la[0] < lb[0] else (lb, la)
        s.wire(p, left); s.wire(right, (xl, p[1]))
        s.label((p[0] + g(1), p[1]), f'{name}_LAMP')
        s.text((p[0] + g(10), p[1] - g(2.6)), f'{name} lamp', size=1.0)
    s.wire((xl, r8['NO6'][1]), (xl, r8['NO8'][1])); s.junction((xl, r8['NO7'][1]))
    s.wire((xl, r8['NO8'][1]), (xl, r8['NO8'][1] + g(4))); s.glabel((xl, r8['NO8'][1] + g(4)), '0V', rot=270)
    s.text((xl + g(2), r8['NO8'][1] + g(5)), 'lamp 0 V: middle strip 6·7·8 → strip 1 terminal 3 (sheet 4)', size=1.1)

    # --- serving keezer strip and flow meters --------------------------------
    kz = s.place(TB[8], *G(222, 60), ref='J1', value='serving keezer 8-way strip')
    kzn = ['0 V for SF800s and probes', '+5 V for SF800s and probes', 'one-wire data, 6 × DS18B20',
           'SF800 tap 1 pulse', 'SF800 tap 2 pulse', 'SF800 tap 3 pulse', 'SF800 tap 4 pulse', 'SF800 tap 5 pulse']
    for i, txt in enumerate(kzn, 1):
        s.text((kz[f'Pin_{i}'][0] + g(6), kz[f'Pin_{i}'][1] - g(.8)), f'{i}: {txt}', size=1.0)
    s.text((kz['Pin_1'][0] + g(6), kz['Pin_1'][1] - g(4)), 'field side: one 8-core cable into the keezer', size=1.1)
    for i in range(5):
        bend(s, r8[f'COM{i+1}'], kz[f'Pin_{i+4}'], r8[f'COM{i+1}'][0] + g(6 + 2 * i))
    uno = s.place(UNO_RP, *G(214, 128), ref='A1', value='Arduino Uno · /dev/rpints · RaspberryPints sketch')
    for i in range(5):
        bend(s, r8[f'NO{i+1}'], uno[f'D{6+i}'], r8[f'NO{i+1}'][0] + g(30 + 2 * i))
    s.text(G(150, 146), 'relays 1–5 break the SF800 pulse lines between the keezer strip and the Uno; D6–D10 use the Uno\'s internal pull-ups.', size=1.1)
    x5 = uno['5V'][0] + g(4)
    bendy(s, (x5, uno['5V'][1]), (kz['Pin_2'][0] - g(4), kz['Pin_2'][1]), g(104)); s.wire(uno['5V'], (x5, uno['5V'][1])); s.wire((kz['Pin_2'][0] - g(4), kz['Pin_2'][1]), kz['Pin_2'])
    xg = uno['GND'][0] + g(6)
    bendy(s, (xg, uno['GND'][1]), (kz['Pin_1'][0] - g(6), kz['Pin_1'][1]), g(102)); s.wire(uno['GND'], (xg, uno['GND'][1])); s.wire((kz['Pin_1'][0] - g(6), kz['Pin_1'][1]), kz['Pin_1'])
    s.junction((xg, uno['GND'][1]))
    s.wire((xg, uno['GND'][1]), (xg + g(4), uno['GND'][1])); s.glabel((xg + g(4), uno['GND'][1]), 'RPINTS_GND')
    s.text((xg + g(12), uno['GND'][1] - g(.8)), 'keezer 0 V tied to the Pi GND at the 3-way block', size=1.1)
    # one-wire data: Pin_3 → 3-way block, 2.2k to Uno 3V3, → IO4
    xd = kz['Pin_3'][0] - g(10)
    s.wire(kz['Pin_3'], (xd, kz['Pin_3'][1])); s.junction((xd, kz['Pin_3'][1]))
    r1 = s.place(RES, xd, kz['Pin_3'][1] - g(6), ref='R1', value='2.2k')
    rb, rt = vres(s, r1, xd, 0, 0)
    s.wire((xd, kz['Pin_3'][1]), rb)
    s.wire(rt, (rt[0], rt[1] - g(2))); s.wire((rt[0], rt[1] - g(2)), (rt[0] - g(6), rt[1] - g(2))); s.glabel((rt[0] - g(6), rt[1] - g(2)), 'UNO_3V3', rot=180)
    s.text((xd - g(2), kz['Pin_3'][1] - g(12)), '3-way block', size=1.0)
    s.wire((xd, kz['Pin_3'][1]), (xd, kz['Pin_3'][1] + g(8))); s.glabel((xd, kz['Pin_3'][1] + g(8)), 'KEEZER_1W', rot=270)
    gl(s, pi['IO4'], 'KEEZER_1W', stub=6)
    s.text((pi['IO4'][0] + g(14), pi['IO4'][1] - g(.8)), 'six serving-keezer probes, via the 3-way block', size=1.1)
    gl(s, uno['3V3'], 'UNO_3V3', stub=4)
    s.text((uno['3V3'][0] + g(12), uno['3V3'][1] - g(.8)), 'feeds both 2.2 kΩ pull-ups (keezer and lager buses)', size=1.1)

    # --- 4-ch dash power module ---------------------------------------------
    r4 = s.place(RELAY4, *G(130, 180), ref='K7', value='SunFounder 4-ch relay module, low-level trigger — dash power')
    for io, inp in (('IO17', 'IN1'), ('IO18', 'IN2'), ('IO10', 'IN3')):
        bend(s, pi[io], r4[inp], g(98 + 2 * (['IN1', 'IN2', 'IN3'].index(inp))))
    gl(s, r4['IN4'], 'BREWPI_IO16', 'left', stub=6)
    gl(s, r4['VCC'], 'RPINTS_5V', 'left', stub=6); gl(s, r4['GND'], 'RPINTS_GND', 'left', stub=6)
    xc4 = r4['COM1'][0] + g(4)
    s.wire(r4['COM1'], (xc4, r4['COM1'][1])); s.wire((xc4, r4['COM1'][1]), (xc4, r4['COM4'][1]))
    for c in ('COM2', 'COM3', 'COM4'):
        s.wire((xc4, r4[c][1]), r4[c])
        if c != 'COM4':
            s.junction((xc4, r4[c][1]))
    s.wire((xc4, r4['COM1'][1]), (xc4, r4['COM1'][1] - g(4))); s.glabel((xc4, r4['COM1'][1] - g(4)), 'DASH_POWER_+12V', rot=90)
    for no, net, txt in (('NO1', 'SPEED_+12V', '"SPEED": both switch pods (Pursuit only)'),
                         ('NO2', 'TACHO_+12V', '"TACHO": speedo, tacho, dummy6'),
                         ('NO3', 'DUMMY3_+12V', '"DUMMY3": dummy3 E and F'),
                         ('NO4', 'COMM_+12V', '"COMM": message centre')):
        p = gl(s, r4[no], net, stub=8)
        s.text((p[0] + g(16), p[1] - g(.8)), txt, size=1.1)
    s.text(G(150, 212), 'low-level trigger: panp.py drives the pin to 0 to power a board.  IN1 (IO17) is low only in Pursuit.  Cable 0 V returns to the DASH POWER block (sheet 4).', size=1.1)

    # --- serial and the 12-way strip ----------------------------------------
    sh = s.place(SHIFT, *G(80, 172), ref='U2', value='TXS0108E (right-hand shifter)')
    bend(s, pi['TXD'], sh['A6'], g(62)); bend(s, pi['CE0'], sh['A7'], g(64))
    gl(s, sh['OE'], 'RPINTS_OE', 'left', stub=6); gl(s, sh['VA'], 'RPINTS_3V3', 'left', stub=6); gl(s, sh['GND'], 'RPINTS_GND', 'left', stub=6)
    gl(s, sh['VB'], 'RPINTS_5V', stub=6)
    gl(s, sh['B6'], 'DUMMY_TX', stub=6); gl(s, sh['B7'], 'TACHO_TX', stub=6)
    s.text(G(58, 194), 'ttyAMA0 (TXD) carries the dummy bus (E then G), ttyAMA1 (CE0) the tacho.  Which shifter channel is which (A6/B6 vs A7/B7) is unconfirmed.', size=1.1)

    tb12 = s.place(TB[12], *G(222, 186), ref='J2', value='12-way strip: serial and panel one-wire')
    rows = [('DUMMY_TX', 'solid green → dummy3 E COM → dummy6 G COM'),
            ('TACHO_TX', 'solid green → tacho A COM'),
            ('RPINTS_GND', 'two green/white → board grounds'),
            ('BREWPI_MSGCTR_TX', 'solid brown → message centre C COM'),
            ('BREWPI_SPEEDO_TX', 'solid orange → speedo B COM → dummy3 F'),
            ('BREWPI_GND', 'brown/white, orange/white → board grounds'),
            ('LAGER_1W', 'red → 3 × DS18B20 data, lagering keezer (parasite powered)'),
            ('UNO_3V3', '2.2 kΩ across to 7, from the RaspberryPints Uno 3V3'),
            ('RPINTS_GND', 'white → lager probe ground'),
            ('HOT_1W', 'red → 2 × DS18B20 data, mash and HLT (parasite powered)'),
            ('BREWPI_3V3', '2.2 kΩ across to 10'),
            ('BREWPI_GND', 'white → hot-side probe ground')]
    for i, (net, txt) in enumerate(rows, 1):
        p = tb12[f'Pin_{i}']
        gl(s, p, net, 'left', stub=10)
        s.text((p[0] + g(6), p[1] - g(.8)), f'{i}: {txt}', size=1.0)
    for (a, b, ref, off) in (('Pin_7', 'Pin_8', 'R2', 20), ('Pin_10', 'Pin_11', 'R3', 26)):
        pa, pb = tb12[a], tb12[b]
        xr = pa[0] - g(off)
        r = s.place(RES, xr, (pa[1] + pb[1]) / 2, ref=ref, value='2.2k')
        rb, rt = vres(s, r, 0, 0, 0)
        s.wire((pa[0] - g(10), pa[1]), rt) if False else None
        s.wire(rt, (xr, pa[1])); s.wire((xr, pa[1]), (pa[0] - g(10), pa[1])); s.junction((pa[0] - g(10), pa[1]))
        s.wire(rb, (xr, pb[1])); s.wire((xr, pb[1]), (pb[0] - g(10), pb[1])); s.junction((pb[0] - g(10), pb[1]))
    gl(s, pi['IO26'], 'LAGER_1W', stub=6)
    s.text((pi['IO26'][0] + g(14), pi['IO26'][1] - g(.8)), 'three lagering-keezer probes, via 12-way strip 7', size=1.1)

    # --- right switch pod -----------------------------------------------------
    sp = s.place(UNO_SP, *G(92, 230), ref='A2', value='Arduino Uno · /dev/switchpod · right pod')
    pod = s.place(POD, *G(52, 230), ref='SP1', value='right switch pod, resistive ladder')
    mid = (snapx((pod['SGN'][0] + sp['A0'][0]) / 2), pod['SGN'][1])
    bend(s, pod['SGN'], sp['A0'], mid[0]); s.junction(mid)
    r4r = s.place(RES, mid[0], mid[1] - g(6), ref='R4', value='3k3')
    rb, rt = vres(s, r4r, 0, 0, 0)
    s.wire(mid, rb)
    bendy(s, rt, sp['5V'], rt[1] - g(2)) if False else None
    s.wire(rt, (rt[0], rt[1] - g(2))); s.wire((rt[0], rt[1] - g(2)), (sp['5V'][0] + g(4), rt[1] - g(2))); s.wire((sp['5V'][0] + g(4), rt[1] - g(2)), (sp['5V'][0] + g(4), sp['5V'][1])); s.wire((sp['5V'][0] + g(4), sp['5V'][1]), sp['5V'])
    xg2 = sp['GND'][0] + g(6)
    s.wire(sp['GND'], (xg2, sp['GND'][1])); s.wire((xg2, sp['GND'][1]), (xg2, pod['GND'][1] + g(6))); s.wire((xg2, pod['GND'][1] + g(6)), (pod['GND'][0] + g(2), pod['GND'][1] + g(6))); s.wire((pod['GND'][0] + g(2), pod['GND'][1] + g(6)), (pod['GND'][0] + g(2), pod['GND'][1])); s.wire((pod['GND'][0] + g(2), pod['GND'][1]), pod['GND'])
    gl(s, pod['+12V'], 'SPEED_+12V', stub=6); gl(s, pod['0V'], 'DASH_0V', stub=6)
    s.text(G(40, 246), 'pod lamps are inside the pod, lit from the SPEED cable in Pursuit only; USB from the Uno to rpints hub port 1.4', size=1.1)
    return s.write(os.path.join(OUT, 'sheet2-rpints.kicad_sch'))


# ================================================================= sheet 3
def sheet3():
    s = Schematic('brew-utils sheet 3 — brewpi and the fermenters', paper='A3')
    header(s, 'Sheet 3 — brewpi: relay drives, mode lines, hot-side probes, serial to the message centre and speedo, the left switch pod, '
              'and the three BrewPi Remix controllers with their heaters and glycol pumps.',
           'H1_FEED–H3_FEED and +24V come from sheet 1; the 8-way relay inputs and the 4-ch IN4 land on sheet 2.')
    pi = s.place(PI_B, *G(30, 90), ref='PI2', value='brewpi · Raspberry Pi 4 · 10.0.1.32 · GeeekPi HAT')
    nets = {'IO2': 'BREWPI_IO2', 'IO3': 'BREWPI_IO3', 'IO27': 'BREWPI_IO27', 'IO21': 'BREWPI_IO21', 'IO13': 'BREWPI_IO13',
            'IO16': 'BREWPI_IO16', 'IO5': 'BREWPI_IO5', 'IO7': 'MODE_AUTO', 'IO22': 'MODE_NORM', 'IO4': 'HOT_1W',
            'IO25': 'BREWPI_OE', '5V': 'BREWPI_5V', '3V3': 'BREWPI_3V3', 'GND': 'BREWPI_GND'}
    notes = {'IO2': 'flow meter 1 relay IN1 (sheet 2), active high', 'IO3': 'flow meter 2 relay IN2', 'IO27': 'flow meter 3 relay IN3',
             'IO21': 'flow meter 4 relay IN4', 'IO13': 'flow meter 5 relay IN5', 'IO16': 'message centre power, 4-ch IN4 (sheet 2); held low = on',
             'IO5': 'interlock relay K3 IN (sheet 1); power-relay-brewpi.service', 'IO7': 'from rpints; input with pull-down',
             'IO22': 'from rpints; input with pull-down', 'IO4': 'mash and HLT probes via 12-way strip 10 (sheet 2)'}
    for pn, net in nets.items():
        p = gl(s, pi[pn], net, stub=6)
        if pn in notes:
            s.text((p[0] + g(16), p[1] - g(.8)), notes[pn], size=1.1)

    sh = s.place(SHIFT, *G(80, 150), ref='U3', value='TXS0108E (left-hand shifter)')
    bend(s, pi['CE0'], sh['A6'], g(62)); bend(s, pi['TXD'], sh['A7'], g(64))
    gl(s, sh['OE'], 'BREWPI_OE', 'left', stub=6); gl(s, sh['VA'], 'BREWPI_3V3', 'left', stub=6); gl(s, sh['GND'], 'BREWPI_GND', 'left', stub=6)
    gl(s, sh['VB'], 'BREWPI_5V', stub=6)
    p = gl(s, sh['B6'], 'BREWPI_SPEEDO_TX', stub=6); s.text((p[0] + g(18), p[1] - g(.8)), 'ttyAMA1 (CE0, blue) → 12-way strip 5 → speedo B → dummy3 F', size=1.1)
    p = gl(s, sh['B7'], 'BREWPI_MSGCTR_TX', stub=6); s.text((p[0] + g(18), p[1] - g(.8)), 'ttyAMA0 (TXD, yellow) → 12-way strip 4 → message centre C', size=1.1)

    sp = s.place(UNO_SP, *G(92, 200), ref='A3', value='Arduino Uno · /dev/switchpod · left pod')
    pod = s.place(POD, *G(52, 200), ref='SP2', value='left switch pod, resistive ladder')
    mid = (snapx((pod['SGN'][0] + sp['A0'][0]) / 2), pod['SGN'][1])
    bend(s, pod['SGN'], sp['A0'], mid[0]); s.junction(mid)
    r5 = s.place(RES, mid[0], mid[1] - g(6), ref='R5', value='3k3')
    rb, rt = vres(s, r5, 0, 0, 0)
    s.wire(mid, rb)
    s.wire(rt, (rt[0], rt[1] - g(2))); s.wire((rt[0], rt[1] - g(2)), (sp['5V'][0] + g(4), rt[1] - g(2))); s.wire((sp['5V'][0] + g(4), rt[1] - g(2)), (sp['5V'][0] + g(4), sp['5V'][1])); s.wire((sp['5V'][0] + g(4), sp['5V'][1]), sp['5V'])
    xg2 = sp['GND'][0] + g(6)
    s.wire(sp['GND'], (xg2, sp['GND'][1])); s.wire((xg2, sp['GND'][1]), (xg2, pod['GND'][1] + g(6))); s.wire((xg2, pod['GND'][1] + g(6)), (pod['GND'][0] + g(2), pod['GND'][1] + g(6))); s.wire((pod['GND'][0] + g(2), pod['GND'][1] + g(6)), (pod['GND'][0] + g(2), pod['GND'][1])); s.wire((pod['GND'][0] + g(2), pod['GND'][1]), pod['GND'])
    gl(s, pod['+12V'], 'SPEED_+12V', stub=6); gl(s, pod['0V'], 'DASH_0V', stub=6)
    s.text(G(40, 216), 'USB from the Uno to brewpi hub port 1.4', size=1.1)

    # --- BrewPi controllers ----------------------------------------------------
    r6 = s.place(RELAY6, *G(236, 100), ref='K8', value='ANMBEST 6-ch relay module, high-level trigger')
    gl(s, r6['VCC'], 'BREWPI_5V', 'left', stub=6); gl(s, r6['GND'], 'BREWPI_GND', 'left', stub=6)
    for i, (name, port, addr, ref, pref) in enumerate((
            ('unitank-1', '1.3', '28BD2057047A3C27', 'A4', 'T1'),
            ('unitank-2', '1.1', '28EACCED582001EE', 'A5', 'T2'),
            ('chronical', '1.2', '283213C85820014B', 'A6', 'T3'))):
        y = 44 + i * 44
        u = s.place(UNO_BP, *G(176, y), ref=ref, value=f'Uno · /dev/{name} · BrewPi Remix · USB port {port}')
        pr = s.place(PROBES, *G(130, y), ref=pref, value=f'DS18B20 {name} thermowell  {addr}')
        xd = snapx((pr['DQ'][0] + u['A0'][0]) / 2)
        bend(s, pr['DQ'], u['A0'], xd); s.junction((xd, pr['DQ'][1]))
        rr = s.place(RES, xd, pr['DQ'][1] - g(6), ref=f'R{6+i}', value='4.7k')
        rb, rt = vres(s, rr, 0, 0, 0)
        s.wire((xd, pr['DQ'][1]), rb)
        ytop = rt[1] - g(2)
        x5 = u['5V'][0] - g(4)
        s.wire(rt, (rt[0], ytop)); s.wire((rt[0], ytop), (x5, ytop)); s.wire((x5, ytop), (x5, u['5V'][1])); s.wire((x5, u['5V'][1]), u['5V'])
        xv = pr['VDD'][0] + g(3)
        s.wire(pr['VDD'], (xv, pr['VDD'][1])); s.wire((xv, pr['VDD'][1]), (xv, ytop - g(2))); s.wire((xv, ytop - g(2)), (x5, ytop - g(2))); s.wire((x5, ytop - g(2)), (x5, ytop)); s.junction((x5, ytop))
        xgn = pr['GND'][0] + g(5); ygn = u['GND'][1] + g(4)
        s.wire(pr['GND'], (xgn, pr['GND'][1])); s.wire((xgn, pr['GND'][1]), (xgn, ygn)); s.wire((xgn, ygn), (u['GND'][0] - g(2), ygn)); s.wire((u['GND'][0] - g(2), ygn), (u['GND'][0] - g(2), u['GND'][1])); s.wire((u['GND'][0] - g(2), u['GND'][1]), u['GND'])
        strip = ('1–3', '4–6', '8–10')[i]
        s.text((pr['GND'][0] - g(24), pr['GND'][1] + g(6)), f'10-way strip positions {strip}: data · 5 V · GND;  4.7 kΩ pull-up to the Uno 5 V', size=1.1)
        heat_in, cool_in = f'IN{i+1}', f'IN{i+4}'
        bend(s, u['D5'], r6[heat_in], r6[heat_in][0] - g(14 + 2 * i))
        bend(s, u['D6'], r6[cool_in], r6[cool_in][0] - g(8 + 2 * i))
        s.label((u['D5'][0] + g(1), u['D5'][1]), f'HEAT{i+1}'); s.label((u['D6'][0] + g(1), u['D6'][1]), f'COOL{i+1}')
        gl(s, r6[f'COM{i+1}'], f'H{i+1}_FEED', stub=6)
        h = s.place(HEATER, r6[f'NO{i+1}'][0] + g(28), r6[f'NO{i+1}'][1] + g(1), ref=f'H{i+1}', value=f'FTSs heating pad 12 V · {name}')
        s.wire(r6[f'NO{i+1}'], h['+'])
        s.wire(h['-'], (h['-'][0] - g(3), h['-'][1])); s.wire((h['-'][0] - g(3), h['-'][1]), (h['-'][0] - g(3), h['-'][1] + g(3))); s.glabel((h['-'][0] - g(3), h['-'][1] + g(3)), '0V', rot=270)
        gl(s, r6[f'COM{i+4}'], '+24V', stub=6)
        m = s.place(PUMP, r6[f'NO{i+4}'][0] + g(28), r6[f'NO{i+4}'][1] + g(1), ref=f'M{i+1}', value=f'Penguin XL glycol pump 24 V · {name}')
        s.wire(r6[f'NO{i+4}'], m['+'])
        s.wire(m['-'], (m['-'][0] - g(3), m['-'][1])); s.wire((m['-'][0] - g(3), m['-'][1]), (m['-'][0] - g(3), m['-'][1] + g(3))); s.glabel((m['-'][0] - g(3), m['-'][1] + g(3)), '24V_RTN', rot=270)
    s.text(G(176, 184), 'BrewPi device config on all three: A0 chamber temp, pin 5 chamber heater (Act 1), pin 6 chamber cooler (Act 2), "not inverted" — a high closes the relay.\n'
                        'Heater feeds are the 7.5 A fuses V3–V5 via strip 2; heater 0 V returns to the fuse block V−.  Pump 24 V comes via strip 3 and leaves on strip 4.\n'
                        'Heat/cool relays: unitank-1 = 1/4, unitank-2 = 2/5, chronical = 3/6.  Unos left→right on the panel in that order.', size=1.1)
    return s.write(os.path.join(OUT, 'sheet3-brewpi.kicad_sch'))


# ================================================================= sheet 4
def sheet4():
    s = Schematic('brew-utils sheet 4 — panel distribution and the dash', paper='A3')
    header(s, 'Sheet 4 — strip 1 to the Pi converters, the DASH POWER block to the six dash boards, and the four PANP switches as they land on the two 8-way strips.',
           'The dash boards take +12 V / GND on screw terminals and receive serial on COM (buses on sheets 2 and 3).')
    s1 = s.place(TB[6], *G(40, 40), ref='J3', value='distribution strip 1 — "P1"')
    for i, txt in enumerate(['0 V → SSLHONG #1 IN−', '0 V → SSLHONG #2 IN−', '0 V in (P1) · PANP lamp 0 V → middle strip 8',
                             '+12 V in (P1) · SSLHONG #1 IN+', '+12 V → SSLHONG #2 IN+', '+12 V → 8-way relay COM6–8 (PANP lamps)'], 1):
        s.text((s1[f'Pin_{i}'][0] + g(6), s1[f'Pin_{i}'][1] - g(.8)), f'{i}: {txt}', size=1.1)
    x0 = s1['Pin_1'][0] - g(6)
    s.wire(s1['Pin_1'], (x0, s1['Pin_1'][1])); s.wire((x0, s1['Pin_1'][1]), (x0, s1['Pin_3'][1])); s.junction((x0, s1['Pin_2'][1])); s.wire((x0, s1['Pin_2'][1]), s1['Pin_2']); s.wire((x0, s1['Pin_3'][1]), s1['Pin_3'])
    s.wire((x0, s1['Pin_3'][1]), (x0, s1['Pin_3'][1] + g(3))); s.glabel((x0, s1['Pin_3'][1] + g(3)), '0V', rot=270)
    s.wire((x0, s1['Pin_1'][1]), (x0, s1['Pin_1'][1] - g(3))); s.glabel((x0, s1['Pin_1'][1] - g(3)), '0V', rot=90)
    x1 = s1['Pin_4'][0] - g(12)
    s.wire(s1['Pin_4'], (x1, s1['Pin_4'][1])); s.wire((x1, s1['Pin_4'][1]), (x1, s1['Pin_6'][1])); s.junction((x1, s1['Pin_5'][1])); s.wire((x1, s1['Pin_5'][1]), s1['Pin_5']); s.wire((x1, s1['Pin_6'][1]), s1['Pin_6'])
    s.wire((x1, s1['Pin_6'][1]), (x1, s1['Pin_6'][1] + g(3))); s.glabel((x1, s1['Pin_6'][1] + g(3)), 'P1_+12V', rot=270)
    b1 = s.place(BUCK, *G(120, 34), ref='U4', value='SSLHONG buck #1 → rpints')
    b2 = s.place(BUCK, *G(120, 52), ref='U5', value='SSLHONG buck #2 → brewpi')
    for b in (b1, b2):
        gl(s, b['IN+'], 'P1_+12V', 'left', stub=6); gl(s, b['IN-'], '0V', 'left', stub=6)
    p = gl(s, b1['USB-C'], 'RPINTS_5V', stub=6); s.text((p[0] + g(12), p[1] - g(.8)), 'USB-C into the rpints Pi 4', size=1.1)
    p = gl(s, b2['USB-C'], 'BREWPI_5V', stub=6); s.text((p[0] + g(12), p[1] - g(.8)), 'USB-C into the brewpi Pi 4', size=1.1)
    s.text(G(100, 64), 'Both Pis are on the switched rail: the POWER latch, or pulling fuse V1, cuts them.', size=1.1)

    dp = s.place(TB[4], *G(40, 96), ref='J4', value='"DASH POWER" 4-way block')
    for i, txt in enumerate(['0 V, jumpered to 2 — return for the four dash cables', '0 V in from fuse V2', '+12 V in from fuse V2', '+12 V, jumpered to 3 → 4-ch relay COM1–4'], 1):
        s.text((dp[f'Pin_{i}'][0] + g(6), dp[f'Pin_{i}'][1] - g(.8)), f'{i}: {txt}', size=1.1)
    xd = dp['Pin_1'][0] - g(6)
    s.wire(dp['Pin_1'], (xd, dp['Pin_1'][1])); s.wire((xd, dp['Pin_1'][1]), (xd, dp['Pin_2'][1])); s.wire((xd, dp['Pin_2'][1]), dp['Pin_2'])
    s.wire((xd, dp['Pin_1'][1]), (xd, dp['Pin_1'][1] - g(3))); s.glabel((xd, dp['Pin_1'][1] - g(3)), 'DASH_0V', rot=90)
    s.wire((xd, dp['Pin_2'][1]), (xd, dp['Pin_2'][1] + g(3))); s.glabel((xd, dp['Pin_2'][1] + g(3)), '0V', rot=270)
    xe = dp['Pin_3'][0] - g(12)
    s.wire(dp['Pin_3'], (xe, dp['Pin_3'][1])); s.wire((xe, dp['Pin_3'][1]), (xe, dp['Pin_4'][1])); s.wire((xe, dp['Pin_4'][1]), dp['Pin_4'])
    s.wire((xe, dp['Pin_4'][1]), (xe, dp['Pin_4'][1] + g(3))); s.glabel((xe, dp['Pin_4'][1] + g(3)), 'DASH_POWER_+12V', rot=270)
    s.text(G(20, 116), 'DASH_0V is the return for every dash board and both pods, back to the fuse block V− (0V).', size=1.1)

    boards = [('DB1', 'A tacho', 'TACHO_+12V', 'TACHO_TX'), ('DB2', 'G dummy6', 'TACHO_+12V', 'DUMMY_TX'),
              ('DB3', 'B speedo', 'TACHO_+12V', 'BREWPI_SPEEDO_TX'), ('DB4', 'F red/green dummy3', 'DUMMY3_+12V', 'BREWPI_SPEEDO_TX'),
              ('DB5', 'E red dummy3', 'DUMMY3_+12V', 'DUMMY_TX'), ('DB6', 'C message centre', 'COMM_+12V', 'BREWPI_MSGCTR_TX')]
    for i, (ref, name, pwr, com) in enumerate(boards):
        x = 200 + (i % 3) * 56; y = 40 + (i // 3) * 30
        b = s.place(BOARD, *G(x, y), ref=ref, value=f'ideegeniali {name}')
        gl(s, b['+12V'], pwr, 'left', stub=6); gl(s, b['GND'], 'DASH_0V', 'left', stub=6); gl(s, b['COM'], com, 'left', stub=6)
    s.text(G(170, 104), 'Dummy chains: E → G on the rpints dummy bus, B → F on the brewpi speedo bus.  The boards\' own TX terminals are unconnected.', size=1.1)

    low = s.place(TB[8], *G(60, 172), ref='J5', value='lower 8-way strip (tagged PURS · AUTO · NORM)')
    mid = s.place(TB[8], *G(60, 222), ref='J6', value='middle 8-way strip')
    lown = [('POWER NO (yellow/green) → interlock chain, sheet 1', 'PWR_BTN_NO'), ('POWER C → latch TRIG_RTN, sheet 1', 'PWR_BTN_C'),
            ('POWER lamp +12 V ← "master relay"', 'MASTER_RELAY'), ('POWER lamp 0 V', '0V'),
            ('AUTO NO → rpints IO5 (purple)', 'AUTO_NO'), ('NORM NO → rpints IO6 (blue)', 'NORM_NO'),
            ('PURSUIT NO → rpints IO16 (green)', 'PURSUIT_NO'), ('C common → middle strip 1', 'RPINTS_GND')]
    midn = [('C common ← lower strip 8', 'RPINTS_GND'), ('C common → rpints GND', 'RPINTS_GND'),
            ('AUTO lamp +12 V ← relay 8 NO (red)', 'AUTO_LAMP'), ('NORM lamp +12 V ← relay 7 NO (brown)', 'NORM_LAMP'),
            ('PURSUIT lamp +12 V ← relay 6 NO (black)', 'PURSUIT_LAMP'), ('lamp 0 V, jumpered from 7', '0V'),
            ('lamp 0 V, jumpered from 8', '0V'), ('lamp 0 V ← strip 1 terminal 3', '0V')]
    for tb, names in ((low, lown), (mid, midn)):
        for i, (txt, net) in enumerate(names, 1):
            p = tb[f'Pin_{i}']
            s.text((p[0] + g(6), p[1] - g(.8)), f'{i}: {txt}', size=1.0)
            gl(s, p, net, 'left', stub=10)
    p = mid['Pin_2']; s.wire((p[0] - g(10), p[1]), (p[0] - g(10), p[1] + g(3))) if False else None
    s.text(G(20, 240), 'The switches\' commoned C reaches rpints GND via middle strip 2; the lamp 0 V lines reach strip 1 terminal 3 (0V) via middle strip 8.', size=1.1)
    for i, (ref, name, no_net, c_net, lamp_p, lamp_n) in enumerate((
            ('SW5', 'PURSUIT', 'PURSUIT_NO', 'RPINTS_GND', 'PURSUIT_LAMP', '0V'),
            ('SW6', 'NORM', 'NORM_NO', 'RPINTS_GND', 'NORM_LAMP', '0V'),
            ('SW7', 'AUTO', 'AUTO_NO', 'RPINTS_GND', 'AUTO_LAMP', '0V'),
            ('SW1', 'POWER', 'PWR_BTN_NO', 'PWR_BTN_C', 'MASTER_RELAY', '0V'))):
        b = s.place(PANPSW, *G(200 + (i % 2) * 76, 150 + (i // 2) * 44), ref=ref, value=f'{name} · MICRO SWITCH 4A13BAA31')
        gl(s, b['NO'], no_net, 'left', stub=6); gl(s, b['C'], c_net, 'left', stub=6); s.nc(b['NC'])
        gl(s, b['LAMP5'], lamp_p, stub=6); gl(s, b['LAMP2'], lamp_n, stub=6)
    s.text(G(176, 236), 'Four wires per switch: C, NO, lamp 5, lamp 2 — two uxcell 8-core cables.  Pursuit and Power are mounted upside down relative to Auto and Norm.', size=1.1)
    return s.write(os.path.join(OUT, 'sheet4-dash.kicad_sch'))


if __name__ == '__main__':
    for fn in (sheet1, sheet2, sheet3, sheet4):
        print(fn())
