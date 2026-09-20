#!/usr/bin/python3

"""Drive the KITT dash from brewery data.

One script runs on both Raspberry Pis and decides which it is from the
hostname: `rpints` owns the PANP buttons, the dash power relays, the tacho and
the two dummy boards; `brewpi` owns the speedo, the message centre and the
flow meter relays. See kitt/README.md for how it fits together and
kitt/DASHBOARD.md for what every gauge means.
"""

import json
import math
import os
import signal
import socket
import ssl
import statistics
import sys
import threading
import time
import urllib.request
from enum import Enum
from queue import Queue
from threading import Thread

import mysql.connector as database
import RPi.GPIO as GPIO
import sdnotify
import serial
from serial.serialutil import PortNotOpenError


# ---------------------------------------------------------------------------
# Pins (board numbering)
# ---------------------------------------------------------------------------

# common to both hosts
SERIAL_ENABLE = 22        # GPIO 25, enables the 3.3V to 5V serial converter
AUTO_MODE_COMM = 26       # GPIO 7 (CE1), rpints tells brewpi it is in auto
NORMAL_MODE_COMM = 15     # GPIO 22, rpints tells brewpi it is in norm

# rpints
LOWER_DASH_POWER = 19     # GPIO 10 (MOSI)
UPPER_DASH_POWER = 12     # GPIO 18
SP_POWER = 11             # GPIO 17

# brewpi
MSGCTR_POWER = 36         # GPIO 16
FLOWMETER = [3, 5, 13, 40, 33]   # GPIO 2 (SCL), 3 (SDA), 27, 21, 13


class PANPState(Enum):
    """PANP lamps, by pin."""
    AUTO = 35       # GPIO 19
    NORM = 38       # GPIO 20
    PURSUIT = 32    # GPIO 12


class PANPButton(Enum):
    """PANP buttons, by pin."""
    AUTO = 29       # GPIO 5
    NORM = 31       # GPIO 6
    PURSUIT = 36    # GPIO 16


# ---------------------------------------------------------------------------
# Timing and cadence
# ---------------------------------------------------------------------------

SERIAL_GAP = 0.1          # the boards drop messages sent closer together than this
LOOP_PERIOD = 0.25        # seconds between passes of the main loop
STARTUP_SETTLE = 3        # seconds for the threads to start before the first pass
DATABASE_EVERY = 11       # passes between RaspberryPints queries
PROBE_PERIOD = 1          # seconds between one-wire probe reads
PROBE_MISSES = 10         # failed reads in a row before a probe is shown as 0.0
BREWPI_PERIOD = 60        # seconds between BrewPi Remix polls
SWITCHPOD_RETRY = 2       # seconds before reopening a switchpod that went away
BRIGHTNESS_REPEATS = 3    # a brightness message is sent this many times

BAUD = 57600


# ---------------------------------------------------------------------------
# Probes and fermenters
# ---------------------------------------------------------------------------

# the six serving keezer probes, in the order the tacho bars show them
KEEZER_PROBES = [
    "/sys/bus/w1/devices/28-012052b92541/w1_slave",
    "/sys/bus/w1/devices/28-012058f936f3/w1_slave",
    "/sys/bus/w1/devices/28-012052ba8dab/w1_slave",
    "/sys/bus/w1/devices/28-012058fbceb5/w1_slave",
    "/sys/bus/w1/devices/28-012058fc2851/w1_slave",
    "/sys/bus/w1/devices/28-012052b426ca/w1_slave"]

# the three lager fridge probes
LAGER_PROBES = [
    "/sys/bus/w1/devices/28-3c01b556f6d0/w1_slave",
    "/sys/bus/w1/devices/28-3cdd04574813/w1_slave",
    "/sys/bus/w1/devices/28-3ce80457395a/w1_slave"]

# mash tun, then hot liquor tank
HOT_SIDE_PROBES = [
    "/sys/bus/w1/devices/28-012052b65be5/w1_slave",
    "/sys/bus/w1/devices/28-0120529d8f20/w1_slave"]

# the BrewPi Remix instances, each with a KITTSOCKET in its home directory
FERMENTERS = ["unitank-1", "unitank-2", "chronical"]


# ---------------------------------------------------------------------------
# Display modes
# ---------------------------------------------------------------------------

class TachoView(Enum):
    """What the tacho digits and arc show."""
    MEAN = 0          # mean of the six serving keezer probes
    KEG_TEMP = 1      # the selected keg's keezer probe
    KEG_LITRES = 2    # the selected keg: litres left on the digits, percent full on the arc
    KEG_GALLONS = 3   # the same keg, gallons to a tenth on the digits
    LAGER_TEMP = 4    # the selected lager probe


class MsgCtrMode(Enum):
    """What the lower speedo display shows, and so what the message centre captions."""
    MASH_TEMP = 0
    HLT_TEMP = 1
    UNITANK1_TEMP = 2
    UNITANK1_SG = 3
    UNITANK2_TEMP = 4
    UNITANK2_SG = 5
    CHRONICAL_TEMP = 6
    CHRONICAL_SG = 7
    BREWPI_UP = 8
    MASH_TEMP_C = 9   # no button selects this
    FLOWMETER = 10


class TankDisplay(Enum):
    """Which of temperature and gravity a fermenter button shows on its next press."""
    TEMP = 0
    SG = 1


# right switch pod, left column: position -> keg 1-5 as an index. keg n sits on
# keezer probe n. each press steps the tacho through that keg's views, in
# KEG_VIEWS order, and the press after the last goes back to the mean
RIGHT_POD_KEGS = {
    0: 0,   # TURBO BOOST
    2: 1,   # 7 DLA
    4: 2,   # 8 PL1
    6: 3,   # 6 RM (orange)
    8: 4,   # H6
}
KEG_VIEWS = [TachoView.KEG_TEMP, TachoView.KEG_LITRES, TachoView.KEG_GALLONS]

# right switch pod, right column: position -> lager probe as an index. one
# press shows it, the next goes back to the mean
RIGHT_POD_LAGERS = {
    1: 0,   # 6 RM (white)
    3: 1,   # P ENG
    5: 2,   # AUTO ROOF R
}

# right switch pod, right column: the brewery room lights over the hue bridge
RIGHT_POD_LIGHTS_ON = 7    # P IND
RIGHT_POD_LIGHTS_OFF = 9   # EJECT R

# each tap's keg when full, in US gallons: four kegs and, on tap 5, the 2.5
# gallon cask on the beer engine. RaspberryPints reports volumes in gallons
KEG_GALLONS = [5.0, 5.0, 5.0, 5.0, 2.5]
LITRES_PER_GALLON = 3.78541

# the arc shows a keg's percent full, and is full from this percentage up
ARC_FULL_PERCENT = 80.0

# left switch pod, left column: position -> a fixed caption
LEFT_POD_CAPTIONS = {
    0: (MsgCtrMode.MASH_TEMP, ">CScDEG F MASH?"),   # SILENT MODE
    2: (MsgCtrMode.HLT_TEMP, ">CScDEG F HLT?"),     # TEAR GAS
}

# left switch pod, left column: position -> a fermenter that alternates
# temperature and gravity: (tank index, temperature mode, gravity mode, name)
LEFT_POD_TANKS = {
    4: (0, MsgCtrMode.UNITANK1_TEMP, MsgCtrMode.UNITANK1_SG, "UTK1"),   # AUTO ROOF L
    6: (1, MsgCtrMode.UNITANK2_TEMP, MsgCtrMode.UNITANK2_SG, "UTK2"),   # MICRO-JAM
    8: (2, MsgCtrMode.CHRONICAL_TEMP, MsgCtrMode.CHRONICAL_SG, "CHRN"),  # EJECT L
}

# left switch pod, right column: position -> flow meter relay to toggle
LEFT_POD_FLOWMETERS = {
    1: 0,   # LASER
    3: 1,   # PAUX
    5: 2,   # GRPLG. HOOK
    7: 3,   # SMOKE RELEASE
    9: 4,   # H6
}


# The speedo's two led lines take a step count, and the board clamps it to the
# line's own maximum. Neither line is one led per step: some steps light a
# pair. Measured on the board, September 2026.
SPEED_LINE_STEPS = 19     # 20 leds above MPH, one step lights two
FUEL_LINE_STEPS = 13      # 16 leds above the multifunction display, three steps light two


def _span(lo, hi):
    """Steps to light on the multifunction line for a value on a linear scale.

    Dark at `lo`, full at `hi`, whatever the line's step count is.
    """
    return lambda value: (value - lo) / (hi - lo) * FUEL_LINE_STEPS


# what the lower speedo display shows in each mode: the value, the multifunction
# line's span for it, and the decimal point position (1 is 000.0, 3 is 0.000).
# gravities arrive already multiplied by 100, so 1.050 is 105.0 here
LOWER_DISPLAY = {
    MsgCtrMode.MASH_TEMP:      (lambda h: h.hot_side_temps[0], _span(110.0, 190.0), 1),
    MsgCtrMode.MASH_TEMP_C:    (lambda h: (h.hot_side_temps[0] - 32.0) * 5.0 / 9.0, _span(20.0, 84.0), 1),
    MsgCtrMode.UNITANK1_TEMP:  (lambda h: h.brewpi_rmx_data[0], _span(34.0, 82.0), 1),
    MsgCtrMode.UNITANK2_TEMP:  (lambda h: h.brewpi_rmx_data[2], _span(34.0, 82.0), 1),
    MsgCtrMode.CHRONICAL_TEMP: (lambda h: h.brewpi_rmx_data[4], _span(34.0, 82.0), 1),
    MsgCtrMode.BREWPI_UP:      (lambda h: h.hot_side_temps[0], _span(45.0, 205.0), 1),
    MsgCtrMode.HLT_TEMP:       (lambda h: h.hot_side_temps[1], _span(110.0, 190.0), 1),
    MsgCtrMode.UNITANK1_SG:    (lambda h: h.brewpi_rmx_data[1] * 100.0, _span(100.0, 106.4), 3),
    MsgCtrMode.UNITANK2_SG:    (lambda h: h.brewpi_rmx_data[3] * 100.0, _span(100.0, 106.4), 3),
    MsgCtrMode.CHRONICAL_SG:   (lambda h: h.brewpi_rmx_data[5] * 100.0, _span(100.0, 106.4), 3),
}

# the message centre's own rotation while the dash is dark
MSGCTR_IDLE = ">CSbBREWPI UP|BREWPI UP~?"


# ---------------------------------------------------------------------------
# Bar graph scaling
# ---------------------------------------------------------------------------
#
# Each bar takes a single byte and lights that many segments. These lists map a
# temperature or a percentage onto the byte values that land on consecutive fill
# steps; tuning the dash means editing them. See kitt/DASHBOARD.md for the
# segment counts and colour layouts they are written against.

# the tacho's rpm arc, 30 leds, one per step: the step number is sent raw
RPM_CIRCLE_TEMPS = [2.0, 4.0, 6.0, 8.0, 10.0,
                    12.5, 15.0, 17.5, 20.0,
                    22.5, 25.0, 27.5, 30.0,
                    33.0, 36.0, 40.0,
                    43.0, 46.0, 49.0,
                    52.5, 55.0, 57.5, 60.0,
                    62.5, 65.0, 67.5, 70.0,
                    72.5, 75.0, 77.5, 80.0]

# the six tacho bars, 8 steps landing on steps 1-8, so never fully dark
TACHO_BAR_TEMPS = [30.0, 33.0, 36.0, 39.0, 43.0, 47.0, 48.0, 51.0]
TACHO_BAR_VALUES = [0x20, 0x38, 0x54, 0x70, 0x90, 0xA8, 0xC4, 0xE0]

# the dummy bars take 16 steps; these 17 values land on steps 0-16
DUMMY_BAR_VALUES = [0x00, 0x10, 0x20, 0x30, 0x40, 0x50,
                    0x60, 0x70, 0x79, 0x88, 0x97,
                    0xA6, 0xB5, 0xC4, 0xD3, 0xE2, 0xFF]

# keg volume, as a percentage of the starting volume
KEG_BAR_VOLS = [4.1, 12.3, 16.4, 24.6, 32.8,
                36.9, 45.1, 49.9, 53.3, 61.5, 65.6,
                73.8, 82.0, 86.1, 94.3, 98.0, 99.9]

# lager fridge temperature
LAGER_BAR_TEMPS = [23.5, 24.0, 25.0, 26.5, 27.5,
                   28.0, 29.0, 29.5, 30.0, 30.5,
                   31.0, 31.5, 32.5, 33.5, 34.0, 35.0, 35.5]

# fermenter temperature; steps 1-16 of the dummy values, so never fully dark
TEMPERATURE_BAR_TEMPS = [30.0, 34.0, 37.0, 41.0, 44.0,
                         46.0, 50.0, 52.0, 54.0, 58.0, 60.0,
                         64.0, 68.0, 70.0, 74.0, 76.0]
TEMPERATURE_BAR_VALUES = DUMMY_BAR_VALUES[1:]


def step(value, thresholds):
    """Index of the first threshold the value is below.

    Falling off the end clamps to the last index on purpose, so a value above
    the top of the list shows a full bar.
    """
    for i, t in enumerate(thresholds):
        if value < t:
            break
    return i


def rpm_circle(temp):
    return step(temp, RPM_CIRCLE_TEMPS)


def arc_percent(percent):
    """Arc steps for a keg's percent full: dark at empty, all 30 from ARC_FULL_PERCENT up."""
    steps = int(percent / ARC_FULL_PERCENT * len(RPM_CIRCLE_TEMPS[:-1]))
    return max(0, min(len(RPM_CIRCLE_TEMPS[:-1]), steps))


def digit_byte(value):
    """A value for the tacho's two digits as a register byte; over 99 the board shows HI.

    Rounds halves up, so 1.25 gallons reads 1.3 rather than Python's 1.2.
    """
    return max(0, min(255, int(math.floor(value + 0.5))))


def tacho_bar(temp):
    return TACHO_BAR_VALUES[step(temp, TACHO_BAR_TEMPS)]


def keg_bar(vol):
    return DUMMY_BAR_VALUES[step(vol, KEG_BAR_VOLS)]


def lager_bar(temp):
    return DUMMY_BAR_VALUES[step(temp, LAGER_BAR_TEMPS)]


def temperature_bar(temp):
    return TEMPERATURE_BAR_VALUES[step(temp, TEMPERATURE_BAR_TEMPS)]


def temperature_digits(value):
    """Split a value into the hundreds, tens, ones and tenths digits the speedo wants."""
    tenths = int(round(value % 1 * 10.0, 0))
    ones = int(value % 10)
    tens = int(value // 10 % 10)
    hundreds = int(value // 100 % 10)
    return hundreds, tens, ones, tenths


# ---------------------------------------------------------------------------
# Plumbing
# ---------------------------------------------------------------------------

def drain(q):
    """Yield everything queued so far without blocking."""
    while q.qsize() > 0:
        yield q.get()


class Wire:
    """Hands out turns to transmit to the dash, first come first served.

    One of these is shared by both buses on a Pi, and every packet on either
    bus takes a turn and waits SERIAL_GAP before going out. That buys two
    things. The boards drop packets that follow another on the same bus by
    less than the gap. And the two buses are neighbouring channels of one
    level shifter on long cable runs, so a packet on one is corrupted when the
    other is transmitting at the same instant; with a single gate that cannot
    happen.

    Turns are served in the order they were asked for. A plain lock lets the
    thread that just released it take it straight back, so a burst of writes
    from the GPIO callback thread would hold the main loop off the bus for
    seconds; with tickets the two interleave packet by packet.
    """

    def __init__(self):
        self.cond = threading.Condition()
        self.next_ticket = 0
        self.serving = 0

    def __enter__(self):
        with self.cond:
            ticket = self.next_ticket
            self.next_ticket += 1
            while self.serving != ticket:
                self.cond.wait()

    def __exit__(self, *exc):
        with self.cond:
            self.serving += 1
            self.cond.notify_all()


class Bus:
    """One serial bus to the dash. All writes go through the shared Wire.

    `boards` is the destination letters of the boards strapped to this bus,
    as traced in HARDWARE.md; a message for any other board need not be sent
    on it.
    """

    def __init__(self, device, wire, boards):
        self.port = serial.Serial(device, BAUD)
        self.wire = wire
        self.boards = boards

    def write(self, msg):
        with self.wire:
            time.sleep(SERIAL_GAP)
            self.port.write(msg.encode())

    def close(self):
        self.port.close()


class Service:
    """Process-wide state: systemd, the shutdown flag, and what to close on exit."""

    def __init__(self):
        self.hostname = socket.gethostname()
        self.pid = os.getpid()
        self.notifier = sdnotify.SystemdNotifier()
        self.running = False
        self.wire = Wire()
        self.buses = []
        self.teardown = None
        # objects whose bound methods are GPIO callbacks; kept so they stay alive
        self.handlers = []

    def status(self, msg):
        print(msg)
        self.notifier.notify(f"STATUS={msg}")

    def open_bus(self, device, boards):
        bus = Bus(device, self.wire, boards)
        self.buses.append(bus)
        return bus

    def sigterm(self, _signo, _frame):
        self.running = False
        self.status(f"PANP service PID {self.pid} on {self.hostname} got SIGTERM")
        if self.teardown is not None:
            self.teardown()
        GPIO.output(SERIAL_ENABLE, 0)
        self.shutdown()

    def shutdown(self):
        """Close the ports and leave. Does not return."""
        self.status(f"PANP service on {self.hostname} shutting down")
        time.sleep(1)
        for bus in reversed(self.buses):
            bus.close()
        self.status(f"PANP service on {self.hostname} exiting cleanly")
        self.notifier.notify("STOPPING=1")
        sys.exit(0)


# ---------------------------------------------------------------------------
# Background readers, each a daemon thread feeding a queue
# ---------------------------------------------------------------------------

def read_probe_f(path):
    """Temperature of a one-wire probe in degrees F, or None if it cannot be read.

    The kernel reports the CRC of the reading on the first line; a reading
    that failed its CRC is treated as no reading.
    """
    try:
        with open(path, 'r') as fileobj:
            lines = fileobj.readlines()
        if 'YES' not in lines[0]:
            return None
        equals_pos = lines[1].find('t=')
        tempstr = lines[1][equals_pos + 2:]
        tempvalue_c = float(tempstr) / 1000.0
        return tempvalue_c * 9.0 / 5.0 + 32.0
    except Exception:
        return None


def get_temps(probes, q):
    """Queue (index, temperature) for each probe, once a second.

    A read that fails is skipped, so the display keeps the last good value:
    the one-wire line hiccups for a second now and then and every probe on it
    comes back empty together, which used to paint zeros across the dash. A
    probe that fails PROBE_MISSES reads in a row is reported as 0.0, so one
    that has really gone still shows.
    """
    misses = [0] * len(probes)
    while True:
        try:
            for i, path in enumerate(probes):
                temp = read_probe_f(path)
                if temp is None:
                    misses[i] += 1
                    if misses[i] < PROBE_MISSES:
                        continue
                    temp = 0.0
                else:
                    misses[i] = 0
                q.put((i, temp))
        except Exception:
            pass
        time.sleep(PROBE_PERIOD)


def get_switchpod(sp_q, service):
    """Queue the position of each switchpod key press."""
    path = "/dev/switchpod"
    rx = None
    while True:
        try:
            if rx is None:
                while os.access(path, os.R_OK) is False:
                    time.sleep(1)
                rx = serial.Serial(path, 9600, timeout=None)
            data = rx.readline().decode().strip()
            if not data:
                # switchpod firmware before 2026-09 wrapped the line every
                # twentieth press; harmless, so skip instead of erroring
                continue
            sp_q.put(int(data))
        except Exception:
            # either the input did not parse or the adapter went away. drop
            # the port and pick it up from scratch, waiting for it to exist.
            # the sleep matters: without it a missing device spins the cpu
            try:
                rx.close()
            except Exception:
                pass
            rx = None
            time.sleep(SWITCHPOD_RETRY)
            service.status(f"PANP service PID {service.pid} on {service.hostname} "
                           "restarting switchpod i/o")


def get_brewpi_rmx_data(q):
    """Queue (index, value) for each fermenter's temperature and gravity.

    Index is 2 * fermenter for the temperature and 2 * fermenter + 1 for the
    gravity. A fermenter that cannot be reached reports 0.0 for both. The
    temperature comes back as the text BrewPi shows on its lcd, which is
    "--.-" for a probe it cannot read; the consumer deals with that.
    """
    while True:
        for i, fermenter in enumerate(FERMENTERS):
            for j, request in enumerate(["lcd", "statusText"]):
                data = 0.0
                try:
                    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                    s.connect(f"/home/brewpi/{fermenter}/KITTSOCKET")
                    s.sendall(request.encode())
                    s.settimeout(2)
                    try:
                        reply = json.loads(s.recv(4096).decode())
                        if request == "lcd":
                            data = reply[1].split()[1]
                        else:
                            data = reply["0"]["Tilt SG: "]
                    except Exception:
                        pass
                    s.close()
                except Exception:
                    try:
                        s.close()
                    except Exception:
                        pass
                q.put((i * 2 + j, data))
        time.sleep(BREWPI_PERIOD)


# ---------------------------------------------------------------------------
# The bench light
# ---------------------------------------------------------------------------

class HueLight:
    """The Hue lights the dash controls: the bench light, switched on by
    pursuit mode so there is enough light to read small print at the
    workbench, and the brewery room lights, switched by two right pod keys.

    Configured from a json file holding the bridge address, an application
    key, the bench light's v2 resource id with optionally the brightness and
    colour temperature to use, and under "room" the ids of the room lights
    and the brightness to switch them on at. That file is deliberately not in
    this repository: the key is a credential and grants full control of
    everything on the bridge. With no config file this class does nothing at
    all, which is what happens on brewpi and on any machine with no bridge
    to talk to.

    The bridge is in another part of the building, so nothing here is allowed
    to hold up the dash: requests go onto a queue and a thread does the talking.
    """

    def __init__(self, path="/usr/local/etc/panp-hue.json"):
        self.cfg = None
        self.q = Queue()
        self.room_lights = []
        try:
            with open(path) as f:
                cfg = json.load(f)
            self.bridge = cfg["bridge"]
            self.bench = cfg["light"]
            self.key = cfg["key"]
            # full brightness and a neutral white by default. both are in the
            # config file so the bench light can be tuned without editing code
            self.brightness = float(cfg.get("brightness", 100.0))
            self.mirek = int(cfg.get("mirek", 200))
            room = cfg.get("room", {})
            self.room_lights = list(room.get("lights", []))
            self.room_brightness = float(room.get("brightness", 100.0))
            # the bridge serves a self signed certificate, so there is nothing
            # to verify it against
            self.ctx = ssl.create_default_context()
            self.ctx.check_hostname = False
            self.ctx.verify_mode = ssl.CERT_NONE
            self.cfg = cfg
        except Exception:
            pass

    def configured(self):
        return self.cfg is not None

    def set(self, on):
        """Ask for the bench light to change and return at once."""
        if self.cfg is not None:
            self.q.put(("bench", on))

    def room(self, on):
        """Ask for the room lights to change and return at once."""
        if self.cfg is not None and self.room_lights:
            self.q.put(("room", on))

    def _put(self, light, body, timeout):
        url = "https://{}/clip/v2/resource/light/{}".format(self.bridge, light)
        try:
            req = urllib.request.Request(url,
                                         data=json.dumps(body).encode(),
                                         method="PUT",
                                         headers={"hue-application-key": self.key,
                                                  "Content-Type": "application/json"})
            urllib.request.urlopen(req, timeout=timeout, context=self.ctx).read()
        except Exception:
            # a bridge that is slow, unreachable or switched off is not a
            # reason for the brewery dashboard to notice anything
            pass

    def put(self, on, timeout=5):
        """Send the bench light a state. Blocks, so keep it off the dash path."""
        if self.cfg is None:
            return
        if on:
            body = {"on": {"on": True},
                    "dimming": {"brightness": self.brightness},
                    "color_temperature": {"mirek": self.mirek}}
        else:
            body = {"on": {"on": False}}
        self._put(self.bench, body, timeout)

    def put_room(self, on, timeout=5):
        """Send every room light a state, one request each. Blocks."""
        if self.cfg is None:
            return
        if on:
            body = {"on": {"on": True}, "dimming": {"brightness": self.room_brightness}}
        else:
            body = {"on": {"on": False}}
        for light in self.room_lights:
            self._put(light, body, timeout)

    def loop(self):
        while True:
            target, on = self.q.get()
            # only the most recent request for each target matters. if several
            # piled up while the bridge was slow, drop the stale ones rather
            # than replaying a burst of presses at it
            latest = {target: on}
            while self.q.qsize() > 0:
                target, on = self.q.get()
                latest[target] = on
            for target, on in latest.items():
                if target == "bench":
                    self.put(on)
                else:
                    self.put_room(on)


# ---------------------------------------------------------------------------
# Shared display handling
# ---------------------------------------------------------------------------

class BrightnessHandler:
    """Dims the dash in norm and brightens it in pursuit.

    Runs as a GPIO callback on brewpi and from the PANP handler on rpints.
    """

    # dim (norm) and bright (pursuit) master brightness for every board. each
    # bus gets the messages for the boards on it, repeated for safety; the
    # message centre is never dimmed
    DIM = ['>ABD60?', '>BBD60?', '>EBD10?', '>FBD10?', '>GBD10?']
    BRIGHT = ['>ABDFF?', '>BBDFF?', '>EBD40?', '>FBD40?', '>GBD40?']

    def __init__(self, buses):
        self.brightness = GPIO.HIGH
        self.buses = buses

    def set_brightness(self, channel, force=False):
        channel_state = GPIO.input(channel)
        if force or channel_state != self.brightness:
            msgs = self.DIM if channel_state else self.BRIGHT
            for bus in self.buses:
                for _ in range(BRIGHTNESS_REPEATS):
                    for m in msgs:
                        if m[1] in bus.boards:
                            bus.write(m)
                self.brightness = channel_state


# ---------------------------------------------------------------------------
# rpints: PANP buttons, tacho, dummy boards
# ---------------------------------------------------------------------------

# the relay and signal levels for each PANP state, in the order they are set:
# lower dash power, upper dash power, switch pod power, normal-mode line,
# auto-mode line. the power relays are active low
PANP_OUTPUTS = {
    PANPState.AUTO:    (1, 1, 1, 0, 1),
    PANPState.NORM:    (0, 0, 1, 1, 0),
    PANPState.PURSUIT: (0, 0, 0, 0, 0),
}

BUTTON_STATE = {
    PANPButton.AUTO.value: PANPState.AUTO,
    PANPButton.NORM.value: PANPState.NORM,
    PANPButton.PURSUIT.value: PANPState.PURSUIT,
}


class PANPHandler:
    """The Auto / Norm / Pursuit buttons and lamps, and what they switch."""

    def __init__(self, tacho, dummy, hue, on_wake):
        self.tacho = tacho
        self.dummy = dummy
        self.hue = hue
        # called when the dash comes back from auto, so the loop can reset what it shows
        self.on_wake = on_wake
        self.brightness = BrightnessHandler([tacho, dummy])
        self.last_push = None
        for p in PANPState:
            GPIO.setup(p.value, GPIO.OUT, initial=0)
        self.state = PANPState.AUTO
        # we come up in auto, so the bench light starts off
        self.hue.set(False)
        GPIO.output(AUTO_MODE_COMM, 1)
        GPIO.output(NORMAL_MODE_COMM, 0)
        GPIO.output(self.state.value, 1)
        for p in PANPButton:
            GPIO.setup(p.value, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(p.value, GPIO.FALLING, callback=self.button, bouncetime=50)

    def clear_display(self):
        """Blank the tacho and the two dummies and put them in user-value mode.

        Needed after the dash has been powered down, because the tacho comes
        back in its factory modes. The dummy mode writes are also what
        persists their bar values, so this must stay occasional.
        """
        for msg in [">AHa01010101010101?",   # all seven tacho bars to user value
                    ">ABo01?",               # tacho digits to user value
                    ">AHh00000000000000?",   # bars dark
                    ">ABp00?",               # digits zero
                    ">EHa010101?",           # red dummy3 bars to user value
                    ">EHm000000?",           # and dark
                    ">GHa010101010101?",     # dummy6 bars to user value
                    ">GHm000000000000?"]:    # and dark
            bus = self.tacho if msg[1] == 'A' else self.dummy
            bus.write(msg)

    def change_state(self, channel):
        old_state = self.state
        new_state = BUTTON_STATE.get(channel)
        if new_state is not None:
            self.state = new_state
            GPIO.output(old_state.value, 0)
            GPIO.output(new_state.value, 1)
            lower, upper, sp, normal, auto = PANP_OUTPUTS[new_state]
            GPIO.output(LOWER_DASH_POWER, lower)
            GPIO.output(UPPER_DASH_POWER, upper)
            GPIO.output(SP_POWER, sp)
            GPIO.output(NORMAL_MODE_COMM, normal)
            GPIO.output(AUTO_MODE_COMM, auto)
            if new_state is not PANPState.AUTO:
                self.brightness.set_brightness(NORMAL_MODE_COMM)
        # the bench light follows pursuit. queued, so the sleeps below and a
        # bridge in another room cannot delay the dash responding to the button
        self.hue.set(self.state is PANPState.PURSUIT)
        if old_state is PANPState.AUTO:
            # the boards were unpowered and have lost their state, and the
            # tacho starts over from the mean
            self.on_wake()
            time.sleep(1)
            self.brightness.set_brightness(NORMAL_MODE_COMM, True)
            self.clear_display()

    def button(self, channel):
        if channel != self.last_push:
            time.sleep(0.05)
            if GPIO.input(channel) == GPIO.LOW:
                self.change_state(channel)
                self.last_push = channel


class RPintsLoopHandler:
    """One pass of the rpints side: keezer and lager probes, keg volumes, tacho and dummies."""

    def __init__(self, service, tacho, dummy, hue, sp_q, keezer_q, lager_q):
        self.service = service
        self.tacho = tacho
        self.dummy = dummy
        self.hue = hue
        self.sp_q = sp_q
        self.keezer_q = keezer_q
        self.lager_q = lager_q

        self.keezer_temps = [0.0] * len(KEEZER_PROBES)
        # min, max and median are computed for a future button; nothing shows them yet
        self.keezer_max = 0.0
        self.keezer_min = 0.0
        self.keezer_mean = 0.0
        self.keezer_median = 0.0

        # what the tacho shows, which pod position put it there, and which
        # keg or lager probe it is about
        self.view = TachoView.MEAN
        self.view_key = None
        self.view_keg = 0
        self.view_lager = 0
        # set from the panp button callback when the dash comes back from auto
        self.wake = False

        self.passes = 0
        self.connection = None

        self.lager_temps = [0.0] * len(LAGER_PROBES)
        self.total_capacity = 0.0
        self.keg_capacity = [0.0] * len(KEG_GALLONS)   # percent of each keg's starting volume
        self.keg_remaining = [0.0] * len(KEG_GALLONS)  # gallons left in each keg

    def read_keg_volumes(self):
        """Refresh keg_capacity and keg_remaining from RaspberryPints."""
        try:
            self.connection = database.connect(user="RaspberryPints",
                                               password="RaspberryPints",
                                               host="localhost",
                                               database="raspberrypints")
            cursor = self.connection.cursor()
            cursor.execute("SELECT id, startAmount, remainAmount FROM vwGetActiveTaps")
            for (idx, start, remain) in cursor:
                idx = int(idx) - 1
                if start < 0.0001 or remain < 0.0001:
                    self.keg_capacity[idx] = 0.0
                    self.keg_remaining[idx] = 0.0
                else:
                    self.keg_capacity[idx] = float(remain) / float(start) * 100.0
                    self.keg_remaining[idx] = float(remain)
        except Exception:
            pass
        finally:
            # if the connect failed this closes the previous pass's connection
            # again, which is harmless
            try:
                self.connection.close()
            except Exception:
                pass

    def show_mean(self):
        """Put the tacho back on the mean at the next pass. Safe from another thread."""
        self.wake = True

    def press(self, pos):
        """Act on a right switch pod position."""
        if pos in RIGHT_POD_KEGS:
            if self.view_key == pos and self.view in KEG_VIEWS:
                # same key again: the next view of this keg, or the mean after the last
                i = KEG_VIEWS.index(self.view) + 1
                if i < len(KEG_VIEWS):
                    self.view = KEG_VIEWS[i]
                else:
                    self.view, self.view_key = TachoView.MEAN, None
            else:
                self.view_key = pos
                self.view_keg = RIGHT_POD_KEGS[pos]
                self.view = KEG_VIEWS[0]
        elif pos in RIGHT_POD_LAGERS:
            if self.view_key == pos and self.view is TachoView.LAGER_TEMP:
                self.view, self.view_key = TachoView.MEAN, None
            else:
                self.view_key = pos
                self.view_lager = RIGHT_POD_LAGERS[pos]
                self.view = TachoView.LAGER_TEMP
        elif pos == RIGHT_POD_LIGHTS_ON:
            self.hue.room(True)
        elif pos == RIGHT_POD_LIGHTS_OFF:
            self.hue.room(False)

    def tacho_reading(self):
        """What the tacho shows this pass: the digits' value, where the decimal
        point goes (0 none, 1 after the first digit), and the arc's step count."""
        if self.view is TachoView.KEG_TEMP:
            t = self.keezer_temps[self.view_keg]
            return digit_byte(t), 0, rpm_circle(t)
        if self.view is TachoView.LAGER_TEMP:
            t = self.lager_temps[self.view_lager]
            return digit_byte(t), 0, rpm_circle(t)
        if self.view in (TachoView.KEG_LITRES, TachoView.KEG_GALLONS):
            gallons = self.keg_remaining[self.view_keg]
            arc = arc_percent(gallons / KEG_GALLONS[self.view_keg] * 100.0)
            if self.view is TachoView.KEG_LITRES:
                return digit_byte(gallons * LITRES_PER_GALLON), 0, arc
            # tenths of a gallon with the point after the first digit: 2.7
            return digit_byte(gallons * 10.0), 1, arc
        t = self.keezer_mean
        return digit_byte(t), 0, rpm_circle(t)

    def loop(self):
        if not self.service.running:
            return

        # the database is polled less often than everything else
        if self.passes % DATABASE_EVERY == 0:
            self.read_keg_volumes()
        self.passes += 1

        # total beer on tap is the mean of the five percentages, not a volume
        self.total_capacity = sum(self.keg_capacity) / len(self.keg_capacity)

        # the dash coming back from auto starts the tacho over from the mean,
        # before any press queued while it was dark
        if self.wake:
            self.wake = False
            self.view, self.view_key = TachoView.MEAN, None

        for sp_val in drain(self.sp_q):
            self.press(sp_val)

        for idx, temp in drain(self.keezer_q):
            self.keezer_temps[idx] = temp
        self.keezer_min = min(self.keezer_temps)
        self.keezer_max = max(self.keezer_temps)
        self.keezer_mean = sum(self.keezer_temps) / float(len(self.keezer_temps))
        self.keezer_median = statistics.median(self.keezer_temps)

        for idx, temp in drain(self.lager_q):
            self.lager_temps[idx] = temp

        digits, point, arc = self.tacho_reading()

        try:
            # the selected reading on the tacho digits, with its decimal point.
            # the point register is firmware 8eb06b4 and later; older tacho
            # firmware ignores it and shows the digits without the point
            self.tacho.write(f">ABq{point:02X}?")
            self.tacho.write(f">ABp{digits:02X}?")

            # the six probes on the six bars, and the selected reading on the arc
            bars = "".join(f"{tacho_bar(t):02X}" for t in self.keezer_temps)
            self.tacho.write(f">AHh{bars}{arc:02X}?")

            # lager temps, total capacity, keg 1 and keg 2 on dummy6
            self.dummy.write(">GHm{:02X}{:02X}{:02X}{:02X}{:02X}{:02X}?".format(
                lager_bar(self.lager_temps[0]),
                lager_bar(self.lager_temps[1]),
                lager_bar(self.lager_temps[2]),
                keg_bar(self.total_capacity),
                keg_bar(self.keg_capacity[0]),
                keg_bar(self.keg_capacity[1])))

            # kegs 3, 4 and 5 on the red dummy3
            self.dummy.write(">EHm{:02X}{:02X}{:02X}?".format(
                keg_bar(self.keg_capacity[2]),
                keg_bar(self.keg_capacity[3]),
                keg_bar(self.keg_capacity[4])))

        except PortNotOpenError:
            # a port closing under us during shutdown is not an error
            if self.service.running:
                raise


# ---------------------------------------------------------------------------
# brewpi: speedo, message centre, red/green dummy3, flow meters
# ---------------------------------------------------------------------------

class BrewPiLoopHandler:
    """One pass of the brewpi side: hot side probes, fermenters, speedo and message centre."""

    def __init__(self, service, speedo, msgctr, sp_q, hot_side_q, brewpi_rmx_q):
        self.service = service
        self.speedo = speedo
        self.msgctr = msgctr

        self.sp_q = sp_q

        self.hot_side_q = hot_side_q
        self.hot_side_temps = [0, 0]   # mash, hlt

        # message centre: the caption for the selected value, and the rotation
        # shown while the dash is dark
        self.msgctr_mode_old = MsgCtrMode.BREWPI_UP
        self.msgctr_mode = MsgCtrMode.MASH_TEMP
        self.msgctr_mode_saved = self.msgctr_mode
        self.msgctr_msg = ">CScDEG F MASH?"
        self.msgctr_auto = MSGCTR_IDLE

        # what each fermenter button will show on its next press
        self.tank_next = [TankDisplay.TEMP] * len(FERMENTERS)

        self.flowmeter_relay_state = [GPIO.LOW] * len(FLOWMETER)

        # follows the auto-mode line from rpints
        self.auto_mode = GPIO.HIGH

        # the red/green dummy3 is unpowered in auto mode, so its bars are put
        # back into user-value mode once each time the dash comes up rather
        # than on every pass: the mode write persists to the board's eeprom,
        # and that eeprom save also rewrites the bar values
        self.dummy3_user_mode = False

        # utk1 temp, utk1 sg, utk2 temp, utk2 sg, chronical temp, chronical sg
        self.brewpi_rmx_q = brewpi_rmx_q
        self.brewpi_rmx_data = [0.0, 0.000, 0.0, 0.000, 0.0, 0.000]

    def start(self):
        """Put the message centre into its idle rotation."""
        self.msgctr.write(self.msgctr_auto)

    def toggle_flowmeter_relay(self, f):
        if self.flowmeter_relay_state[f] == GPIO.LOW:
            self.flowmeter_relay_state[f] = GPIO.HIGH
            flow_msg = "on"
        else:
            self.flowmeter_relay_state[f] = GPIO.LOW
            flow_msg = "off"
        GPIO.output(FLOWMETER[f], self.flowmeter_relay_state[f])
        # flash the change on the message centre for a second
        self.msgctr.write(">CBa00?")
        self.msgctr.write(f">CScFLOW {f + 1} {flow_msg}?")
        time.sleep(1)
        # and make the next pass restore the caption
        self.msgctr_mode_old = MsgCtrMode.FLOWMETER
        # the idle rotation lists the meters that are running
        flow_stat_msg = "".join(f"F{n + 1}" if state == GPIO.HIGH else "  "
                                for n, state in enumerate(self.flowmeter_relay_state))
        if flow_stat_msg.isspace():
            self.msgctr_auto = MSGCTR_IDLE
        else:
            self.msgctr_auto = f">CSbBREWPI UP|{flow_stat_msg}~?"

    def set_auto_mode(self, channel):
        """GPIO callback for the auto-mode line from rpints."""
        time.sleep(0.1)
        channel_state_tmp = GPIO.input(channel)
        time.sleep(0.1)
        channel_state = GPIO.input(channel)
        if channel_state_tmp != channel_state:
            time.sleep(0.5)
            channel_state = GPIO.input(channel)
        if channel_state != self.auto_mode:
            self.auto_mode = channel_state
            if channel_state == GPIO.HIGH:
                # the dash is going dark: hand the message centre back to its
                # rotation, remembering what was selected so it can come back
                self.msgctr_mode_saved = self.msgctr_mode
                self.msgctr_mode = MsgCtrMode.BREWPI_UP
                self.msgctr_mode_old = self.msgctr_mode
                self.msgctr.write(self.msgctr_auto)
                self.msgctr.write(">CBa01?")
                self.msgctr.write(">CBa00?")
            else:
                # the dash is lighting up: put back the selection from before
                # auto, unless a button was pressed while it was dark, so the
                # lower display and its caption agree
                if self.msgctr_mode is MsgCtrMode.BREWPI_UP:
                    self.msgctr_mode = self.msgctr_mode_saved
                # the caption is not sent from here. the main loop may be
                # part way through a dark pass, and its trailing override-off
                # packet would land after a caption sent now and lapse it back
                # to the rotation (seen on the dash). marking the caption as
                # stale makes the loop send it on its first lit pass, after
                # its own dark writes, in order on one thread
                self.msgctr_mode_old = None

    def press(self, sp_val):
        """Act on a left switch pod position."""
        if sp_val in LEFT_POD_CAPTIONS:
            self.msgctr_mode, self.msgctr_msg = LEFT_POD_CAPTIONS[sp_val]
        elif sp_val in LEFT_POD_TANKS:
            tank, temp_mode, sg_mode, name = LEFT_POD_TANKS[sp_val]
            if self.tank_next[tank] == TankDisplay.TEMP:
                self.tank_next[tank] = TankDisplay.SG
                self.msgctr_mode = temp_mode
                self.msgctr_msg = f">CScDEG F {name}?"
            else:
                self.tank_next[tank] = TankDisplay.TEMP
                self.msgctr_mode = sg_mode
                self.msgctr_msg = f">CScSG {name}?"
        elif sp_val in LEFT_POD_FLOWMETERS:
            self.toggle_flowmeter_relay(LEFT_POD_FLOWMETERS[sp_val])
        else:
            self.msgctr_mode, self.msgctr_msg = LEFT_POD_CAPTIONS[0]

    def loop(self):
        if not self.service.running:
            return

        for idx, temp in drain(self.hot_side_q):
            self.hot_side_temps[idx] = temp

        for idx, value in drain(self.brewpi_rmx_q):
            try:
                self.brewpi_rmx_data[idx] = float(value)
            except (ValueError, IndexError):
                # brewpi puts "--.-" on its lcd for a probe it cannot read,
                # which is not a float. keep the last good value: letting this
                # raise would leave the loop and stop the service, and the
                # unit does not restart
                pass

        for sp_val in drain(self.sp_q):
            self.press(sp_val)

        if self.auto_mode == GPIO.LOW:
            # the dash is lit
            try:
                if self.msgctr_mode is not self.msgctr_mode_old:
                    self.msgctr.write(">CBa00?")
                    self.msgctr.write(self.msgctr_msg)
                    self.msgctr_mode_old = self.msgctr_mode

                # hlt temperature on the upper display, and one step of its
                # led line per ten degrees, so the line is full at 190 F
                hundreds, tens, ones, tenths = temperature_digits(round(self.hot_side_temps[1]))
                steps = min(SPEED_LINE_STEPS, hundreds * 10 + tens)
                self.speedo.write(f">BBc{steps:02X}?")
                self.speedo.write(f">BHd0{hundreds}0{tens}0{ones}03?")

                # the selected value on the lower display and its led line
                source, span, dp_mode = LOWER_DISPLAY[self.msgctr_mode]
                value = source(self)
                steps = int(math.floor(span(value)))
                steps = max(0, min(FUEL_LINE_STEPS, steps))
                self.speedo.write(f">BBb{steps:02X}?")
                hundreds, tens, ones, tenths = temperature_digits(value)
                self.speedo.write(f">BHe0{hundreds}0{tens}0{ones}0{tenths}0{dp_mode}?")

                # fermenter temperatures on the red/green dummy3
                if self.dummy3_user_mode is False:
                    self.speedo.write(">FHa010101?")
                    self.dummy3_user_mode = True
                self.speedo.write(">FHm{:02X}{:02X}{:02X}?".format(
                    temperature_bar(self.brewpi_rmx_data[0]),
                    temperature_bar(self.brewpi_rmx_data[2]),
                    temperature_bar(self.brewpi_rmx_data[4])))

            except PortNotOpenError:
                if self.service.running:
                    raise

        else:
            # the dash is powered down, so the dummy3 will need its mode set
            # again when it comes back; meanwhile the message centre runs its
            # own rotation, which is constant text and so safe to re-send
            self.dummy3_user_mode = False
            try:
                self.msgctr.write(self.msgctr_auto)
                self.msgctr.write(">CBa01?")
            except PortNotOpenError:
                if self.service.running:
                    raise


# ---------------------------------------------------------------------------
# Startup
# ---------------------------------------------------------------------------

def start_thread(target, *args):
    t = Thread(target=target, args=args)
    t.daemon = True
    t.start()
    return t


def setup_rpints(service, sp_q):
    # the dashboard power relays, active low, start open
    GPIO.setwarnings(False)
    GPIO.setup(LOWER_DASH_POWER, GPIO.OUT, initial=1)
    GPIO.setup(UPPER_DASH_POWER, GPIO.OUT, initial=1)
    GPIO.setup(SP_POWER, GPIO.OUT, initial=1)

    # the mode lines to brewpi
    GPIO.setup(NORMAL_MODE_COMM, GPIO.OUT, initial=0)
    GPIO.setup(AUTO_MODE_COMM, GPIO.OUT, initial=0)
    GPIO.setwarnings(True)

    dummy = service.open_bus("/dev/ttyAMA0", "EG")   # red dummy3, dummy6
    tacho = service.open_bus("/dev/ttyAMA1", "A")

    # the bench light and the room lights, if a bridge is configured on this machine
    hue = HueLight()
    if hue.configured():
        start_thread(hue.loop)
        service.status(f"PANP service PID {service.pid} on {service.hostname} has a bench light "
                       f"and {len(hue.room_lights)} room lights")
    else:
        service.status(f"PANP service PID {service.pid} on {service.hostname} "
                       "has no bench light configured")

    keezer_q = Queue()
    start_thread(get_temps, KEEZER_PROBES, keezer_q)
    lager_q = Queue()
    start_thread(get_temps, LAGER_PROBES, lager_q)

    loop_handler = RPintsLoopHandler(service, tacho, dummy, hue, sp_q, keezer_q, lager_q)
    service.handlers.append(PANPHandler(tacho, dummy, hue, loop_handler.show_mean))

    def teardown():
        for p in PANPState:
            GPIO.output(p.value, 0)
        # the lamps are going out, so the bench light goes with them. sent
        # here rather than queued: the worker thread will not outlive us
        hue.put(False, 2)
        GPIO.output(LOWER_DASH_POWER, 1)
        GPIO.output(UPPER_DASH_POWER, 1)
        GPIO.output(SP_POWER, 1)
        GPIO.output(NORMAL_MODE_COMM, 0)
        GPIO.output(AUTO_MODE_COMM, 1)
    service.teardown = teardown

    return loop_handler


def setup_brewpi(service, sp_q):
    # the message centre power relay
    GPIO.setwarnings(False)
    GPIO.setup(MSGCTR_POWER, GPIO.OUT, initial=0)

    # the mode lines from rpints
    GPIO.setup(NORMAL_MODE_COMM, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(AUTO_MODE_COMM, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setwarnings(True)

    for f in FLOWMETER:
        GPIO.setup(f, GPIO.OUT, initial=0)

    msgctr = service.open_bus("/dev/ttyAMA0", "C")
    speedo = service.open_bus("/dev/ttyAMA1", "BF")   # speedo, red/green dummy3

    # clear the red/green dummy3 and set to user mode
    time.sleep(1)
    speedo.write(">FHa010101?")
    speedo.write(">FHm000000?")

    # dim the speedo as we are in auto mode initially
    brightness = BrightnessHandler([speedo])
    brightness.set_brightness(NORMAL_MODE_COMM, True)
    GPIO.add_event_detect(NORMAL_MODE_COMM, GPIO.BOTH, callback=brightness.set_brightness, bouncetime=50)
    service.handlers.append(brightness)

    hot_side_q = Queue()
    start_thread(get_temps, HOT_SIDE_PROBES, hot_side_q)

    brewpi_rmx_q = Queue()
    start_thread(get_brewpi_rmx_data, brewpi_rmx_q)

    loop_handler = BrewPiLoopHandler(service, speedo, msgctr, sp_q, hot_side_q, brewpi_rmx_q)
    loop_handler.start()
    GPIO.add_event_detect(AUTO_MODE_COMM, GPIO.BOTH, callback=loop_handler.set_auto_mode, bouncetime=50)

    def teardown():
        GPIO.remove_event_detect(NORMAL_MODE_COMM)
        GPIO.remove_event_detect(AUTO_MODE_COMM)
        GPIO.output(MSGCTR_POWER, 0)
    service.teardown = teardown

    return loop_handler


def main():
    service = Service()
    signal.signal(signal.SIGTERM, service.sigterm)

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(SERIAL_ENABLE, GPIO.OUT, initial=1)

    # allow boot to continue
    service.notifier.notify("READY=1")
    service.status(f"PANP service PID {service.pid} running on {service.hostname}")

    sp_q = Queue()
    start_thread(get_switchpod, sp_q, service)

    if service.hostname == 'rpints':
        loop_handler = setup_rpints(service, sp_q)
    elif service.hostname == 'brewpi':
        loop_handler = setup_brewpi(service, sp_q)
    else:
        service.status("Unknown hostname, exiting")
        service.notifier.notify("ERRNO=1")
        sys.exit(1)

    time.sleep(STARTUP_SETTLE)
    service.status(f"PANP service PID {service.pid} on {service.hostname} entering main loop")
    service.running = True
    try:
        while service.running:
            loop_handler.loop()
            time.sleep(LOOP_PERIOD)
    except KeyboardInterrupt:
        pass
    service.shutdown()


if __name__ == "__main__":
    main()
