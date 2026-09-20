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

class RPMMode(Enum):
    """What the tacho digits and arc show."""
    PROBE1 = 0
    PROBE2 = 1
    PROBE3 = 2
    PROBE4 = 3
    PROBE5 = 4
    PROBE6 = 5
    LAGER1 = 6
    LAGER2 = 7
    LAGER3 = 8
    MEAN = 9


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


# right switch pod: position -> what the tacho shows. anything else means the mean
RIGHT_POD = {
    0: RPMMode.PROBE1,    # TURBO BOOST
    2: RPMMode.PROBE2,    # 7 DLA
    4: RPMMode.PROBE3,    # 8 PL1
    6: RPMMode.PROBE4,    # 6 RM (orange)
    8: RPMMode.PROBE5,    # H6
    1: RPMMode.PROBE6,    # 6 RM (white)
    3: RPMMode.LAGER1,    # P ENG
    5: RPMMode.LAGER2,    # AUTO ROOF R
    7: RPMMode.LAGER3,    # P IND
    9: RPMMode.MEAN,      # EJECT R
}

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


def _leds(offset, per_led):
    """Led count for a value on a linear scale starting at `offset`."""
    return lambda value: (value - offset) // per_led


def _sg_leds(value):
    """Led count for a gravity, which arrives here already multiplied by 100."""
    return (value * 10.0 - 1000.0) // 4.0


# what the lower speedo display shows in each mode: the value, the number of
# leds to light for it, and the decimal point position (1 is 000.0, 3 is 0.000)
LOWER_DISPLAY = {
    MsgCtrMode.MASH_TEMP:      (lambda h: h.hot_side_temps[0], _leds(110.0, 5.0), 1),
    MsgCtrMode.MASH_TEMP_C:    (lambda h: (h.hot_side_temps[0] - 32.0) * 5.0 / 9.0, _leds(20.0, 4.0), 1),
    MsgCtrMode.UNITANK1_TEMP:  (lambda h: h.brewpi_rmx_data[0], _leds(34.0, 3.0), 1),
    MsgCtrMode.UNITANK2_TEMP:  (lambda h: h.brewpi_rmx_data[2], _leds(34.0, 3.0), 1),
    MsgCtrMode.CHRONICAL_TEMP: (lambda h: h.brewpi_rmx_data[4], _leds(34.0, 3.0), 1),
    MsgCtrMode.BREWPI_UP:      (lambda h: h.hot_side_temps[0], _leds(45.0, 10.0), 1),
    MsgCtrMode.HLT_TEMP:       (lambda h: h.hot_side_temps[1], _leds(110.0, 5.0), 1),
    MsgCtrMode.UNITANK1_SG:    (lambda h: h.brewpi_rmx_data[1] * 100.0, _sg_leds, 3),
    MsgCtrMode.UNITANK2_SG:    (lambda h: h.brewpi_rmx_data[3] * 100.0, _sg_leds, 3),
    MsgCtrMode.CHRONICAL_SG:   (lambda h: h.brewpi_rmx_data[5] * 100.0, _sg_leds, 3),
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
    """Temperature of a one-wire probe in degrees F, or 0.0 if it cannot be read."""
    tempvalue_f = 0.0
    try:
        with open(path, 'r') as fileobj:
            lines = fileobj.readlines()
        equals_pos = lines[1].find('t=')
        tempstr = lines[1][equals_pos + 2:]
        tempvalue_c = float(tempstr) / 1000.0
        tempvalue_f = tempvalue_c * 9.0 / 5.0 + 32.0
    except Exception:
        pass
    return tempvalue_f


def get_temps(probes, q):
    """Queue (index, temperature) for each probe, once a second."""
    while True:
        try:
            for i, path in enumerate(probes):
                q.put((i, read_probe_f(path)))
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
    """The bench light, switched on by pursuit mode so there is enough light
    to read small print at the workbench.

    Configured from a json file holding the bridge address, an application key,
    the light's v2 resource id, and optionally the brightness and colour
    temperature to use. That file is deliberately not in this repository: the
    key is a credential and grants full control of everything on the bridge.
    With no config file this class does nothing at all, which is what happens
    on brewpi and on any machine with no bridge to talk to.

    The bridge is in another part of the building, so nothing here is allowed
    to hold up the dash: requests go onto a queue and a thread does the talking.
    """

    def __init__(self, path="/usr/local/etc/panp-hue.json"):
        self.cfg = None
        self.q = Queue()
        try:
            with open(path) as f:
                cfg = json.load(f)
            self.url = "https://{}/clip/v2/resource/light/{}".format(
                cfg["bridge"], cfg["light"])
            self.key = cfg["key"]
            # full brightness and a neutral white by default. both are in the
            # config file so the bench light can be tuned without editing code
            self.brightness = float(cfg.get("brightness", 100.0))
            self.mirek = int(cfg.get("mirek", 200))
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
        """Ask for a state change and return at once."""
        if self.cfg is not None:
            self.q.put(on)

    def put(self, on, timeout=5):
        """Send a state to the bridge. Blocks, so keep it off the dash path."""
        if self.cfg is None:
            return
        if on:
            body = {"on": {"on": True},
                    "dimming": {"brightness": self.brightness},
                    "color_temperature": {"mirek": self.mirek}}
        else:
            body = {"on": {"on": False}}
        try:
            req = urllib.request.Request(self.url,
                                         data=json.dumps(body).encode(),
                                         method="PUT",
                                         headers={"hue-application-key": self.key,
                                                  "Content-Type": "application/json"})
            urllib.request.urlopen(req, timeout=timeout, context=self.ctx).read()
        except Exception:
            # a bridge that is slow, unreachable or switched off is not a
            # reason for the brewery dashboard to notice anything
            pass

    def loop(self):
        while True:
            on = self.q.get()
            # only the most recent request matters. if several piled up while
            # the bridge was slow, drop the stale ones rather than replaying
            # a burst of panp presses at it
            while self.q.qsize() > 0:
                on = self.q.get()
            self.put(on)


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

    def __init__(self, tacho, dummy, hue):
        self.tacho = tacho
        self.dummy = dummy
        self.hue = hue
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
            # the boards were unpowered and have lost their state
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

    def __init__(self, service, tacho, dummy, sp_q, keezer_q, lager_q):
        self.service = service
        self.tacho = tacho
        self.dummy = dummy
        self.sp_q = sp_q
        self.keezer_q = keezer_q
        self.lager_q = lager_q

        self.keezer_temps = [0.0] * len(KEEZER_PROBES)
        # min, max and median are computed for a future button; nothing shows them yet
        self.keezer_max = 0.0
        self.keezer_min = 0.0
        self.keezer_mean = 0.0
        self.keezer_median = 0.0
        self.rpm_mode = RPMMode.MEAN

        self.passes = 0
        self.connection = None

        self.lager_temps = [0.0] * len(LAGER_PROBES)
        self.total_capacity = 0.0
        self.keg_capacity = [0.0, 0.0, 0.0, 0.0, 0.0]

    def read_keg_volumes(self):
        """Refresh keg_capacity from RaspberryPints, as a percentage of each keg's start."""
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
                else:
                    self.keg_capacity[idx] = float(remain) / float(start) * 100.0
        except Exception:
            pass
        finally:
            # if the connect failed this closes the previous pass's connection
            # again, which is harmless
            try:
                self.connection.close()
            except Exception:
                pass

    def selected_temperature(self):
        """The temperature the right switch pod has put on the tacho digits."""
        if self.rpm_mode is RPMMode.MEAN:
            return self.keezer_mean
        if self.rpm_mode is RPMMode.LAGER1:
            return self.lager_temps[0]
        if self.rpm_mode is RPMMode.LAGER2:
            return self.lager_temps[1]
        if self.rpm_mode is RPMMode.LAGER3:
            return self.lager_temps[2]
        return self.keezer_temps[self.rpm_mode.value]

    def loop(self):
        if not self.service.running:
            return

        # the database is polled less often than everything else
        if self.passes % DATABASE_EVERY == 0:
            self.read_keg_volumes()
        self.passes += 1

        # total beer on tap is the mean of the five percentages, not a volume
        self.total_capacity = sum(self.keg_capacity) / len(self.keg_capacity)

        for sp_val in drain(self.sp_q):
            self.rpm_mode = RIGHT_POD.get(sp_val, RPMMode.MEAN)

        for idx, temp in drain(self.keezer_q):
            self.keezer_temps[idx] = temp
        self.keezer_min = min(self.keezer_temps)
        self.keezer_max = max(self.keezer_temps)
        self.keezer_mean = sum(self.keezer_temps) / float(len(self.keezer_temps))
        self.keezer_median = statistics.median(self.keezer_temps)

        for idx, temp in drain(self.lager_q):
            self.lager_temps[idx] = temp

        rpm = self.selected_temperature()

        try:
            # the selected temperature on the tacho digits
            self.tacho.write(f">ABp{int(round(rpm)):02X}?")

            # the six probes on the six bars, and the selected temperature on the arc
            bars = "".join(f"{tacho_bar(t):02X}" for t in self.keezer_temps)
            self.tacho.write(f">AHh{bars}{rpm_circle(rpm):02X}?")

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
                self.msgctr_mode_old = self.msgctr_mode
                self.msgctr.write(">CBa00?")
                self.msgctr.write(self.msgctr_msg)

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

                # hlt temperature on the upper display and its led line
                hundreds, tens, ones, tenths = temperature_digits(round(self.hot_side_temps[1]))
                self.speedo.write(f">BBc{hundreds * 10 + tens:02X}?")
                self.speedo.write(f">BHd0{hundreds}0{tens}0{ones}03?")

                # the selected value on the lower display and its led line
                source, leds, dp_mode = LOWER_DISPLAY[self.msgctr_mode]
                value = source(self)
                n_leds = int(math.floor(leds(value)))
                n_leds = max(0, min(16, n_leds))
                self.speedo.write(f">BBb{n_leds:02X}?")
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

    # the bench light, if a bridge is configured on this machine
    hue = HueLight()
    if hue.configured():
        start_thread(hue.loop)
        service.status(f"PANP service PID {service.pid} on {service.hostname} has a bench light")
    else:
        service.status(f"PANP service PID {service.pid} on {service.hostname} "
                       "has no bench light configured")

    service.handlers.append(PANPHandler(tacho, dummy, hue))

    keezer_q = Queue()
    start_thread(get_temps, KEEZER_PROBES, keezer_q)
    lager_q = Queue()
    start_thread(get_temps, LAGER_PROBES, lager_q)

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

    return RPintsLoopHandler(service, tacho, dummy, sp_q, keezer_q, lager_q)


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
