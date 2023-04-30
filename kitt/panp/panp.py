#!/usr/bin/python

import os
import sys
import time
import socket
import signal
import serial
import _thread
import sdnotify
import RPi.GPIO as GPIO
from enum import Enum
from threading import Thread
from queue import Queue
from serial.serialutil import PortNotOpenError


# common gpio pins
serial_enable = 22 # GPIO 25

# gpio pin configuration for rpints
rpints_power_relay = 16 # GPIO 23
lower_dash_power = 19   # GPIO 10 (MOSI)
upper_dash_power = 12   # GPIO 18
sp_power = 11           # GPIO 17
normal_mode_out = 15    # GPIO 22

# gpio pin configurations for brewpi
brewpi_power_relay = 29 # GPIO 5
msgctr_power = 36       # GPIO 16
normal_mode_in = 15     # GPIO 22


class TempProbe:
    def __init__(self):
        pass

    def get_probe_temp(self, probe):
        tempvalue_f = 0
        try:
            fileobj = open(probe,'r')
            lines = fileobj.readlines()
            fileobj.close()
            status = lines[0][-4:-1]
            equals_pos = lines[1].find('t=')
            tempstr = lines[1][equals_pos+2:]
            tempvalue_c = float(tempstr)/1000.0
            tempvalue_f = tempvalue_c * 9.0 / 5.0 + 32.0
        except:
            pass
        return tempvalue_f

    def get_probe_temp_units(self, probe):
        tempvalue_f = self.get_probe_temp(probe)
        tenths = int(round(tempvalue_f % 1 * 10.0, 0))
        ones = int(tempvalue_f % 10)
        tens = int(tempvalue_f // 10 % 10)
        hundreds = int(tempvalue_f // 100 % 10)
        return hundreds, tens, ones, tenths


class RPMMode(Enum):
    PROBE1 = 0
    PROBE2 = 1
    PROBE3 = 2
    PROBE4 = 3
    PROBE5 = 4
    PROBE6 = 5
    MIN = 6
    MAX = 7
    MEDIAN = 8
    MEAN = 9


class RPintsLoopHandler:
    def __init__(self, tacho_tx, dummy_tx, sp_q, keezer_q):
        self.tacho_tx = tacho_tx
        self.dummy_tx = dummy_tx
        self.sp_q = sp_q
        self.keezer_q = keezer_q

        self.keezer_temps = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
        self.keezer_max = 0.0
        self.keezer_min = 0.0
        self.keezer_mean = 0.0
        self.keezer_median = 0.0
        self.rpm_mode = RPMMode.MEAN

        self.lager_temps = [ 0.0, 0.0, 0.0 ]
        self.keg_capacity = [ 0.0, 0.0, 0.0, 0.0, 0.0]
        self.total_capacity = 0.0

    def rpm_circle(self, temp):
        rpm_idx = [ 2.0, 4.0, 6.0, 8.0, 10.0,
                12.5, 15.0, 17.5, 20.0,
                22.5, 25.0, 27.5, 30.0,
                33.3, 36.6, 40.0,
                43.3, 46.6, 50.0,
                52.5, 55.0, 57.5, 60.0,
                62.5, 65.0, 67.5, 70.0,
                72.5, 75.0, 77.5, 80.0]
        for i, t in enumerate(rpm_idx):
            if temp < t: break
        return i

    def tacho_bar(self, temp):
        bar_temps = [34.0, 37.0, 40.0, 43.0, 47.0, 51.0, 53.0, 56.0]
        bar_vals = [0x20, 0x38, 0x54, 0x70, 0x90, 0xA8, 0xC4, 0xE0]
        for i, t in enumerate(bar_temps):
            if temp < t: break
        return bar_vals[i]

    def loop(self):
        global run_loop
        if run_loop is False:
            return

        # Update the mode if there was a button press
        if self.sp_q.qsize() > 0:
            try:
                sp_val = int(sp_q.get())
            except:
                sp_val = None
            if sp_val == 0:   # TURBO BOOST
                self.rpm_mode = RPMMode.PROBE1
            elif sp_val == 2: # 7DLA
                self.rpm_mode = RPMMode.PROBE2
            elif sp_val == 4: # 8PL1
                self.rpm_mode = RPMMode.PROBE3
            elif sp_val == 6: # 6RM
                self.rpm_mode = RPMMode.PROBE4
            elif sp_val == 8: # H6
                self.rpm_mode = RPMMode.PROBE5
            elif sp_val == 1: # 6RM
                self.rpm_mode = RPMMode.PROBE6
            elif sp_val == 3: # PENG
                self.rpm_mode = RPMMode.MAX
            elif sp_val == 5: # AUTO ROOF R 
                self.rpm_mode = RPMMode.MIN
            elif sp_val == 7: # PIND
                self.rpm_mode = RPMMode.MEDIAN
            elif sp_val == 9: # EJECT R
                self.rpm_mode = RPMMode.MEAN
            else:
                self.rpm_mode = RPMMode.MEAN

        while self.keezer_q.qsize() > 0:
            keezer_data = self.keezer_q.get()
            idx, temp = keezer_data.split(',')
            self.keezer_temps[int(idx)] = float(temp)

        self.keezer_min = min(self.keezer_temps)
        self.keezer_max = max(self.keezer_temps)
        self.keezer_mean = sum(self.keezer_temps) / float(len(self.keezer_temps))
        temps = self.keezer_temps.copy()
        n = len(temps)
        temps.sort()
        if n % 2 == 0:
            median1 = temps[n//2]
            median2 = temps[n//2 - 1]
            self.keezer_median = (median1 + median2)/2
        else:
            self.keezer_median = temps[n//2]

        if self.rpm_mode is RPMMode.MEAN:
            rpm = self.keezer_mean
        elif self.rpm_mode is RPMMode.MEDIAN:
            rpm = self.keezer_median
        elif self.rpm_mode is RPMMode.MIN:
            rpm = self.keezer_min
        elif self.rpm_mode is RPMMode.MAX:
            rpm = self.keezer_max
        else:
            rpm = self.keezer_temps[self.rpm_mode.value]

        try:
            # write the temperature to the tacho seven segment display
            msg = ">ABp{:0>2X}?".format(int(round(rpm)))
            self.tacho_tx.write(str.encode(msg))

            # write the probe temps to the six bars
            msg = ">AHh"
            for t in self.keezer_temps:
                msg = "{}{:0>2X}".format(msg,self.tacho_bar(t))

            # write the temperature to the rpm circle
            msg = "{}{:0>2X}?".format(msg,self.rpm_circle(rpm))
            self.tacho_tx.write(str.encode(msg))
        except PortNotOpenError:
            if run_loop is True:
                raise PortNotOpenError
            else:
                pass


class BrewPiLoopHandler(TempProbe):
    def __init__(self, speedo_tx):
        self.mash_probe = "/sys/bus/w1/devices/28-012052b65be5/w1_slave"
        self.hlt_probe = "/sys/bus/w1/devices/28-0120529d8f20/w1_slave"
        self.speedo_tx = speedo_tx

    def loop(self):
        global run_loop
        if run_loop is False:
            return

        try:
            # write hlt temp to upper display
            hundreds, tens, ones, tenths = self.get_probe_temp_units(self.hlt_probe)
            msg = ">BBc{:0>2X}?".format(hundreds*10 + tens)
            self.speedo_tx.write(str.encode(msg))
            msg = '>BHd0{0}0{1}0{2}03?'.format(hundreds, tens, ones)
            self.speedo_tx.write(str.encode(msg))

            # write mash temp to lower display
            hundreds, tens, ones, tenths = self.get_probe_temp_units(self.mash_probe)
            n = ((hundreds*100+tens*10+ones)-110)//5
            if ( n < 0 ): n = 0
            msg = ">BBb{:0>2X}?".format(n)
            self.speedo_tx.write(str.encode(msg))
            msg = '>BHe0{0}0{1}0{2}0{3}01?'.format(hundreds, tens, ones, tenths)
            self.speedo_tx.write(str.encode(msg))
        except PortNotOpenError:
            if run_loop is True:
                raise PortNotOpenError
            else:
                pass


class PANPState(Enum):
    # GPIO pins for PANP lamps
    AUTO = 35    # GPIO 19
    NORM = 38    # GPIO 20
    PURSUIT = 32 # GPIO 12


class PANPButton(Enum):
    # GPIO pins for PANP buttons
    AUTO = 29    # GPIO 5
    NORM = 31    # GPIO 6
    PURSUIT = 36 # GPIO 16


class PANPHandler:
    # current state
    state = None
    last_push = None

    # initalize panp lamps on boot
    def __init__(self,tacho_tx, dummy_tx):
        self.tacho_tx = tacho_tx
        self.dummy_tx = dummy_tx
        self.brightness = BrightnessHandler([tacho_tx, dummy_tx])
        for p in PANPState:
            GPIO.setup(p.value, GPIO.OUT, initial=0)
        self.state = PANPState.AUTO
        GPIO.output(self.state.value,1)
        for p in PANPButton:
            GPIO.setup(p.value, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(p.value, GPIO.FALLING, callback=self.button, bouncetime=50)

    def clear_display(self):
        # clear the tacho and set to user mode
        msg = ">AHa01010101010101?"
        self.tacho_tx.write(str.encode(msg))
        time.sleep(0.1)
        msg = ">ABo01?"
        self.tacho_tx.write(str.encode(msg))
        time.sleep(0.1)
        msg = ">AHh00000000000000?"
        self.tacho_tx.write(str.encode(msg))
        time.sleep(0.1)
        msg = ">ABp00?"
        self.tacho_tx.write(str.encode(msg))
        time.sleep(0.1)

        # clear the red dummy3 and set to user mode
        msg = ">EHa010101?"
        self.dummy_tx.write(str.encode(msg))
        time.sleep(0.1)
        msg = ">EHm000000?"
        self.dummy_tx.write(str.encode(msg))
        time.sleep(0.1)

        # clear the red dummy6 and set to user mode
        msg = ">GHa010101010101?"
        self.dummy_tx.write(str.encode(msg))
        time.sleep(0.1)
        msg = ">GHm000000000000?"
        self.dummy_tx.write(str.encode(msg))
        time.sleep(0.1)

    def change_state(self, channel):
        old_state = self.state
        if channel is PANPButton.AUTO.value:
            GPIO.output(lower_dash_power, 1)
            GPIO.output(upper_dash_power, 1)
            GPIO.output(sp_power, 1)
            GPIO.output(normal_mode_out, 0)
            self.state = PANPState.AUTO
        elif channel is PANPButton.NORM.value:
            GPIO.output(lower_dash_power, 0)
            GPIO.output(upper_dash_power, 0)
            GPIO.output(sp_power, 1)
            GPIO.output(normal_mode_out, 1)
            self.brightness.set_brightness(normal_mode_out)
            self.state = PANPState.NORM
        elif channel is PANPButton.PURSUIT.value:
            GPIO.output(lower_dash_power, 0)
            GPIO.output(upper_dash_power, 0)
            GPIO.output(sp_power, 0)
            GPIO.output(normal_mode_out, 0)
            self.brightness.set_brightness(normal_mode_out)
            self.state = PANPState.PURSUIT
        GPIO.output(old_state.value,0)
        GPIO.output(self.state.value,1)
        if old_state is PANPState.AUTO:
            for i in range(2,0,-1):
                time.sleep(i)
                self.brightness.set_brightness(normal_mode_out, True)
                self.clear_display()

    def button(self, channel):
        if channel is not self.last_push:
            time.sleep(0.05)
            if GPIO.input(channel) == GPIO.LOW:
                self.change_state(channel)
                self.last_push = channel


class BrightnessHandler:
    def __init__(self, tx_devs):
        self.brightness = GPIO.HIGH
        self.tx_devs = tx_devs
        
    def set_brightness(self, channel, force=False):
        channel_state = GPIO.input(channel)
        if force or (channel_state is not self.brightness):
            for tx in self.tx_devs:
                if channel_state:
                    msg = '>@BD20?'
                else:
                    msg = '>@BDFF?'
                for i in range(3):
                    tx.write(str.encode(msg))
                    time.sleep(0.01)
                self.brightness = channel_state

def cleanup_exit():
    global speedo_tx
    global tacho_tx
    global dummy_tx
    global keezer_t
    global sp_t

    msg="PANP service on {0} got KeyboardInterrupt".format(my_hostname)
    print(msg)
    n.notify("STATUS={0}".format(msg))

    if my_hostname == 'rpints':
        keezer_t.stop()
        sp_t.stop()

    time.sleep(1)
    if my_hostname == 'brewpi':
        speedo_tx.close()
    else:
        tacho_tx.close()
        dummy_tx.close()

    msg="PANP service on {0} exiting cleanly".format(my_hostname)
    print(msg)
    n.notify("STATUS={0}".format(msg))
    n.notify("STOPPING=1")
    sys.exit(0)

# set up exit signal handler for systemd
def sigterm_handler(_signo, _stack_frame):
    global main_pid
    global run_loop
    run_loop = False
    msg="PANP service PID {} on {} got SIGTERM".format(os.getpid(), my_hostname)
    print(msg)
    n.notify("STATUS={0}".format(msg))
    if my_hostname == 'rpints':
        for p in PANPState:
            GPIO.output(p.value, 0)
        GPIO.output(lower_dash_power, 1)
        GPIO.output(upper_dash_power, 1)
        GPIO.output(sp_power, 1)
        GPIO.output(normal_mode_out, 0)
    else:
        GPIO.remove_event_detect(normal_mode_in)
        GPIO.output(msgctr_power,0)
    GPIO.output(serial_enable,0)
    msg="PANP service on {} sending PID {} SIGINT".format(my_hostname, main_pid)
    print(msg)
    n.notify("STATUS={0}".format(msg))
    try:
        os.kill(main_pid, signal.SIGINT)
    except KeyboardInterrupt:
        cleanup_exit()


# function to listen to switchpod
def get_switchpod(rx, sp_q):
    while True:
        state = rx.readline()
        data = state.decode().strip()
        sp_q.put(data)

# function to get the keezer temperatures
def get_keezer_temps(keezer_q):
    t = TempProbe()
    keezer_probes = [
            "/sys/bus/w1/devices/28-012052b92541/w1_slave",
            "/sys/bus/w1/devices/28-012058f936f3/w1_slave",
            "/sys/bus/w1/devices/28-012052ba8dab/w1_slave",
            "/sys/bus/w1/devices/28-012058fbceb5/w1_slave",
            "/sys/bus/w1/devices/28-012058fc2851/w1_slave",
            "/sys/bus/w1/devices/28-012052b426ca/w1_slave" ]
    while True:
        for i, kp in enumerate(keezer_probes):
            temp = t.get_probe_temp(kp)
            keezer_q.put("{},{}".format(i,temp))
        time.sleep(1)


# get the hostname
my_hostname=socket.gethostname()

# install the signal handler
signal.signal(signal.SIGTERM, sigterm_handler)

# set up the gpio system
GPIO.setmode(GPIO.BOARD)

# disable the power button by opening the relay
if my_hostname == 'rpints':
  GPIO.setup(rpints_power_relay, GPIO.OUT, initial=1)
elif my_hostname == 'brewpi':
  GPIO.setup(brewpi_power_relay, GPIO.OUT, initial=1)
else:
    # fail with an error
    msg = "Unknown hostname, exiting"
    print(msg)
    n.notify("STATUS={}".format(msg))
    n.notify("ERRNO=1")
    sys.exit(1)

# enable the 3.3V to 5V serial converter
GPIO.setup(serial_enable, GPIO.OUT, initial=1)

main_pid = os.getpid()

# allow boot to continue
n = sdnotify.SystemdNotifier()
n.notify("READY=1")
msg = "PANP service PID {} running on {}".format(main_pid, my_hostname)
print(msg)
n.notify("STATUS={}".format(msg))

if my_hostname == 'rpints':
    # initailize the dashboard power relays
    GPIO.setup(lower_dash_power, GPIO.OUT, initial=1)
    GPIO.setup(upper_dash_power, GPIO.OUT, initial=1)
    GPIO.setup(sp_power, GPIO.OUT, initial=1)

    # set up output pin for brewpi
    GPIO.setup(normal_mode_out, GPIO.OUT, initial=0)

    # open the serial ports
    dummy_tx = serial.Serial("/dev/ttyAMA0", 57600)
    tacho_tx = serial.Serial("/dev/ttyAMA1", 57600)
    sp_rx = serial.Serial("/dev/switchpod", 9600)

    # set up the panp button handler
    panp_handler = PANPHandler(tacho_tx, dummy_tx)

    # create the listener for the switchpod
    sp_q = Queue()
    sp_t = Thread(target=get_switchpod, args=(sp_rx, sp_q,))
    sp_t.daemon = True
    sp_t.start()

    # create the tread for the keezer probes
    keezer_q = Queue()
    keezer_t = Thread(target=get_keezer_temps, args=(keezer_q,))
    keezer_t.daemon = True
    keezer_t.start()

    # create the loop handler
    loop_handler = RPintsLoopHandler(tacho_tx, dummy_tx, sp_q, keezer_q)

elif my_hostname == 'brewpi':
    # initailize the message center power relay
    GPIO.setup(msgctr_power, GPIO.OUT, initial=0)

    # set up input pin for auto mode from rpints
    GPIO.setup(normal_mode_in, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    # open the serial port to the speedo display
    speedo_tx = serial.Serial("/dev/ttyAMA1", 57600)

    # dim the speedo as we are in auto mode initially
    brightness = BrightnessHandler([speedo_tx])
    GPIO.add_event_detect(normal_mode_in, GPIO.BOTH, callback=brightness.set_brightness, bouncetime=10)

    loop_handler = BrewPiLoopHandler(speedo_tx)

else:
    # fail with an error
    msg = "Unknown hostname, exiting"
    print(msg)
    n.notify("STATUS={}".format(msg))
    n.notify("ERRNO=1")
    sys.exit(1)

# sleep forever waiting for events
run_loop=True
try:
    while run_loop:
        loop_handler.loop()
        time.sleep(0.25)

except KeyboardInterrupt:
    cleanup_exit()
