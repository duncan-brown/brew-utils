#!/usr/bin/python

import os
import sys
import time
import json
import math
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
serial_enable = 22    # GPIO 25
auto_mode_comm = 26   # GPIO 7 (CE1)
normal_mode_comm = 15 # GPIO 22

# gpio pin configuration for rpints
lower_dash_power = 19   # GPIO 10 (MOSI)
upper_dash_power = 12   # GPIO 18
sp_power = 11           # GPIO 17

# gpio pin configurations for brewpi
msgctr_power = 36       # GPIO 16


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


class MsgCtrMode(Enum):
    MASH_TEMP = 0
    HLT_TEMP = 1
    UNITANK1_TEMP = 2
    UNITANK1_SG = 3
    UNITANK2_TEMP = 4
    UNITANK2_SG = 5
    CHRONICAL_TEMP = 6
    CHRONICAL_SG = 7
    BREWPI_UP = 8
    MASH_TEMP_C = 9


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
                33.0, 36.0, 40.0,
                43.0, 46.0, 50.0,
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
        while self.sp_q.qsize() > 0:
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


class BrewPiLoopHandler():
    def __init__(self, speedo_tx, msgctr_tx, sp_q, hot_side_q, brewpi_rmx_q):
        # serial communications for speedo and message center
        self.speedo_tx = speedo_tx
        self.msgctr_tx = msgctr_tx

        # queue for switchpod keypresses
        self.sp_q = sp_q

        # hot side temperature data
        self.hot_side_q = hot_side_q
        self.hot_side_temps = [0, 0] # mash, hlt

        # message center display text
        self.msgctr_mode_old = MsgCtrMode.BREWPI_UP
        self.msgctr_mode = MsgCtrMode.BREWPI_UP

        # auto mode status
        self.auto_mode = GPIO.HIGH

        # brewpi data utk1, utk2, chrn
        self.brewpi_rmx_q = brewpi_rmx_q
        self.brewpi_rmx_data = [0.0, 0.000, 0.0, 0.000, 0.0, 0.000]

    def get_probe_temp_units(self, tempvalue_f):
        tenths = int(round(tempvalue_f % 1 * 10.0, 0))
        ones = int(tempvalue_f % 10)
        tens = int(tempvalue_f // 10 % 10)
        hundreds = int(tempvalue_f // 100 % 10)
        return hundreds, tens, ones, tenths

    def temperature_bar(self, temp):
        bar_temps = [30.0, 34.0, 36.0, 40.0, 44.0,
                46.0, 50.0, 52.0, 54.0, 58.0, 60.0,
                64.0, 68.0, 70.0, 74.0, 76.0]
        bar_vals = [0x10, 0x20, 0x30, 0x40, 0x50,
                0x60, 0x70, 0x79, 0x88, 0x97,
                0xA6, 0xB5, 0xC4, 0xD3, 0xE2, 0xFF]
        for i, t in enumerate(bar_temps):
            if temp < t: break
        return bar_vals[i]

    def set_auto_mode(self,channel):
        channel_state = GPIO.input(channel)
        if channel_state is not self.auto_mode:
            self.auto_mode = channel_state
            if channel_state == GPIO.HIGH:
                self.msgctr_mode = MsgCtrMode.BREWPI_UP
                self.msgctr_msg = ">CScBREWPI UP?"
                time.sleep(0.25)
                self.msgctr_tx.write(str.encode(self.msgctr_msg))
                self.msgctr_mode_old = self.msgctr_mode
                time.sleep(0.25)

    def loop(self):
        global run_loop
        if run_loop is False:
            return

        # get any updated temperatures from the queue
        while self.hot_side_q.qsize() > 0:
            hot_side_data = self.hot_side_q.get()
            idx, temp = hot_side_data.split(',')
            self.hot_side_temps[int(idx)] = float(temp)

        # get any updated brewpi data from the queue
        while self.brewpi_rmx_q.qsize() > 0:
            brewpi_rmx_data = self.brewpi_rmx_q.get()
            idx, value = brewpi_rmx_data.split(',')
            self.brewpi_rmx_data[int(idx)] = float(value)

        # Update the mode if there was a button press
        while self.sp_q.qsize() > 0:
            try:
                sp_val = int(sp_q.get())
            except:
                sp_val = None
            if sp_val == 0:   # SILENT MODE
                self.msgctr_mode = MsgCtrMode.MASH_TEMP
                self.msgctr_msg = ">CScDEG F MASH?"
            elif sp_val == 1: # LASER
                self.msgctr_mode = MsgCtrMode.MASH_TEMP_C
                self.msgctr_msg = ">CScDEG C MASH?"
            elif sp_val == 2: # TEAR GAS
                self.msgctr_mode = MsgCtrMode.UNITANK1_TEMP
                self.msgctr_msg = ">CScDEG F UTK1?"
            elif sp_val == 3: # X-RAY
                self.msgctr_mode = MsgCtrMode.UNITANK1_SG
                self.msgctr_msg = ">CScSG UTK1?"
            elif sp_val == 4: # AUTO ROOF L
                self.msgctr_mode = MsgCtrMode.UNITANK2_TEMP
                self.msgctr_msg = ">CScDEG F UTK2?"
            elif sp_val == 5: # GRPLG. HOOK
                self.msgctr_mode = MsgCtrMode.UNITANK2_SG
                self.msgctr_msg = ">CScSG UTK2?"
            elif sp_val == 6: # MICRO-JAM
                self.msgctr_mode = MsgCtrMode.CHRONICAL_TEMP
                self.msgctr_msg = ">CScDEG F CHRN?"
            elif sp_val == 7: # SMOKE RELEASE
                self.msgctr_mode = MsgCtrMode.CHRONICAL_SG
                self.msgctr_msg = ">CScSG CHRN?"
            elif sp_val == 8: # EJECT L
                self.msgctr_mode = MsgCtrMode.BREWPI_UP
                self.msgctr_msg = ">CScBREWPI UP?"
            elif sp_val == 9: # H6
                self.msgctr_mode = MsgCtrMode.HLT_TEMP
                self.msgctr_msg = ">CScDEG F HLT?"
            else:
                self.msgctr_mode = MsgCtrMode.MASH_TEMP
                self.msgctr_msg = ">CScDEG F MASH?"

        # if we are in auto mode no one is listening
        if self.auto_mode == GPIO.LOW:
            try:
                if self.msgctr_mode is not self.msgctr_mode_old:
                    self.msgctr_tx.write(str.encode(self.msgctr_msg))
                    self.msgctr_mode_old = self.msgctr_mode
                    time.sleep(0.1)

                # write hlt temp to upper display
                hundreds, tens, ones, tenths = self.get_probe_temp_units(self.hot_side_temps[1])
                msg = ">BBc{:0>2X}?".format(hundreds*10 + tens)

                self.speedo_tx.write(str.encode(msg))
                msg = ">BHd0{0}0{1}0{2}03?".format(hundreds, tens, ones)
                self.speedo_tx.write(str.encode(msg))

                dp_mode = 1
                if (self.msgctr_mode == MsgCtrMode.MASH_TEMP):
                    value = self.hot_side_temps[0]
                    n_leds = (value-110.0)//5.0
                elif (self.msgctr_mode == MsgCtrMode.MASH_TEMP_C):
                    value = (self.hot_side_temps[0] - 32.0) * 5.0/9.0
                    n_leds = (value-20.0)//4.0
                elif (self.msgctr_mode == MsgCtrMode.UNITANK1_TEMP):
                    value = self.brewpi_rmx_data[0]
                    n_leds = (value-34.0)//3.0
                elif (self.msgctr_mode == MsgCtrMode.UNITANK2_TEMP):
                    value = self.brewpi_rmx_data[2]
                    n_leds = (value-34.0)//3.0
                elif (self.msgctr_mode == MsgCtrMode.CHRONICAL_TEMP):
                    value = self.brewpi_rmx_data[4]
                    n_leds = (value-34.0)//3.0
                elif (self.msgctr_mode == MsgCtrMode.BREWPI_UP):
                    value = self.hot_side_temps[0]
                    n_leds = (value-45.0)//10.0
                elif (self.msgctr_mode == MsgCtrMode.HLT_TEMP):
                    value = self.hot_side_temps[1]
                    n_leds = (value-110.0)//5.0
                elif (self.msgctr_mode == MsgCtrMode.UNITANK1_SG):
                    dp_mode = 3
                    value = self.brewpi_rmx_data[1] * 100.0
                    n_leds = (value*10.0-1000.0)//4.0
                elif (self.msgctr_mode == MsgCtrMode.UNITANK2_SG):
                    dp_mode = 3
                    value = self.brewpi_rmx_data[3] * 100.0
                    n_leds = (value*10.0-1000.0)//4.0
                elif (self.msgctr_mode == MsgCtrMode.CHRONICAL_SG):
                    dp_mode = 3
                    value = self.brewpi_rmx_data[5] * 100.0
                    n_leds = (value*10.0-1000.0)//4.0

                # update lower display leds
                n_leds = int(math.floor(n_leds))
                if ( n_leds < 0 ):
                    n_leds = 0
                elif ( n_leds > 16 ):
                    n_leds = 16
                msg = ">BBb{:0>2X}?".format(n_leds)
                self.speedo_tx.write(str.encode(msg))

                # update lower display digits
                hundreds, tens, ones, tenths = self.get_probe_temp_units(value)
                msg = ">BHe0{0}0{1}0{2}0{3}0{4}?".format(hundreds, tens, ones, tenths, dp_mode)
                self.speedo_tx.write(str.encode(msg))

                # write the fermenter temps to the green/red dummy3
                msg = ">FHm{:0>2X}{:0>2X}{:0>2X}?".format(
                        self.temperature_bar(self.brewpi_rmx_data[0]),
                        self.temperature_bar(self.brewpi_rmx_data[2]),
                        self.temperature_bar(self.brewpi_rmx_data[4]))
                self.speedo_tx.write(str.encode(msg))

            except PortNotOpenError:
                if run_loop is True:
                    raise PortNotOpenError
                else:
                    pass


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
        GPIO.output(auto_mode_comm, 1)
        GPIO.output(normal_mode_comm, 0)
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
            self.state = PANPState.AUTO
            GPIO.output(old_state.value,0)
            GPIO.output(self.state.value,1)
            GPIO.output(lower_dash_power, 1)
            GPIO.output(upper_dash_power, 1)
            GPIO.output(sp_power, 1)
            GPIO.output(normal_mode_comm, 0)
            GPIO.output(auto_mode_comm, 1)
        elif channel is PANPButton.NORM.value:
            self.state = PANPState.NORM
            GPIO.output(old_state.value,0)
            GPIO.output(self.state.value,1)
            GPIO.output(lower_dash_power, 0)
            GPIO.output(upper_dash_power, 0)
            GPIO.output(sp_power, 1)
            GPIO.output(normal_mode_comm, 1)
            GPIO.output(auto_mode_comm, 0)
            self.brightness.set_brightness(normal_mode_comm)
        elif channel is PANPButton.PURSUIT.value:
            self.state = PANPState.PURSUIT
            GPIO.output(old_state.value,0)
            GPIO.output(self.state.value,1)
            GPIO.output(lower_dash_power, 0)
            GPIO.output(upper_dash_power, 0)
            GPIO.output(sp_power, 0)
            GPIO.output(normal_mode_comm, 0)
            GPIO.output(auto_mode_comm, 0)
            self.brightness.set_brightness(normal_mode_comm)
        if old_state is PANPState.AUTO:
            for i in range(1,0,-1):
                time.sleep(i)
                self.brightness.set_brightness(normal_mode_comm, True)
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
                    time.sleep(0.1)
                self.brightness = channel_state


# function to clean up on exit
def cleanup_exit():
    global speedo_tx
    global tacho_tx
    global dummy_tx
    global msgctr_tx

    msg="PANP service on {0} got KeyboardInterrupt".format(my_hostname)
    print(msg)
    n.notify("STATUS={0}".format(msg))

    # close all the serial ports
    time.sleep(1)
    if my_hostname == 'brewpi':
        speedo_tx.close()
        msgctr_tx.close()
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
        GPIO.output(normal_mode_comm, 0)
        GPIO.output(auto_mode_comm, 1)
    else:
        GPIO.remove_event_detect(normal_mode_comm)
        GPIO.remove_event_detect(auto_mode_comm)
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
def get_switchpod(sp_q):
    path = "/dev/switchpod"
    while os.access(path, os.R_OK) is False:
        time.sleep(1)
    rx = serial.Serial("/dev/switchpod", 9600, timeout=None)
    while True:
        try:
            state = rx.read(2)
            data = state.decode().strip()
            if data:
                sp_q.put(data)
        except:
            pass


# function to get the temperatures from one-wire probes
def get_temps(probes,q):
    t = TempProbe()
    while True:
        try:
            for i, kp in enumerate(probes):
                temp = t.get_probe_temp(kp)
                q.put("{},{}".format(i,temp))
        except:
            pass
        time.sleep(1)


def get_brewpi_rmx_data(q):
    while True:
        for i, fermenter in enumerate(["unitank-1", "unitank-2", "chronical"]):
            for j, msg in enumerate(["lcd", "statusText"]):
                try:
                    time.sleep(1)
                    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                    s.connect('/home/brewpi/{}/KITTSOCKET'.format(fermenter))
                    s.sendall(msg.encode())
                    s.settimeout(2)
                    if j == 0:
                        data = json.loads(s.recv(4096).decode())[1].split()[1]
                    else:
                        data = json.loads(s.recv(4096).decode())["0"]["Tilt SG: "]
                    s.close()
                except:
                    data = 0.0
                    try:
                        s.close()
                    except:
                        pass
                q.put("{},{}".format(i*2+j,data))
        time.sleep(60)


if __name__ == "__main__":
    # get the hostname
    my_hostname=socket.gethostname()

    # install the signal handler
    signal.signal(signal.SIGTERM, sigterm_handler)

    # set up the gpio system
    GPIO.setmode(GPIO.BOARD)

    # enable the 3.3V to 5V serial converter
    GPIO.setup(serial_enable, GPIO.OUT, initial=1)

    # store the pid of the parent process
    main_pid = os.getpid()

    # allow boot to continue
    n = sdnotify.SystemdNotifier()
    n.notify("READY=1")
    msg = "PANP service PID {} running on {}".format(main_pid, my_hostname)
    print(msg)
    n.notify("STATUS={}".format(msg))

    # create the listener for the switchpod
    sp_q = Queue()
    sp_t = Thread(target=get_switchpod, args=(sp_q,))
    sp_t.daemon = True
    sp_t.start()

    if my_hostname == 'rpints':
        # initailize the dashboard power relays
        GPIO.setwarnings(False)
        GPIO.setup(lower_dash_power, GPIO.OUT, initial=1)
        GPIO.setup(upper_dash_power, GPIO.OUT, initial=1)
        GPIO.setup(sp_power, GPIO.OUT, initial=1)

        # set up output pin for brewpi
        GPIO.setup(normal_mode_comm, GPIO.OUT, initial=0)
        GPIO.setup(auto_mode_comm, GPIO.OUT, initial=0)
        GPIO.setwarnings(True)

        # open the serial ports
        dummy_tx = serial.Serial("/dev/ttyAMA0", 57600)
        tacho_tx = serial.Serial("/dev/ttyAMA1", 57600)

        # set up the panp button handler
        panp_handler = PANPHandler(tacho_tx, dummy_tx)

        # create the tread for the keezer probes
        keezer_probes = [
                "/sys/bus/w1/devices/28-012052b92541/w1_slave",
                "/sys/bus/w1/devices/28-012058f936f3/w1_slave",
                "/sys/bus/w1/devices/28-012052ba8dab/w1_slave",
                "/sys/bus/w1/devices/28-012058fbceb5/w1_slave",
                "/sys/bus/w1/devices/28-012058fc2851/w1_slave",
                "/sys/bus/w1/devices/28-012052b426ca/w1_slave" ]
        keezer_q = Queue()
        keezer_t = Thread(target=get_temps, args=(keezer_probes, keezer_q,))
        keezer_t.daemon = True
        keezer_t.start()

        # create the loop handler
        loop_handler = RPintsLoopHandler(tacho_tx, dummy_tx, sp_q, keezer_q)

    elif my_hostname == 'brewpi':
        # initailize the message center power relay
        GPIO.setwarnings(False)
        GPIO.setup(msgctr_power, GPIO.OUT, initial=0)

        # set up input pin for auto mode from rpints
        GPIO.setup(normal_mode_comm, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(auto_mode_comm, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setwarnings(True)

        # open the serial port to the speedo display
        msgctr_tx = serial.Serial("/dev/ttyAMA0", 57600)
        speedo_tx = serial.Serial("/dev/ttyAMA1", 57600)

        # dim the speedo as we are in auto mode initially
        brightness = BrightnessHandler([speedo_tx])
        GPIO.add_event_detect(normal_mode_comm, GPIO.BOTH, callback=brightness.set_brightness, bouncetime=10)

        hot_side_probes = [
                "/sys/bus/w1/devices/28-012052b65be5/w1_slave",
                "/sys/bus/w1/devices/28-0120529d8f20/w1_slave" ]
        hot_side_q = Queue()
        hot_side_t = Thread(target=get_temps, args=(hot_side_probes, hot_side_q,))
        hot_side_t.daemon = True
        hot_side_t.start()

        brewpi_rmx_q = Queue()
        brewpi_rmx_t = Thread(target=get_brewpi_rmx_data, args=(brewpi_rmx_q,))
        brewpi_rmx_t.daemon = True
        brewpi_rmx_t.start()

        loop_handler = BrewPiLoopHandler(speedo_tx, msgctr_tx, sp_q, hot_side_q, brewpi_rmx_q)
        GPIO.add_event_detect(auto_mode_comm, GPIO.BOTH, callback=loop_handler.set_auto_mode, bouncetime=10)

    else:
        # fail with an error
        msg = "Unknown hostname, exiting"
        print(msg)
        n.notify("STATUS={}".format(msg))
        n.notify("ERRNO=1")
        sys.exit(1)

    # sleep forever waiting for events
    time.sleep(3)
    msg = "PANP service PID {} on {} entering main loop".format(main_pid, my_hostname)
    print(msg)
    n.notify("STATUS={}".format(msg))
    run_loop=True
    try:
        while run_loop:
            loop_handler.loop()
            time.sleep(0.25)

    except KeyboardInterrupt:
        cleanup_exit()
