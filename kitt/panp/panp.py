#!/usr/bin/python

import os
import sys
import time
import socket
import signal
import serial
import sdnotify
import RPi.GPIO as GPIO
from enum import Enum

GPIO.cleanup()

# common gpio pins
serial_enable = 22 # GPIO 25

# gpio pin configuration for rpints
power_relay = 16      # GPIO 23
lower_dash_power = 19 # GPIO 10 (MOSI)
upper_dash_power = 12 # GPIO 18
sp_power = 11         # GPIO 17
normal_mode_out = 15 # GPIO 22

# gpio pin configurations for brewpi
msgctr_power = 36    # GPIO 16
normal_mode_in = 15 # GPIO 22

# serial configuration for rpints
tacho_tx_dev = '/dev/ttyAMA0'
dummy_tx_dev = '/dev/ttyAMA1'

# serial configuration for brewpi
msg_ctr_tx_dev = '/dev/ttyAMA0'
speedo_tx_dev = '/dev/ttyAMA1'

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
    def __init__(self):
        for p in PANPState:
            GPIO.setup(p.value, GPIO.OUT, initial=0)
        self.state = PANPState.AUTO
        GPIO.output(self.state.value,1)
        for p in PANPButton:
            GPIO.setup(p.value, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(p.value, GPIO.FALLING, callback=self.button, bouncetime=50)

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
            self.state = PANPState.NORM
        elif channel is PANPButton.PURSUIT.value:
            GPIO.output(lower_dash_power, 0)
            GPIO.output(upper_dash_power, 0)
            GPIO.output(sp_power, 0)
            GPIO.output(normal_mode_out, 0)
            self.state = PANPState.PURSUIT
        GPIO.output(old_state.value,0)
        GPIO.output(self.state.value,1)

    def button(self, channel):
        if channel is not self.last_push:
            time.sleep(0.05)
            if GPIO.input(channel) == GPIO.LOW:
                self.change_state(channel)
                self.last_push = channel


# set up exit signal handler for systemd
def sigterm_handler(_signo, _stack_frame):
    msg="PANP service on {0} exiting on SIGTERM".format(my_hostname)
    print(msg)
    n.notify("STATUS={0}".format(msg))
    n.notify("STOPPING=1")
    if my_hostname == 'rpints':
        for p in PANPState:
            GPIO.output(p.value, 0)
        GPIO.output(lower_dash_power, 1)
        GPIO.output(upper_dash_power, 1)
        GPIO.output(sp_power, 1)
        GPIO.output(normal_mode_out, 0)
    else:
        GPIO.output(msgctr_power,0)
    GPIO.output(serial_enable,0)
    sys.exit(0)


# send a command to dim the speedo
class SpeedoBrightnessHandler:
    def __init__(self):
        self.speedo_brightness = GPIO.HIGH
        self.boot = True
        
    def brightness(self, channel):
        if self.boot:
            time.sleep(1)
            self.boot = False
        speedo_tx = serial.Serial(speedo_tx_dev, 57600)
        channel_state = GPIO.input(channel)
        if channel_state is not self.speedo_brightness:
            if channel_state:
                print("setting speedo dim")
                speedo_tx.write(str.encode('>BBD40?'))
            else:
                print("setting speedo bright")
                speedo_tx.write(str.encode('>BBDFF?'))
            self.speedo_brightness = channel_state
        speedo_tx.close()

# install the signal handler
signal.signal(signal.SIGTERM, sigterm_handler)

# set up the gpio system
GPIO.setmode(GPIO.BOARD)

# disable the power button by opening the relay
GPIO.setup(power_relay, GPIO.OUT, initial=1)

# enable the 3.3V to 5V serial converter
GPIO.setup(serial_enable, GPIO.OUT, initial=1)

# allow boot to continue
n = sdnotify.SystemdNotifier()
n.notify("READY=1")

my_hostname=socket.gethostname()
n.notify("STATUS=PANP service running on {0}".format(my_hostname))

if my_hostname == 'rpints':
    # initailize the dashboard power relays
    GPIO.setup(lower_dash_power, GPIO.OUT, initial=1)
    GPIO.setup(upper_dash_power, GPIO.OUT, initial=1)
    GPIO.setup(sp_power, GPIO.OUT, initial=1)

    # set up output pin for brewpi
    GPIO.setup(normal_mode_out, GPIO.OUT, initial=0)

    # set up the panp button handler
    h = PANPHandler()

elif my_hostname == 'brewpi':
    # initailize the message center power relay
    GPIO.setup(msgctr_power, GPIO.OUT, initial=0)

    # set up input pin for auto mode from rpints
    GPIO.setup(normal_mode_in, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    # dim the speedo as we are in auto mode initially
    speedo_brightness = SpeedoBrightnessHandler()
    GPIO.add_event_detect(normal_mode_in, GPIO.BOTH, callback=speedo_brightness.brightness, bouncetime=10)

else:
    # fail with an error
    n.notify("Unknown hostname, exiting")
    n.notify("ERRNO=1")
    sys.exit(1)

# sleep forever waiting for events
while True:
    time.sleep(1)
