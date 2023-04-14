#!/usr/bin/python

import os
import sys
import time
import socket
import signal
import sdnotify
import RPi.GPIO as GPIO
from enum import Enum

GPIO.cleanup()

# gpio pin configuration
power_relay = 29 # GPIO 5

class PANPState(Enum):
    AUTO = 13 # GPIO 27
    NORM = 40 # GPIO 21
    PURSUIT = 37 # GPIO 26

class PANPButton(Enum):
    AUTO = 19 # GPIO 10
    NORM = 21 # GPIO 9
    PURSUIT = 26 # GPIO 7

class PANPHandler:
    # current state
    state = None
    last_push = None

    # initalize lamps on boot
    def __init__(self):
        for p in PANPState:
            GPIO.setup(p.value, GPIO.OUT, initial=1)
        self.state = PANPState.AUTO
        GPIO.output(self.state.value,0)
        for p in PANPButton:
            GPIO.setup(p.value, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(p.value, GPIO.FALLING, callback=self.button, bouncetime=50)

    def change_state(self, channel):
        old_state = self.state
        if channel is PANPButton.AUTO.value:
            self.state = PANPState.AUTO
        elif channel is PANPButton.NORM.value:
            self.state = PANPState.NORM
        elif channel is PANPButton.PURSUIT.value:
            self.state = PANPState.PURSUIT
        GPIO.output(old_state.value,1)
        GPIO.output(self.state.value,0)

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
    for p in PANPState:
        GPIO.output(p.value, 1)
    sys.exit(0)
signal.signal(signal.SIGTERM, sigterm_handler)

# set up the gpio system
GPIO.setmode(GPIO.BOARD)

# disable the power button by opening the relay
GPIO.setup(power_relay, GPIO.OUT, initial=1)

# allow boot to continue
n = sdnotify.SystemdNotifier()
n.notify("READY=1")

my_hostname=socket.gethostname()
n.notify("STATUS=PANP service running on {0}".format(my_hostname))

if my_hostname == 'rpints':
    h = PANPHandler()
    while True:
        time.sleep(1)
elif my_hostname == 'brewpi':
    while True:
        time.sleep(1)
else:
    n.notify("Unknown hostname, exiting")
    n.notify("ERRNO=1")
    sys.exit(1)
