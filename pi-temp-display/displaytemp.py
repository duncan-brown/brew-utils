import os
import glob
import time
from threading import Thread
from multiprocessing import Queue
 
# GPIO ports for the 7seg pins + dp
# (b,a,e,d,f,g,c)
segments =  (5,6,13,19,26,12,21)

# GPIO ports for the 8 digit ground pins
# need to add the following to /boot/config.txt
# dtparam=spi=off
# to use spi pins as gpio
# mash temp h,t,u.f
mash_digits = (7,8,22,24)

# high temp t,u
high_digits = (11,9)

# low temp t,u
low_digits = (25,10)

digits = mash_digits + high_digits + low_digits

num = {' ':(0,0,0,0,0,0,0),
    '0':(1,1,1,1,1,0,1),
    '1':(1,0,0,0,0,0,1),
    '2':(1,1,1,1,0,1,0),
    '3':(1,1,0,1,0,1,1),
    '4':(1,0,0,0,1,1,1),
    '5':(0,1,0,1,1,1,1),
    '6':(0,1,1,1,1,1,1),
    '7':(1,1,0,0,0,0,1),
    '8':(1,1,1,1,1,1,1),
    '9':(1,1,0,0,1,1,1)}

keezer_probes = ['/sys/devices/w1_bus_master1/28-3c01b5562fd2/w1_slave',
                 '/sys/devices/w1_bus_master1/28-3c01b556f835/w1_slave']

def producer(out_q):
    while True: 
        keezer_temps = []
        for k in keezer_probes:
            fileobj = open(k,'r')
            lines = fileobj.readlines()
            fileobj.close()
            status = lines[0][-4:-1]
            equals_pos = lines[1].find('t=')
            tempstr = lines[1][equals_pos+2:]
            tempvalue_c = float(tempstr)/1000.0
            tempvalue_f = tempvalue_c * 9.0 / 5.0 + 32.0
            tempvalue = int(round(tempvalue_f,0))
            keezer_temps.append(tempvalue)

        k_min = str(min(keezer_temps)).rjust(2)
        k_max = str(max(keezer_temps)).rjust(2)
        
        display_str='    '+k_max+k_min
        out_q.put(display_str)
        time.sleep(5)

def consumer(in_q):
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)

    for segment in segments:
        GPIO.setup(segment, GPIO.OUT)
        GPIO.output(segment, 0)

    GPIO.setup(16, GPIO.OUT)
    GPIO.output(16, 0)

    for digit in digits:
        GPIO.setup(digit, GPIO.OUT)
        GPIO.output(digit, 1)

    ticks = 0
    my_str = '12345678'
    while True:
        if ticks == 0:
            try:
                display_str = in_q.get(False)
                my_str = display_str
            except:
                pass
        for digit in range(len(digits)):
            ticks += 1
            for loop in range(0,7):
                GPIO.output(segments[loop], num[my_str[digit]][loop])
            if (digits[digit] == 22):
                GPIO.output(16,1)
            else:
                GPIO.output(16,0)
            GPIO.output(digits[digit], 0)
            time.sleep(0.001)
            GPIO.output(digits[digit], 1)
        if ticks > 4998: ticks = 0

try:
    q = Queue()
    t1 = Thread(target = consumer, args=(q,))
    t2 = Thread(target = producer, args=(q,))
    t1.daemon = True
    t2.daemon = True
    t1.start()
    t2.start()
    while True:
        time.sleep(0.1)
finally:
    import RPi.GPIO as GPIO
    try:
        GPIO.cleanup()
    except:
        pass

