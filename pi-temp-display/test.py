# code modified, tweaked and tailored from code by bertwert 
# on RPi forum thread topic 91796
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
 
# GPIO ports for the 7seg pins + dp
# (b,a,e,d,f,g,c)
segments =  (5,6,13,19,26,12,21)

for segment in segments:
    GPIO.setup(segment, GPIO.OUT)
    GPIO.output(segment, 0)

GPIO.setup(16, GPIO.OUT)
GPIO.output(16, 0)

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

digits = low_digits + high_digits + mash_digits

for digit in digits:
    GPIO.setup(digit, GPIO.OUT)
    GPIO.output(digit, 1)

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

n = 0
ticks = 0

try:
    while True:
        for digit in range(len(digits)):
            GPIO.output(digits[digit], 1)
            for loop in range(0,7):
                GPIO.output(segments[loop], num[str(n)][loop])
            if (digits[digit] == 22) and (ticks < 500):
                GPIO.output(16,1)
            else:
                GPIO.output(16,0)
            GPIO.output(digits[digit], 0)
            time.sleep(0.001)
            ticks += 1
        if ticks > 1000:
            ticks = 0
            if n > 8:
                n = 0
            else:
                n += 1
finally:
    GPIO.cleanup()
