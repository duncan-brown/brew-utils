# code modified, tweaked and tailored from code by bertwert 
# on RPi forum thread topic 91796
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
 
# GPIO ports for the 7seg pins + dp
# (b,a,e,d,f,g,dp,c)
segments =  (5,6,13,19,26,16,20,21)

for segment in segments:
    GPIO.setup(segment, GPIO.OUT)
    GPIO.output(segment, 0)

# GPIO ports for the 8 digit ground pins 
digits = (4,17,18,27,22,23,25)

for digit in digits:
    GPIO.setup(digit, GPIO.OUT)
    GPIO.output(digit, 1)

num = {' ':(0,0,0,0,0,0,0,0),
    '0':(1,1,1,1,1,0,1,1),
    '1':(1,0,0,0,0,0,1,1),
    '2':(1,1,1,1,0,1,1,0),
    '3':(1,1,0,1,0,1,1,1),
    '4':(1,0,0,0,1,1,1,1),
    '5':(0,1,0,1,1,1,1,1),
    '6':(0,1,1,1,1,1,1,1),
    '7':(1,1,0,0,0,0,1,1),
    '8':(1,1,1,1,1,1,1,1),
    '9':(1,1,0,0,1,1,1,1)}

n = 0
ticks = 0

try:
    while True:
        for loop in range(0,8):
            GPIO.output(segments[loop], num[str(n)][loop])
            for digit in range(8):
                GPIO.output(digits[digit], 0)
                time.sleep(0.001)
                GPIO.output(digits[digit], 1)
            ticks += 1
            if ticks > 999:
                ticks = 0
                if n > 9:
                    n = 0
                else:
                    n += 1
finally:
    GPIO.cleanup()
