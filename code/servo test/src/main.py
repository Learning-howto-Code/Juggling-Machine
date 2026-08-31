import time

import board
import pwmio
from adafruit_motor import servo

# signal wire on GP22 (physical pin 29)
pwm = pwmio.PWMOut(board.GP22, duty_cycle=0, frequency=50)
my_servo = servo.Servo(pwm)

while True:
    my_servo.angle = 0
    time.sleep(1)
    my_servo.angle = 90
    time.sleep(1)
    my_servo.angle = 180
    time.sleep(1)
