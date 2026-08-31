from machine import Pin, PWM  # type: ignore
import time

# Standard hobby servo: 50 Hz, ~0.5 ms (0 deg) to ~2.5 ms (180 deg) pulse
MIN_NS = 500_000
MAX_NS = 2_500_000


def make_servo(gpio):
    pwm = PWM(Pin(gpio))
    pwm.freq(50)
    return pwm


def set_angle(pwm, angle):
    angle = min(max(angle, 0), 180)
    pwm.duty_ns(int(MIN_NS + (MAX_NS - MIN_NS) * angle / 180))


servo1 = make_servo(22)

try:
    while True:
        for angle in (0, 90, 180):
            print("turning to", angle)
            set_angle(servo1, angle)
            time.sleep(1)
except KeyboardInterrupt:
    servo1.deinit()
