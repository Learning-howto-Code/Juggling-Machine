
from machine import Pin, PWM, machine # type: ignore
from time import sleep, time
import logging
import threading

from tmc.TMC_5160 import  ( # type: ignore
    stopMode,
    Loglevel,
    Board,
    tmc_gpio,
    MovementPhase,
    MovementAbsRel,
    TMC_5160,
    TmcEnableControlPin,
    TmcMotionControlStepDir,
)

range_angle = 120
distance = -((round(range_angle/360)*200))
motor = TMC_5160(
    spi_id=0,
    cs_pin=17,
    sck_pin=18,
    mosi_pin=19,
    miso_pin=16,
    step_pin=2,
    dir_pin=3,
    en_pin=6,
    DIAG0=31
)
servoLeft= machine.Pin(29)
servoRight= machine.Pin(27)
# these functions change settings in the TMC register
# -----------------------------------------------------------------------
tmc.set_direction_reg(False)
tmc.set_current_rms(300)
tmc.set_interpolation(True)
tmc.set_spreadcycle(False)
tmc.set_microstepping_resolution(2)
tmc.set_internal_rsense(False)

# these functions read and print the current settings in the TMC register
tmc.read_register("ioin")
tmc.read_register("chopconf")
tmc.read_register("drvstatus")
tmc.read_register("gconf")

print("---\n---")

# set the Acceleration and maximal Speed in fullsteps
# -----------------------------------------------------------------------
tmc.acceleration_fullstep = 1000
tmc.max_speed_fullstep = 250

# sets up log verbosity, and absolute cords
tmc.tmc_logger.loglevel = Loglevel.DEBUG
tmc.movement_abs_rel = MovementAbsRel.ABSOLUTE
# activate the motor current output
# -----------------------------------------------------------------------
tmc.set_motor_enabled(True)

tmc.test_stallguard_threshold(200) # rotates once

def my_callback():
    """StallGuard callback"""
    print("StallGuard detected!!")
    tmc.tmc_mc.stop()

# 1, pin, threshhold(50 is default), callback funtion, min speed threshold(optional)
tmc.set_stallguard_callback( # will call my_callback when stallguard detected
    26, 50, my_callback
)  # after this function call, StallGuard is active


# uses STEP/DIR to move the motor
result = tmc.run_to_position_steps(4000, MovementAbsRel.RELATIVE)
if result is StopMode.NO:
    print("Movement finished successfully")
else:
    print("Movement was not completed")

# ^^is all setup^^
tmc.set_motor_enabled(True)
def home():
    stopLeft = tmc.do_homing(26, 1, 50)
    positionLeft = tmc.current_pos

    stopRight= tmc.do_homing(27, 1, 50)
    positionRight = tmc.current_pos

servo = PWM(servoLeft)
max_duty = 7864
min_duty = 1802
half_duty = int(max_duty/2)
servo.freq (50)
def moveLeft(): # needs to move clockwise to launch ball
    start = time.time_ns()
    def run_motor():
        tmc.run_to_position_fullsteps(distance)
        tmc.run_to_position_fullsteps(-distance)
    def run_servo():
        servo.duty_u16(half_duty)
        servo.duty_u16(0)
    left_thread_motor = threading.Thread(target=run_motor, name="left_motor_thread")
    left_thread_servo = threading.Thread(target=run_servo, name="left_servo_thread")
    left_thread_motor.start(), left_thread_servo.start() # uses threads so that both can move at the same time
    left_total = time.time_ns() - start
    return left_total
def moveRight():
    start = time.time_ns()
    def run_motor():
        tmc.run_to_position_fullsteps(-distance)
        tmc.run_to_position_fullsteps(distance)
    def run_servo():
        servo.duty_u16(-(half_duty))
        servo.duty_u16(0)
    right_thread_motor = threading.Thread(target=run_motor, name="right_motor_thread")
    right_thread_servo = threading.Thread(target=run_servo, name="right_servo_thread")
    right_thread_motor.start(), right_thread_servo.start() # uses threads so that both can move at the same time    
    end = time.time_ns()
    right_total = end - start
    return right_total

while True:
    left_total = moveLeft()/1_000_000_000 # converts nanoseconds to seconds
    print("left arm run time is {left_total} seconds")
    sleep(left_total/2)
    right_total = moveRight()/1_000_000_000 # converts nanoseconds to seconds
    print("right arm run time is {right_total} seconds")
    sleep(right_total/2)






    









