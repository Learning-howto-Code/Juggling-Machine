

from tmc.TMC_5160 import  (
    Tmc2209,
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
stop_one = tmc.do_homing(26, 1, 50)








