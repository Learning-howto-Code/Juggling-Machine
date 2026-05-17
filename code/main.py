from tmc.TMC_5160 import TMC_5160

motor = TMC_5160(
    spi_id=0,
    cs_pin=17,
    sck_pin=18,
    mosi_pin=19,
    miso_pin=16,
    step_pin=2,
    dir_pin=3,
    en_pin=6
)

motor.set_direction(0)
motor.set_current(run_current_ma=2000, hold_current_ma=500)
motor.set_microstepping_resolution(8)
motor.set_motor_enabled(True)

# Move with step/dir
motor.make_a_step()           # single step
motor.run_to_position_steps(1600)  # move 8 revolutions (200 steps/rev)

def stop():
    tmc.tmc_mc.stop()
tmc.set_stallguard_callback(16, 50, stop) # pin 16, threshhold 50, runs stop funtion