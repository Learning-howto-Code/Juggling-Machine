"""
Top speed test for one stepper driver: BIGTREETECH TMC5160T Pro V1.0 on a
Raspberry Pi Pico 2 running MicroPython. Tests one driver (ACTIVE_DRIVER).

The motor ramps up through SPEED_STEPS_RPM, holding each speed for HOLD_S seconds.
At each speed it prints SG_RESULT and the speed the driver measured (from TSTEP).
SG_RESULT drops as the motor runs out of torque, and sits at 0 once it loses sync
(it buzzes instead of turning). The last speed that ran smoothly is about the top
speed at this current and supply voltage.

STEP pulses come from hardware PWM, so the speed isn't limited by how fast
MicroPython can toggle a pin. Press Ctrl-C at any time to ramp down and stop.

VM (24 V) must be on before running this, the TMC5160 logic is powered from it.
"""

from machine import Pin, SPI, PWM  # type: ignore
import math
import time

# Shared SPI bus (both drivers)
PIN_SCK = 18
PIN_MOSI = 19
PIN_MISO = 16

# Per-driver pins. DIAG0 is open drain with a 10k pull-up on the PCB, low = driver error.
# EN 2 is not routed on the PCB - it needs a jumper wire to GPIO8.
DRIVER_PINS = {
    1: dict(en=20, csn=17, step=10, dir=11, diag0=6, diag1=7),
    2: dict(en=8, csn=13, step=15, dir=14, diag0=4, diag1=5),
}
ACTIVE_DRIVER = 2     # which driver to test (1 or 2), the other stays disabled

RSENSE = 0.075        # ohms, fitted on the TMC5160T Pro V1.0 (3.1 A RMS max)
RUN_CURRENT = 0.8     # A RMS, motor is rated 2.8 A - more current = more torque at speed
HOLD_CURRENT = 0.4    # A RMS
MRES = 4              # 0=256 ... 4=16 ... 8=full step microsteps per STEP pulse
MICROSTEPS = 256 >> MRES
FULL_STEPS_PER_REV = 200

# Speed ramp
FORWARD = True
START_RPM = 30        # starts here instantly, below the speed where it could fail to start
SPEED_STEPS_RPM = [60, 120, 180, 240, 300]
ACCEL_RPM_PER_S = 50 # acceleration between speeds, lower it if it stalls while speeding up
HOLD_S = 2            # time at each speed
RAMP_UPDATE_MS = 5    # how often the speed is updated while ramping

# Stall check: SG_RESULT stuck at 0 means the motor lost sync
STOP_ON_STALL = True
STALL_CHECK_MIN_RPM = 100  # SG_RESULT isn't meaningful at low speed
STALL_SAMPLES = 10    # consecutive SG_RESULT=0 reads counted as a stall
SAMPLE_MS = 20        # time between SG_RESULT reads

# TMC5160 registers
GCONF = 0x00
GSTAT = 0x01
IOIN = 0x04
GLOBALSCALER = 0x0B
IHOLD_IRUN = 0x10
TPOWERDOWN = 0x11
TSTEP = 0x12
CHOPCONF = 0x6C
DRV_STATUS = 0x6F

VFS = 0.325           # full scale sense voltage (TMC5160 datasheet)
FCLK = 12_000_000     # internal clock, CLK is tied to GND on the module
TMC5160_VERSION = 0x30

spi = SPI(
    0,
    baudrate=1_000_000,
    polarity=1,       # SPI mode 3
    phase=1,
    bits=8,
    firstbit=SPI.MSB,
    sck=Pin(PIN_SCK),
    mosi=Pin(PIN_MOSI),
    miso=Pin(PIN_MISO),
)


class Driver:
    def __init__(self, number, en, csn, step, dir, diag0, diag1):
        self.number = number
        self.csn = Pin(csn, Pin.OUT, value=1)  # high keeps it off the shared MISO line
        self.en = Pin(en, Pin.OUT, value=1)  # active low, start with the power stage off
        self.en_gpio = en
        self.step = Pin(step, Pin.OUT, value=0)
        self.direction = Pin(dir, Pin.OUT, value=0)
        self.diag0 = Pin(diag0, Pin.IN)
        self.diag1 = Pin(diag1, Pin.IN)

    def transfer(self, addr, data=0):
        """Send one 40-bit datagram, return (status byte, 32-bit data)."""
        tx = bytes([addr]) + data.to_bytes(4, "big")
        rx = bytearray(5)
        self.csn.value(0)
        spi.write_readinto(tx, rx)
        self.csn.value(1)
        return rx[0], int.from_bytes(rx[1:], "big")

    def write_reg(self, addr, value):
        self.transfer(addr | 0x80, value)

    def read_reg(self, addr):
        # The TMC5160 answers a read on the following datagram
        self.transfer(addr)
        return self.transfer(addr)[1]

    def set_current(self, run_a, hold_a):
        # I_rms = GLOBALSCALER/256 * (CS+1)/32 * VFS/RSENSE / sqrt(2)
        scaler = int(run_a * 256 * math.sqrt(2) * RSENSE / VFS + 0.5)
        scaler = max(32, min(256, scaler))  # 1..31 not allowed, 256 is written as 0

        def cs(amps):
            value = int(amps * 256 * 32 * math.sqrt(2) * RSENSE / (scaler * VFS) - 0.5)
            return max(0, min(31, value))

        self.write_reg(GLOBALSCALER, 0 if scaler == 256 else scaler)
        self.write_reg(IHOLD_IRUN, (6 << 16) | (cs(run_a) << 8) | cs(hold_a))  # IHOLDDELAY=6

    def setup(self):
        ioin = self.read_reg(IOIN)
        version = ioin >> 24
        if version != TMC5160_VERSION:
            raise RuntimeError(
                "Driver %d: no TMC5160 found (IOIN=0x%08x). Check VM power, SPI wiring and CSN."
                % (self.number, ioin)
            )
        print("Driver %d: TMC5160 found, SD_MODE = %d" % (self.number, (ioin >> 6) & 1))

        self.write_reg(GCONF, self.read_reg(GCONF) | (1 << 5))  # diag0_error: DIAG0 low on driver error

        # SpreadCycle: TOFF=3, HSTRT=4, HEND=1, TBL=2 (datasheet defaults), intpol to 256
        chopconf = 3 | (4 << 4) | (1 << 7) | (2 << 15) | (MRES << 24) | (1 << 28)
        self.write_reg(CHOPCONF, chopconf)
        if self.read_reg(CHOPCONF) != chopconf:
            raise RuntimeError("Driver %d: CHOPCONF readback mismatch, check MOSI wiring" % self.number)

        self.set_current(RUN_CURRENT, HOLD_CURRENT)
        self.write_reg(TPOWERDOWN, 10)

        self.write_reg(GSTAT, 0b111)  # clear reset / drv_err / uv_cp flags

    def enable(self):
        self.en.value(0)
        time.sleep_ms(100)
        if (self.read_reg(IOIN) >> 4) & 1:
            raise RuntimeError(
                "Driver %d still sees EN high, check the EN -> GPIO%d connection"
                % (self.number, self.en_gpio)
            )

    def disable(self):
        self.en.value(1)

    def check_error(self):
        if self.diag0.value() == 0:
            raise RuntimeError("Driver %d: DIAG0 went low (driver error)" % self.number)

    def print_status(self):
        gstat = self.read_reg(GSTAT)
        drv = self.read_reg(DRV_STATUS)
        print("Driver %d:" % self.number)
        if gstat == 0xFFFFFFFF or drv == 0xFFFFFFFF:
            print("  No SPI response (all ones), status unknown. Check VM power and the driver module.")
            return
        print("  GSTAT  reset=%d drv_err=%d uv_cp=%d" % (gstat & 1, (gstat >> 1) & 1, (gstat >> 2) & 1))
        print(
            "  DRV_STATUS 0x%08x  ot=%d otpw=%d s2ga=%d s2gb=%d s2vsa=%d s2vsb=%d ola=%d olb=%d cs_actual=%d"
            % (
                drv,
                (drv >> 25) & 1,
                (drv >> 26) & 1,
                (drv >> 27) & 1,
                (drv >> 28) & 1,
                (drv >> 12) & 1,
                (drv >> 13) & 1,
                (drv >> 29) & 1,
                (drv >> 30) & 1,
                (drv >> 16) & 0x1F,
            )
        )
        print("  DIAG0 =", self.diag0.value(), " DIAG1 =", self.diag1.value())


class StepGenerator:
    """STEP pulses from hardware PWM at 50% duty, one pulse per microstep."""

    def __init__(self, pin):
        self.pin = pin
        self.pwm = None
        self.rpm = 0

    def set_rpm(self, rpm):
        if self.pwm is None:
            self.pwm = PWM(self.pin)
        self.pwm.freq(int(rpm * FULL_STEPS_PER_REV * MICROSTEPS / 60))
        self.pwm.duty_u16(32768)
        self.rpm = rpm

    def stop(self):
        if self.pwm is not None:
            self.pwm.deinit()
            self.pwm = None
        self.pin.init(Pin.OUT, value=0)
        self.rpm = 0


def ramp(driver, gen, to_rpm):
    """Change speed linearly from the current speed at ACCEL_RPM_PER_S."""
    from_rpm = gen.rpm
    duration_ms = abs(to_rpm - from_rpm) * 1000 / ACCEL_RPM_PER_S
    start = time.ticks_ms()
    while True:
        elapsed = time.ticks_diff(time.ticks_ms(), start)
        if elapsed >= duration_ms:
            break
        gen.set_rpm(from_rpm + (to_rpm - from_rpm) * elapsed / duration_ms)
        driver.check_error()
        time.sleep_ms(RAMP_UPDATE_MS)
    gen.set_rpm(to_rpm)


def hold(driver, gen, seconds):
    """Run at the current speed, reading SG_RESULT. Returns (samples, stalled)."""
    samples = []
    zeros = 0
    end = time.ticks_add(time.ticks_ms(), int(seconds * 1000))
    while time.ticks_diff(end, time.ticks_ms()) > 0:
        driver.check_error()
        sg = driver.read_reg(DRV_STATUS) & 0x3FF
        samples.append(sg)
        zeros = zeros + 1 if sg == 0 else 0
        if STOP_ON_STALL and gen.rpm >= STALL_CHECK_MIN_RPM and zeros >= STALL_SAMPLES:
            return samples, True
        time.sleep_ms(SAMPLE_MS)
    return samples, False


print("---")
print("SCRIPT START")
print("---")

# Create both so every CSN is high and every EN is off before any SPI traffic
all_drivers = {n: Driver(n, **pins) for n, pins in DRIVER_PINS.items()}
driver = all_drivers[ACTIVE_DRIVER]
gen = StepGenerator(driver.step)
top_rpm = 0

try:
    driver.setup()
    driver.enable()
    driver.direction.value(1 if FORWARD else 0)
    time.sleep_us(10)

    print("Ramping %s at %d RPM/s, %d s at each speed, %d A" % (
        "forward" if FORWARD else "backward", ACCEL_RPM_PER_S, HOLD_S, RUN_CURRENT))
    print("  target RPM | measured RPM | step freq | SG_RESULT min / avg / max")
    gen.set_rpm(START_RPM)
    for target in SPEED_STEPS_RPM:
        ramp(driver, gen, target)
        samples, stalled = hold(driver, gen, HOLD_S)

        # TSTEP is the time between 1/256 microsteps in fCLK cycles, as measured by the driver
        tstep = driver.read_reg(TSTEP) & 0xFFFFF
        measured = FCLK * 60 / (tstep * FULL_STEPS_PER_REV * 256) if tstep else 0
        print("  %10d | %12d | %6.1f kHz | %d / %d / %d" % (
            target, measured, target * FULL_STEPS_PER_REV * MICROSTEPS / 60_000,
            min(samples), sum(samples) // len(samples), max(samples)))

        if stalled:
            print("  Stalled at %d RPM (SG_RESULT stuck at 0)" % target)
            gen.stop()
            break
        top_rpm = target

    if gen.rpm:
        print("Ramping down")
        ramp(driver, gen, START_RPM)
except KeyboardInterrupt:
    if gen.rpm:
        print("Ctrl-C, ramping down")
        ramp(driver, gen, START_RPM)
finally:
    gen.stop()
    if top_rpm:
        print("Top speed without stalling: %d RPM (%.1f rev/s)" % (top_rpm, top_rpm / 60))
    driver.print_status()  # before disabling, turning EN off clears the fault flags
    driver.disable()
    print("---")
    print("SCRIPT FINISHED")
    print("---")
