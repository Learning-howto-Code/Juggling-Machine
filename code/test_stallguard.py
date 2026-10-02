"""
Sensorless homing test with StallGuard2: BIGTREETECH TMC5160T Pro V1.0 on a
Raspberry Pi Pico 2 running MicroPython. Tests one driver (ACTIVE_DRIVER).

The motor turns toward the end stop until StallGuard reports a stall, then backs
off. This repeats HOMING_RUNS times - after the first run the stall should come
back at the same place, so the step count shows how repeatable homing is.

StallGuard2 only works in SpreadCycle and above a minimum speed. A stall is
reported on DIAG1 (diag1_stall) and in DRV_STATUS, the script stops on either.

VM (24 V) must be on before running this, the TMC5160 logic is powered from it.
"""

from machine import Pin, SPI  # type: ignore
import math
import time

# Shared SPI bus (both drivers)
PIN_SCK = 18
PIN_MOSI = 19
PIN_MISO = 16

# Per-driver pins. DIAG0/DIAG1 are open drain with 10k pull-ups on the PCB, active low.
# EN 2 is not routed on the PCB - it needs a jumper wire to GPIO8.
DRIVER_PINS = {
    1: dict(en=20, csn=17, step=10, dir=11, diag0=6, diag1=7),
    2: dict(en=8, csn=13, step=15, dir=14, diag0=4, diag1=5),
}
ACTIVE_DRIVER = 2     # which driver to test (1 or 2), the other stays disabled

RSENSE = 0.075        # ohms, fitted on the TMC5160T Pro V1.0 (3.1 A RMS max)
RUN_CURRENT = 0.8     # A RMS, motor is rated 2.8 A - kept low for bench testing
HOLD_CURRENT = 0.4    # A RMS
MRES = 4              # 0=256 ... 4=16 ... 8=full step microsteps per STEP pulse
MICROSTEPS = 256 >> MRES
FULL_STEPS_PER_REV = 200

# Homing
HOMING_RPM = 120       # StallGuard gets noisy at low speed, try 90-120 if SG_RESULT jumps around
HOME_FORWARD = True  # direction toward the end stop
MAX_HOMING_REVS = 2   # give up if nothing stalls within this
BACKOFF_REVS = 0.5    # distance away from the stop after each stall (direction is automatic), more than the blanking
HOMING_RUNS = 5
STALL_BLANK_FULL_STEPS = 20  # ignore stalls while the motor gets going, it grinds this long if it starts at the stop

# StallGuard threshold, -64..63. Higher = less sensitive.
# Stops before reaching the stop -> raise SGT. Grinds against the stop without stopping -> lower SGT.
# Good tuning: SG_RESULT well above 0 while moving freely, 0 when stalled.
SGT = 3
SFILT = 1             # 1 = filter SG_RESULT over 4 full steps, evens out the high/low alternation between full steps

# TMC5160 registers
GCONF = 0x00
GSTAT = 0x01
IOIN = 0x04
GLOBALSCALER = 0x0B
IHOLD_IRUN = 0x10
TPOWERDOWN = 0x11
TCOOLTHRS = 0x14
CHOPCONF = 0x6C
COOLCONF = 0x6D
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

    def setup_stallguard(self, rpm):
        # TSTEP is the time between 1/256 microsteps in fCLK cycles. Stall detection is on
        # while TSTEP <= TCOOLTHRS, so this turns it on above 2/3 of the homing speed.
        tstep = FCLK * 60 / (rpm * FULL_STEPS_PER_REV * 256)
        tcoolthrs = int(tstep * 1.5)
        self.write_reg(TCOOLTHRS, tcoolthrs)
        self.write_reg(COOLCONF, (SFILT << 24) | ((SGT & 0x7F) << 16))  # SEMIN=0 keeps coolStep off

        gconf = self.read_reg(GCONF) | (1 << 8)  # diag1_stall: DIAG1 low on stall
        self.write_reg(GCONF, gconf)
        if self.read_reg(GCONF) != gconf:
            raise RuntimeError("Driver %d: GCONF readback mismatch" % self.number)
        if gconf & (1 << 2):
            raise RuntimeError("Driver %d: StealthChop is on, StallGuard2 needs SpreadCycle" % self.number)
        print(
            "Driver %d: StallGuard SGT=%d SFILT=%d, TCOOLTHRS=%d (TSTEP at %d RPM ~%d), GCONF=0x%02x"
            % (self.number, SGT, SFILT, tcoolthrs, rpm, tstep, gconf)
        )

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
        print("  stallguard=%d sg_result=%d" % ((drv >> 24) & 1, drv & 0x3FF))
        print("  DIAG0 =", self.diag0.value(), " DIAG1 =", self.diag1.value())

        # Short to supply (s2vsa/s2vsb) or open load (ola/olb) at shutdown usually means
        # the motor's coil pairs are mixed up across the screw terminal
        if (gstat >> 1) & 1 and drv & ((1 << 12) | (1 << 13) | (1 << 29) | (1 << 30)):
            print("  HINT: check the motor wire order in the screw terminal. Each coil must be on")
            print("        its own pair (pins 1-2 = coil A, pins 3-4 = coil B). Two wires of the same")
            print("        coil read ~1 ohm between them. Power off 24 V before rewiring.")


def move(driver, steps, rpm, forward, stop_on_stall=False):
    """Step one driver at constant speed, sampling SG_RESULT once per full step.

    With stop_on_stall, stops as soon as DIAG1 or DRV_STATUS reports a stall.
    Returns (steps taken, what reported the stall or None, SG_RESULT samples).
    """
    period_us = int(60_000_000 / (rpm * FULL_STEPS_PER_REV * MICROSTEPS))
    blank = STALL_BLANK_FULL_STEPS * MICROSTEPS
    samples = []
    driver.direction.value(1 if forward else 0)
    time.sleep_us(10)
    next_us = time.ticks_us()
    for i in range(steps):
        if driver.diag0.value() == 0:
            raise RuntimeError("Driver %d: DIAG0 went low after %d steps (driver error)" % (driver.number, i))
        if i >= blank and i % MICROSTEPS == 0:
            # One datagram per full step: each read returns the previous full step's DRV_STATUS,
            # the first one is left over from before the move so it is skipped
            drv = driver.transfer(DRV_STATUS)[1]
            if i > blank:
                samples.append(drv & 0x3FF)
                if stop_on_stall and (drv >> 24) & 1:
                    return i, "DRV_STATUS", samples
        if stop_on_stall and i >= blank and driver.diag1.value() == 0:
            return i, "DIAG1", samples
        # Timed from a running deadline so the SPI reads don't slow the motor down
        while time.ticks_diff(next_us, time.ticks_us()) > 0:
            pass
        driver.step.value(1)
        driver.step.value(0)
        next_us = time.ticks_add(next_us, period_us)
    return steps, None, samples


def print_samples(samples):
    if not samples:
        print("  SG_RESULT: no samples")
        return
    print(
        "  SG_RESULT min %d  avg %d  max %d  (%d full steps), last: %s"
        % (min(samples), sum(samples) // len(samples), max(samples), len(samples), samples[-8:])
    )


print("---")
print("SCRIPT START")
print("---")

# Create both so every CSN is high and every EN is off before any SPI traffic
all_drivers = {n: Driver(n, **pins) for n, pins in DRIVER_PINS.items()}
driver = all_drivers[ACTIVE_DRIVER]

max_steps = int(MAX_HOMING_REVS * FULL_STEPS_PER_REV * MICROSTEPS)
backoff_steps = int(BACKOFF_REVS * FULL_STEPS_PER_REV * MICROSTEPS)
if BACKOFF_REVS * FULL_STEPS_PER_REV <= STALL_BLANK_FULL_STEPS:
    raise ValueError("BACKOFF_REVS must be positive and longer than STALL_BLANK_FULL_STEPS")

try:
    driver.setup()
    driver.setup_stallguard(HOMING_RPM)
    driver.enable()

    for run in range(1, HOMING_RUNS + 1):
        print("Homing run %d: %s at %d RPM" % (run, "forward" if HOME_FORWARD else "backward", HOMING_RPM))
        steps, source, samples = move(driver, max_steps, HOMING_RPM, HOME_FORWARD, stop_on_stall=True)
        print_samples(samples)
        if source is None:
            print("  No stall within %d rev. If it hit the stop, lower SGT or raise HOMING_RPM." % MAX_HOMING_REVS)
            break
        print("  Stall on %s after %d full steps" % (source, steps // MICROSTEPS))
        if run > 1:
            # Started BACKOFF_REVS away from the last stall, so it should stall there again
            print("  Off by %+.1f full steps from the last stall" % ((steps - backoff_steps) / MICROSTEPS))

        time.sleep_ms(200)
        print("  Backing off %.2f rev" % BACKOFF_REVS)
        _, _, samples = move(driver, backoff_steps, HOMING_RPM, not HOME_FORWARD)
        print_samples(samples)
        time.sleep_ms(500)
finally:
    driver.print_status()  # before disabling, turning EN off clears the fault flags
    driver.disable()
    print("---")
    print("SCRIPT FINISHED")
    print("---")
