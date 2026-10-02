# Juggling Machine TMC5160 Stepper Bring-up — Context Handoff
*Compacted 2026-09-29 · ~20 turns, covering the rewrite of the stepper test script, the EN 2 PCB bug, coil-wiring fault debugging, and the two-motor test*

## Objective
Get the two NEMA 23 stepper motors on the juggling machine turning under MicroPython on a Raspberry Pi Pico 2 (RP2350). Each motor is driven by a BIGTREETECH TMC5160T Pro V1.0 driver on the custom PCB (`juggling_machine_pcb`). `code/test_stepper.py` is the bench test that spins the motors slowly. The machine's main code still has to be written: `code/main.py` is currently broken.

## Current State
- `code/test_stepper.py` **works on real hardware**. It configures both drivers over SPI and jogs both motors together: 1 revolution forward, a 1 s pause, then 1 revolution back, at `RPM = 10`.
  - It was confirmed working on driver 1 earlier.
  - The two-motor version was tested **only in a CPython simulation** (mocked `machine` module). It hasn't been run on the Pico yet.
- **EN 2 PCB bug** is worked around with a jumper wire from U2 EN to Pico pin 11 (GP8). The script's EN check has since passed for driver 2.
- **Coil B fault (`s2vsb=1`, `olb=1`, `drv_err=1`)** on driver 1 was fixed by the user rearranging motor wires in the screw terminal. The correct wire/colour order was **not recorded**.
- Driver 2's coil B fault and its later "no response" state are **not confirmed resolved** (see Open Questions).
- `code/main.py` is untouched and broken.

## Key Facts & Values

### Hardware
| Item | Value | Notes |
|---|---|---|
| MCU | Raspberry Pi Pico 2 (RP2350) | MicroPython (the old code was CircuitPython) |
| Driver | BIGTREETECH TMC5160T Pro V1.0 (chip TMC5160-TA) | KiCad footprint is named `BTT_TMC5160T_Pro_V1.1` |
| Sense resistor | 0.075 Ω | 3.1 A RMS / 4.4 A peak max (BTT manual) |
| SD_MODE / SPI_MODE | both tied to VCC_IO via 0 Ω (R7, R6) on module | → **STEP/DIR mode only**; SPI only configures. Internal ramp generator unusable |
| Module CLK | tied to GND via 10K (R4) | internal 12 MHz clock |
| Motor | StepperOnline 23HS22-2804S | 2.8 A rated, 1.26 Nm, ~1 Ω/phase ("~1 Ω" used in checks) |
| VM | +24 V | TMC5160 logic is powered from VM. **VM must be on** or SPI reads 0 or all-ones |
| VCC_IO | +3V3 from Pico | |
| DIAG pull-ups | 10k to 3V3 (R1–R4) | DIAG0 is open drain, active low |

### Pico pin map (from PCB netlist)
| Signal | Driver 1 (U1) | Driver 2 (U2) |
|---|---|---|
| EN (active low) | GPIO20 | GPIO8 (**jumper wire**; not routed on PCB) |
| CSN | GPIO17 | GPIO13 |
| STEP | GPIO10 | GPIO15 |
| DIR | GPIO11 | GPIO14 |
| DIAG0 | GPIO6 | GPIO4 |
| DIAG1 | GPIO7 | GPIO5 |
| Shared SPI0 | SCK GPIO18, MOSI GPIO19, MISO GPIO16 | same |
| Servos | Servo 1 GPIO22, Servo 2 GPIO21 | |

- Pico physical pin 11 = GP8. Pins 8 and 13 are GND. Pins 9/10 = GP6/GP7, which carry driver 1's DIAG with 10k pull-ups.
- Motor terminals: J10 (driver 1) and J11 (driver 2). **Pins 1–2 = coil A, pins 3–4 = coil B.**
  - J10: pin1→U1 A2, pin2→U1 A1, pin3→U1 B2, pin4→U1 B1.
  - J11: pin1→U2 A1, pin2→U2 A2, pin3→U2 B1, pin4→U2 B2.

### PCB findings
- **EN 2 bug:** schematic global label "EN 2" near the Pico is at x=46.99, while neighbouring labels (CSN 2, DIR 2, STEP 2) are at x=45.72. The label is dangling, so the net EN 2 has only 1 pad (U2 EN) and 0 traces. The PCB lists `unconnected-(A1-GPIO8-Pad11)`.
- All other nets are routed. `kicad-cli pcb drc` on the current `juggling_machine_pcb.kicad_pcb` gives 0 violations and 0 unconnected. `kicad-cli` is at `/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli`.
- Pad coordinates (mm): U2 EN (109.85, 87.81); Pico pad 11 (77.11, 92.66), about 33 mm apart. U2 B2 (122.55, 100.51) sits next to U2 VCC_IO (122.55, 103.05). The nearest GND pads to U2 EN (C1 pad 2, U2 GND1) are each next to a +24 V pad.

### Driver configuration used in the script
| Register | Value | Meaning |
|---|---|---|
| GCONF 0x00 | read-modify-write `\| (1<<5)` | diag0_error; result 0x28 with the default multistep_filt |
| CHOPCONF 0x6C | `0x140100C3` | TOFF=3, HSTRT=4, HEND=1, TBL=2, SpreadCycle, MRES=4 (16 µsteps), intpol=1 |
| GLOBALSCALER 0x0B | 67 (0x43) | for 0.8 A RMS |
| IHOLD_IRUN 0x10 | `0x00061F0F` | IRUN=31, IHOLD=15, IHOLDDELAY=6 → 0.8 A run / 0.4 A hold |
| TPOWERDOWN 0x11 | 10 | |
| GSTAT 0x01 | write `0b111` to clear | |
| IOIN 0x04 | VERSION bits 31..24 = 0x30; bit4 DRV_ENN; bit6 SD_MODE | used for the chip-present and EN-reaching-driver checks |

- SPI: mode 3 (polarity=1, phase=1), 1 MHz, 40-bit datagrams, write = addr|0x80. A read returns its data on the **next** datagram.
- Current formula: `I_rms = GLOBALSCALER/256 * (CS+1)/32 * VFS/RSENSE / sqrt(2)`, with VFS = 0.325 V. GLOBALSCALER 1–31 is not allowed; 256 is written as 0.

### DRV_STATUS (0x6F) bits
| Bit | Name | Meaning |
|---|---|---|
| 0–9 | SG_RESULT | stallGuard result |
| 12 | s2vsa | short to supply, coil A |
| 13 | s2vsb | short to supply, coil B |
| 16–20 | CS_ACTUAL | actual current scale |
| 25 | ot | overtemperature |
| 26 | otpw | overtemperature pre-warning |
| 27 | s2ga | short to GND, coil A |
| 28 | s2gb | short to GND, coil B |
| 29 | ola | open load, coil A |
| 30 | olb | open load, coil B |
| 31 | stst | standstill |

GSTAT bits: 0 = reset, 1 = drv_err, 2 = uv_cp.

### Observed fault signatures
| Output | Meaning |
|---|---|
| `drv_err=1`, `s2vsb=1`, `olb=1`, `DRV_STATUS 0x411f2000`, trips after 0–9 steps | Coil pairs mixed across the terminal (fixed on driver 1 by rewiring) |
| `DRV_STATUS 0x800f23ff`, `s2vsb=1` | Same fault on driver 2 |
| `IOIN=0xffffffff`, all status bits 1 | Nothing answering on SPI: VM off, a dead module, or a module pulling down the shared MISO line |
| `IOIN=0x00000000` | No VM, or chip not seated |
| `Driver still sees EN high` | EN wire not reaching the driver (hit on driver 2 before the jumper was fixed) |

## Decisions Made
- **Raw SPI register code, no library.** `tmc_driver` / pytmcstepper (in `code/venv`) is CircuitPython/Linux-only and targets TMC2209 over UART.
- **STEP/DIR motion from Pico GPIO.** SD_MODE is hard-wired to 1 on the module, so the internal ramp generator can't drive the motor. The user's original pin list omitted STEP; GPIO15/GPIO10 came from the PCB netlist.
- **SpreadCycle, not StealthChop.** It needs no auto-tuning, so it's more robust for first spin.
- **0.8 A run / 0.4 A hold**, deliberately low for the bench (motor is rated 2.8 A).
- **16 microsteps with interpolation to 256**, for smooth slow motion.
- **Status is read before disabling EN.** Disabling clears the fault flags, which hid them on the first driver 2 fault.
- **Wire-order HINT prints only when `drv_err=1` and (s2vsa|s2vsb|ola|olb)**, because ola/olb can false-trigger at standstill after a normal run.
- **EN 2 workaround = jumper U2 EN → Pico pin 11 (GP8)**, so the code keeps using GPIO8. The user chose and did this.
- **Both drivers share one step loop.** Both STEP pins are pulsed in the same pass, and a DIAG0-low on either driver stops both. Driver objects are created before any SPI traffic so all CSNs are high.

## Ruled Out
- **Using the TMC5160 internal ramp generator (RAMPMODE/XTARGET)** — SD_MODE=1 is hard-wired on the BTT module.
- **`tmc_driver` Tmc2209 / CircuitPython `board` code** — wrong chip, wrong bus, wrong firmware.
- **Leaving EN 2 floating** — DRV_ENN pull behaviour is unconfirmed, and a floating input is unreliable near motor wiring.
- **PCB design short on coil B** — DRC is clean. The fault was the motor wire order at the terminal.
- **Speed as the cause of the coil B fault** — it tripped at 0–9 steps at 10–12 RPM too.

## Artifacts
- `code/test_stepper.py` — **current, working** two-driver test. It uses the `Driver` class and the `DRIVER_PINS` dict.
- Simulation mocks, in the session scratchpad (temporary, may be gone): `mock2/machine.py` (two-chip TMC5160 SPI mock), `run2.py` (pass any argument to fake a driver 2 coil fault). A clean run gave 6400 step pulses per driver.
- Reference docs used:
  - BTT TMC5160T Pro V1.0 User Manual: `github.com/bigtreetech/BIGTREETECH-Stepper-Motor-Driver/.../TMC5160(T) Pro V1.1/TMC5160T Pro V1.0 User Manual.pdf`
  - Module schematic: `.../TMC5160(T) Pro V1.1/Hardware/BIGTREETECH TMC5160T_Pro-SCH.pdf`
  - BTT docs: `github.com/bigtreetech/docs/blob/master/docs/TMC5160T Pro V1.0.md`

## Code & Config
Current per-driver pin table and core SPI read (in `code/test_stepper.py`):
```python
DRIVER_PINS = {
    1: dict(en=20, csn=17, step=10, dir=11, diag0=6, diag1=7),
    2: dict(en=8, csn=13, step=15, dir=14, diag0=4, diag1=5),
}

def read_reg(self, addr):
    # The TMC5160 answers a read on the following datagram
    self.transfer(addr)
    return self.transfer(addr)[1]
```
Run it with: `cd /Users/jakehopkins/Projects/Juggling-Machine/code && mpremote run test_stepper.py`. `mpremote` is at `~/.local/bin/mpremote`; the MicroPico VS Code extension is also installed.

## Open Questions
- **Is driver 2 healthy now?** It showed the same `s2vsb` coil B fault. A later run returned `IOIN=0xffffffff` (no response). Possible causes: VM supply tripped, the module damaged by repeated shorts, or J11 wiring still mixed. The user never reported fixing J11 or re-testing driver 2 alone. Settle it by running the new two-motor script, or driver 2 alone, after checking J11 pairs with a meter.
- **Were the driver modules swapped between U1 and U2?** Asked, never answered.
- **Correct motor wire colours are unknown.** The assistant's table (black/green = coil A, red/blue = coil B, StepperOnline typical) may be wrong for this motor, since the fix involved changing the wire order. Record the actual working order.
- **Two-motor script not yet run on hardware.**
- **Does Ctrl-C under `mpremote run` stop the script on the device?** Unverified; `mpremote soft-reset` was given as the fallback.
- **Does TMC5160 DRV_ENN have an internal pull resistor?** Not confirmed; the datasheet download failed.
- **Direction:** both motors turn the same way. Mirrored arms may need one reversed (swap one coil's pair, or add per-driver inversion).
- **Which PCB revision was fabricated?** The working tree has modified gerbers and a new `manufacturing.zip`. The DRC result applies to the current file only.

## Next Steps
1. Power off. On J11 (driver 2), verify about 1 Ω across pins 1–2 and across 3–4, open between pairs and open to the motor case. Fix pairing the same way driver 1 was fixed. Record the working colour order.
2. Power the 24 V first, then USB, and run `mpremote run test_stepper.py`. Expect "Driver 1/2: TMC5160 found" and both motors turning 1 rev each way.
3. If driver 2 returns `0xffffffff`: check VM at U2, pull the U2 module (a dead module can pull down the shared MISO line), and test driver 1 alone by temporarily removing key `2` from `DRIVER_PINS`.
4. Fix the schematic for the next PCB rev: move the "EN 2" label onto the GPIO8 wire at x=45.72, update the PCB from the schematic, route the new trace, and run ERC (it flags dangling labels) before ordering.
5. Build the real motion code for `code/main.py`. It currently imports a nonexistent `tmc.TMC_5160`, uses `threading`, and has no `tmc` object. It needs PIO/timer step generation with acceleration ramps (the Python loop can't go much past about 60–100 RPM, and there's no ramp) plus the servo PWM from `code/test_servo.py` (50 Hz, 500–2500 µs).

## Working Context
- The user runs code with `mpremote run test_stepper.py` from `code/` (sometimes inside `(venv)`), and pastes terminal output for diagnosis.
- Caveman mode is active for chat replies: terse, articles dropped. Code, comments and commits are written normally.
- Code style follows `code/test_servo.py`: `from machine import Pin, ...  # type: ignore`, sparse comments, `%`-formatting.
- The user edits the script directly (RPM, REVOLUTIONS, pin blocks). Read the file before editing.
- Before telling the user what to rewire, always flag the risks: never hot-plug motors or drivers with 24 V on, and watch for +24 V pads next to GND pads.
