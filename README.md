# BalanceBot

A two-wheel self-balancing robot: an Arduino Uno, an MPU6050 IMU, and two bipolar stepper
motors driven by L298N boards, all built into a wooden box with a wheel at each end.

The firmware in [`BalanceBot/BalanceBot.ino`](BalanceBot/BalanceBot.ino) replaces the
original sketch, whose source was lost. The original was recovered by disassembling the
board's flash. It used on/off control: one step forward or back per 5 ms loop, with a ±2°
dead band. That capped it at about 150 steps/s and gave no proportional response, so it
could never catch itself. A backup of that firmware is in [`backup/`](backup/).

## Hardware

| Part | Notes |
|---|---|
| Arduino Uno | Genuine board (USB VID 2341, PID 0043) |
| MPU6050 (GY-521) | I2C at 0x68, mounted flat on the box floor |
| 2 × bipolar steppers | 200 steps/rev, driven half-step (400/rev) |
| 2 × L298N boards | One per motor, ENA/ENB jumpers fitted, 5V jumper fitted |
| 12V supply | Unregulated wall adapter on the bench (reads ~16V unloaded) |

### Wiring

Pin assignments come from the original firmware.

| L298N #1 (motor A) | Uno | L298N #2 (motor B) | Uno |
|---|---|---|---|
| IN1 | D8 | IN1 | D3 |
| IN2 | D9 | IN2 | D4 |
| IN3 | D10 | IN3 | D5 |
| IN4 | D11 | IN4 | D6 |

| MPU6050 | Uno |
|---|---|
| VCC | 5V |
| GND | GND |
| SDA | A4 |
| SCL | A5 |
| INT | D2 (not used by this firmware) |

Each stepper's two coils go to OUT1/OUT2 and OUT3/OUT4. To find the pairs, measure
resistance between the wires: a pair that reads a few ohms is one coil.

### Power

Feed the motor boards straight from the supply. Don't route motor current through the
Uno: its VIN path has a protection diode rated for about 1A.

```
Supply + ──┬── L298N #1 12V
           ├── L298N #2 12V
           └── Uno VIN
Supply − ──┬── L298N #1 GND
           ├── L298N #2 GND
           └── Uno GND        (shared ground is required)
```

USB can stay plugged in alongside VIN. If the supply is a large battery (for example
LiFePO4), put a fuse at the battery terminal.

## Build and upload

Uses [arduino-cli](https://arduino.github.io/arduino-cli/) with the `arduino:avr` core:

```bash
arduino-cli core install arduino:avr
arduino-cli compile --fqbn arduino:avr:uno BalanceBot
arduino-cli upload --fqbn arduino:avr:uno -p COM3 BalanceBot
```

Uploading resets the Uno, so a balancing bot will drop.

To restore the original firmware:

```bash
arduino-cli upload --fqbn arduino:avr:uno -p COM3 --input-file backup/original_firmware.hex
```

## How it works

- **Tilt sensing:** raw accelerometer and gyro readings are blended by a complementary
  filter at 200 Hz. The IMU's low-pass filter is set to 44 Hz, and the gyro rate used for
  damping is smoothed further, so stepper vibration doesn't reach the control loop.
- **Balance loop:** a PD controller on tilt commands wheel *acceleration*, which is
  integrated into wheel speed. Holding a lean requires the wheels to keep accelerating, so
  this matches the physics.
- **Position loop:** wheel speed and distance from the arming spot lean the target angle
  back, so the bot holds its place instead of wandering.
- **Auto-trim:** if the balance point estimate is wrong, or a load shifts the center of
  gravity, the bot ends up sitting away from its spot. The trim keeps shifting until it is
  back home with the wheels stopped. Together with the position loop this acts as a PID on
  wheel position. The learned trim is saved to EEPROM when the bot disarms after more than
  5 s of balancing.
- **Step generation:** a 20 kHz Timer1 interrupt generates half-steps at any speed up to
  800 full steps/s. Each motor's four pins sit on one port (PORTB and PORTD), so a coil
  update is a single register write.
- **Adaptive coil power:** a stepper on an L298N draws the same current whether it is
  holding still or fighting a push, so the coils are chopped with sigma-delta modulation at
  10–20 kHz. Drive is `max(floor, 1.5 × estimated need)`, with instant full power when the
  bot is more than 4° off target. Half-steps that energize only one coil get 1.41× drive so
  torque stays even.

### Safety

- **Arming:** the bot arms only after being held within 3° of the balance point for 0.5 s.
- **Fall cutoff:** it disarms (coils off) beyond 30° of tilt.
- **Runaway cutoff:** it disarms after 1.5 s pinned at maximum speed. At that speed the
  wheels can't get ahead of the body to recover.
- **IMU failure:** it disarms if the IMU stops responding.

## Serial interface

115200 baud, one command per line.

| Command | Effect |
|---|---|
| `p <n>` | kp: acceleration per degree of tilt |
| `d <n>` | kd: acceleration per deg/s of tilt rate |
| `v <n>` | kv: lean per step/s of wheel speed |
| `x <n>` | kx: lean per step from home |
| `i <n>` | ktrim: auto-trim rate (0 = off) |
| `t <n>` | Set the trim angle |
| `z` | Set the trim to the current angle |
| `P <n>` | Idle coil drive % (100 = always full power) |
| `a` | Toggle auto-arm |
| `k` | Disarm now |
| `f` | Flip the drive direction |
| `M` | Motor test (disarmed only; lift the wheels first) |
| `m` | Toggle telemetry |
| `s` | Show settings |
| `w` | Save settings to EEPROM (deferred until disarm while balancing) |
| `r` | Reset to defaults |

Telemetry streams at 25 Hz as tab-separated columns: angle, accelerometer-only angle, gyro
rate, target angle, wheel speed (full steps/s), armed, trim, coil drive %.

## Tuned settings

These are the firmware defaults, found by testing on the floor:

| Setting | Value | Notes |
|---|---|---|
| kp / kd | 250 / 22 | kd sized for the ~17 cm effective pendulum length measured from a free fall |
| kv / kx | 0.01 / 0.01 | At kv 0.004 it reached the speed cap and ran away |
| ktrim | 0.002 | 0.003 swayed about ±1 rev around home; 0.002 settles, but takes ~20 s to fully adapt to a new load |
| trim | 2.4° | Balance point of the bare bot |
| Idle power | 40% | Held in every run, loaded and bare, at about a quarter of full-power coil heat. 30% lost sync. |

## Tuning tips

- **It rolls off steadily in one direction:** the trim is wrong. Auto-trim corrects this;
  `z` sets the trim to the current angle as a quick fix.
- **Fast shaking:** kd (or kp) is too high.
- **Slow growing wobbles:** kd is too low.
- **It falls while the wheels chase it:** kp is too low, or the steppers are stalling.
- **The wheel command ramps up but the body doesn't respond:** the steppers lost sync. Raise
  the idle power with `P`.
- **Cables matter at this scale.** A stiff USB or power cable can prop the bot up and
  corrupt both the tuning and the learned trim. Hang the cables from above with slack.

## Limitations and next steps

- **L298N drivers waste power and run hot.** They have no current control and drop 2–3V.
  TMC2209 or A4988 drivers would give real current limiting and microstepping, and the
  TMC2209 can detect stalls.
- **Heavier loads need more torque,** but the power boost only looks at acceleration and
  speed. Raise the idle power if you carry weight regularly.
- **Planned:** drive and turn commands, then remote control through an ESP32 bridge (phone
  web page or game controller), then sensors on the free pins D7, D12 and A0–A3.
