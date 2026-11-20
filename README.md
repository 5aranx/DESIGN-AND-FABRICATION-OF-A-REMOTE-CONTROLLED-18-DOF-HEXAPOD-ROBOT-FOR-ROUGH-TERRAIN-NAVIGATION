# DESIGN AND FABRICATION OF A REMOTE-CONTROLLED 18-DOF HEXAPOD ROBOT FOR ROUGH TERRAIN NAVIGATION

Firmware for a 6-legged, 18-servo walking robot built around a **Teensy 4.1** and a custom
PCB. It is driven by a **USB gamepad** (via the Teensy USB host port) and shows live
battery voltage, current, power and consumed capacity on a **128x64 SSD1306 OLED**.

The gait engine is an inverse-kinematics walker with two gaits (TRIPOD / WAVE), adjustable
step length, step height, cadence and body height, plus a battery monitor with low-voltage
alarm.

---

## Table of contents

- [Features](#features)
- [Hardware](#hardware)
- [Wiring / pin map](#wiring--pin-map)
- [Controls](#controls)
- [Serial commands](#serial-commands)
- [Battery monitor](#battery-monitor)
- [First-time setup & servo calibration](#first-time-setup--servo-calibration)
- [Building and uploading](#building-and-uploading)
- [How the gait engine works](#how-the-gait-engine-works)
- [Tuning](#tuning)
- [Troubleshooting](#troubleshooting)
- [Safety](#safety)
- [Credits](#credits)

---

## Features

- **18-DOF hexapod** – 6 legs x (coxa, femur, tibia), 3-DOF inverse kinematics per leg.
- **USB gamepad control** – analog walk / strafe / yaw, body height on the right stick.
- **Two gaits** – TRIPOD (fast, 3 legs always on the ground) and WAVE (slow, very stable,
  one leg in the air at a time).
- **Live tuning from the pad** – step length, step height and gait speed without reflashing.
- **50 Hz servo frame** with per-axis smoothing, so the motion is not jerky.
- **Safety defaults** – boots **DISARMED**; if the gamepad is unplugged the robot
  automatically disarms.
- **OLED battery monitor** – voltage, current, watts, percent, mAh consumed, low-battery
  blink alarm; current sensor is auto-zeroed at boot.
- **Serial console** – status print, raw gamepad axis/button dump, I2C scan.

---

## Hardware

| Part | Notes |
|---|---|
| Teensy 4.1 | Main controller, runs the USB host stack |
| Custom PCB | Servo headers, power, battery sensing |
| 18x hobby servos | 3 per leg (coxa / femur / tibia) |
| USB gamepad | Any HID gamepad recognised by `USBHost_t36` |
| SSD1306 OLED | 128x64, I2C, address `0x3C` or `0x3D` |
| Battery pack | 3S Li-ion/LiPo nominal 11.1 V (3.3 Ah used for the fuel gauge) |
| Voltage divider | Ratio **11.352:1** into pin 38 |
| Current sensor | **18.18 A/V** output into pin 39 |

Leg geometry and the IK are based on Emre Kalem & Özgür Toprak Önsoy's hexapod (MIT).

| Link | Length |
|---|---|
| Coxa | 38.0 mm |
| Femur | 86.25 mm |
| Tibia | 160.5 mm |

Standing body height is `Z = -40 mm` (negative Z is down), sit pose `Z = -5 mm`, and the
body-height range is limited to `-110 mm .. 0 mm`.

---

## Wiring / pin map

### Servos — `{coxa, femur, tibia}`

| Leg | Position | Coxa | Femur | Tibia |
|---|---|---|---|---|
| 1 | Right Front (RF) | 10 | 9 | 8 |
| 2 | Right Middle (RM) | 7 | 6 | 5 |
| 3 | Right Rear (RR) | 4 | 3 | 2 |
| 4 | Left Rear (LR) | 36 | 33 | 24 |
| 5 | Left Middle (LM) | 28 | 25 | 37 |
| 6 | Left Front (LF) | 15 | 14 | 29 |

Pulse range is **500–2500 us** (`SERVO_US_MIN` / `SERVO_US_MAX`) on all channels.

### Sensors and display

| Signal | Pin | Notes |
|---|---|---|
| Battery voltage sense | 38 (A14) | divider ratio 11.352 |
| Battery current sense | 39 (A15) | 18.18 A/V, zeroed at boot |
| OLED SDA | 18 | `Wire`, 400 kHz |
| OLED SCL | 19 | `Wire`, 400 kHz |

> The OLED is optional at runtime: if it is not found the firmware keeps running and
> reports `oled=NOT FOUND` over Serial.

---

## Controls

| Input | Action |
|---|---|
| Left stick up/down | Walk forward / backward |
| Left stick left/right | Strafe left / right |
| Right stick X | Turn (yaw) |
| Right stick Y | Body height (up = higher) |
| D-pad Up / Down | Step height +5 / -5 mm (20–100 mm) |
| D-pad Left / Right | Gait speed -0.1 / +0.1 Hz (0.3–1.5 Hz) |
| LB / RB | Step length -5 / +5 mm (20–90 mm) |
| LT / RT | Body height +5 / -5 mm (5 mm steps) |
| A | Stand pose (`Z = -40 mm`) |
| B | Sit / low pose (`Z = -5 mm`) |
| X | Toggle gait: TRIPOD <-> WAVE |
| Y | Print status to Serial |
| START | ARM / DISARM walking (boots DISARMED) |
| BACK | Reset all settings to defaults |

The left/right stick axes pass through a **0.15 deadzone** with rescaling so full stick
travel still maps to full speed. Walking only happens while **ARMED**.

---

## Serial commands

Open the Serial Monitor at **115200 baud**:

| Key | Effect |
|---|---|
| `d` | Toggle raw gamepad dump (buttons + axes 0-9, every 200 ms) |
| `s` | Print the current status line |
| `i` | Scan the I2C bus (use this first if the OLED stays blank) |

Example status line:

```
armed=1 gait=TRIPOD stepLen=50.00 stepH=60.00 speed=0.70Hz bodyZ=-40.00 batt=11.86V 1.42A 62% used=184mAh oled=OK
```

---

## Battery monitor

- The ADC runs at **12-bit** resolution with **16x hardware averaging**.
- Every reading is the **median of 9 samples** after a dummy read, which rejects servo
  current spikes.
- Voltage is filtered with a slow IIR (`0.92 old + 0.08 new`).
- Current is filtered faster (`0.85 / 0.15`) and the zero-current offset is measured over
  20 samples at boot, before the servos are attached.
- Percent comes from a piecewise **3S Li-ion discharge curve** (12.60 V = 100 % down to
  10.20 V = 0 %), with a small IR-sag compensation while under load.
- `mAh consumed` is integrated from the filtered current.
- Below **10.35 V for 3.5 s** the alarm latches and the top of the OLED blinks
  `!! LOW BATTERY !!`.

OLED layout:

```
ARMED  TRIPOD
62%        11.86V
           1.42A
--------------------------------
16.8W     Used:184mAh
```

The OLED redraw is asynchronous to the 50 Hz servo frame, so a slow I2C transfer never
stalls the legs.

---

## First-time setup & servo calibration

1. **Mount the horns at 90 deg.** In the sketch set:

   ```cpp
   #define CAL_MODE 1
   ```

   Upload, power the robot, and the firmware holds every servo at 90 deg and does
   nothing else. Mount each horn so the leg is in its neutral pose, then power down.
2. Set `CAL_MODE 0` and upload again.
3. **Lift the robot onto a stand** so the legs can move freely and test all controls.
4. If a leg is mirrored, flip the corresponding entry in `SERVO_INV[leg][j]`.
5. If a leg is slightly off-centre, trim it with `SERVO_TRIM[leg][j]` (degrees, added
   after any inversion).
6. Only then put the robot on the floor and try TRIPOD, then WAVE.

---

## Building and uploading

Requirements:

- **Arduino IDE 2.x** (or `arduino-cli`) with **Teensyduino** for Teensy 4.1
- Libraries:
  - `USBHost_t36` (bundled with Teensyduino)
  - `Servo` (bundled with Teensyduino)
  - `Wire` (bundled)
  - `Adafruit GFX Library`
  - `Adafruit SSD1306`

Board settings:

- Board: **Teensy 4.1**
- USB Type: **Serial** (or Serial + Keyboard/Mouse; USB host works either way)
- CPU speed: 600 MHz (default is fine)
- Serial Monitor: **115200 baud**

The sketch folder must keep the same name as the `.ino` file (`hexapod-18dof`).

---

## How the gait engine works

1. **Home positions** – each leg's rest foot position is placed radially outward from its
   hip at `reach = coxa + femur*cos45 + tibia*cos(-45)`, at `STAND_Z`.
2. **Phase** – `gaitT` advances at `cycleHz` while a direction input is present. Each leg
   has a phase offset `off[i]`, so the legs are grouped:
   - TRIPOD: `0, 0.5, 0, 0.5, 0, 0.5` (alternating tripods)
   - WAVE: `4/6, 5/6, 0, 3/6, 2/6, 1/6` (one leg at a time)
3. **Swing / stance** – during the first `swing` fraction of a leg's cycle the foot moves
   along the stride and lifts by `sin(u*pi) * stepHeight`; for the rest it sweeps back on
   the ground.
4. **Body motion** – each stride is offset by `vx/vy * stride * stepLength`, and yaw rotates
   the home position by `rot * stride * 15 deg`.
5. **IK** – the target is rotated into the leg's neutral frame, then solved with the
   law of cosines (coxa yaw, femur angle, tibia angle), clamped to reachable distance and
   to `45..135 deg` on the coxa.
6. **Smoothing** – the servo targets chase the IK goal with a `0.35` lerp per 20 ms frame,
   which removes snapping when inputs change.

---

## Tuning

All in the `CONFIG` section at the top of the sketch:

| Constant | Meaning |
|---|---|
| `STAND_Z`, `SIT_Z`, `Z_MIN`, `Z_MAX` | Body heights and limits (mm) |
| `DEF_STEP_LENGTH` | Default stride (mm) |
| `DEF_STEP_HEIGHT` | Default foot lift (mm) |
| `DEF_CYCLE_HZ` | Default gait frequency |
| `ROTATION_STEP_RAD` | Yaw per stride (default 15 deg) |
| `SMOOTHING` | Target lerp per frame (higher = snappier) |
| `DEADZONE` | Stick deadzone (0-1) |
| `SERVO_US_MIN/MAX` | Pulse width calibration |
| `V_DIVIDER_RATIO`, `AMPS_PER_VOLT` | Battery sensing calibration |
| `WARN_VOLTS`, `MIN_VOLTS` | Alarm / empty thresholds |

Axis and button indices in `AXIS_*` / `BTN_*` are gamepad-specific. If your pad does not
respond as expected, send `d` over Serial and press each control to see which index moves.

---

## Troubleshooting

| Symptom | Check |
|---|---|
| OLED blank | Send `i` to scan I2C, check SDA=18 / SCL=19, 3.3 V, GND and pull-ups; the code tries `0x3C` then `0x3D` |
| Gamepad not detected | Must be plugged into the **Teensy USB host** port (not the programming port); `USBHost_t36` supports most HID pads |
| Robot does not walk | Press **START** first (boots DISARMED), then move the left stick |
| Robot disarms by itself | Gamepad was unplugged/lost – this is intentional |
| Legs twitch / buzz | Servo power sag or bad ground; check battery voltage on the OLED |
| Wrong joint direction | Flip `SERVO_INV[leg][j]` |
| Legs collide / over-reach | Reduce `stepLength`, `bodyZ` range or `stepHeight` |
| Battery % jumps | Re-check `V_DIVIDER_RATIO` and `AMPS_PER_VOLT`, and the current-sensor zero offset |
| Servo jitter at boot | Normal until `batteryInit()` finishes (~0.5 s); servos are attached afterwards |

---

## Safety

- **Always test on a stand first.** An 18-servo hexapod can move violently on the first
  power-up if a horn is mounted wrong.
- Keep fingers clear of the legs while armed.
- Use a fuse and a proper BMS on the battery pack.
- Do not exceed the mechanical reach limits; the IK clamps, but the structure may still
  bind.
- The firmware never arms itself: START is required after every boot and after any
  gamepad disconnect.

---

## Credits

- IK / leg geometry based on the hexapod by **Emre Kalem** and **Özgür Toprak Önsoy** (MIT).
- Adafruit GFX and SSD1306 libraries for the display.
- PJRC Teensy 4.1 `USBHost_t36` and `Servo` libraries.

## Contributors

- Saransh Singh — firmware, electronics, mechanical design
- shahidjnp60@gmail.com
- smarthpanwar191@gmail.com
- swayampanwar191@gmail.com
- shrutishravya15@gmail.com

## License

MIT (see the upstream hexapod project for the original IK work).
