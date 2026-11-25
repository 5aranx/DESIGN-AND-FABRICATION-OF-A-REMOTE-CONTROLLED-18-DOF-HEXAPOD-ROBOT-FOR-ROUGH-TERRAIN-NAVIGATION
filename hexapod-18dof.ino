/**
 * Hexapod - USB gamepad control + OLED battery monitor + MPU6050 body leveling
 * (Teensy 4.1, custom PCB). Version for 18x MG995 180-degree servos.
 *
 * IK / leg geometry based on Emre Kalem & Ozgur Toprak Onsoy's hexapod (MIT).
 *
 * CONTROLS
 *   Left stick .......... walk forward/back + strafe left/right
 *   Right stick X ....... turn (yaw)
 *   Right stick Y ....... body height (up = higher)
 *   D-pad Up/Down ....... step height  + / -
 *   D-pad Left/Right .... gait speed   - / +
 *   LB / RB ............. step length  - / +
 *   LT / RT ............. body height  - / +  (5 mm steps)
 *   A ................... stand        B ... sit / low pose
 *   X ................... toggle gait (TRIPOD <-> WAVE)
 *   Y ................... print status to Serial
 *   L3 (left stick click) TOGGLE BODY LEVELING (boots OFF)  [BTN_L3, verify with 'd' dump]
 *   START ............... ARM / DISARM walking (boots DISARMED)
 *   BACK ................ reset all settings to defaults
 *
 * Serial monitor (115200):  'd' = toggle raw gamepad dump, 's' = print status,
 *                           'i' = I2C scan, 'l' = toggle leveling,
 *                           'c' = recalibrate IMU (robot flat and still!)
 *
 * I2C bus (Wire: SDA=pin 18, SCL=pin 19): SSD1306 OLED (0x3C) + MPU6050 (0x68).
 * Battery sense: pin 38 (voltage), pin 39 (current).
 *
 * SERVO PINS (from wiring sheet)      coxa femur tibia
 *   Leg1 RF                             2     3     4
 *   Leg2 RM                             5     6     7
 *   Leg3 RR                             8     9    10
 *   Leg4 LR                            29    14    15
 *   Leg5 LM                            37    25    28
 *   Leg6 LF                            24    33    36
 *
 * BODY FRAME: +X forward, +Y left, +Z up (matches LEG_X / LEG_Y below).
 * MPU6050 is mounted with X and Y inverted (180 deg about Z) -> see imuRemapAxes().
 *
 * IMU CALIBRATION happens at power-up (gyro bias + accel offset, about 4 s).
 * Power the robot on while it stands/sits on FLAT ground and do not touch it.
 *
 * FIRST TIME: set CAL_MODE 1, upload, mount all servo horns at center (1500 us), then CAL_MODE 0.
 * Test with the robot lifted on a stand first!
 */

#include <USBHost_t36.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define CAL_MODE 0   // 1 = hold every servo at 90 deg and do nothing else

// ======================= CONFIG =======================

// ---- Leg geometry (mm) ----
const float COXA_LENGTH  = 38.0f;
const float FEMUR_LENGTH = 86.25f;
const float TIBIA_LENGTH = 160.5f;

// Order: Leg1=RF, Leg2=RM, Leg3=RR, Leg4=LR, Leg5=LM, Leg6=LF
const float LEG_X[6] = {  100.94f,    0.0f, -100.94f, -100.94f,    0.0f,  100.94f };
const float LEG_Y[6] = {  -57.11f, -105.72f,  -57.11f,   57.11f, 105.72f,   57.11f };

// ---- Servo pins from wiring sheet: {coxa, femur, tibia} ----
const uint8_t SERVO_PIN[6][3] = {
  {  2,  3,  4 },   // Leg1 (RF)
  {  5,  6,  7 },   // Leg2 (RM)
  {  8,  9, 10 },   // Leg3 (RR)
  { 29, 14, 15 },   // Leg4 (LR)
  { 37, 25, 28 },   // Leg5 (LM)
  { 24, 33, 36 }    // Leg6 (LF)
};

float SERVO_TRIM[6][3] = {
  { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 },
  { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }
};
// If a joint moves the wrong way, set its entry to true (e.g. { false, true, true }).
const bool SERVO_INV[6][3] = {
  { false, false, false }, { false, false, false }, { false, false, false },
  { false, false, false }, { false, false, false }, { false, false, false }
};

// ---- Servo pulse scaling (MG995, 180 deg) ----
// 500..2500 us = 180 deg  ->  ~11.11 us per degree.
// If your servos sweep less than 180 deg, lower SERVO_SPAN_DEG to the measured value.
const float SERVO_FULL_US    = 2000.0f;   // (max us - min us) for the full span
const float SERVO_SPAN_DEG   = 180.0f;    // full mechanical span of the servo
const float SERVO_US_PER_DEG = SERVO_FULL_US / SERVO_SPAN_DEG;

// Full 180 deg range centered on 1500 us
const int SERVO_US_MIN = 1500 - (int)(90.0f * SERVO_US_PER_DEG + 0.5f);   // 500 us  (0 deg)
const int SERVO_US_MAX = 1500 + (int)(90.0f * SERVO_US_PER_DEG + 0.5f);   // 2500 us (180 deg)

// ---- Defaults / limits ----
const float STAND_Z          = -40.0f;
const float SIT_Z            = -5.0f;
const float Z_MIN            = -110.0f;
const float Z_MAX            =    0.0f;
const float DEF_STEP_LENGTH  = 50.0f;
const float DEF_STEP_HEIGHT  = 60.0f;
const float DEF_CYCLE_HZ     = 0.7f;
const float ROTATION_STEP_RAD = 15.0f * M_PI / 180.0f;
const float SMOOTHING        = 0.35f;

// ---- Gamepad mapping ----
const int AXIS_LX = 0, AXIS_LY = 1, AXIS_RX = 2, AXIS_RY = 5, AXIS_HAT = 9;
const bool INV_LX = false, INV_LY = false, INV_RX = false, INV_RY = false;
const float DEADZONE = 0.15f;

const int BTN_X = 0, BTN_A = 1, BTN_B = 2, BTN_Y = 3;
const int BTN_LB = 4, BTN_RB = 5, BTN_LT = 6, BTN_RT = 7;
const int BTN_BACK = 8, BTN_START = 9;
const int BTN_L3 = 10;   // left stick click - check the number with the 'd' dump

// ---- OLED + battery monitor ----
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
const int V_SENSE_PIN = 38;           // A14
const int I_SENSE_PIN = 39;           // A15
const float V_DIVIDER_RATIO = 11.352;
const float AMPS_PER_VOLT   = 18.18;
const float BATTERY_MAH     = 3300.0;
const float MIN_VOLTS       = 10.20;
const float WARN_VOLTS      = 10.35;
const unsigned long WARN_DELAY_MS = 3500;
const unsigned long OLED_PERIOD_MS = 500;   // redraw rate (I2C transfer blocks ~25 ms)

// ---- MPU6050 + body leveling ----
const uint8_t  MPU_ADDR        = 0x68;
const float    GYRO_SENS       = 65.5f;      // LSB per deg/s at +-500 dps
const float    ACC_SENS        = 16384.0f;   // LSB per g at +-2 g
const float    IMU_ALPHA       = 0.98f;      // gyro weight in the complementary filter
const uint32_t IMU_PERIOD_US   = 5000;       // ~200 Hz attitude update
const int      IMU_CAL_SAMPLES = 1500;       // ~4 s at power-up

const float LEVEL_MAX_DEG    = 12.0f;   // max body correction applied to the feet
const float LEVEL_MAX_RATE   = 60.0f;   // deg/s slew limit on the correction
const float TILT_CUTOFF_DEG  = 45.0f;   // leveling switches OFF beyond this (fallen / lifted)
// If the body tilts the WRONG way when you enable leveling, set the matching sign to -1.
const float LEVEL_ROLL_SIGN  = 1.0f;
const float LEVEL_PITCH_SIGN = 1.0f;

// ======================= STATE =======================

USBHost myusb;
USBHub hub1(myusb);
USBHIDParser hid1(myusb);
USBHIDParser hid2(myusb);
JoystickController joystick(myusb);

struct Vec3 { float x, y, z; };

Vec3 HOME[6];
Vec3 target[6];
float neutralAng[6];

struct Gait { const char* name; float swing; float off[6]; };
const Gait GAITS[2] = {
  { "TRIPOD", 0.5f,        { 0.0f, 0.5f, 0.0f, 0.5f, 0.0f, 0.5f } },
  { "WAVE",   1.0f / 6.0f, { 4.0f/6, 5.0f/6, 0.0f, 3.0f/6, 2.0f/6, 1.0f/6 } }
};

int   gaitIdx    = 0;
float stepLength = DEF_STEP_LENGTH;
float stepHeight = DEF_STEP_HEIGHT;
float cycleHz    = DEF_CYCLE_HZ;
float bodyZ      = STAND_Z;
float gaitT      = 0.0f;
bool  armed      = false;
bool  dbg        = false;

uint32_t prevButtons = 0;
int      prevHat     = 15;

// OLED / battery state
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
bool  oledOk = false;
float filteredVolts = 0.0, shownVolts = 0.0, filteredAmps = 0.0;
float currentZeroOffsetV = 0.0, mahConsumed = 0.0;
unsigned long lastBattMs = 0, lastOledMs = 0, lowVoltStartMs = 0;
bool lowBattAlarm = false;
int  battPct = 0;

// ======================= IMU / LEVELING =======================

struct PID {
  float kp, ki, kd, i, last, outMax;
  float run(float err, float dt) {
    i = constrain(i + ki * err * dt, -outMax, outMax);        // anti-windup clamp
    float d = (dt > 0.0f) ? kd * (err - last) / dt : 0.0f;
    last = err;
    return constrain(kp * err + i + d, -outMax, outMax);
  }
  void reset() { i = 0.0f; last = 0.0f; }
};

// Error is the measured tilt itself; the output is the angle the feet are rotated by.
// Start with these, then raise ki for a firmer hold or lower it if the body hunts.
PID rollPID  = { 0.3f, 1.0f, 0.0f, 0, 0, LEVEL_MAX_DEG };
PID pitchPID = { 0.3f, 1.0f, 0.0f, 0, 0, LEVEL_MAX_DEG };

float gxBias = 0, gyBias = 0, gzBias = 0;     // raw counts, sensor frame
float axOff  = 0, ayOff  = 0, azOff  = 0;     // raw counts, sensor frame

float imuRoll = 0, imuPitch = 0;              // deg, body frame (roll + = left side up, pitch + = nose down)
float imuYawRate = 0;                         // deg/s, body frame (not used yet)
bool  imuOk = false;
bool  levelOn = false;
uint8_t  imuFails = 0;
uint32_t lastImuUs = 0;

float levelRoll = 0, levelPitch = 0;          // correction currently applied to the feet (deg)

void imuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static inline int16_t imuRead16() {
  uint8_t hi = Wire.read();     // separate statements: operand order of | is unspecified in C++
  uint8_t lo = Wire.read();
  return (int16_t)((hi << 8) | lo);
}

// Accel + gyro in one 14-byte burst (registers 0x3B..0x48)
bool imuReadRaw(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14) != 14) return false;

  ax = imuRead16();
  ay = imuRead16();
  az = imuRead16();
  imuRead16();                  // skip temperature
  gx = imuRead16();
  gy = imuRead16();
  gz = imuRead16();
  return true;
}

// Sensor frame -> body frame (chip rotated 180 deg about Z): X and Y inverted, Z unchanged
static inline void imuRemapAxes(float &x, float &y, float &z) {
  x = -x;
  y = -y;
  (void)z;
}

// Robot must be flat and still. Gyro bias + accel offset (Z is expected to read +1 g).
bool imuCalibrate(int samples) {
  double sgx = 0, sgy = 0, sgz = 0, sax = 0, say = 0, saz = 0;
  int n = 0;
  int16_t ax, ay, az, gx, gy, gz;

  for (int i = 0; i < samples; i++) {
    if (imuReadRaw(ax, ay, az, gx, gy, gz)) {
      sgx += gx; sgy += gy; sgz += gz;
      sax += ax; say += ay; saz += az;
      n++;
    }
    delay(2);
  }
  if (n < samples / 2) return false;

  gxBias = sgx / n;  gyBias = sgy / n;  gzBias = sgz / n;
  axOff  = sax / n;  ayOff  = say / n;  azOff  = saz / n - ACC_SENS;   // +1 g on Z when flat

  imuRoll = 0; imuPitch = 0;
  rollPID.reset(); pitchPID.reset();
  levelRoll = 0; levelPitch = 0;
  return true;
}

void imuInit() {
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) {
    imuOk = false;
    Serial.println("MPU6050 not found at 0x68 - leveling disabled. Send 'i' to scan I2C.");
    return;
  }
  imuWrite(0x6B, 0x01);   // wake up, gyro-X PLL clock
  delay(100);
  imuWrite(0x1A, 0x03);   // DLPF ~44 Hz
  imuWrite(0x1B, 0x08);   // gyro +-500 dps
  imuWrite(0x1C, 0x00);   // accel +-2 g
  delay(500);

  Serial.println("Calibrating IMU - keep the robot flat and still...");
  imuOk = imuCalibrate(IMU_CAL_SAMPLES);
  if (imuOk) {
    Serial.print("IMU OK. gyro bias raw: ");
    Serial.print(gxBias, 1); Serial.print(' ');
    Serial.print(gyBias, 1); Serial.print(' ');
    Serial.println(gzBias, 1);
  } else {
    Serial.println("IMU calibration failed - leveling disabled.");
  }
  lastImuUs = micros();
}

void setLeveling(bool on) {
  if (on && !imuOk) { Serial.println("IMU not available - leveling stays OFF"); return; }
  levelOn = on;
  rollPID.reset();
  pitchPID.reset();
  Serial.println(levelOn ? "LEVELING ON" : "LEVELING OFF");
}

// Rotates a foot target (body frame) about the body origin. Used on every foot before the IK.
void levelFoot(float &x, float &y, float &z, float rollDeg, float pitchDeg) {
  const float r = rollDeg  * DEG_TO_RAD;
  const float p = pitchDeg * DEG_TO_RAD;

  const float y1 = y * cosf(r) - z * sinf(r);     // roll about X
  const float z1 = y * sinf(r) + z * cosf(r);

  const float x2 =  x * cosf(p) + z1 * sinf(p);   // pitch about Y
  const float z2 = -x * sinf(p) + z1 * cosf(p);

  x = x2; y = y1; z = z2;
}

// Runs at ~200 Hz from loop(): attitude estimate + leveling PID + slew limit.
void imuUpdate() {
  if (!imuOk) return;

  const uint32_t nowUs = micros();
  if (nowUs - lastImuUs < IMU_PERIOD_US) return;
  float dt = (nowUs - lastImuUs) * 1e-6f;
  lastImuUs = nowUs;
  if (dt > 0.05f) dt = 0.05f;               // OLED redraw can stall the loop for ~25 ms

  int16_t rax, ray, raz, rgx, rgy, rgz;
  if (!imuReadRaw(rax, ray, raz, rgx, rgy, rgz)) {
    if (++imuFails >= 10) {
      imuOk = false;
      levelOn = false;
      Serial.println("IMU read failing - leveling disabled");
    }
    return;
  }
  imuFails = 0;

  float gx = (rgx - gxBias) / GYRO_SENS;    // deg/s
  float gy = (rgy - gyBias) / GYRO_SENS;
  float gz = (rgz - gzBias) / GYRO_SENS;
  float ax = (rax - axOff) / ACC_SENS;      // g
  float ay = (ray - ayOff) / ACC_SENS;
  float az = (raz - azOff) / ACC_SENS;

  imuRemapAxes(gx, gy, gz);
  imuRemapAxes(ax, ay, az);
  imuYawRate = gz;

  // Complementary filter. When the accel magnitude is far from 1 g (leg impacts,
  // acceleration) the accel is ignored for that sample and only the gyro is used.
  const float amag = sqrtf(ax * ax + ay * ay + az * az);
  const float a = (amag > 0.85f && amag < 1.15f) ? IMU_ALPHA : 1.0f;
  const float accRoll  = atan2f(ay, az) * RAD_TO_DEG;
  const float accPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

  imuRoll  = a * (imuRoll  + gx * dt) + (1.0f - a) * accRoll;
  imuPitch = a * (imuPitch + gy * dt) + (1.0f - a) * accPitch;

  // Safety: fallen over or picked up -> stop correcting
  if (levelOn && (fabsf(imuRoll) > TILT_CUTOFF_DEG || fabsf(imuPitch) > TILT_CUTOFF_DEG)) {
    setLeveling(false);
    Serial.println("Tilt too large - leveling OFF");
  }

  float wantRoll = 0.0f, wantPitch = 0.0f;
  if (levelOn) {
    // Positive roll (left up) -> feet rotated by +roll shorten the left legs / lengthen the right.
    // Positive pitch (nose down) -> front legs lengthen. The integral term holds the correction
    // while the body is level, so P alone never has to fight itself.
    wantRoll  = LEVEL_ROLL_SIGN  * rollPID.run(imuRoll,  dt);
    wantPitch = LEVEL_PITCH_SIGN * pitchPID.run(imuPitch, dt);
  }

  // Slew limit so the legs never jerk (also eases back to 0 when leveling is switched off)
  const float maxStep = LEVEL_MAX_RATE * dt;
  levelRoll  += constrain(wantRoll  - levelRoll,  -maxStep, maxStep);
  levelPitch += constrain(wantPitch - levelPitch, -maxStep, maxStep);
}

// ======================= HELPERS =======================

// Servo output uses Teensy hardware PWM (50 Hz) instead of the Servo library,
// because the Servo library only drives 12 servos and this robot has 18.
void servoAttach(uint8_t pin) {
  pinMode(pin, OUTPUT);
  analogWriteFrequency(pin, 50);
}
void servoWriteUs(uint8_t pin, int us) {
  analogWrite(pin, (uint32_t)(us * (65535.0f / 20000.0f) + 0.5f));   // 16-bit, 20 ms period
}

static inline float rad2deg(float r) { return r * 180.0f / M_PI; }

float normAxis(int raw, bool inv) {
  float n = (raw - 127.5f) / 127.5f;
  if (inv) n = -n;
  if (fabsf(n) < DEADZONE) return 0.0f;
  n = (n > 0) ? (n - DEADZONE) / (1.0f - DEADZONE) : (n + DEADZONE) / (1.0f - DEADZONE);
  return constrain(n, -1.0f, 1.0f);
}

void calculateIK(int leg, const Vec3& tgt, float out[3]) {
  float rx = tgt.x - LEG_X[leg];
  float ry = tgt.y - LEG_Y[leg];
  float rz = tgt.z;

  const float c = cosf(-neutralAng[leg]);
  const float s = sinf(-neutralAng[leg]);
  const float tx = rx * c - ry * s;
  const float ty = rx * s + ry * c;

  const float lxy = sqrtf(tx * tx + ty * ty);
  const float lf  = lxy - COXA_LENGTH;
  float D = sqrtf(lf * lf + rz * rz);

  const float maxD = FEMUR_LENGTH + TIBIA_LENGTH - 0.001f;
  const float minD = fabsf(FEMUR_LENGTH - TIBIA_LENGTH) + 0.001f;
  D = constrain(D, minD, maxD);

  out[2] = rad2deg(acosf((FEMUR_LENGTH * FEMUR_LENGTH + TIBIA_LENGTH * TIBIA_LENGTH - D * D) /
                         (2.0f * FEMUR_LENGTH * TIBIA_LENGTH)) - M_PI);
  const float b1 = atan2f(rz, lf);
  const float b2 = acosf((FEMUR_LENGTH * FEMUR_LENGTH + D * D - TIBIA_LENGTH * TIBIA_LENGTH) /
                         (2.0f * FEMUR_LENGTH * D));
  out[1] = rad2deg(b1 + b2);
  out[0] = rad2deg(atan2f(ty, tx));
}

// angle is 0..180 deg (90 = center). Converted to pulse width using SERVO_US_PER_DEG.
void writeServo(int leg, int j, float angle) {
  if (SERVO_INV[leg][j]) angle = 180.0f - angle;
  angle += SERVO_TRIM[leg][j];
  angle = constrain(angle, 0.0f, 180.0f);          // hard 180 deg limit
  int us = SERVO_US_MIN + (int)(angle * SERVO_US_PER_DEG + 0.5f);
  servoWriteUs(SERVO_PIN[leg][j], us);
}

void writeLeg(int leg) {
  // Leveling is applied to a copy, so the smoothed target[] used by the gait stays untouched
  Vec3 t = target[leg];
  if (levelRoll != 0.0f || levelPitch != 0.0f) levelFoot(t.x, t.y, t.z, levelRoll, levelPitch);

  float a[3];
  calculateIK(leg, t, a);
  float ax = constrain(90.0f + a[0], 45.0f, 135.0f);   // coxa limited to protect the physical stopper
  float ay = 90.0f + a[1];
  float az = fabsf(a[2]);
  writeServo(leg, 0, ax);
  writeServo(leg, 1, ay);
  writeServo(leg, 2, az);
}

Vec3 footTarget(int i, float vx, float vy, float rot, bool moving) {
  Vec3 h = HOME[i];
  if (!moving) return { h.x, h.y, bodyZ };

  const Gait& g = GAITS[gaitIdx];
  float p = fmodf(gaitT + g.off[i], 1.0f);
  float stride, lift = 0.0f;
  if (p < g.swing) {
    float u = p / g.swing;
    stride = u - 0.5f;
    lift = sinf(u * M_PI) * stepHeight;
  } else {
    float u = (p - g.swing) / (1.0f - g.swing);
    stride = 0.5f - u;
  }

  const float ang = rot * stride * ROTATION_STEP_RAD;
  const float cr = cosf(ang), sr = sinf(ang);
  Vec3 r;
  r.x = h.x * cr - h.y * sr + vx * stride * stepLength;
  r.y = h.x * sr + h.y * cr + vy * stride * stepLength;
  r.z = bodyZ + lift;
  return r;
}

void resetDefaults() {
  stepLength = DEF_STEP_LENGTH;
  stepHeight = DEF_STEP_HEIGHT;
  cycleHz = DEF_CYCLE_HZ;
  bodyZ = STAND_Z;
  gaitIdx = 0;
}

void printStatus() {
  Serial.print("armed="); Serial.print(armed);
  Serial.print(" gait="); Serial.print(GAITS[gaitIdx].name);
  Serial.print(" stepLen="); Serial.print(stepLength);
  Serial.print(" stepH="); Serial.print(stepHeight);
  Serial.print(" speed="); Serial.print(cycleHz);
  Serial.print("Hz bodyZ="); Serial.print(bodyZ);
  Serial.print(" batt="); Serial.print(shownVolts, 2);
  Serial.print("V "); Serial.print(filteredAmps, 2);
  Serial.print("A "); Serial.print(battPct);
  Serial.print("% used="); Serial.print((int)mahConsumed);
  Serial.print("mAh oled="); Serial.print(oledOk ? "OK" : "NOT FOUND");
  Serial.print(" imu="); Serial.print(imuOk ? "OK" : "NOT FOUND");
  Serial.print(" level="); Serial.print(levelOn ? "ON" : "OFF");
  Serial.print(" roll="); Serial.print(imuRoll, 1);
  Serial.print(" pitch="); Serial.print(imuPitch, 1);
  Serial.print(" corr="); Serial.print(levelRoll, 1);
  Serial.print("/"); Serial.println(levelPitch, 1);
}

// ======================= BATTERY / OLED =======================

// Dummy read to settle the ADC mux, then median of N samples (rejects servo-noise spikes)
float readMedianPinVolts(int pin) {
  const int N = 9;
  float samples[N];
  analogRead(pin);
  delayMicroseconds(50);
  for (int i = 0; i < N; i++) {
    samples[i] = analogRead(pin) * (3.3 / 4095.0);
    delayMicroseconds(50);
  }
  for (int i = 0; i < N - 1; i++)
    for (int j = i + 1; j < N; j++)
      if (samples[j] < samples[i]) { float t = samples[i]; samples[i] = samples[j]; samples[j] = t; }
  return samples[N / 2];
}

int getVoltagePercent(float v) {
  if (v >= 12.60) return 100;
  if (v >= 12.33) return map(v * 100, 1233, 1260, 90, 100);
  if (v >= 12.06) return map(v * 100, 1206, 1233, 75, 90);
  if (v >= 11.55) return map(v * 100, 1155, 1206, 50, 75);
  if (v >= 11.13) return map(v * 100, 1113, 1155, 25, 50);
  if (v >= 10.50) return map(v * 100, 1050, 1113, 10, 25);
  if (v > MIN_VOLTS) return map(v * 100, MIN_VOLTS * 100, 1050, 0, 10);
  return 0;
}

void i2cScan() {
  Serial.println("I2C scan (Wire: SDA=18, SCL=19):");
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print("  device at 0x"); Serial.println(a, HEX);
      found++;
    }
  }
  if (!found) Serial.println("  nothing found - check SDA/SCL wiring, pull-ups, 3.3V and GND");
}

void batteryInit() {
  analogReadResolution(12);
  analogReadAveraging(16);

  Wire.begin();
  Wire.setClock(400000);
  oledOk = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (!oledOk) oledOk = display.begin(SSD1306_SWITCHCAPVCC, 0x3D);   // some modules use 0x3D
  if (oledOk) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 24);
    display.println("Calibrating sensors...");
    display.display();
  } else {
    Serial.println("OLED not found (tried 0x3C, 0x3D) - continuing without display. Send 'i' to scan I2C.");
  }

  delay(300);   // let rails settle at idle (servos not attached yet)

  filteredVolts = readMedianPinVolts(V_SENSE_PIN) * V_DIVIDER_RATIO;
  shownVolts = filteredVolts;

  float offsetSum = 0.0;
  for (int i = 0; i < 20; i++) { offsetSum += readMedianPinVolts(I_SENSE_PIN); delay(10); }
  currentZeroOffsetV = offsetSum / 20.0;
  lastBattMs = millis();
}

void updateBattery(uint32_t now) {
  float dtHours = (now - lastBattMs) / 3600000.0;
  lastBattMs = now;

  float instantVolts = readMedianPinVolts(V_SENSE_PIN) * V_DIVIDER_RATIO;
  filteredVolts = 0.92f * filteredVolts + 0.08f * instantVolts;
  if (fabsf(filteredVolts - shownVolts) >= 0.02f) shownVolts = filteredVolts;

  float rawIVolts = readMedianPinVolts(I_SENSE_PIN) - currentZeroOffsetV;
  if (rawIVolts < 0.0f) rawIVolts = 0.0f;
  filteredAmps = 0.85f * filteredAmps + 0.15f * (rawIVolts * AMPS_PER_VOLT);

  mahConsumed += (filteredAmps * 1000.0f) * dtHours;

  float sagComp = shownVolts;
  if (filteredAmps > 0.5f) sagComp += filteredAmps * 0.035f;
  battPct = constrain(getVoltagePercent(sagComp), 0, 100);

  if (shownVolts <= WARN_VOLTS) {
    if (lowVoltStartMs == 0) lowVoltStartMs = now;
    if (now - lowVoltStartMs >= WARN_DELAY_MS) lowBattAlarm = true;
  } else {
    lowVoltStartMs = 0;
    lowBattAlarm = false;
  }
}

void drawOLED(uint32_t now) {
  display.clearDisplay();

  display.setTextSize(1);
  display.setCursor(0, 0);
  if (lowBattAlarm && ((now / 400) % 2 == 0)) {
    display.fillRect(0, 0, 128, 10, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.print(" !! LOW BATTERY !! ");
    display.setTextColor(SSD1306_WHITE);
  } else {
    display.print(armed ? "ARMED  " : "DISARM ");
    display.print(GAITS[gaitIdx].name);
  }

  display.setTextSize(2);
  display.setCursor(0, 16);
  display.print(battPct);
  display.print("%");

  display.setTextSize(1);
  display.setCursor(64, 16);
  display.print(shownVolts, 2);
  display.print("V");
  display.setCursor(64, 26);
  display.print(filteredAmps, 2);
  display.print("A");

  // Attitude line (y = 33..41, just above the divider)
  display.setCursor(0, 33);
  if (imuOk) {
    char buf[28];
    snprintf(buf, sizeof(buf), "R%+5.1f P%+5.1f %s", imuRoll, imuPitch, levelOn ? "LVL" : "---");
    display.print(buf);
  } else {
    display.print("IMU NOT FOUND");
  }

  display.drawLine(0, 42, 128, 42, SSD1306_WHITE);
  display.setCursor(0, 48);
  display.print(shownVolts * filteredAmps, 1);
  display.print("W");
  display.setCursor(54, 48);
  display.print("Used:");
  display.print((int)mahConsumed);
  display.print("mAh");

  display.display();
}

// ======================= SETUP =======================

void setup() {
  Serial.begin(115200);
  myusb.begin();

  // Battery + OLED first, while servos are still idle (auto-zeroes the current sensor)
  batteryInit();

  // IMU calibration also happens while the servos are idle. Keep the robot flat and still.
  imuInit();

  analogWriteResolution(16);

  const float reach = COXA_LENGTH + FEMUR_LENGTH * cosf(M_PI / 4) +
                      TIBIA_LENGTH * cosf(-M_PI / 4);
  for (int i = 0; i < 6; i++) {
    neutralAng[i] = atan2f(LEG_Y[i], LEG_X[i]);
    HOME[i] = { LEG_X[i] + reach * cosf(neutralAng[i]),
                LEG_Y[i] + reach * sinf(neutralAng[i]), STAND_Z };
    target[i] = HOME[i];
  }

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 3; j++) servoAttach(SERVO_PIN[i][j]);
#if CAL_MODE
    for (int j = 0; j < 3; j++) writeServo(i, j, 90.0f);
#else
    writeLeg(i);
#endif
    delay(150);
  }

#if CAL_MODE
  Serial.println("CAL MODE: all servos at 90 deg");
  while (true) { delay(1000); }
#endif

  Serial.println("Hexapod ready. Plug gamepad, press START to ARM. L3 or 'l' toggles leveling.");
  lastImuUs = micros();
  delay(500);
  lastImuUs = micros();
}

// ======================= LOOP =======================

void loop() {
  myusb.Task();

  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'd') { dbg = !dbg; Serial.println(dbg ? "dump ON" : "dump OFF"); }
    if (ch == 's') printStatus();
    if (ch == 'i') i2cScan();
    if (ch == 'l') setLeveling(!levelOn);
    if (ch == 'c') {
      levelOn = false;
      Serial.println("Recalibrating IMU - keep flat and still...");
      imuOk = imuCalibrate(IMU_CAL_SAMPLES);
      Serial.println(imuOk ? "IMU recalibrated" : "IMU calibration failed");
      lastImuUs = micros();
    }
  }

  imuUpdate();                            // ~200 Hz, own timer

  static uint32_t lastMs = 0;
  uint32_t now = millis();
  if (now - lastMs < 20) return;          // 50 Hz servo frame
  float dt = (now - lastMs) / 1000.0f;
  lastMs = now;
  if (dt > 0.1f) dt = 0.02f;

  float vx = 0, vy = 0, rot = 0;

  if (joystick) {
    const uint32_t b = joystick.getButtons();
    const uint32_t pressed = b & ~prevButtons;
    prevButtons = b;

    const float lx = normAxis(joystick.getAxis(AXIS_LX), INV_LX);
    const float ly = normAxis(joystick.getAxis(AXIS_LY), INV_LY);
    const float rx = normAxis(joystick.getAxis(AXIS_RX), INV_RX);
    const float ry = normAxis(joystick.getAxis(AXIS_RY), INV_RY);
    const int   hat = joystick.getAxis(AXIS_HAT);

    if (pressed & (1u << BTN_START)) {
      armed = !armed;
      Serial.println(armed ? "ARMED" : "DISARMED");
    }
    if (pressed & (1u << BTN_BACK)) { resetDefaults(); Serial.println("defaults"); }
    if (pressed & (1u << BTN_L3)) setLeveling(!levelOn);
    if (pressed & (1u << BTN_A)) bodyZ = STAND_Z;
    if (pressed & (1u << BTN_B)) bodyZ = SIT_Z;
    if (pressed & (1u << BTN_X)) {
      gaitIdx = (gaitIdx + 1) % 2;
      Serial.print("gait: "); Serial.println(GAITS[gaitIdx].name);
    }
    if (pressed & (1u << BTN_Y)) printStatus();
    if (pressed & (1u << BTN_LB)) stepLength = constrain(stepLength - 5, 20, 90);
    if (pressed & (1u << BTN_RB)) stepLength = constrain(stepLength + 5, 20, 90);
    if (pressed & (1u << BTN_LT)) bodyZ = constrain(bodyZ + 5, Z_MIN, Z_MAX);
    if (pressed & (1u << BTN_RT)) bodyZ = constrain(bodyZ - 5, Z_MIN, Z_MAX);

    if (hat != prevHat && hat >= 0 && hat <= 7) {
      const bool up    = (hat == 7 || hat == 0 || hat == 1);
      const bool right = (hat >= 1 && hat <= 3);
      const bool down  = (hat >= 3 && hat <= 5);
      const bool left  = (hat >= 5 && hat <= 7);
      if (up)    stepHeight = constrain(stepHeight + 5, 20, 100);
      if (down)  stepHeight = constrain(stepHeight - 5, 20, 100);
      if (right) cycleHz    = constrain(cycleHz + 0.1f, 0.3f, 1.5f);
      if (left)  cycleHz    = constrain(cycleHz - 0.1f, 0.3f, 1.5f);
    }
    prevHat = hat;

    bodyZ = constrain(bodyZ + ry * 40.0f * dt, Z_MIN, Z_MAX);

    if (armed) {
      vx  = -ly;
      vy  = -lx;
      rot = -rx;
      float mag = sqrtf(vx * vx + vy * vy);
      if (mag > 1.0f) { vx /= mag; vy /= mag; }
    }

    if (dbg) {
      static uint32_t lastDump = 0;
      if (now - lastDump > 200) {
        lastDump = now;
        Serial.print("btn=0x"); Serial.print(b, HEX);
        for (int a = 0; a < 10; a++) { Serial.print(" a"); Serial.print(a); Serial.print("="); Serial.print(joystick.getAxis(a)); }
        Serial.println();
      }
    }
  } else {
    if (armed) { armed = false; Serial.println("Gamepad lost - DISARMED"); }
    prevButtons = 0;
  }

  // ---- gait ----
  const bool moving = (fabsf(vx) > 0.01f || fabsf(vy) > 0.01f || fabsf(rot) > 0.01f);
  if (moving) {
    gaitT += cycleHz * dt;
    if (gaitT >= 1.0f) gaitT -= 1.0f;
  }

  for (int i = 0; i < 6; i++) {
    Vec3 goal = footTarget(i, vx, vy, rot, moving);
    target[i].x += (goal.x - target[i].x) * SMOOTHING;
    target[i].y += (goal.y - target[i].y) * SMOOTHING;
    target[i].z += (goal.z - target[i].z) * SMOOTHING;
    writeLeg(i);
  }

  // ---- battery + OLED (after servos are updated, so legs never wait on it) ----
  updateBattery(now);
  if (oledOk && now - lastOledMs >= OLED_PERIOD_MS) {
    lastOledMs = now;
    drawOLED(now);
  }
}
