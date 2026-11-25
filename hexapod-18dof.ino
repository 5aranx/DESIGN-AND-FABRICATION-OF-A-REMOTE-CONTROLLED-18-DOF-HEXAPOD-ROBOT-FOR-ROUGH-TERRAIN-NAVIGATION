/**
 * Hexapod - USB gamepad control + OLED battery monitor + MPU6050 body leveling
 * (Teensy 4.1, custom PCB). 18x MG995 180-degree servos.
 *
 * Walking is done in SERVO-ANGLE space around the measured STAND pose (no IK).
 *
 * CONTROLS
 *   A ................... STAND (and ARM walking if AUTO_ARM)
 *   B ................... SIT (boot pose)
 *   START ............... ARM / DISARM walking
 *   Left stick Y ........ walk forward / back
 *   Right stick X ....... turn (yaw)
 *   Right stick Y ....... body height (femur offset)
 *   D-pad Up/Down ....... step height + / -
 *   D-pad Left/Right .... gait speed  - / +
 *   LB / RB ............. step length - / +
 *   LT / RT ............. body height - / +
 *   X ................... toggle gait (TRIPOD <-> WAVE)
 *   Y ................... print status
 *   L3 .................. toggle body leveling
 *   BACK ................ reset settings
 *
 * Serial monitor (115200): 'd' gamepad dump, 's' status, 'i' I2C scan,
 *                          'l' leveling, 'c' recalibrate IMU
 *
 * POSES:  SIT   = coxa 90, femur 180, tibia 160
 *         STAND = coxa 90, femur 105, tibia 160
 * Tibia is hard-limited to 0..160 deg.
 *
 * Test with the robot lifted first!
 */

#include <USBHost_t36.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define CAL_MODE 0   // 1 = hold every servo at 90 deg and do nothing else

// ======================= CONFIG =======================

const float LEG_X[6] = {  100.94f,    0.0f, -100.94f, -100.94f,    0.0f,  100.94f };
const float LEG_Y[6] = {  -57.11f, -105.72f,  -57.11f,   57.11f, 105.72f,   57.11f };

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
const bool SERVO_INV[6][3] = {
  { false, false, false }, { false, false, false }, { false, false, false },
  { false, false, false }, { false, false, false }, { false, false, false }
};

// ---- Servo pulse scaling (MG995, 180 deg) ----
const float SERVO_FULL_US    = 2000.0f;
const float SERVO_SPAN_DEG   = 180.0f;
const float SERVO_US_PER_DEG = SERVO_FULL_US / SERVO_SPAN_DEG;
const int SERVO_US_MIN = 1500 - (int)(90.0f * SERVO_US_PER_DEG + 0.5f);   // 500 us
const int SERVO_US_MAX = 1500 + (int)(90.0f * SERVO_US_PER_DEG + 0.5f);   // 2500 us

// ---- Per-joint hard angle limits ----
//                                   coxa   femur  tibia
const float JOINT_MIN_DEG[3] = {     0.0f,   0.0f,   0.0f };
const float JOINT_MAX_DEG[3] = {   180.0f, 180.0f, 160.0f };   // tibia can NEVER exceed 160

// ---- Walking tuning (servo-angle gait) ----
const float STRIDE_DEG_PER_MM = 0.75f;   // coxa sweep (deg) per mm of step length
const float LIFT_DEG_PER_MM   = 0.8f;    // femur lift (deg) per mm of step height
const float TIBIA_LIFT_RATIO  = 0.8f;    // tibia bends this many deg per deg of femur lift
const float TURN_DEG          = 15.0f;   // coxa sweep when turning at full stick

// DIRECTION SIGNS - if the robot moves the wrong way, flip these:
const float FEMUR_LIFT_SIGN = -1.0f;     // -1: foot lifts when femur angle goes DOWN from stand
const float TIBIA_LIFT_SIGN = -1.0f;     // -1: tibia angle goes DOWN while lifting
const float FWD_SIGN[6]  = {  1,  1,  1,  1,  1,  1 };    // flip a leg that sweeps backward
const float TURN_SIGN[6] = {  1,  1,  1, -1, -1, -1 };    // right/left sides mirrored

// ---- Defaults / limits ----
const bool  AUTO_ARM         = true;     // pressing A (stand) also arms walking
const float DEF_STEP_LENGTH  = 40.0f;
const float DEF_STEP_HEIGHT  = 35.0f;
const float DEF_CYCLE_HZ     = 0.6f;
const float COXA_MIN_DEG     = 45.0f;
const float COXA_MAX_DEG     = 135.0f;
const float BODY_OFF_MAX     = 25.0f;

// ---- Anti-jitter + current limiting ----
const float SERVO_DEADBAND_DEG = 0.3f;
const float SERVO_MAX_DPS      = 90.0f;
const float FRAME_DT           = 0.02f;

// ---- Gamepad mapping ----
const int AXIS_LX = 0, AXIS_LY = 1, AXIS_RX = 2, AXIS_RY = 5, AXIS_HAT = 9;
const bool INV_LX = false, INV_LY = false, INV_RX = false, INV_RY = false;
const float DEADZONE = 0.15f;

const int BTN_X = 0, BTN_A = 1, BTN_B = 2, BTN_Y = 3;
const int BTN_LB = 4, BTN_RB = 5, BTN_LT = 6, BTN_RT = 7;
const int BTN_BACK = 8, BTN_START = 9;
const int BTN_L3 = 10;

// ---- OLED + battery monitor ----
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
const int V_SENSE_PIN = 38;
const int I_SENSE_PIN = 39;
const float V_DIVIDER_RATIO = 11.352;
const float AMPS_PER_VOLT   = 18.18;
const float BATTERY_MAH     = 3300.0;
const float MIN_VOLTS       = 10.20;
const float WARN_VOLTS      = 10.35;
const unsigned long WARN_DELAY_MS = 3500;
const unsigned long OLED_PERIOD_MS = 500;

// ---- MPU6050 + body leveling ----
const uint8_t  MPU_ADDR        = 0x68;
const float    GYRO_SENS       = 65.5f;
const float    ACC_SENS        = 16384.0f;
const float    IMU_ALPHA       = 0.98f;
const uint32_t IMU_PERIOD_US   = 5000;
const int      IMU_CAL_SAMPLES = 1500;

const float LEVEL_MAX_DEG    = 20.0f;
const float LEVEL_MAX_RATE   = 120.0f;
const float TILT_CUTOFF_DEG  = 45.0f;
const float LEVEL_ROLL_SIGN  = 1.0f;
const float LEVEL_PITCH_SIGN = 1.0f;
const float LEVEL_FEMUR_GAIN = 1.5f;

// ======================= STATE =======================

USBHost myusb;
USBHub hub1(myusb);
USBHIDParser hid1(myusb);
USBHIDParser hid2(myusb);
JoystickController joystick(myusb);

struct Gait { const char* name; float swing; float off[6]; };
const Gait GAITS[2] = {
  { "TRIPOD", 0.5f,        { 0.0f, 0.5f, 0.0f, 0.5f, 0.0f, 0.5f } },
  { "WAVE",   1.0f / 6.0f, { 4.0f/6, 5.0f/6, 0.0f, 3.0f/6, 2.0f/6, 1.0f/6 } }
};

int   gaitIdx    = 0;
float stepLength = DEF_STEP_LENGTH;
float stepHeight = DEF_STEP_HEIGHT;
float cycleHz    = DEF_CYCLE_HZ;
float bodyOff    = 0.0f;
float gaitT      = 0.0f;
bool  armed      = false;
bool  dbg        = false;
bool  wasMoving  = false;

//                              coxa  femur tibia
const float STAND_POSE[3] = {  90.0f, 105.0f, 160.0f };
const float SIT_POSE[3]   = {  90.0f, 180.0f, 160.0f };
bool poseStand = false;

uint32_t prevButtons = 0;
int      prevHat     = 15;

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
    i = constrain(i + ki * err * dt, -outMax, outMax);
    float d = (dt > 0.0f) ? kd * (err - last) / dt : 0.0f;
    last = err;
    return constrain(kp * err + i + d, -outMax, outMax);
  }
  void reset() { i = 0.0f; last = 0.0f; }
};

PID rollPID  = { 1.0f, 2.0f, 0.0f, 0, 0, LEVEL_MAX_DEG };
PID pitchPID = { 1.0f, 2.0f, 0.0f, 0, 0, LEVEL_MAX_DEG };

float gxBias = 0, gyBias = 0, gzBias = 0;
float axOff  = 0, ayOff  = 0, azOff  = 0;

float imuRoll = 0, imuPitch = 0;
float imuYawRate = 0;
bool  imuOk = false;
bool  levelOn = false;
uint8_t  imuFails = 0;
uint32_t lastImuUs = 0;

float levelRoll = 0, levelPitch = 0;

void imuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static inline int16_t imuRead16() {
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return (int16_t)((hi << 8) | lo);
}

bool imuReadRaw(int16_t &ax, int16_t &ay, int16_t &az,
                int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14) != 14) return false;

  ax = imuRead16();
  ay = imuRead16();
  az = imuRead16();
  imuRead16();
  gx = imuRead16();
  gy = imuRead16();
  gz = imuRead16();
  return true;
}

static inline void imuRemapAxes(float &x, float &y, float &z) {
  x = -x;
  y = -y;
  (void)z;
}

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
  axOff  = sax / n;  ayOff  = say / n;  azOff  = saz / n - ACC_SENS;

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
  imuWrite(0x6B, 0x01);
  delay(100);
  imuWrite(0x1A, 0x03);
  imuWrite(0x1B, 0x08);
  imuWrite(0x1C, 0x00);
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

void imuUpdate() {
  if (!imuOk) return;

  const uint32_t nowUs = micros();
  if (nowUs - lastImuUs < IMU_PERIOD_US) return;
  float dt = (nowUs - lastImuUs) * 1e-6f;
  lastImuUs = nowUs;
  if (dt > 0.05f) dt = 0.05f;

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

  float gx = (rgx - gxBias) / GYRO_SENS;
  float gy = (rgy - gyBias) / GYRO_SENS;
  float gz = (rgz - gzBias) / GYRO_SENS;
  float ax = (rax - axOff) / ACC_SENS;
  float ay = (ray - ayOff) / ACC_SENS;
  float az = (raz - azOff) / ACC_SENS;

  imuRemapAxes(gx, gy, gz);
  imuRemapAxes(ax, ay, az);
  imuYawRate = gz;

  const float amag = sqrtf(ax * ax + ay * ay + az * az);
  const float a = (amag > 0.85f && amag < 1.15f) ? IMU_ALPHA : 1.0f;
  const float accRoll  = atan2f(ay, az) * RAD_TO_DEG;
  const float accPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

  imuRoll  = a * (imuRoll  + gx * dt) + (1.0f - a) * accRoll;
  imuPitch = a * (imuPitch + gy * dt) + (1.0f - a) * accPitch;

  if (levelOn && (fabsf(imuRoll) > TILT_CUTOFF_DEG || fabsf(imuPitch) > TILT_CUTOFF_DEG)) {
    setLeveling(false);
    Serial.println("Tilt too large - leveling OFF");
  }

  float wantRoll = 0.0f, wantPitch = 0.0f;
  if (levelOn) {
    wantRoll  = LEVEL_ROLL_SIGN  * rollPID.run(imuRoll,  dt);
    wantPitch = LEVEL_PITCH_SIGN * pitchPID.run(imuPitch, dt);
  }

  const float maxStep = LEVEL_MAX_RATE * dt;
  levelRoll  += constrain(wantRoll  - levelRoll,  -maxStep, maxStep);
  levelPitch += constrain(wantPitch - levelPitch, -maxStep, maxStep);
}

// ======================= HELPERS =======================

void servoAttach(uint8_t pin) {
  pinMode(pin, OUTPUT);
  analogWriteFrequency(pin, 50);
}
void servoWriteUs(uint8_t pin, int us) {
  analogWrite(pin, (uint32_t)(us * (65535.0f / 20000.0f) + 0.5f));
}

float normAxis(int raw, bool inv) {
  float n = (raw - 127.5f) / 127.5f;
  if (inv) n = -n;
  if (fabsf(n) < DEADZONE) return 0.0f;
  n = (n > 0) ? (n - DEADZONE) / (1.0f - DEADZONE) : (n + DEADZONE) / (1.0f - DEADZONE);
  return constrain(n, -1.0f, 1.0f);
}

float servoLast[6][3];
bool  servoInit[6][3];

void writeServo(int leg, int j, float angle) {
  if (SERVO_INV[leg][j]) angle = 180.0f - angle;
  angle += SERVO_TRIM[leg][j];
  angle = constrain(angle, JOINT_MIN_DEG[j], JOINT_MAX_DEG[j]);

  if (!servoInit[leg][j]) {
    servoInit[leg][j] = true;
    servoLast[leg][j] = angle;
  } else {
    float diff = angle - servoLast[leg][j];
    if (fabsf(diff) >= SERVO_DEADBAND_DEG) {
      const float maxStep = SERVO_MAX_DPS * FRAME_DT;
      servoLast[leg][j] += constrain(diff, -maxStep, maxStep);
    }
  }
  float out = constrain(servoLast[leg][j], JOINT_MIN_DEG[j], JOINT_MAX_DEG[j]);
  int us = SERVO_US_MIN + (int)(out * SERVO_US_PER_DEG + 0.5f);
  servoWriteUs(SERVO_PIN[leg][j], us);
}

void writePose(const float pose[3]) {
  for (int leg = 0; leg < 6; leg++)
    for (int j = 0; j < 3; j++) writeServo(leg, j, pose[j]);
}

// Servo angles of one leg while standing / walking (offsets from STAND_POSE).
void computeLegAngles(int i, float vx, float rot, bool moving, float out[3]) {
  float coxaOff = 0.0f, lift = 0.0f;

  if (moving) {
    const Gait& g = GAITS[gaitIdx];
    float p = fmodf(gaitT + g.off[i], 1.0f);
    float stride;
    if (p < g.swing) {                              // swing: foot in the air, moves forward
      float u = p / g.swing;
      stride = u - 0.5f;
      lift = sinf(u * M_PI) * stepHeight * LIFT_DEG_PER_MM;
    } else {                                        // stance: foot on ground, moves backward
      float u = (p - g.swing) / (1.0f - g.swing);
      stride = 0.5f - u;
    }
    coxaOff = stride * (vx * stepLength * STRIDE_DEG_PER_MM * FWD_SIGN[i] +
                        rot * TURN_DEG * TURN_SIGN[i]);
  }

  float cox = STAND_POSE[0] + coxaOff;
  float fem = STAND_POSE[1] + bodyOff + FEMUR_LIFT_SIGN * lift;
  float tib = STAND_POSE[2] + TIBIA_LIFT_SIGN * lift * TIBIA_LIFT_RATIO;

  if (levelOn) {
    const float sy = (LEG_Y[i] > 1.0f) ? 1.0f : (LEG_Y[i] < -1.0f ? -1.0f : 0.0f);
    const float sx = (LEG_X[i] > 1.0f) ? 1.0f : (LEG_X[i] < -1.0f ? -1.0f : 0.0f);
    fem += LEVEL_FEMUR_GAIN * ( levelRoll * sy - levelPitch * sx);
  }

  out[0] = constrain(cox, COXA_MIN_DEG, COXA_MAX_DEG);
  out[1] = constrain(fem, JOINT_MIN_DEG[1], JOINT_MAX_DEG[1]);
  out[2] = constrain(tib, JOINT_MIN_DEG[2], JOINT_MAX_DEG[2]);   // tibia 0..160 only
}

void resetDefaults() {
  stepLength = DEF_STEP_LENGTH;
  stepHeight = DEF_STEP_HEIGHT;
  cycleHz = DEF_CYCLE_HZ;
  bodyOff = 0.0f;
  gaitIdx = 0;
}

void printStatus() {
  Serial.print("armed="); Serial.print(armed);
  Serial.print(" pose="); Serial.print(poseStand ? "STAND" : "SIT");
  Serial.print(" gait="); Serial.print(GAITS[gaitIdx].name);
  Serial.print(" stepLen="); Serial.print(stepLength);
  Serial.print(" stepH="); Serial.print(stepHeight);
  Serial.print(" speed="); Serial.print(cycleHz);
  Serial.print("Hz bodyOff="); Serial.print(bodyOff);
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
  if (!oledOk) oledOk = display.begin(SSD1306_SWITCHCAPVCC, 0x3D);
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

  delay(300);

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
    display.print(poseStand ? "STAND" : "SIT");
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

  batteryInit();
  imuInit();

  analogWriteResolution(16);

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 3; j++) servoAttach(SERVO_PIN[i][j]);
#if CAL_MODE
    for (int j = 0; j < 3; j++) writeServo(i, j, 90.0f);
#else
    for (int j = 0; j < 3; j++) writeServo(i, j, SIT_POSE[j]);
#endif
    delay(150);
  }

#if CAL_MODE
  Serial.println("CAL MODE: all servos at 90 deg");
  while (true) { delay(1000); }
#endif

  Serial.println("Hexapod ready (sitting). A = STAND (+ARM), left stick = walk.");
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

  imuUpdate();

  static uint32_t lastMs = 0;
  uint32_t now = millis();
  if (now - lastMs < 20) return;          // 50 Hz servo frame
  float dt = (now - lastMs) / 1000.0f;
  lastMs = now;
  if (dt > 0.1f) dt = 0.02f;

  float vx = 0, rot = 0;

  if (joystick) {
    const uint32_t b = joystick.getButtons();
    const uint32_t pressed = b & ~prevButtons;
    prevButtons = b;

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
    if (pressed & (1u << BTN_A)) {
      poseStand = true;
      Serial.println("STAND");
      if (AUTO_ARM && !armed) { armed = true; Serial.println("ARMED (auto)"); }
    }
    if (pressed & (1u << BTN_B)) { poseStand = false; Serial.println("SIT"); }
    if (pressed & (1u << BTN_X)) {
      gaitIdx = (gaitIdx + 1) % 2;
      Serial.print("gait: "); Serial.println(GAITS[gaitIdx].name);
    }
    if (pressed & (1u << BTN_Y)) printStatus();
    if (pressed & (1u << BTN_LB)) stepLength = constrain(stepLength - 5, 20, 90);
    if (pressed & (1u << BTN_RB)) stepLength = constrain(stepLength + 5, 20, 90);
    if (pressed & (1u << BTN_LT)) bodyOff = constrain(bodyOff + 5.0f, -BODY_OFF_MAX, BODY_OFF_MAX);
    if (pressed & (1u << BTN_RT)) bodyOff = constrain(bodyOff - 5.0f, -BODY_OFF_MAX, BODY_OFF_MAX);

    if (hat != prevHat && hat >= 0 && hat <= 7) {
      const bool up    = (hat == 7 || hat == 0 || hat == 1);
      const bool right = (hat >= 1 && hat <= 3);
      const bool down  = (hat >= 3 && hat <= 5);
      const bool left  = (hat >= 5 && hat <= 7);
      if (up)    stepHeight = constrain(stepHeight + 5, 20, 100);
      if (down)  stepHeight = constrain(stepHeight - 5, 20, 100);
      if (right) cycleHz    = constrain(cycleHz + 0.1f, 0.2f, 1.0f);
      if (left)  cycleHz    = constrain(cycleHz - 0.1f, 0.2f, 1.0f);
    }
    prevHat = hat;

    bodyOff = constrain(bodyOff - ry * 30.0f * dt, -BODY_OFF_MAX, BODY_OFF_MAX);

    if (armed && poseStand) {
      vx  = -ly;          // stick up = forward
      rot = -rx;
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

  // ---- gait clock ----
  const bool moving = (fabsf(vx) > 0.01f || fabsf(rot) > 0.01f);
  if (moving != wasMoving) {
    wasMoving = moving;
    Serial.println(moving ? "WALKING" : "stopped");
  }
  if (moving) {
    float deflect = constrain(fmaxf(fabsf(vx), fabsf(rot)), 0.4f, 1.0f);
    gaitT += cycleHz * deflect * dt;
    if (gaitT >= 1.0f) gaitT -= 1.0f;
  }

  // ---- legs ----
  if (poseStand) {
    for (int i = 0; i < 6; i++) {
      float a[3];
      computeLegAngles(i, vx, rot, moving, a);
      writeServo(i, 0, a[0]);
      writeServo(i, 1, a[1]);
      writeServo(i, 2, a[2]);
    }
  } else {
    writePose(SIT_POSE);
  }

  // ---- battery + OLED ----
  static uint32_t lastBatt = 0;
  if (now - lastBatt >= 100) { lastBatt = now; updateBattery(now); }
  if (oledOk && now - lastOledMs >= OLED_PERIOD_MS) {
    lastOledMs = now;
    drawOLED(now);
  }
}
