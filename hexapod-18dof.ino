/**
 * Hexapod – USB gamepad control + OLED battery monitor (Teensy 4.1, custom PCB)
 *
 * IK / leg geometry based on Emre Kalem & Özgür Toprak Önsoy's hexapod (MIT).
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
 *   START ............... ARM / DISARM walking (boots DISARMED)
 *   BACK ................ reset all settings to defaults
 *
 * Serial monitor (115200):  'd' = toggle raw gamepad dump, 's' = print status,
 *                           'i' = I2C scan (use this if the OLED stays blank)
 *
 * OLED: SSD1306 128x64 on I2C (Wire: SDA=pin 18, SCL=pin 19).
 * Battery sense: pin 38 (voltage), pin 39 (current).
 *
 * FIRST TIME: set CAL_MODE 1, upload, mount all servo horns at 90 deg, then CAL_MODE 0.
 * Test with the robot lifted on a stand first!
 */

#include <USBHost_t36.h>
#include <Servo.h>
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

// ---- Servo pins from schematic: {coxa, femur, tibia} ----
const uint8_t SERVO_PIN[6][3] = {
  { 10,  9,  8 },   // Leg1 (RF)
  {  7,  6,  5 },   // Leg2 (RM)
  {  4,  3,  2 },   // Leg3 (RR)
  { 36, 33, 24 },   // Leg4 (LR)
  { 28, 25, 37 },   // Leg5 (LM)
  { 15, 14, 29 }    // Leg6 (LF)
};

float SERVO_TRIM[6][3] = {
  { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 },
  { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }
};
const bool SERVO_INV[6][3] = {
  { false, false, false }, { false, false, false }, { false, false, false },
  { false, false, false }, { false, false, false }, { false, false, false }
};

const int SERVO_US_MIN = 500;
const int SERVO_US_MAX = 2500;

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

// ======================= STATE =======================

USBHost myusb;
USBHub hub1(myusb);
USBHIDParser hid1(myusb);
USBHIDParser hid2(myusb);
JoystickController joystick(myusb);

Servo servos[6][3];

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

// ======================= HELPERS =======================

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

void writeServo(int leg, int j, float angle) {
  if (SERVO_INV[leg][j]) angle = 180.0f - angle;
  angle += SERVO_TRIM[leg][j];
  angle = constrain(angle, 0.0f, 180.0f);
  int us = SERVO_US_MIN + (int)(angle * (SERVO_US_MAX - SERVO_US_MIN) / 180.0f + 0.5f);
  servos[leg][j].writeMicroseconds(us);
}

void writeLeg(int leg) {
  float a[3];
  calculateIK(leg, target[leg], a);
  float ax = constrain(90.0f + a[0], 45.0f, 135.0f);
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
  Serial.print("mAh oled="); Serial.println(oledOk ? "OK" : "NOT FOUND");
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

  const float reach = COXA_LENGTH + FEMUR_LENGTH * cosf(M_PI / 4) +
                      TIBIA_LENGTH * cosf(-M_PI / 4);
  for (int i = 0; i < 6; i++) {
    neutralAng[i] = atan2f(LEG_Y[i], LEG_X[i]);
    HOME[i] = { LEG_X[i] + reach * cosf(neutralAng[i]),
                LEG_Y[i] + reach * sinf(neutralAng[i]), STAND_Z };
    target[i] = HOME[i];
  }

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 3; j++) servos[i][j].attach(SERVO_PIN[i][j], SERVO_US_MIN, SERVO_US_MAX);
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

  Serial.println("Hexapod ready. Plug gamepad, press START to ARM.");
  delay(500);
}

// ======================= LOOP =======================

void loop() {
  myusb.Task();

  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'd') { dbg = !dbg; Serial.println(dbg ? "dump ON" : "dump OFF"); }
    if (ch == 's') printStatus();
    if (ch == 'i') i2cScan();
  }

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
