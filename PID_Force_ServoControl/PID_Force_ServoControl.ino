/*
  Teensy 4.x Five-Finger Force PID Controller
  - Controls servo angles from 0..180 deg
  - Each finger closes under PID until force >= setpoint
  - Per finger command bit over UART: 1 = close under PID, 0 = open toward 0
  - Packet: <SP=320;CMD=10110>\n
    Thumb=1, Index=0, Middle=1, Ring=1, Pinky=0 in this example
*/

#include <Arduino.h>
#include <Servo.h>

// ===================== User wiring =====================
const int servoPins[5] = {3, 5, 6, 9, 10};      // Thumb, Index, Middle, Ring, Pinky
const int forcePins[5] = {A0, A1, A2, A3, A4};  // Analog inputs for force sensors

// Teensy 4.x default analog resolution is 10 bits in Arduino API unless changed
// You can do analogReadResolution(12) if your sensors benefit from it
// analogReadAveraging(8) can also help if needed

// ===================== PID settings =====================
// Start with these, then tune per finger
float Kp[5] = {0.8, 0.8, 0.8, 0.8, 0.8};
float Ki[5] = {0.05, 0.05, 0.05, 0.05, 0.05};
float Kd[5] = {0.02, 0.02, 0.02, 0.02, 0.02};

// Derivative low-pass filter coefficient in [0..1]. Higher is smoother but slower derivative.
// y_d = alpha*y_d + (1-alpha)*raw
const float dFilterAlpha = 0.85f;

// Optional input low-pass on force readings to reduce noise
const float forceLPFAlpha = 0.85f;

// PID compute period in microseconds
const uint32_t PID_PERIOD_US = 5000;  // 5 ms = 200 Hz

// Output limits
const float ANGLE_MIN = 0.0f;
const float ANGLE_MAX = 180.0f;

// Open behavior when command bit is 0
const float OPEN_SPEED_DEG_PER_SEC = 180.0f;  // how fast to open toward 0 when not commanded to close

// Anti-windup clamp on integral term contribution (in angle units)
const float I_MIN = -50.0f;
const float I_MAX = 50.0f;

// Safety force ceiling. If exceeded, freeze angle and report fault.
const int FORCE_HARD_LIMIT = 4095;  // set to your ADC scale if using 12 bit

// ===================== State =====================
Servo servos[5];

float angleCmd[5]     = {0, 0, 0, 0, 0};   // commanded angle to servos
float integ[5]        = {0, 0, 0, 0, 0};   // integral accumulator
float prevErr[5]      = {0, 0, 0, 0, 0};   // previous error for derivative
float dFiltered[5]    = {0, 0, 0, 0, 0};   // filtered derivative
float forceFilt[5]    = {0, 0, 0, 0, 0};   // filtered force

// Command interface
int   setpoint = 300;          // default force setpoint in sensor units
uint8_t closeMask = 0b00000;   // bit i = 1 means close finger i under PID

// Timing
elapsedMicros pidTimer;

// ===================== Helpers =====================

// Clamp utility
template <typename T>
T clamp(T v, T lo, T hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// Parse packets of the form <SP=320;CMD=10110>\n
void parseSerial() {
  static String buf;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (buf.length() > 0) {
        // Process one line
        int spPos = buf.indexOf("SP=");
        int cmdPos = buf.indexOf("CMD=");
        if (spPos >= 0) {
          int sep = buf.indexOf(';', spPos);
          String spStr = (sep > spPos) ? buf.substring(spPos + 3, sep) : buf.substring(spPos + 3);
          setpoint = spStr.toInt();
        }
        if (cmdPos >= 0) {
          // read until '>' or end
          int endPos = buf.indexOf('>', cmdPos);
          String cmdStr;
          if (endPos > cmdPos) cmdStr = buf.substring(cmdPos + 4, endPos);
          else                 cmdStr = buf.substring(cmdPos + 4);
          // Expect 5 chars of 0 or 1
          if (cmdStr.length() >= 5) {
            uint8_t m = 0;
            for (int i = 0; i < 5; ++i) {
              if (cmdStr[i] == '1') m |= (1 << i);
            }
            closeMask = m;
          }
        }
        // Optional echo for debugging
        Serial.print(F("[ACK] SP="));
        Serial.print(setpoint);
        Serial.print(F(" CMD="));
        for (int i = 0; i < 5; ++i) Serial.print((closeMask >> i) & 1);
        Serial.println();
      }
      buf = "";
    } else {
      if (buf.length() < 128) buf += c;
    }
  }
}

// Read and optionally low-pass filter force sensors
void readForces(int raw[5]) {
  for (int i = 0; i < 5; ++i) {
    raw[i] = analogRead(forcePins[i]);
    // optional resolution change. Uncomment if you want 12 bit.
    // analogReadResolution(12);
    // raw[i] = analogRead(forcePins[i]);
    // low-pass
    forceFilt[i] = forceLPFAlpha * forceFilt[i] + (1.0f - forceLPFAlpha) * (float)raw[i];
  }
}

// One PID step for a single finger i
void pidStep(int i, float dtSec, float forceMeas, bool enablePID) {
  if (!enablePID) {
    // Opening mode. Disable integrator and move toward 0 at a fixed speed.
    integ[i] = 0.0f;
    prevErr[i] = 0.0f;
    dFiltered[i] = 0.0f;
    angleCmd[i] -= OPEN_SPEED_DEG_PER_SEC * dtSec;
    angleCmd[i] = clamp(angleCmd[i], ANGLE_MIN, ANGLE_MAX);
    return;
  }

  // Safety ceiling
  if (forceMeas >= FORCE_HARD_LIMIT) {
    // Freeze command for this finger and report
    Serial.print(F("[FAULT] Force hard limit on finger "));
    Serial.println(i);
    return;
  }

  // Error is positive if we want more force
  float err = (float)setpoint - forceMeas;

  // Proportional
  float P = Kp[i] * err;

  // Integral with anti-windup clamp on the I term contribution
  integ[i] += Ki[i] * err * dtSec;
  integ[i] = clamp(integ[i], I_MIN, I_MAX);

  // Derivative on measurement with simple low-pass
  float derr = (err - prevErr[i]) / dtSec;
  dFiltered[i] = dFilterAlpha * dFiltered[i] + (1.0f - dFilterAlpha) * derr;
  float D = Kd[i] * dFiltered[i];

  // PID output is an angle correction
  float u = P + integ[i] + D;

  // Apply to angle, then clamp
  angleCmd[i] += u;
  angleCmd[i] = clamp(angleCmd[i], ANGLE_MIN, ANGLE_MAX);

  prevErr[i] = err;

  // Stop integrating if we have reached the setpoint and are clamped at a limit to prevent windup
  bool atUpper = (angleCmd[i] >= ANGLE_MAX && err > 0);
  bool atLower = (angleCmd[i] <= ANGLE_MIN && err < 0);
  if (atUpper || atLower) {
    // back out the last integral increment
    integ[i] -= Ki[i] * err * dtSec;
  }
}

// ===================== Setup and loop =====================
void setup() {
  Serial.begin(115200);
  delay(200);

  // Optionally increase ADC resolution and averaging for Teensy 4.x
  analogReadResolution(12);     // 0..4095
  analogReadAveraging(8);

  for (int i = 0; i < 5; ++i) {
    servos[i].attach(servoPins[i]);
    angleCmd[i] = 0.0f;
    servos[i].write((int)angleCmd[i]);
    forceFilt[i] = 0.0f;
  }

  pidTimer = 0;
  Serial.println(F("Five-finger PID ready. Send packets like <SP=320;CMD=10110>"));
}

void loop() {
  parseSerial();

  if (pidTimer >= PID_PERIOD_US) {
    float dt = pidTimer / 1e6f;
    pidTimer = 0;

    int forceRaw[5];
    readForces(forceRaw);

    for (int i = 0; i < 5; ++i) {
      bool closeEnable = ((closeMask >> i) & 0x01) == 1;
      pidStep(i, dt, forceFilt[i], closeEnable);
      servos[i].write((int)angleCmd[i]);
    }

    // Optional telemetry at 10 Hz
    static uint16_t tCnt = 0;
    if (++tCnt >= (uint16_t)(0.1f / (PID_PERIOD_US / 1e6f))) {
      tCnt = 0;
      Serial.print(F("SP=")); Serial.print(setpoint);
      Serial.print(F(" F=["));
      for (int i = 0; i < 5; ++i) { Serial.print((int)forceFilt[i]); if (i < 4) Serial.print(','); }
      Serial.print(F("] A=["));
      for (int i = 0; i < 5; ++i) { Serial.print((int)angleCmd[i]); if (i < 4) Serial.print(','); }
      Serial.println(']');
    }
  }
}
