#include <Arduino.h>
#include <Servo.h>

// Servos
Servo Thumb, Index, Middle, Ring, Pinky;

// Servo signal pins {Thumb, Index, Middle, Ring, Pinky}
const int servoPins[] = { 9, 8, 7, 6, 5 };

// Calibrated servo endpoints for each finger
const int thumbMin = 0,   thumbMax = 180;
const int indexMin = 0,   indexMax = 180;\ 
const int middleMin = 0,  middleMax = 180;
const int ringMin  = 0,   ringMax  = 180;
const int pinkyMin = 0,   pinkyMax = 180;

// Convenience arrays aligned {Thumb, Index, Middle, Ring, Pinky}
const int minAngles[5] = { thumbMin, indexMin, middleMin, ringMin, pinkyMin };
const int maxAngles[5] = { thumbMax, indexMax, middleMax, ringMax, pinkyMax };

// Current commanded angles
int servoAngles[5];

// Forward declare
bool parseFiveFloats(const String& s, float outVals[5]);
void writeAllServos();
static inline float clamp01(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}
  Serial.setTimeout(25);

  Thumb.attach(servoPins[0]);
  Index.attach(servoPins[1]);
  Middle.attach(servoPins[2]);
  Ring.attach(servoPins[3]);
  Pinky.attach(servoPins[4]);

  // Start at each finger's max (fully open if that matches your mechanics)
  for (int i = 0; i < 5; i++) {
    servoAngles[i] = maxAngles[i];
  }
  writeAllServos();

  Serial.println("Ready. Send 5 floats in [0..1], for example:");
  Serial.println("0 0.25 0.5 0.75 1");
  Serial.println("or: 0,0.25,0.5,0.75,1");
  Serial.println("or: [0, 0.25, 0.5, 0.75, 1]");
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    float vals[5];
    if (parseFiveFloats(line, vals)) {
      for (int i = 0; i < 5; i++) {
        float v = clamp01(vals[i]); // 0 = open, 1 = closed
        float fAngle = maxAngles[i] - v * (maxAngles[i] - minAngles[i]); // open->max, closed->min
        servoAngles[i] = (int)lroundf(fAngle);
      }
      writeAllServos();

      // Echo
      Serial.print("Set angles: ");
      for (int i = 0; i < 5; i++) {
        Serial.print(servoAngles[i]);
        if (i < 4) Serial.print(' ');
      }
      Serial.print("   from inputs: ");
      for (int i = 0; i < 5; i++) {
        Serial.print(vals[i], 3);
        if (i < 4) Serial.print(' ');
      }
      Serial.println();
    } else if (line.length() > 0) {
      Serial.println("Input error. Send 5 floats in [0..1], like 0 0.5 1 0.2 0.75");
    }
  }

  delay(5);
}

// -------- Helpers --------

void writeAllServos() {
  Thumb.write(servoAngles[0]);
  Index.write(servoAngles[1]);
  Middle.write(servoAngles[2]);
  Ring.write(servoAngles[3]);
  Pinky.write(servoAngles[4]);
}

// Parse 5 floats from a String. Accepts spaces, commas, tabs, and [] as separators.
bool parseFiveFloats(const String& s, float outVals[5]) {
  if (s.length() == 0) return false;

  // Copy to a mutable C string
  char buf[160];
  size_t n = s.length();
  if (n >= sizeof(buf)) n = sizeof(buf) - 1;
  s.substring(0, n).toCharArray(buf, n + 1);

  const char* delims = " ,\t\r\n[]";
  int count = 0;
  for (char* tok = strtok(buf, delims); tok != nullptr && count < 5; tok = strtok(nullptr, delims)) {
    // Handle possible trailing commas or non numeric tokens gracefully
    char* endp = nullptr;
    float v = strtof(tok, &endp);
    if (endp == tok) return false; // not a number
    outVals[count++] = v;
  }
  return count == 5;
}
