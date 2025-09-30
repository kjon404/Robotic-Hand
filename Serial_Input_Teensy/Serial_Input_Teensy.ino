 #include <Arduino.h>

// Use Teensy’s built-in Servo library.
// If you still have the third-party Servo library installed, this include will pick the wrong one.
#include <Servo.h>

// Servos
Servo Thumb;
Servo Index;
Servo Middle;
Servo Ring;
Servo Pinky;

// Servo signal pins {Thumb, Index, Middle, Ring, Pinky}
// Teensy 4.0 supports these as PWM outputs
const int servoPins[] = { 3, 5, 6, 9, 10 };

// Calibrated servo endpoints for each finger
const int thumbMin = 0,   thumbMax = 180;
const int indexMin = 0,   indexMax = 180;
const int middleMin = 0,  middleMax = 180;
const int ringMin  = 0,   ringMax  = 180;
const int pinkyMin = 0,   pinkyMax = 180;

// Convenience arrays aligned {Thumb, Index, Middle, Ring, Pinky}
const int minAngles[5] = { thumbMin, indexMin, middleMin, ringMin, pinkyMin };
const int maxAngles[5] = { thumbMax, indexMax, middleMax, ringMax, pinkyMax };

// Current commanded angles
int servoAngles[5];

// Forward declare
bool parseFiveBits(const String& s, int outBits[5]);
void writeAllServos();

void setup() {
  // Teensy USB serial comes up fast, but give the host a moment
  Serial.begin(9600);
  while (!Serial && millis() < 2000) { }
  Serial.setTimeout(25); // faster readStringUntil

  // Attach servos. You can add pulse ranges if needed, example:
  // Thumb.attach(servoPins[0], 1000, 2000);
  Thumb.attach(servoPins[0]);
  Index.attach(servoPins[1]);
  Middle.attach(servoPins[2]);
  Ring.attach(servoPins[3]);
  Pinky.attach(servoPins[4]);

  // Start open (use max as open, min as closed; flip if your mechanics are reversed)
  for (int i = 0; i < 5; i++) {
    servoAngles[i] = maxAngles[i];
  }
  writeAllServos();

  Serial.println("Ready. Send 5 bits like 10101 or space-separated 1 0 1 0 1");
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');   // read up to newline
    line.trim();                                  // strip spaces and CR

    int bits[5];
    if (parseFiveBits(line, bits)) {
      // bits[i] is 0 for open, 1 for close
      for (int i = 0; i < 5; i++) {
        servoAngles[i] = bits[i] ? minAngles[i] : maxAngles[i];
      }
      writeAllServos();

      // Echo
      Serial.print("Set: ");
      for (int i = 0; i < 5; i++) {
        Serial.print(bits[i]);
        if (i < 4) Serial.print(' ');
      }
      Serial.println();
    } else if (line.length() > 0) {
      Serial.println("Input error. Send 5 binary values like 10101 or 1 0 1 0 1");
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

// Parse 5 binary digits from a String
bool parseFiveBits(const String& s, int outBits[5]) {
  // Compact form "10101"
  if (s.length() == 5) {
    for (int i = 0; i < 5; i++) {
      char c = s.charAt(i);
      if (c != '0' && c != '1') return false;
      outBits[i] = (c == '1') ? 1 : 0;
    }
    return true;
  }

  // Space or comma separated form
  int count = 0;
  int idx = 0;
  while (idx < s.length() && count < 5) {
    // Skip separators
    while (idx < s.length() && (s[idx] == ' ' || s[idx] == ',' || s[idx] == '\r' || s[idx] == '\t'))
      idx++;
    if (idx >= s.length()) break;
    char c = s[idx++];
    if (c != '0' && c != '1') return false;
    outBits[count++] = (c == '1') ? 1 : 0;
  }
  return count == 5;
}
