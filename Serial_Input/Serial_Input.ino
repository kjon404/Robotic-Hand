#include <Servo.h>

// Servos
Servo Thumb;
Servo Index;
Servo Middle;
Servo Ring;
Servo Pinky;

// Servo signal pins {Thumb, Index, Middle, Ring, Pinky}
const int servoPins[] = { 3, 5, 6, 9, 10 };

// Calibrated servo endpoints for each finger
const int thumbMin = 0,   thumbMax = 180;
const int indexMin = 0,   indexMax = 180;
const int middleMin = 0,  middleMax = 180;
const int ringMin = 0,    ringMax = 180;
const int pinkyMin = 0,   pinkyMax = 180;

// Convenience arrays aligned {Thumb, Index, Middle, Ring, Pinky}
const int minAngles[5] = { thumbMin, indexMin, middleMin, ringMin, pinkyMin };
const int maxAngles[5] = { thumbMax, indexMax, middleMax, ringMax, pinkyMax };

// Current commanded angles
int servoAngles[5];

void setup() {
  Serial.begin(9600);
  delay(200);

  // Attach servos
  Thumb.attach(servoPins[0]);
  Index.attach(servoPins[1]);
  Middle.attach(servoPins[2]);
  Ring.attach(servoPins[3]);
  Pinky.attach(servoPins[4]);

  // Start open (use max as open, min as closed, adjust if your mechanics are flipped)
  for (int i = 0; i < 5; i++) {
    servoAngles[i] = maxAngles[i];
  }
  writeAllServos();
}

void loop() {
  // If a full line is available, parse it
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');   // read up to newline
    line.trim();                                  // remove spaces and CR

    // Accept formats "10101" or "1 0 1 0 1"
    int bits[5];
    if (parseFiveBits(line, bits)) {
      // bits[i] is 0 for open, 1 for close
      for (int i = 0; i < 5; i++) {
        servoAngles[i] = bits[i] ? minAngles[i] : maxAngles[i];
      }
      writeAllServos();
      // Optional echo
      Serial.print("Set: ");
      for (int i = 0; i < 5; i++) { Serial.print(bits[i]); if (i < 4) Serial.print(' '); }
      Serial.println();
    } else {
      Serial.println("Input error. Send 5 binary values like 10101 or 1 0 1 0 1");
    }
  }

  delay(10); // small pacing delay
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
  // Try compact form first
  if (s.length() == 5) {
    for (int i = 0; i < 5; i++) {
      char c = s.charAt(i);
      if (c != '0' && c != '1') return false;
      outBits[i] = (c == '1') ? 1 : 0;
    }
    return true;
  }

  // Try space or comma separated form
  int count = 0;
  int idx = 0;
  while (idx < s.length() && count < 5) {
    // Skip separators
    while (idx < s.length() && (s[idx] == ' ' || s[idx] == ',' || s[idx] == '\r' || s[idx] == '\t')) idx++;
    if (idx >= s.length()) break;
    char c = s[idx++];
    if (c != '0' && c != '1') return false;
    outBits[count++] = (c == '1') ? 1 : 0;
  }
  return count == 5;
}
