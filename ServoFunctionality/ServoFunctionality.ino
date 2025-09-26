#include <Servo.h>

// Define servo objects
Servo Thumb;
Servo Index;
Servo Middle;
Servo Ring;
Servo Pinky;

// Define pin numbers
const int ForcePins[] = { A0, A1, A2, A3, A4 };  // FSR pins {Thumb, Index, Middle, Ring, Pinky}
const int servoPins[] = { 3, 5, 6, 9, 10 };    // Servo signal pins {Thumb, Index, Middle, Ring, Pinky}

// Calibrated Servo Angles
const int thumbMin = 0,   thumbMax = 180;
const int indexMin = 0,   indexMax = 180;
const int middleMin = 0,  middleMax = 180;
const int ringMin = 0,    ringMax = 180;
const int pinkyMin = 0,   pinkyMax = 180;

// Arrays for readings
int ForceValues[5];
int servoAngles[5];

void setup() {
  Serial.begin(9600);
  delay(500);



  // Attach and initialize servos
  Thumb.attach(servoPins[0]);
  Index.attach(servoPins[1]);
  Middle.attach(servoPins[2]);
  Ring.attach(servoPins[3]);
  Pinky.attach(servoPins[4]);

  Thumb.write(thumbMax);
  Index.write(indexMax);
  Middle.write(middleMax);
  Ring.write(ringMax);
  Pinky.write(pinkyMax);
}

void loop() {
  // Read potentiometers
  for (int i = 0; i < 5; i++) {
    ForceValues[i] = analogRead(ForcePins[0]); // only reads from one force sensor for all fingers
  }

  // Map to servo angles
  servoAngles[0] = map(ForceValues[0], 0, 650, thumbMax, thumbMin);
  servoAngles[1] = map(ForceValues[1], 0, 650, indexMax, indexMin);
  servoAngles[2] = map(ForceValues[2], 0, 650, middleMax, middleMin);
  servoAngles[3] = map(ForceValues[3], 0, 650, ringMax, ringMin);
  servoAngles[4] = map(ForceValues[4], 0, 650, pinkyMax, pinkyMin);

  // Write servo positions
  Thumb.write(servoAngles[0]);
  Index.write(servoAngles[1]);
  Middle.write(servoAngles[2]);
  Ring.write(servoAngles[3]);
  Pinky.write(servoAngles[4]);

  
// pacing delay
  delay(20);
}