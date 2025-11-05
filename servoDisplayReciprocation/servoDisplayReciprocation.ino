#include <Servo.h>

Servo myServo;  // Create a Servo object

int angle1 = 0;     // First position
int angle2 = 45;    // Second position
int stepDelay = 5; // Delay between steps in milliseconds

void setup() {
  myServo.attach(3);   // Attach servo to pin D3
  myServo.write(angle1);
}

void loop() {
  // Move slowly from angle1 to angle2
  for (int pos = angle1; pos <= angle2; pos++) {
    myServo.write(pos);
    delay(stepDelay);
  }

  delay(1000); // Pause at end

  // Move slowly from angle2 back to angle1
  for (int pos = angle2; pos >= angle1; pos--) {
    myServo.write(pos);
    delay(stepDelay);
  }

  delay(1000); // Pause at start
}
