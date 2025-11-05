#include <Servo.h>

Servo myServo;  // Create a Servo object
int angle = 0;

void setup() {
  myServo.attach(3);   // Attach servo to digital pin D3
  myServo.write(angle);
}

void loop() {
  myServo.write(angle);   // Move servo to the new position
  delay(2000);            // Wait 2 seconds
}
