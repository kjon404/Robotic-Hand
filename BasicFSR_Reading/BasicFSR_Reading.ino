#include <Servo.h>

// FSR voltage divider on A0 with 10k resistor to GND
// Reads analog data and maps it to a servo angle

const int fsrPin = A0;     // FSR connected to analog pin A0
const int servoPin = 9;    // Servo control pin

int fsrReading = 0;        // Variable to store analog value
float ratio = 0;           // Ratio for scaling
int servoAngle = 0;        // Servo angle

Servo myServo;

void setup() {
  Serial.begin(9600);      // Initialize serial communication
  myServo.attach(servoPin);
}

void loop() {
  fsrReading = analogRead(fsrPin);   // Read FSR value (0-1023)

  // Calculate ratio relative to reference
  ratio = (float)fsrReading / 650.0;

  // Map FSR reading to servo angle (0–1023 mapped to 0–180°)
  servoAngle = map(fsrReading, 0, 1023, 0, 180);

  // Clamp to 0–180 range (safety check)
  servoAngle = constrain(servoAngle, 0, 180);

  myServo.write(servoAngle);

  // Print readings
  Serial.print("FSR Reading: ");
  Serial.print(fsrReading);
  Serial.print("\t Ratio: ");
  Serial.print(ratio, 2);
  Serial.print("\t Servo Angle: ");
  Serial.println(servoAngle);

  delay(100);  // Small delay for readability
}
