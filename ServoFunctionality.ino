#include <Servo.h>
#include <max6675.h>

// Define servo objects
Servo Thumb;
Servo Index;
Servo Middle;
Servo Ring;
Servo Pinky;

// Define ThermoCouple
#define SCK 13
#define CS 11
#define SO 12

MAX6675 thermocouple(SCK, CS, SO);

// Define pin numbers
const int potPins[] = { A0, A1, A2, A3, A4 };  // Potentiometer pins {Thumb, Index, Middle, Ring, Pinky}
const int servoPins[] = { 3, 5, 6, 9, 10 };    // Servo signal pins {Thumb, Index, Middle, Ring, Pinky}
const int ledPin = 2;                          // Red LED pin
const int buzzerPin = 4;                       // Buzzer pin (PWM)

// Calibrated Servo Angles
const int thumbMin = 0,   thumbMax = 180;
const int indexMin = 0,   indexMax = 180;
const int middleMin = 0,  middleMax = 180;
const int ringMin = 0,    ringMax = 180;
const int pinkyMin = 0,   pinkyMax = 180;

const int tempThreshold = 100;  // °C

// Arrays for readings
int potValues[5];
int servoAngles[5];

// Timing variables
unsigned long lastTempRead = 0;
const unsigned long tempInterval = 1000;  // ms
float tempC = 0;

void setup() {
  Serial.begin(9600);
  delay(500);  // Allow MAX6675 startup

  pinMode(10, OUTPUT);  // Required for SPI to stay in master mode on AVR

  // Initial thermocouple read
  tempC = thermocouple.readCelsius();
  Serial.print("Initial Temp: ");
  Serial.println(tempC);

  // Set pins
  pinMode(ledPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

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
    potValues[i] = analogRead(potPins[i]);
  }

  // Map to servo angles
  servoAngles[0] = map(potValues[0], 0, 1023, thumbMax, thumbMin);
  servoAngles[1] = map(potValues[1], 0, 1023, indexMax, indexMin);
  servoAngles[2] = map(potValues[2], 0, 1023, middleMax, middleMin);
  servoAngles[3] = map(potValues[3], 0, 1023, ringMax, ringMin);
  servoAngles[4] = map(potValues[4], 0, 1023, pinkyMax, pinkyMin);

  // Write servo positions
  Thumb.write(servoAngles[0]);
  Index.write(servoAngles[1]);
  Middle.write(servoAngles[2]);
  Ring.write(servoAngles[3]);
  Pinky.write(servoAngles[4]);

  // Read temperature every 1 second
  if (millis() - lastTempRead >= tempInterval) {
    lastTempRead = millis();
    tempC = thermocouple.readCelsius();
    float tempF = tempC * 1.8 + 32.0;
    Serial.print("Temp: ");
    Serial.println(tempF);
  }

  // Alarm logic
  if (tempC > tempThreshold) {
    digitalWrite(ledPin, HIGH);
    tone(buzzerPin, 1000);  // 1kHz tone
  } else {
    digitalWrite(ledPin, LOW);
    noTone(buzzerPin);
  }

  delay(20);  // pacing delay
}
