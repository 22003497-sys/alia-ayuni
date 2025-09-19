# hi my first time.
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

// ================= LCD =================
LiquidCrystal_I2C lcd(0x27, 16, 2); // 16x2 LCD at I2C address 0x27

// ================= Motor Driver L298N =================
const int IN1 = 9;
const int IN2 = 8;
const int IN3 = 7;
const int IN4 = 6;
const int ENA = 10;
const int ENB = 5;

// ================= Speed Sensor =================
const int speedSensorPin = 2;
volatile unsigned int pulseCount = 0;
unsigned long prevTime = 0;
int speedRPM = 0;

// ================= Ultrasonic =================
const int trigPin = 12;
const int echoPin = 13;

// ================= Servo =================
Servo myServo;
const int servoPin = 11;

// ================= Car / Physics Params =================
const float wheelDiameter = 0.065;   // 6.5 cm wheel
const float reactionTime = 0.5;      // seconds
const float decel = 2.0;             // assumed deceleration (m/s^2)

// ================= Interrupt =================
void countPulse() {
  pulseCount++;
}

// ================= Safe Distance Formula =================
float calcSafeDistance(int rpm) {
  // Convert RPM → linear speed (m/s)
  float circumference = 3.1416 * wheelDiameter;
  float revPerSec = rpm / 60.0;
  float v = revPerSec * circumference;  // m/s

  // Safe distance formula
  float reactionDist = v * reactionTime;
  float brakingDist = (v * v) / (2 * decel);
  float safeDist = reactionDist + brakingDist;

  return safeDist * 100.0; // return in cm
}

// ================= Distance Measurement =================
long getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 20000); // timeout 20ms
  long distance = duration * 0.034 / 2; // cm
  return distance;
}

// ================= Setup =================
void setup() {
  // LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Smart Car System");

  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Ultrasonic pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Speed sensor
  pinMode(speedSensorPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(speedSensorPin), countPulse, RISING);

  // Servo
  myServo.attach(servoPin);

  // Start motors forward at medium speed
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 150);
  analogWrite(ENB, 150);

  delay(2000);
  lcd.clear();
}

// ================= Loop =================
void loop() {
  // --- Calculate RPM every 1 sec ---
  if (millis() - prevTime >= 1000) {
    detachInterrupt(digitalPinToInterrupt(speedSensorPin));
    speedRPM = pulseCount * 60; // assume 1 pulse = 1 rotation
    pulseCount = 0;
    prevTime = millis();
    attachInterrupt(digitalPinToInterrupt(speedSensorPin), countPulse, RISING);
  }

  // --- Convert RPM to km/h ---
  float circumference = 3.1416 * wheelDiameter; // m
  float revPerSec = speedRPM / 60.0;
  float v = revPerSec * circumference; // m/s
  float speedKmh = v * 3.6;

  // --- Measure distance ---
  long distance = getDistance();

  // --- Calculate safe distance ---
  float safeDist = calcSafeDistance(speedRPM);

  // --- Servo scanning ---
  for (int angle = 60; angle <= 120; angle += 30) {
    myServo.write(angle);
    delay(200);
  }
  for (int angle = 120; angle >= 60; angle -= 30) {
    myServo.write(angle);
    delay(200);
  }

  // --- Decide Safe / Not Safe ---
  String statusMsg = (distance < safeDist) ? "NOT SAFE" : "SAFE";

  // --- Display on LCD ---
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Spd:");
  lcd.print(speedKmh, 1);
  lcd.print("kmh");

  lcd.setCursor(0, 1);
  lcd.print("Dst:");
  lcd.print(distance);
  lcd.print("cm ");

  lcd.setCursor(10, 0);
  lcd.print("S:");
  lcd.print((int)safeDist);
  lcd.print("cm");

  lcd.setCursor(10, 1);
  lcd.print(statusMsg);

  delay(500);
}
