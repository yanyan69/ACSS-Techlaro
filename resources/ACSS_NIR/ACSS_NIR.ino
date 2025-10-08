#include <Servo.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include "AS726X.h"

// === PIN ASSIGNMENTS ===
#define TB6612_AIN1 7
#define TB6612_AIN2 8
#define TB6612_PWMA 9
#define TB6612_STBY 6
#define IR_SENSOR 2
#define SERVO_PIN 5
#define LED_PIN 4
#define LED_COUNT 24

// Servo object
Servo sorterServo;

// NeoPixel object
Adafruit_NeoPixel ring(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// AS7263 object
AS726X sensor;

// IR debounce
unsigned long lastIrTime = 0;
const unsigned long IR_DEBOUNCE = 500;

// Servo safety limits (adjusted to standard range)
const int SERVO_LEFT_ANGLE = 80;  // Left position
const int SERVO_CENTER = 90;     // Neutral/center position
const int SERVO_RIGHT_ANGLE = 100; // Right position
const int SERVO_MIN_SAFE = 80;
const int SERVO_MAX_SAFE = 100;

// AS7263 LED control
unsigned long asLedOnTime = 0;
const unsigned long AS_LED_DURATION = 1000;  // 1s

// === SETUP ===
void setup() {
  Serial.begin(9600);
  Serial.setTimeout(100);  // Faster serial timeout
  Wire.begin();

  // Motor driver pins
  pinMode(TB6612_AIN1, OUTPUT);
  pinMode(TB6612_AIN2, OUTPUT);
  pinMode(TB6612_PWMA, OUTPUT);
  pinMode(TB6612_STBY, OUTPUT);

  // IR sensor
  pinMode(IR_SENSOR, INPUT);

  // Servo setup
  sorterServo.attach(SERVO_PIN, 500, 2500);
  sorterServo.write(SERVO_CENTER);

  // Enable motor driver standby
  digitalWrite(TB6612_STBY, HIGH);

  // LED ring setup
  ring.begin();
  ring.show();
  for (int i = 0; i < LED_COUNT; i++) {
    ring.setPixelColor(i, ring.Color(255, 255, 255));
  }
  ring.show();

  // AS7263 setup
  if (!sensor.begin()) {
    Serial.println("AS7263 not detected. Check wiring!");
  } else {
    sensor.enableBulb();
    sensor.setBulbCurrent(3);
    sensor.disableIndicator();  // Start with LED off
  }

  Serial.println("Arduino ready for commands.");
}

// === MOTOR CONTROL ===
void motorForward(int speed) {
  Serial.print("DEBUG: Motor starting at speed ");
  Serial.println(speed);
  digitalWrite(TB6612_AIN1, HIGH);
  digitalWrite(TB6612_AIN2, LOW);
  analogWrite(TB6612_PWMA, speed);
}

void motorStop() {
  digitalWrite(TB6612_AIN1, LOW);
  digitalWrite(TB6612_AIN2, LOW);
  analogWrite(TB6612_PWMA, 0);
}

// === SERVO SAFE WRITE ===
void safeServoWrite(int angle) {
  angle = constrain(angle, SERVO_MIN_SAFE, SERVO_MAX_SAFE);
  sorterServo.write(angle);
  Serial.print("DEBUG: Servo moved to ");
  Serial.println(angle);
}

// === LOOP ===
void loop() {
  // AS7263 LED timeout
  if (asLedOnTime > 0 && millis() - asLedOnTime > AS_LED_DURATION) {
    sensor.disableIndicator();
    asLedOnTime = 0;
  }

  // IR monitoring
  int irVal = digitalRead(IR_SENSOR);
  if (irVal == HIGH && (millis() - lastIrTime > IR_DEBOUNCE)) {
    Serial.println("IR");
    lastIrTime = millis();
  }

  // Check serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toUpperCase();

    if (command.length() == 0) return;

    Serial.print("Received: ");
    Serial.println(command);

    // SORT commands
    if (command.startsWith("SORT,")) {
      String dir = command.substring(5);
      if (dir == "L") {
        safeServoWrite(SERVO_LEFT_ANGLE);
        Serial.println("ACK");
        delay(200);  // Reduced for responsiveness
      } else if (dir == "C") {
        safeServoWrite(SERVO_CENTER);
        Serial.println("ACK");
        delay(200);
      } else if (dir == "R") {
        safeServoWrite(SERVO_RIGHT_ANGLE);
        Serial.println("ACK");
        delay(200);
      } else {
        Serial.println("Unknown sort direction.");
      }
    }
    // MOTOR commands
    else if (command == "MOTOR,ON") {
      motorForward(255);
      Serial.println("ACK");
    }
    else if (command == "MOTOR,OFF") {
      motorStop();
      Serial.println("ACK");
    }
    // AS7263 commands
    else if (command == "REQ_AS") {
      sensor.takeMeasurements();
      if (sensor.getVersion() == SENSORTYPE_AS7263) {
        Serial.print("AS:");
        Serial.print(sensor.getCalibratedR(), 2); Serial.print(",");
        Serial.print(sensor.getCalibratedS(), 2); Serial.print(",");
        Serial.print(sensor.getCalibratedT(), 2); Serial.print(",");
        Serial.print(sensor.getCalibratedU, 2); Serial.print(",");
        Serial.print(sensor.getCalibratedV(), 2); Serial.print(",");
        Serial.println(sensor.getCalibratedW(), 2);
      }
    }
    else if (command == "AS_IND_ON") {
      sensor.enableIndicator();
      sensor.setIndicatorCurrent(3);
      asLedOnTime = millis();
      Serial.println("ACK");
    }
    // BACKWARD COMPAT
    else if (command.length() == 1) {
      char cmd = command.charAt(0);
      if (cmd == 'M') {
        motorForward(255);
        Serial.println("Motor running forward.");
        delay(2000);
        motorStop();
        Serial.println("Motor stopped.");
      }
      else if (cmd == 'S') {
        Serial.println("Servo test: moving left(80°), center(90°), right(100°).");
        safeServoWrite(SERVO_LEFT_ANGLE);
        delay(1000);
        safeServoWrite(SERVO_CENTER);
        delay(1000);
        safeServoWrite(SERVO_RIGHT_ANGLE);
        delay(1000);
        safeServoWrite(SERVO_CENTER);
        Serial.println("Servo test done.");
      }
      else if (cmd == 'I') {
        int irVal = digitalRead(IR_SENSOR);
        Serial.print("IR Sensor value: ");
        Serial.println(irVal);
      }
      else if (cmd == 'A') {
        sensor.takeMeasurements();
        if (sensor.getVersion() == SENSORTYPE_AS7263) {
          Serial.print("AS:");
          Serial.print(sensor.getCalibratedR(), 2); Serial.print(",");
          Serial.print(sensor.getCalibratedS(), 2); Serial.print(",");
          Serial.print(sensor.getCalibratedT(), 2); Serial.print(",");
          Serial.print(sensor.getCalibratedU, 2); Serial.print(",");
          Serial.print(sensor.getCalibratedV(), 2); Serial.print(",");
          Serial.println(sensor.getCalibratedW(), 2);
        }
      }
      else {
        Serial.println("Unknown command.");
      }
    }
    else {
      Serial.println("Unknown command format.");
    }
  }
}