#include <Servo.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>   // Library for WS2812B LEDs
#include "AS726X.h"              // Use SparkFun AS726X library

// === PIN ASSIGNMENTS ===
#define TB6612_AIN1 7
#define TB6612_AIN2 8
#define TB6612_PWMA 9
#define TB6612_STBY 6

#define IR_SENSOR 2   // digital output from IR sensor
#define SERVO_PIN 5   // servo PWM pin

#define LED_PIN 4     // WS2812B data pin
#define LED_COUNT 24  // Number of LEDs on CJMCU-2812B ring

// Servo object
Servo sorterServo;

// NeoPixel object
Adafruit_NeoPixel ring(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// AS7263 object
AS726X sensor;

// IR debounce
unsigned long lastIrTime = 0;
const unsigned long IR_DEBOUNCE = 500;  // ms

// Servo safety limits (adjusted: left=80°, center=90°, right=100° for 10° deflection)
const int SERVO_LEFT_ANGLE = 65;  // Safe left position
const int SERVO_CENTER = 30;      // Neutral/center position
const int SERVO_RIGHT_ANGLE = 5; // Center + 10° (right position)
const int SERVO_MIN_SAFE = 0;    // Absolute min angle to prevent over-rotation
const int SERVO_MAX_SAFE = 70;   // Absolute max angle

// === SETUP ===
void setup() {
  // Serial for communication with RPi
  Serial.begin(9600);
  Wire.begin();

  // Motor driver pins
  pinMode(TB6612_AIN1, OUTPUT);
  pinMode(TB6612_AIN2, OUTPUT);
  pinMode(TB6612_PWMA, OUTPUT);
  pinMode(TB6612_STBY, OUTPUT);

  // IR proximity sensor
  pinMode(IR_SENSOR, INPUT);

  // Servo setup with pulse width limits for extra safety (prevents signal overdrive)
  sorterServo.attach(SERVO_PIN, 500, 2500);  // minPulse=500µs, maxPulse=2500µs (safer than defaults)
  sorterServo.write(SERVO_CENTER);  // Start at center

  // Enable motor driver standby
  digitalWrite(TB6612_STBY, HIGH);

  // LED ring setup
  ring.begin();
  ring.show(); // Initialize all pixels to 'off'
  
  // Light up LEDs by default (solid white)
  for (int i = 0; i < LED_COUNT; i++) {
    ring.setPixelColor(i, ring.Color(255, 255, 255)); // White
  }
  ring.show();

  // AS7263 setup
  if (sensor.begin() == false) {
    Serial.println("AS7263 not detected. Check wiring!");
    // No halt - continue for motor testing/integration
  } else {
    // Turn on the built-in indicator LED
    sensor.enableIndicator();
    sensor.setIndicatorCurrent(3); // 0=1mA, 1=2mA, 2=4mA, 3=8mA (max brightness)

    // Turn on the built-in illumination bulb (for measurement lighting)
    sensor.disableBulb();
    sensor.setBulbCurrent(3); // same scale
  }

  Serial.println("Arduino ready for commands.");
}

// === MOTOR CONTROL ===
void motorForward(int speed) {
  Serial.print("DEBUG: Motor starting at speed ");  // Added for verbose debugging
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

// === SERVO SAFE WRITE (NEW FUNCTION FOR LIMITS) ===
void safeServoWrite(int angle) {
  // Clamp to safe range to prevent exceeding physical limits
  angle = constrain(angle, SERVO_MIN_SAFE, SERVO_MAX_SAFE);
  sorterServo.write(angle);
  Serial.print("DEBUG: Servo moved to ");  // Debug output
  Serial.println(angle);
}

// === LOOP ===
void loop() {
  // Continuous IR monitoring
  int irVal = digitalRead(IR_SENSOR);
  if (irVal == HIGH && (millis() - lastIrTime > IR_DEBOUNCE)) {
    Serial.println("IR");
    lastIrTime = millis();
  }

  // Check if RPi sent a command
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();  // Remove any whitespace
    command.toUpperCase();  // Ensure uppercase for matching (added for robustness)

    Serial.print("DEBUG: Raw command length: ");  // Added for verbose debugging
    Serial.println(command.length());

    if (command.length() == 0) return;

    Serial.print("Received: ");
    Serial.println(command);

    // === SORT COMMANDS ===
    if (command.startsWith("SORT,")) {
      String dir = command.substring(5);
      if (dir == "L") {
        safeServoWrite(SERVO_LEFT_ANGLE);  // 80° for left
        Serial.println("ACK");
        delay(500);  // Brief settle time to avoid jitter
      } else if (dir == "C") {
        safeServoWrite(SERVO_CENTER);  // 90° for center
        Serial.println("ACK");
        delay(500);
      } else if (dir == "R") {
        safeServoWrite(SERVO_RIGHT_ANGLE);  // 100° for right
        Serial.println("ACK");
        delay(500);
      } else {
        Serial.println("Unknown sort direction.");
      }
    }

    // === MOTOR COMMANDS ===
    else if (command == "MOTOR,ON") {
      motorForward(255); // Max speed for gear motor inertia
      Serial.println("ACK");
    }
    else if (command == "MOTOR,OFF") {
      motorStop();
      Serial.println("ACK");
    }
    else if (command.indexOf("MOTOR") != -1) {  // Added for debugging partial matches
      Serial.println("DEBUG: Motor command detected but not exact match.");
    }

    // === REQ_AS: Get calibrated AS7263 values ===
    else if (command == "REQ_AS") {
      sensor.takeMeasurements();

      if (sensor.getVersion() == SENSORTYPE_AS7263) {
        Serial.print(sensor.getCalibratedR(), 2); Serial.print(",");
        Serial.print(sensor.getCalibratedS(), 2); Serial.print(",");
        Serial.print(sensor.getCalibratedT(), 2); Serial.print(",");
        Serial.print(sensor.getCalibratedU(), 2); Serial.print(",");
        Serial.print(sensor.getCalibratedV(), 2); Serial.print(",");
        Serial.println(sensor.getCalibratedW(), 2);
      }
      else if (sensor.getVersion() == SENSORTYPE_AS7262) {
        Serial.print(sensor.getCalibratedViolet(), 2); Serial.print(",");
        Serial.print(sensor.getCalibratedBlue(), 2); Serial.print(",");
        Serial.print(sensor.getCalibratedGreen(), 2); Serial.print(",");
        Serial.print(sensor.getCalibratedYellow(), 2); Serial.print(",");
        Serial.print(sensor.getCalibratedOrange(), 2); Serial.print(",");
        Serial.println(sensor.getCalibratedRed(), 2);
      }
    }

    // === BACKWARD COMPAT: SINGLE CHAR TESTS ===
    else if (command.length() == 1) {
      char cmd = command.charAt(0);

      if (cmd == 'M') { // Motor test
        motorForward(255);  // Max speed
        Serial.println("Motor running forward.");
        delay(2000);
        motorStop();
        Serial.println("Motor stopped.");
      }
      else if (cmd == 'S') { // Servo test
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
      else if (cmd == 'I') { // IR sensor test
        int irVal = digitalRead(IR_SENSOR);
        Serial.print("IR Sensor value: ");
        Serial.println(irVal);
      }
      else if (cmd == 'A') { // AS7263 quick test
        sensor.takeMeasurements();
        if (sensor.getVersion() == SENSORTYPE_AS7263) {
          Serial.print("AS7263: ");
          Serial.print(sensor.getCalibratedR(), 2); Serial.print(",");
          Serial.print(sensor.getCalibratedS(), 2); Serial.print(",");
          Serial.print(sensor.getCalibratedT(), 2); Serial.print(",");
          Serial.print(sensor.getCalibratedU(), 2); Serial.print(",");
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