/*
  Automated Copra Segregation - Arduino Controller (L298N + Ultrasonic)
  Team Techlaro - Raspberry Pi Controlled

  Components:
    - Stepper: 42BYGH48 (NEMA17) via L298N (4-wire coil control)
    - 2x MG996R Servos (LEFT and RIGHT flappers)
    - AS7263 NIR Sensor (I2C)
    - Ultrasonic Sensor (Start zone trigger)
*/

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include "AS726X.h"


// === CONFIG CONSTANTS ===

// --- Stepper Motor ---
#define STP_IN1 10
#define STP_IN2 11
#define STP_IN3 12
#define STP_IN4 2

const int STEPS_TO_CAM = 400;      // steps to reach camera zone
const int STEPS_TO_NIR = 100;      // steps to reach NIR zone
const int STEPS_TO_FLAPPER = 200;  // steps to reach flapper zone
const int STEP_DELAY_MS = 2;       // delay between steps (speed control)
const int STEP_PAUSE_MS = 10;      // pause between movements

// --- Servo Angles ---
#define LEFT_SERVO_PIN 5
#define RIGHT_SERVO_PIN 13

const int LEFT_PUSH_ANGLE = 75;     // angle for left push
const int RIGHT_PUSH_ANGLE = 105;   // angle for right push
const int LEFT_NEUTRAL_ANGLE = 0;   // neutral position
const int RIGHT_NEUTRAL_ANGLE = 0;
const int SERVO_PUSH_HOLD_MS = 200; // hold time for push

// --- LED Ring (optional) ---
#define LED_PIN 4
#define LED_COUNT 24

// --- Ultrasonic Sensor (Proximity trigger) ---
#define START_ULTRA_TRIG 3
#define START_ULTRA_ECHO 14
const int DETECT_DISTANCE_CM = 7;       // trigger distance (adjust)
const unsigned long DETECT_COOLDOWN_MS = 2500;  // debounce time before next trigger

// === GLOBAL OBJECTS ===
Servo leftServo, rightServo;
Adafruit_NeoPixel ring(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
AS726X sensor;

int stepSequence[4][4] = {
  {1, 0, 1, 0},
  {0, 1, 1, 0},
  {0, 1, 0, 1},
  {1, 0, 0, 1}
};
int currentStep = 0;
unsigned long lastDetectTime = 0;


// === INITIAL SETUP ===
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Stepper pins
  pinMode(STP_IN1, OUTPUT);
  pinMode(STP_IN2, OUTPUT);
  pinMode(STP_IN3, OUTPUT);
  pinMode(STP_IN4, OUTPUT);

  // Servo setup
  leftServo.attach(LEFT_SERVO_PIN, 500, 2500);
  rightServo.attach(RIGHT_SERVO_PIN, 500, 2500);
  servosNeutral();

  // Ultrasonic pins
  pinMode(START_ULTRA_TRIG, OUTPUT);
  pinMode(START_ULTRA_ECHO, INPUT);

  // LED setup
  ring.begin();
  ring.show();
  for (int i = 0; i < LED_COUNT; i++) ring.setPixelColor(i, ring.Color(255, 255, 255));
  ring.show();

  // Sensor
  if (!sensor.begin()) Serial.println("AS7263 not detected!");
  else Serial.println("AS7263 ready");

  Serial.println("Arduino ready: L298N Stepper + 2 Servos + AS7263 + Ultrasonic");
}


// === STEPPER CONTROL ===
void stepMotor(int stepIdx) {
  digitalWrite(STP_IN1, stepSequence[stepIdx][0]);
  digitalWrite(STP_IN2, stepSequence[stepIdx][1]);
  digitalWrite(STP_IN3, stepSequence[stepIdx][2]);
  digitalWrite(STP_IN4, stepSequence[stepIdx][3]);
}

void stepperMove(int steps, bool forward = true) {
  int dir = forward ? 1 : -1;
  for (int i = 0; i < abs(steps); i++) {
    currentStep += dir;
    if (currentStep > 3) currentStep = 0;
    if (currentStep < 0) currentStep = 3;
    stepMotor(currentStep);
    delay(STEP_DELAY_MS);
  }
  delay(STEP_PAUSE_MS);
  Serial.print("ACK,STEPPER,MOVED,");
  Serial.println(steps);
}


// === SERVO CONTROL ===
void servosNeutral() {
  leftServo.write(LEFT_NEUTRAL_ANGLE);
  rightServo.write(RIGHT_NEUTRAL_ANGLE);
}

void sortLeft() {
  leftServo.write(LEFT_PUSH_ANGLE);
  delay(SERVO_PUSH_HOLD_MS);
  leftServo.write(LEFT_NEUTRAL_ANGLE);
  Serial.println("ACK,SORT,L");
}

void sortRight() {
  rightServo.write(RIGHT_PUSH_ANGLE);
  delay(SERVO_PUSH_HOLD_MS);
  rightServo.write(RIGHT_NEUTRAL_ANGLE);
  Serial.println("ACK,SORT,R");
}


// === AS7263 SENSOR ===
void sendASReadings() {
  sensor.takeMeasurements();
  if (sensor.getVersion() == SENSORTYPE_AS7263) {
    Serial.print("AS:");
    Serial.print(sensor.getCalibratedR(), 2); Serial.print(",");
    Serial.print(sensor.getCalibratedS(), 2); Serial.print(",");
    Serial.print(sensor.getCalibratedT(), 2); Serial.print(",");
    Serial.print(sensor.getCalibratedU(), 2); Serial.print(",");
    Serial.print(sensor.getCalibratedV(), 2); Serial.print(",");
    Serial.println(sensor.getCalibratedW(), 2);
  } else {
    Serial.println("AS:ERR");
  }
}


// === ULTRASONIC READ ===
float getDistanceCM() {
  digitalWrite(START_ULTRA_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(START_ULTRA_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(START_ULTRA_TRIG, LOW);

  long duration = pulseIn(START_ULTRA_ECHO, HIGH, 20000); // timeout 20ms
  float distance = duration * 0.034 / 2; // convert to cm
  return distance;
}

void checkUltrasonicTrigger() {
  float dist = getDistanceCM();
  unsigned long now = millis();

  if (dist > 0 && dist < DETECT_DISTANCE_CM && (now - lastDetectTime > DETECT_COOLDOWN_MS)) {
    lastDetectTime = now;
    Serial.println("TRIG,OBJECT_DETECTED");
    stepperMove(STEPS_TO_CAM);
  }
}


// === SERIAL COMMANDS ===
void handleCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "STEP,TEST") stepperMove(200, true);
  else if (cmd.startsWith("STEP,FWD,")) stepperMove(cmd.substring(9).toInt(), true);
  else if (cmd.startsWith("STEP,REV,")) stepperMove(cmd.substring(9).toInt(), false);
  else if (cmd == "STEP,CAM") stepperMove(STEPS_TO_CAM);
  else if (cmd == "STEP,NIR") stepperMove(STEPS_TO_NIR);
  else if (cmd == "STEP,FLAPPER") stepperMove(STEPS_TO_FLAPPER);
  else if (cmd == "SORT,L") sortLeft();
  else if (cmd == "SORT,R") sortRight();
  else if (cmd == "SORT,NEUTRAL") { servosNeutral(); Serial.println("ACK,SORT,NEUTRAL"); }
  else if (cmd == "REQ_AS") sendASReadings();
  else Serial.print("ACK,UNKNOWN,"), Serial.println(cmd);
}


// === MAIN LOOP ===
void loop() {
  checkUltrasonicTrigger();

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    handleCommand(cmd);
  }
}