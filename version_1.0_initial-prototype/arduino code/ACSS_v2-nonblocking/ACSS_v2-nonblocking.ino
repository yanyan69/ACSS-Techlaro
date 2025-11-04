/*
  Automated Copra Segregation - Arduino Controller (FIFO improvements)
  - Adds per-item pipeline flags so multiple copra can be queued while the
    first is being processed (start ultrasonic -> camera -> NIR -> flapper).
  - Classifications from RPi are assigned to the earliest item that reached camera.
  - If an item arrives at flapper without classification it falls back to OVERCOOKED.
*/

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include "AS726X.h"

// =================== CONFIGURABLE CONSTANTS ===================
#define MOTOR_AIN1 7
#define MOTOR_AIN2 8
#define MOTOR_PWMA 9   // PWM pin
#define MOTOR_STBY 6

const int MOTOR_SPEED = 255;
const unsigned long TIME_TO_CAM_MS = 3000; // start → camera (unused now, but kept for reference)
const unsigned long TIME_TO_NIR_MS = 750;  // camera → NIR
const unsigned long TIME_TO_FLAPPER_MS = 10000; // NIR → flapper (max window)
const unsigned long CLEAR_TIME_MS = 1000;
const unsigned long MOVE_PAUSE_MS = 10;
const unsigned long NIR_READ_DELAY_MS = 1000;

#define LEFT_SERVO_PIN 5
#define RIGHT_SERVO_PIN 13

const int LEFT_NEUTRAL_ANGLE = 40;
const int RIGHT_NEUTRAL_ANGLE = 40;
const int LEFT_PUSH_ANGLE = LEFT_NEUTRAL_ANGLE + 90;
const int RIGHT_PUSH_ANGLE = RIGHT_NEUTRAL_ANGLE + 90;
const unsigned long SERVO_PUSH_HOLD_MS = 500;
const unsigned long SERVO_COOLDOWN_MS = 500;

#define START_ULTRA_TRIG 3
#define START_ULTRA_ECHO 14
const int START_DETECT_DISTANCE_CM = 7;
const unsigned long DETECT_COOLDOWN_MS = 500;
const unsigned long ULTRA_CHECK_INTERVAL_MS = 100;

#define CAM_ULTRA_TRIG 11
#define CAM_ULTRA_ECHO 15  // A1
const int CAM_DETECT_DISTANCE_CM = 7;
const unsigned long CAM_DETECT_COOLDOWN_MS = 500;
const unsigned long CAM_ULTRA_CHECK_INTERVAL_MS = 100;

#define FLAP_ULTRA_TRIG 10
#define FLAP_ULTRA_ECHO 2
const int FLAP_DETECT_DISTANCE_CM = 5;
const unsigned long FLAP_DETECT_COOLDOWN_MS = 500;
const unsigned long FLAP_ULTRA_CHECK_INTERVAL_MS = 100;

#define LED_PIN 4
#define LED_COUNT 24

const int SERVO_MIN_SAFE = 40;
const int SERVO_MAX_SAFE = 180;

// ===============================================================

Servo leftServo, rightServo;
Adafruit_NeoPixel ring(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
AS726X sensor;

enum SystemState {
  IDLE,
  MOVING_TO_FLAPPER,
  AT_FLAPPER
};
SystemState currentState = IDLE;

bool isMotorMoving = false;
unsigned long motorStartTime = 0;
unsigned long motorDuration = 0;
unsigned long moveEndPauseTime = 0;
bool motorForward = true;

bool isServoActive = false;
unsigned long servoStartTime = 0;
char activeServo = 'N';

unsigned long lastStartDetectTime = 0;
unsigned long lastStartUltraCheck = 0;
unsigned long lastCamDetectTime = 0;
unsigned long lastCamUltraCheck = 0;
unsigned long lastFlapDetectTime = 0;
unsigned long lastFlapUltraCheck = 0;

unsigned long nirReadStartTime = 0;
bool nirReadingStarted = false;

bool autoMode = true; // false = manual
String copraClass = "";
bool sentAtCam = false;

// ===============================================================
// FIFO QUEUE (per-item timing/state)
#define MAX_QUEUE 10

struct CopraItem {
  String classification;     // "", "OVERCOOKED", "RAW", "STANDARD"
  bool valid;                // true if slot used
  bool atCamNotified;        // ACK,AT_CAM sent for this item
  bool nirRequested;         // NIR read has been requested for this item
  unsigned long detectedAt;  // millis when START ultrasonic detected this item
  unsigned long camArrivedAt; // millis when CAM ultrasonic detected this item
};

CopraItem fifoQueue[MAX_QUEUE];
int headIndex = 0;
int tailIndex = 0;
int queueCount = 0;

void clearQueue() {
  for (int i = 0; i < MAX_QUEUE; i++) {
    fifoQueue[i].classification = "";
    fifoQueue[i].valid = false;
    fifoQueue[i].atCamNotified = false;
    fifoQueue[i].nirRequested = false;
    fifoQueue[i].detectedAt = 0;
    fifoQueue[i].camArrivedAt = 0;
  }
  headIndex = 0;
  tailIndex = 0;
  queueCount = 0;
}

bool isQueueFull() {
  return queueCount >= MAX_QUEUE;
}

bool isQueueEmpty() {
  return queueCount == 0;
}

void printQueueState() {
  Serial.print("QUEUE(");
  Serial.print(queueCount);
  Serial.print("): ");
  for (int i = 0; i < queueCount; i++) {
    int idx = (headIndex + i) % MAX_QUEUE;
    Serial.print("[");
    Serial.print(idx);
    Serial.print(":");
    Serial.print(fifoQueue[idx].valid ? "V" : " ");
    Serial.print(",");
    Serial.print(fifoQueue[idx].atCamNotified ? "CAM" : "");
    Serial.print(",");
    Serial.print(fifoQueue[idx].classification);
    Serial.print("] ");
  }
  Serial.println();
}

// Enqueue a placeholder when start-ultrasonic detects an object
void enqueuePlaceholder(unsigned long detectedTime) {
  if (isQueueFull()) {
    Serial.println("ERR,FIFO_FULL_ON_DETECT");
    return;
  }
  fifoQueue[tailIndex].classification = "";
  fifoQueue[tailIndex].valid = true;
  fifoQueue[tailIndex].atCamNotified = false;
  fifoQueue[tailIndex].nirRequested = false;
  fifoQueue[tailIndex].detectedAt = detectedTime;
  fifoQueue[tailIndex].camArrivedAt = 0;
  tailIndex = (tailIndex + 1) % MAX_QUEUE;
  queueCount++;
  Serial.print("ACK,ENQUEUE_PLACEHOLDER,idx=");
  Serial.print((tailIndex + MAX_QUEUE - 1) % MAX_QUEUE);
  Serial.print(",time=");
  Serial.println(detectedTime);
}

// Assign classification to earliest item that has atCamNotified==true but classification empty
void assignClassificationToEarliestAtCam(String cls) {
  if (queueCount == 0) {
    // nothing waiting — treat as simple enqueue
    if (!isQueueFull()) {
      fifoQueue[tailIndex].classification = cls;
      fifoQueue[tailIndex].valid = true;
      fifoQueue[tailIndex].atCamNotified = false;
      fifoQueue[tailIndex].nirRequested = false;
      fifoQueue[tailIndex].detectedAt = millis();
      fifoQueue[tailIndex].camArrivedAt = 0;
      tailIndex = (tailIndex + 1) % MAX_QUEUE;
      queueCount++;
      Serial.print("ACK,ENQUEUE_DIRECT,");
      Serial.println(cls);
    } else {
      Serial.println("ERR,FIFO_FULL_ASSIGN");
    }
    return;
  }

  // search from head to tail for first item that reached camera and still has empty class
  for (int i = 0; i < queueCount; i++) {
    int idx = (headIndex + i) % MAX_QUEUE;
    if (fifoQueue[idx].valid && fifoQueue[idx].atCamNotified && fifoQueue[idx].classification == "") {
      fifoQueue[idx].classification = cls;
      Serial.print("ACK,ASSIGN_CLASS,idx=");
      Serial.print(idx);
      Serial.print(",");
      Serial.println(cls);
      return;
    }
  }

  // No at-cam pending item found — append to queue as fallback
  if (!isQueueFull()) {
    fifoQueue[tailIndex].classification = cls;
    fifoQueue[tailIndex].valid = true;
    fifoQueue[tailIndex].atCamNotified = false;
    fifoQueue[tailIndex].nirRequested = false;
    fifoQueue[tailIndex].detectedAt = millis();
    fifoQueue[tailIndex].camArrivedAt = 0;
    tailIndex = (tailIndex + 1) % MAX_QUEUE;
    queueCount++;
    Serial.print("ACK,ENQUEUE_LATE,");
    Serial.println(cls);
  } else {
    Serial.println("ERR,FIFO_FULL_ASSIGN2");
  }
}

// Dequeue and return classification; returns "" if queue empty
String dequeueItem() {
  if (queueCount == 0) return "";
  String cls = fifoQueue[headIndex].classification;
  int idxOut = headIndex;
  fifoQueue[headIndex].classification = "";
  fifoQueue[headIndex].valid = false;
  fifoQueue[headIndex].atCamNotified = false;
  fifoQueue[headIndex].nirRequested = false;
  fifoQueue[headIndex].detectedAt = 0;
  fifoQueue[headIndex].camArrivedAt = 0;
  headIndex = (headIndex + 1) % MAX_QUEUE;
  queueCount--;
  Serial.print("ACK,DEQUEUE,idx=");
  Serial.print(idxOut);
  Serial.print(",");
  Serial.println(cls);
  return cls;
}

// ===============================================================
// INITIALIZATION
void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(MOTOR_AIN1, OUTPUT);
  pinMode(MOTOR_AIN2, OUTPUT);
  pinMode(MOTOR_PWMA, OUTPUT);
  pinMode(MOTOR_STBY, OUTPUT);
  digitalWrite(MOTOR_STBY, HIGH);
  digitalWrite(MOTOR_AIN1, LOW);
  digitalWrite(MOTOR_AIN2, LOW);
  analogWrite(MOTOR_PWMA, 0);

  leftServo.attach(LEFT_SERVO_PIN, 500, 2500);
  rightServo.attach(RIGHT_SERVO_PIN, 500, 2500);
  servosNeutral();

  pinMode(START_ULTRA_TRIG, OUTPUT);
  pinMode(START_ULTRA_ECHO, INPUT);
  pinMode(CAM_ULTRA_TRIG, OUTPUT);
  pinMode(CAM_ULTRA_ECHO, INPUT);
  pinMode(FLAP_ULTRA_TRIG, OUTPUT);
  pinMode(FLAP_ULTRA_ECHO, INPUT);

  ring.begin();
  ring.show();
  for (int i = 0; i < LED_COUNT; i++) {
    ring.setPixelColor(i, ring.Color(255, 255, 255));
  }
  ring.show();

  if (!sensor.begin()) {
    Serial.println("AS7263 not detected. Check wiring!");
  } else {
    sensor.enableIndicator();
    sensor.setIndicatorCurrent(3);
    sensor.disableBulb();
    sensor.setBulbCurrent(3);
    Serial.println("AS7263 ready");
  }

  clearQueue();
  Serial.println("Arduino ready (FIFO-enabled). Default: AUTO mode");
}

// ===============================================================
// MOTOR
void startMotorMove(unsigned long duration_ms, bool forward = true) {
  if (isMotorMoving) return;
  motorForward = forward;
  if (forward) {
    digitalWrite(MOTOR_AIN1, HIGH);
    digitalWrite(MOTOR_AIN2, LOW);
  } else {
    digitalWrite(MOTOR_AIN1, LOW);
    digitalWrite(MOTOR_AIN2, HIGH);
  }
  analogWrite(MOTOR_PWMA, MOTOR_SPEED);
  motorStartTime = millis();
  motorDuration = duration_ms;
  isMotorMoving = true;
  moveEndPauseTime = 0;
  Serial.print("ACK,MOTOR,START,");
  Serial.println(duration_ms);
}

void stopMotor() {
  digitalWrite(MOTOR_AIN1, LOW);
  digitalWrite(MOTOR_AIN2, LOW);
  analogWrite(MOTOR_PWMA, 0);
  isMotorMoving = false;
  moveEndPauseTime = 0;
  Serial.println("ACK,MOTOR,DONE");
}

void updateMotor() {
  if (!isMotorMoving) return;
  unsigned long now = millis();
  if (now - motorStartTime >= motorDuration && moveEndPauseTime == 0) {
    stopMotor();
  }
  if (moveEndPauseTime > 0 && now - moveEndPauseTime >= MOVE_PAUSE_MS) {
    moveEndPauseTime = 0;
  }
}

// ===============================================================
// SERVOS
void servosNeutral() {
  leftServo.write(constrain(LEFT_NEUTRAL_ANGLE, SERVO_MIN_SAFE, SERVO_MAX_SAFE));
  rightServo.write(constrain(RIGHT_NEUTRAL_ANGLE, SERVO_MIN_SAFE, SERVO_MAX_SAFE));
}

void triggerServo(char side) {
  if (isServoActive) return;
  activeServo = side;
  servoStartTime = millis();
  isServoActive = true;
  if (side == 'L') {
    leftServo.write(constrain(LEFT_PUSH_ANGLE, SERVO_MIN_SAFE, SERVO_MAX_SAFE));
  } else if (side == 'R') {
    rightServo.write(constrain(RIGHT_PUSH_ANGLE, SERVO_MIN_SAFE, SERVO_MAX_SAFE));
  }
}

void updateServo() {
  if (!isServoActive) return;
  if (millis() - servoStartTime >= SERVO_PUSH_HOLD_MS) {
    if (activeServo == 'L') {
      leftServo.write(constrain(LEFT_NEUTRAL_ANGLE, SERVO_MIN_SAFE, SERVO_MAX_SAFE));
      Serial.println("ACK,SORT,L");
    } else if (activeServo == 'R') {
      rightServo.write(constrain(RIGHT_NEUTRAL_ANGLE, SERVO_MIN_SAFE, SERVO_MAX_SAFE));
      Serial.println("ACK,SORT,R");
    }
    isServoActive = false;
    activeServo = 'N';
    startMotorMove(CLEAR_TIME_MS);
  }
}

// ===============================================================
// ULTRASONIC
float getDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 999;
  return duration * 0.034 / 2.0;
}

void checkStartUltrasonicTrigger() {
  unsigned long now = millis();
  if (now - lastStartUltraCheck < ULTRA_CHECK_INTERVAL_MS) return;
  lastStartUltraCheck = now;

  float dist = getDistanceCM(START_ULTRA_TRIG, START_ULTRA_ECHO);
  if (autoMode && dist > 0 && dist < START_DETECT_DISTANCE_CM && (now - lastStartDetectTime > DETECT_COOLDOWN_MS)) {
    lastStartDetectTime = now;
    Serial.println("TRIG,START_OBJECT_DETECTED");
    // Enqueue placeholder for this object (timestamped)
    enqueuePlaceholder(now);

    // If system idle, kick off motor journey for this object's pipeline
    if (currentState == IDLE) {
      currentState = MOVING_TO_FLAPPER;
      // motor covers camera + NIR + flapper windows (keeps running for max window)
      startMotorMove(TIME_TO_CAM_MS + TIME_TO_NIR_MS + TIME_TO_FLAPPER_MS);
    } else {
      // If already moving, we just keep the motor as-is (objects are in pipeline)
      // Optionally you could extend motorDuration if needed — keep simple for now
    }
  }
}

void checkCamUltrasonicTrigger() {
  unsigned long now = millis();
  if (now - lastCamUltraCheck < CAM_ULTRA_CHECK_INTERVAL_MS) return;
  lastCamUltraCheck = now;

  float dist = getDistanceCM(CAM_ULTRA_TRIG, CAM_ULTRA_ECHO);
  if (autoMode && dist > 0 && dist < CAM_DETECT_DISTANCE_CM && (now - lastCamDetectTime > CAM_DETECT_COOLDOWN_MS)) {
    lastCamDetectTime = now;
    Serial.println("TRIG,CAM_OBJECT_DETECTED");
    // Assign to earliest non-notified item
    for (int i = 0; i < queueCount; i++) {
      int idx = (headIndex + i) % MAX_QUEUE;
      if (fifoQueue[idx].valid && !fifoQueue[idx].atCamNotified) {
        fifoQueue[idx].atCamNotified = true;
        fifoQueue[idx].camArrivedAt = now;
        Serial.print("ACK,AT_CAM,idx=");
        Serial.println(idx);
        fifoQueue[idx].nirRequested = true;
        break; // assign to one per detection (earliest)
      }
    }
  }
}

void checkFlapUltrasonicTrigger() {
  unsigned long now = millis();
  if (now - lastFlapUltraCheck < FLAP_ULTRA_CHECK_INTERVAL_MS || currentState == IDLE) return;
  lastFlapUltraCheck = now;

  float dist = getDistanceCM(FLAP_ULTRA_TRIG, FLAP_ULTRA_ECHO);
  if (autoMode && dist > 0 && dist < FLAP_DETECT_DISTANCE_CM && (now - lastFlapDetectTime > FLAP_DETECT_COOLDOWN_MS)) {
    lastFlapDetectTime = now;
    Serial.println("TRIG,FLAP_OBJECT_DETECTED");
    stopMotor();
    currentState = AT_FLAPPER;

    // Dequeue classification for oldest item
    String nextClass = dequeueItem();
    if (nextClass == "") {
      nextClass = "OVERCOOKED";
      Serial.println("ACK,CLASS,OVERCOOKED (default failsafe)");
    } else {
      Serial.print("ACK,CLASS,");
      Serial.println(nextClass);
    }

    copraClass = nextClass;

    if (copraClass == "OVERCOOKED") triggerServo('L');
    else if (copraClass == "RAW") triggerServo('R');
    else if (copraClass == "STANDARD") {
      startMotorMove(CLEAR_TIME_MS);
      lastFlapDetectTime = now;
    }
  }
}

// ===============================================================
// AS Readings (NIR)
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
  } else if (sensor.getVersion() == SENSORTYPE_AS7262) {
    Serial.print("AS:");
    Serial.print(sensor.getCalibratedViolet(), 2); Serial.print(",");
    Serial.print(sensor.getCalibratedBlue(), 2); Serial.print(",");
    Serial.print(sensor.getCalibratedGreen(), 2); Serial.print(",");
    Serial.print(sensor.getCalibratedYellow(), 2); Serial.print(",");
    Serial.print(sensor.getCalibratedOrange(), 2); Serial.print(",");
    Serial.println(sensor.getCalibratedRed(), 2);
  } else {
    Serial.println("AS:ERR");
  }
}

// ===============================================================
// COMMAND HANDLER
void handleCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "MOTOR,TEST") {
    startMotorMove(1000, true);
  }
  else if (cmd.startsWith("MOTOR,FWD,")) {
    startMotorMove(cmd.substring(10).toInt(), true);
  }
  else if (cmd.startsWith("MOTOR,REV,")) {
    startMotorMove(cmd.substring(10).toInt(), false);
  }
  else if (cmd == "MOTOR,CAM" && (!autoMode || currentState == IDLE)) {
    currentState = MOVING_TO_FLAPPER;
    startMotorMove(TIME_TO_CAM_MS + TIME_TO_NIR_MS + TIME_TO_FLAPPER_MS);
  }
  else if (cmd.startsWith("SORT,")) {
    String dir = cmd.substring(5);
    if (dir == "L" && (!autoMode || currentState == AT_FLAPPER)) {
      triggerServo('L');
    } else if (dir == "R" && (!autoMode || currentState == AT_FLAPPER)) {
      triggerServo('R');
    } else if (dir == "C") {
      startMotorMove(CLEAR_TIME_MS);
      Serial.println("ACK,SORT,C");
    } else {
      Serial.println("Unknown sort direction.");
    }
  }
  else if (cmd == "SORT,NEUTRAL") {
    servosNeutral();
    Serial.println("ACK,SORT,NEUTRAL");
  }
  else if (cmd == "REQ_AS") {
    sendASReadings();
  }
  else if (cmd == "MODE,AUTO") {
    autoMode = true;
    Serial.println("ACK,MODE,AUTO_ENABLED");
  }
  else if (cmd == "MODE,MANUAL") {
    autoMode = false;
    currentState = IDLE;
    Serial.println("ACK,MODE,MANUAL_ENABLED");
  }
  else if (cmd == "RESET") {
    stopMotor();
    servosNeutral();
    currentState = IDLE;
    copraClass = "";
    lastStartDetectTime = 0;
    clearQueue();
    Serial.println("ACK,RESET");
  }
  // RPi sends classification strings — assign to earliest at-cam item
  else if (cmd == "OVERCOOKED" || cmd == "RAW" || cmd == "STANDARD") {
    assignClassificationToEarliestAtCam(cmd);
  }
  else if (cmd == "MOTOR,ON") {
    startMotorMove(999999, true);
    Serial.println("ACK");
  }
  else if (cmd == "MOTOR,OFF") {
    stopMotor();
    Serial.println("ACK");
  }
  else if (cmd == "AS_BULB_ON") {
    if (sensor.getVersion() == SENSORTYPE_AS7263) {
      sensor.enableBulb();
      Serial.println("ACK");
    } else {
      Serial.println("AS7263 not detected.");
    }
  }
  else if (cmd == "AS_BULB_OFF") {
    if (sensor.getVersion() == SENSORTYPE_AS7263) {
      sensor.disableBulb();
      Serial.println("ACK");
    } else {
      Serial.println("AS7263 not detected.");
    }
  }
  else if (cmd.length() == 1) {
    char ch = cmd.charAt(0);
    if (ch == 'M') {
      startMotorMove(2000, true);
      Serial.println("Motor running forward.");
    } else if (ch == 'S') {
      Serial.println("Servo test.");
      triggerServo('L');
      delay(1000 + SERVO_PUSH_HOLD_MS);
      servosNeutral();
      delay(1000);
      triggerServo('R');
      delay(1000 + SERVO_PUSH_HOLD_MS);
      servosNeutral();
      Serial.println("Servo test done.");
    } else if (ch == 'I') {
      float startDist = getDistanceCM(START_ULTRA_TRIG, START_ULTRA_ECHO);
      float camDist = getDistanceCM(CAM_ULTRA_TRIG, CAM_ULTRA_ECHO);
      float flapDist = getDistanceCM(FLAP_ULTRA_TRIG, FLAP_ULTRA_ECHO);
      Serial.print("Start Ultra: ");
      Serial.print(startDist);
      Serial.print(" cm, Cam Ultra: ");
      Serial.print(camDist);
      Serial.print(" cm, Flap Ultra: ");
      Serial.print(flapDist);
      Serial.println(" cm");
    } else if (ch == 'A') {
      sendASReadings();
    } else {
      Serial.println("Unknown command.");
    }
  }
  else {
    Serial.print("ACK,UNKNOWN,");
    Serial.println(cmd);
  }
}

// ===============================================================
// STATE MACHINE
void updateStateMachine() {
  unsigned long now = millis();

  switch (currentState) {
    case IDLE:
      // waiting
      break;

    case MOVING_TO_FLAPPER:
    {
      unsigned long elapsed = now - motorStartTime;

      // Handle NIR read window (per-item, based on cam arrival)
      for (int i = 0; i < queueCount; i++) {
        int idx = (headIndex + i) % MAX_QUEUE;
        if (fifoQueue[idx].valid && fifoQueue[idx].nirRequested && fifoQueue[idx].camArrivedAt > 0 && (now - fifoQueue[idx].camArrivedAt >= TIME_TO_NIR_MS)) {
          Serial.print("ACK,AT_NIR,idx=");
          Serial.println(idx);
          sendASReadings();
          fifoQueue[idx].nirRequested = false;
          break; // one read per cycle (assume sensor handles one at a time)
        }
      }

      if (!isMotorMoving) {
        currentState = IDLE;
        lastStartDetectTime = 0;
        Serial.println("ERR,MOTOR_TIMEOUT");
      }
    }
      break;

    case AT_FLAPPER:
      if (!isMotorMoving && !isServoActive) {
        // ✅ Modified section: Auto-continue if queue still has items
        if (queueCount > 0) {
          currentState = MOVING_TO_FLAPPER;
          startMotorMove(TIME_TO_CAM_MS + TIME_TO_NIR_MS + TIME_TO_FLAPPER_MS);
          Serial.println("ACK,NEXT_ITEM_START");
        } else {
          currentState = IDLE;
        }

        lastFlapDetectTime = now;
        lastStartDetectTime = 0;
        copraClass = "";
        Serial.println("ACK,PROCESS_COMPLETE");
        delay(200);
      }
      break;
  }
}

// ===============================================================
// MAIN LOOP
void loop() {
  updateMotor();
  updateServo();
  checkStartUltrasonicTrigger();
  checkCamUltrasonicTrigger();
  checkFlapUltrasonicTrigger();
  updateStateMachine();

  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    handleCommand(command);
  }
}