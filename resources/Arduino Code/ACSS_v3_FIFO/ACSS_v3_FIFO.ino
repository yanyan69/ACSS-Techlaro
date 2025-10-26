/*
  Automated Copra Segregation - Arduino Controller (Simplified FIFO Automation)
  - Fully automated sequence: start ultrasonic → camera (stop for classification) → flapper (stop for sorting).
  - Stops at camera for CLASS_WAIT_MS to wait for RPi classification, then resumes.
  - FIFO queue tracks multiple copra items; classifications from RPi assigned to earliest item at camera.
  - Falls back to OVERCOOKED if no classification at flapper.
  - AS7263 (NIR sensor) functionality disabled.
  - Debug distance messages only printed when object detected for start/flapper, always for camera in MOVING_TO_FLAPPER.
*/

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

// =================== CONFIGURABLE CONSTANTS ===================
#define MOTOR_AIN1 7
#define MOTOR_AIN2 8
#define MOTOR_PWMA 9   // PWM pin
#define MOTOR_STBY 6

const int MOTOR_SPEED = 255;
const unsigned long TIME_TO_CAM_MS = 5000; // Increased to 5s for slower conveyors
const unsigned long TIME_TO_FLAPPER_MS = 10750; // Camera → flapper
const unsigned long CLEAR_TIME_MS = 2000; // Ensure object clears flapper
const unsigned long CLASS_WAIT_MS = 3000; // Wait at camera for classification

#define LEFT_SERVO_PIN 5
#define RIGHT_SERVO_PIN 13

const int LEFT_NEUTRAL_ANGLE = 40;
const int RIGHT_NEUTRAL_ANGLE = 40;
const int LEFT_PUSH_ANGLE = LEFT_NEUTRAL_ANGLE + 90;
const int RIGHT_PUSH_ANGLE = RIGHT_NEUTRAL_ANGLE + 90;
const unsigned long SERVO_PUSH_HOLD_MS = 1000; // Stronger push

#define START_ULTRA_TRIG 3
#define START_ULTRA_ECHO 14
const int START_DETECT_DISTANCE_CM = 8;
const unsigned long DETECT_COOLDOWN_MS = 500;
const unsigned long ULTRA_CHECK_INTERVAL_MS = 100;

#define CAM_ULTRA_TRIG 11
#define CAM_ULTRA_ECHO 15  // A1
const int CAM_DETECT_DISTANCE_CM = 10; // Increased for better detection
const unsigned long CAM_DETECT_COOLDOWN_MS = 500;
const unsigned long CAM_ULTRA_CHECK_INTERVAL_MS = 100;

#define FLAP_ULTRA_TRIG 10
#define FLAP_ULTRA_ECHO 2
const int FLAP_DETECT_DISTANCE_CM = 5;
const unsigned long FLAP_DETECT_COOLDOWN_MS = 2000; // Prevent multiple triggers
const unsigned long FLAP_ULTRA_CHECK_INTERVAL_MS = 100;

#define LED_PIN 4
#define LED_COUNT 24

const int SERVO_MIN_SAFE = 40;
const int SERVO_MAX_SAFE = 180;

// ===============================================================

Servo leftServo, rightServo;
Adafruit_NeoPixel ring(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

enum SystemState {
  IDLE,
  MOVING_TO_FLAPPER,
  AT_FLAPPER,
  AT_CAM
};
SystemState currentState = IDLE;

bool isMotorMoving = false;
unsigned long motorStartTime = 0;
unsigned long motorDuration = 0;
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

unsigned long classWaitStart = 0;

String copraClass = "";
bool autoEnabled = true;

// ===============================================================
// FIFO QUEUE
#define MAX_QUEUE 10

struct CopraItem {
  String classification;     // "", "OVERCOOKED", "RAW", "STANDARD"
  bool valid;                // True if slot used
  bool atCamNotified;        // ACK,AT_CAM sent
  unsigned long detectedAt;  // Millis at start detection
  unsigned long camArrivedAt; // Millis at camera detection
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

void enqueuePlaceholder(unsigned long detectedTime) {
  if (isQueueFull()) {
    Serial.println("ERR,FIFO_FULL_ON_DETECT");
    return;
  }
  fifoQueue[tailIndex].classification = "";
  fifoQueue[tailIndex].valid = true;
  fifoQueue[tailIndex].atCamNotified = false;
  fifoQueue[tailIndex].detectedAt = detectedTime;
  fifoQueue[tailIndex].camArrivedAt = 0;
  tailIndex = (tailIndex + 1) % MAX_QUEUE;
  queueCount++;
  Serial.print("ACK,ENQUEUE_PLACEHOLDER,idx=");
  Serial.print((tailIndex + MAX_QUEUE - 1) % MAX_QUEUE);
  Serial.print(",time=");
  Serial.println(detectedTime);

  // Extend motor duration if already moving
  if (isMotorMoving) {
    unsigned long now = millis();
    unsigned long remaining = motorDuration - (now - motorStartTime);
    if (remaining < (TIME_TO_CAM_MS + TIME_TO_FLAPPER_MS)) {
      motorDuration = now - motorStartTime + TIME_TO_CAM_MS + TIME_TO_FLAPPER_MS;
      Serial.print("ACK,MOTOR_EXTENDED,");
      Serial.println(motorDuration);
    }
  }
}

void assignClassificationToEarliestAtCam(String cls) {
  if (queueCount == 0) {
    if (!isQueueFull()) {
      fifoQueue[tailIndex].classification = cls;
      fifoQueue[tailIndex].valid = true;
      fifoQueue[tailIndex].atCamNotified = false;
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

  if (!isQueueFull()) {
    fifoQueue[tailIndex].classification = cls;
    fifoQueue[tailIndex].valid = true;
    fifoQueue[tailIndex].atCamNotified = false;
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

String dequeueItem() {
  if (queueCount == 0) return "";
  String cls = fifoQueue[headIndex].classification;
  int idxOut = headIndex;
  fifoQueue[headIndex].classification = "";
  fifoQueue[headIndex].valid = false;
  fifoQueue[headIndex].atCamNotified = false;
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

  clearQueue();
  Serial.println("Arduino ready (FIFO-enabled). AUTO mode enabled");
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
  Serial.print("ACK,MOTOR,START,");
  Serial.println(duration_ms);
}

void stopMotor() {
  digitalWrite(MOTOR_AIN1, LOW);
  digitalWrite(MOTOR_AIN2, LOW);
  analogWrite(MOTOR_PWMA, 0);
  isMotorMoving = false;
  Serial.println("ACK,MOTOR,DONE");
}

void updateMotor() {
  if (!isMotorMoving) return;
  unsigned long now = millis();
  if (now - motorStartTime >= motorDuration) {
    stopMotor();
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
  if (autoEnabled && dist > 0 && dist < START_DETECT_DISTANCE_CM && (now - lastStartDetectTime > DETECT_COOLDOWN_MS)) {
    Serial.print("DBG,START_ULTRA,DIST=");
    Serial.print(dist);
    Serial.println(",DETECTED");
    lastStartDetectTime = now;
    Serial.println("TRIG,START_OBJECT_DETECTED");
    enqueuePlaceholder(now);

    if (currentState == IDLE) {
      currentState = MOVING_TO_FLAPPER;
      startMotorMove(TIME_TO_CAM_MS + TIME_TO_FLAPPER_MS);
    }
  }
}

void checkCamUltrasonicTrigger() {
  unsigned long now = millis();
  if (now - lastCamUltraCheck < CAM_ULTRA_CHECK_INTERVAL_MS || currentState == AT_FLAPPER) return;
  lastCamUltraCheck = now;

  float dist = getDistanceCM(CAM_ULTRA_TRIG, CAM_ULTRA_ECHO);
  if (currentState == MOVING_TO_FLAPPER) {
    Serial.print("DBG,CAM_ULTRA,DIST=");
    Serial.println(dist); // Log all distances during MOVING_TO_FLAPPER
  }
  if (dist > 0 && dist < CAM_DETECT_DISTANCE_CM && (now - lastCamDetectTime > CAM_DETECT_COOLDOWN_MS)) {
    Serial.print("DBG,CAM_ULTRA,DIST=");
    Serial.print(dist);
    Serial.println(",DETECTED");
    lastCamDetectTime = now;
    Serial.println("TRIG,CAM_OBJECT_DETECTED");
    stopMotor();
    currentState = AT_CAM;
    classWaitStart = now;
    for (int i = 0; i < queueCount; i++) {
      int idx = (headIndex + i) % MAX_QUEUE;
      if (fifoQueue[idx].valid && !fifoQueue[idx].atCamNotified) {
        fifoQueue[idx].atCamNotified = true;
        fifoQueue[idx].camArrivedAt = now;
        Serial.print("ACK,AT_CAM,idx=");
        Serial.println(idx);
        break;
      }
    }
  }
}

void checkFlapUltrasonicTrigger() {
  unsigned long now = millis();
  if (now - lastFlapUltraCheck < FLAP_ULTRA_CHECK_INTERVAL_MS || currentState == IDLE || currentState == AT_CAM) return;
  lastFlapUltraCheck = now;

  float dist = getDistanceCM(FLAP_ULTRA_TRIG, FLAP_ULTRA_ECHO);
  if (dist > 0 && dist < FLAP_DETECT_DISTANCE_CM && (now - lastFlapDetectTime > FLAP_DETECT_COOLDOWN_MS)) {
    Serial.print("DBG,FLAP_ULTRA,DIST=");
    Serial.print(dist);
    Serial.println(",DETECTED");
    lastFlapDetectTime = now;
    Serial.println("TRIG,FLAP_OBJECT_DETECTED");
    stopMotor();
    currentState = AT_FLAPPER;

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
  Serial.println("AS:DISABLED");
}

// ===============================================================
// COMMAND HANDLER
void handleCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "AUTO_ENABLE") {
    autoEnabled = true;
    Serial.println("ACK,AUTO_ENABLED");
  } else if (cmd == "AUTO_DISABLE") {
    autoEnabled = false;
    stopMotor();
    currentState = IDLE;
    Serial.println("ACK,AUTO_DISABLED");
  } else if (cmd == "RESET") {
    stopMotor();
    servosNeutral();
    currentState = IDLE;
    copraClass = "";
    lastStartDetectTime = 0;
    lastCamDetectTime = 0;
    lastFlapDetectTime = 0;
    clearQueue();
    Serial.println("ACK,RESET");
  } else if (cmd == "OVERCOOKED" || cmd == "RAW" || cmd == "STANDARD") {
    Serial.print("DBG,RPI_CLASS_RECEIVED,");
    Serial.println(cmd);
    assignClassificationToEarliestAtCam(cmd);
  } else {
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
      break;

    case MOVING_TO_FLAPPER:
      if (!isMotorMoving) {
        currentState = IDLE;
        lastStartDetectTime = 0;
        Serial.println("ERR,MOTOR_TIMEOUT");
      }
      break;

    case AT_CAM:
      if (now - classWaitStart >= CLASS_WAIT_MS) {
        startMotorMove(TIME_TO_FLAPPER_MS);
        currentState = MOVING_TO_FLAPPER;
      }
      break;

    case AT_FLAPPER:
      if (!isMotorMoving && !isServoActive) {
        if (queueCount > 0) {
          currentState = MOVING_TO_FLAPPER;
          startMotorMove(TIME_TO_CAM_MS + TIME_TO_FLAPPER_MS);
          Serial.println("ACK,NEXT_ITEM_START");
        } else {
          currentState = IDLE;
        }
        lastFlapDetectTime = now;
        lastStartDetectTime = 0;
        copraClass = "";
        Serial.println("ACK,PROCESS_COMPLETE");
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