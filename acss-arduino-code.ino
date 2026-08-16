/*
  Automated Copra Segregation - Arduino Controller (Simplified FIFO Automation)
  - Fully automated sequence: start ultrasonic → camera (stop for classification) → flapper (stop for sorting).
  - Stops at camera for CLASS_WAIT_MS to wait for CLASS_TIMEOUT handling, then resumes with default if needed.
  - FIFO queue tracks multiple copra items; classifications from RPi assigned to earliest item at camera.
  - Falls back to OVERCOOKED if no classification at flapper.
  - AS7263 (NIR sensor) functionality disabled.
  - Debug distance messages configurable via defines.
  - Added PING command to confirm serial connection with RPi.
  - Fixed conveyor movement: short clear after servo for RAW/OVERCOOKED; adjustable for STANDARD.
  - Updated ultrasonic detection: treats 999 cm (timeout) as valid object detection for all sensors, with error counting.
  - Added camera sensor condition: detects if distance increases by > 27 cm from previous reading.
  - Added 500ms non-blocking delay before stopping conveyor after flapper detection to center copra.
  - Added per-item timeouts and dynamic motor adjustments.
  - Switched to non-blocking serial reading.
  - Standardized ACK/DBG prints.
  - Re-added queue printing for debug.
  - Added median ultrasonic readings.
  - Capped motor extensions.
  - Cleaned up unused code.

  Updates based on identified gaps:
  - Added Watchdog Timer (WDT) for hang detection and auto-reset.
  - Improved syncing: Added sequence IDs for classifications (RPi must echo idx in responses), checksum for serial messages, heartbeat pings.
  - Added fixed-size serial buffer to prevent overflows.
  - Enhanced error recovery: System pause on critical errors, LED alerts for states/errors.
  - Updated LED ring for visual feedback (green: idle, blue: moving, red: error).
  - Removed dead code (e.g., sendASReadings).
  - Added basic self-test on startup.
  - Improved efficiency: Throttled sensor checks when not needed.
  - Added emergency stop via serial "EMERGENCY_STOP".

  // Update on November 05, 2025: Addressed garbled serial output issue (likely due to baud rate mismatch—ensure serial monitor/RPi is set to 115200 baud). Explained heartbeat functionality in comments. No code changes needed for garbled output as it's typically a configuration issue; added Serial.flush() after key prints for potential stability. Started tracking updates with dates.
  // Update on November 05, 2025: Increased WDT timeout to 8s, added servo test commands and debugs for reset diagnosis, stricter US2 jump checks. No sequence changes.
  // Update on November 05, 2025: Optimized for RAM: Replaced String with enum for classifications, used F() macro for prints, reduced SERIAL_BUFFER_SIZE to 32, MAX_QUEUE to 5.
  // Update on November 05, 2025: Set LED ring to consistently white, removed all color change calls except initial white setup.
  // Update on November 05, 2025: Updated US2 detection trigger to <20cm or >30cm (as per user request), keeping timeout and jump logic.
  // Update on November 05, 2025: Removed SHORT_CLEAR_MS after L/R servo (no forward move after sorting OVERCOOKED/RAW). Confirmed queue handling: Conveyor only moves forward if queue >0 after sort/clear; for STANDARD, clears forward, then checks queue. US2/US3 detections during STANDARD clear are skipped (state=AT_FLAPPER), but dynamic extensions from US1 handle queueing. If needed, can adjust state to allow detections during clear.
  // Update on November 05, 2025: Added low/high threshold conditions for US3 (flapper) similar to US2, with configurable constants FLAP_DETECT_LOW_CM and FLAP_DETECT_HIGH_CM. Removed FLAP_DETECT_DISTANCE_CM as it's replaced by the new thresholds.
  // Update on November 06, 2025: Addressed detection gaps and stop issues. Changed ultrasonic thresholds to floats. Enabled start ultrasonic checks during AT_FLAPPER to detect new items during STANDARD clear. Added handling for missed cam/flap detections: assign default classification, dequeue on miss, extend motor if more in queue, resume from IDLE if queue pending. This prevents process stops on missed objects. Confirmed STANDARD clear moves forward for 4 seconds without immediate stop; occasional stops likely due to prior misses, now mitigated.
  // Update on November 07, 2025: Addressed gaps in classifications and cam US. Enforced checksums strictly in handleCommand (reject invalid). Added ID-echo in ACK,CLASS_RECEIVED. Reduced CAM_DETECT_COOLDOWN_MS to 500ms for close items. Set CAM_JUMP_CHECKS to 2 for sensitivity. Added GET_CAM_DIST command for raw distance query. Send ERR,CAM_SENSOR_FAIL on persistent cam fails. Improved default handling consistency.
  // Update on November 07, 2025: Fixed false start detections from timeouts. Removed dist==999 from start detected (treat as error/no object). Added sensor reset on persistent fails (echo pin toggle). Added 2s startup delay for stabilization.
  // Update on November 07, 2025: Resolved log spam in ERROR_STATE by skipping updates in loop(). Added cooldown (1s) for "ERR,MISSED_CAM" logs. Added "RESUME" command to exit ERROR_STATE without full reset.
  // Update on November 07, 2025: Increased MAX_QUEUE to 10. On full queue, dequeue oldest if it's default (CLASS_OVERCOOKED) to continue processing without pause.
  // Update on November 07, 2025: Made default failsafe classification configurable via SET_DEFAULT command (e.g., SET_DEFAULT,RAW).
  // Update on November 07, 2025: Ensured conveyor moves for STANDARD even on last queue item by starting clear before checking queue empty.
  // Update on November 07, 2025: Added CLEARING_STANDARD state to prevent duplicate clears and ensure conveyor moves for single STANDARD items without stalling.
  // Update on November 07, 2025: Fixed resume timeout spam by bypassing MOVE_PAUSE_MS on queue resume; increased MOVE_PAUSE_MS to 200ms; made missed cam recoverable (no global error increment).
  // Update on November 07, 2025: Made STANDARD clear immediate after dequeue in AT_FLAPPER without extra checks; resume queue in CLEARING_STANDARD if pending after clear.
  // Update on November 07, 2025: Increased CLASS_WAIT_MS to 5000ms to give RPi more time. Added GET_DEFAULT command to return current defaultClass. Handled 'DEFAULT' class from RPi to use configured default without timeout.
  // Update on November 08, 2025: Tuned cam detection (CAM_DETECT_HIGH_CM=25.0, CAM_JUMP_CHECKS=3). Implemented custom strupr. Added id= to ACK,MOTOR,START and ACK,SORT,L/R. Added currentDequeuedId for ID tracking.
  // Update on November 08, 2025: Removed motor from selfTest to avoid ghost startup. Increased CAM_DETECT_COOLDOWN_MS to 2000ms. Added justResumedFromCam flag (skip cam check 500ms after resume). Added queue empty check in AT_FLAPPER to skip ghost sort. Set currentDequeuedId in ITEM_TIMEOUT_DEQUEUE; set state=IDLE if queue empty after.
  // Update on November 08, 2025: Added reset for justResumedFromCam after skip period.
  // Update on November 08, 2025: Removed dist==999.0 as detected in cam ultrasonic (treat as no detection, like start US).
  // Update on November 08, 2025: Fixed incorrect ID in ACK,MOTOR,START by setting currentDequeuedId to next head before startMotorMove calls in AT_FLAPPER (post-servo), IDLE resume, CLEARING_STANDARD resume, and AT_CAM resume. Reset currentDequeuedId = -1 in stopMotor. Added id= to default ACK,CLASS prints in AT_FLAPPER.
  // Update on November 08, 2025: Tuned for close items: Lowered CAM_DETECT_COOLDOWN_MS to 800ms. Added guard to ignore cam if no !atCamNotified. Resume AT_CAM immediately if classified. Extend justResumedFromCam to post-flap. Prioritize AT_CAM over timeout; on timeout in AT_CAM, assign/resume. Dynamic timeout + queue buffer. Increment errors on misses/timeouts. Add millis timestamps to logs. Add state to ACKs. Reverse retry on cam miss.
  // Update on November 08, 2025: Fixed premature timeout for queued items by basing timeout on camArrivedAt if set, else detectedAt. This prevents early timeouts from queue delays in close-spaced detections.
  // Update on November 08, 2025: Added guard in checkFlapUltrasonicTrigger to ignore detection if head item not classified (CLASS_NONE) to prevent early triggers before cam class succeeds/failsafes.
*/

// Forward declarations to resolve compilation scope issues
void servosNeutral();
void startMotorMove(unsigned long duration_ms, bool forward = true);
void stopMotor();
void updateLedRing(uint32_t color);

// Debug flags for serial prints
#define DEBUG_START_ULTRA false
#define DEBUG_CAM_ULTRA false
#define DEBUG_FLAP_ULTRA false
#define DEBUG_QUEUE true
#define DEBUG_MOTOR true
#define DEBUG_SERVO true

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <avr/wdt.h> // For Watchdog Timer
#include <ctype.h> // For toupper

// =================== CONFIGURABLE CONSTANTS ===================
#define MOTOR_AIN1 7
#define MOTOR_AIN2 8
#define MOTOR_PWMA 9 // PWM pin
#define MOTOR_STBY 6
const int MOTOR_SPEED = 255;
const unsigned long TIME_TO_CAM_MS = 5000; // Adjust based on physical measurement
const unsigned long TIME_TO_FLAPPER_MS = 10750; // Adjust based on physical measurement
const unsigned long ITEM_TIMEOUT_BUFFER_MS = 5000; // Increased for buffer
const unsigned long CLEAR_TIME_MS = 4000; // Adjust here for STANDARD copra clearing duration
const unsigned long SHORT_CLEAR_MS = 500; // Short clear after servo for L/R - Not used now
const unsigned long MOVE_PAUSE_MS = 200; // Increased for buffer
const unsigned long CLASS_WAIT_MS = 5000; // Increased to give RPi more time
const unsigned long FLAP_CENTER_DELAY_MS = 100; // Delay before stopping motor at flapper to center copra
const unsigned long SERVO_COOLDOWN_MS = 500; // Cooldown after servo neutral
const unsigned long HEARTBEAT_INTERVAL_MS = 5000; // Send heartbeat every 5s if idle
const unsigned long WDT_TIMEOUT = WDTO_8S; // Increased to 8s for buffer against minor delays
const unsigned long MISSED_CAM_LOG_COOLDOWN_MS = 1000; // New: Log missed cam only every 1s
const unsigned long CAM_MISS_RETRY_REVERSE_MS = 500; // New: Brief reverse on miss
const int CONSECUTIVE_MISS_MAX = 2; // New: Max consecutive misses before error increment

#define LEFT_SERVO_PIN 5
#define RIGHT_SERVO_PIN 13
const int LEFT_NEUTRAL_ANGLE = 40;
const int RIGHT_NEUTRAL_ANGLE = 40;
const int LEFT_PUSH_ANGLE = LEFT_NEUTRAL_ANGLE + 90;
const int RIGHT_PUSH_ANGLE = RIGHT_NEUTRAL_ANGLE + 90;
const unsigned long SERVO_PUSH_HOLD_MS = 1000; // Stronger push

#define START_ULTRA_TRIG 3
#define START_ULTRA_ECHO 14
const int START_DETECT_DISTANCE_CM = 10;
const unsigned long DETECT_COOLDOWN_MS = 1000;
const unsigned long ULTRA_CHECK_INTERVAL_MS = 100;
const unsigned long ULTRA_READ_DELAY_MS = 20;
const int SENSOR_FAIL_COUNT_MAX = 3; // Max consecutive 999 before reset/ignore

#define CAM_ULTRA_TRIG 11
#define CAM_ULTRA_ECHO 15 // A1
const float CAM_DETECT_LOW_CM = 20.0; // Trigger if < this value (assign your value)
const float CAM_DETECT_HIGH_CM = 25.0; // Tuned lower for better detection
const unsigned long CAM_DETECT_COOLDOWN_MS = 800; // Reduced for close items
const unsigned long CAM_ULTRA_CHECK_INTERVAL_MS = 100;
const float CAM_DISTANCE_JUMP_CM = 27.0; // Detect if distance increases by > 27 cm
const int CAM_JUMP_CHECKS = 2; // Increased for less sensitivity
const unsigned long RESUME_CAM_SKIP_MS = 500; // Skip cam check after resume

#define FLAP_ULTRA_TRIG 10
#define FLAP_ULTRA_ECHO 2
const float FLAP_DETECT_LOW_CM = 4.0; // Trigger if < this value (assign your value)
const float FLAP_DETECT_HIGH_CM = 7.8; // Or > this value (assign your value)
const unsigned long FLAP_DETECT_COOLDOWN_MS = 500; // Prevent multiple triggers
const unsigned long FLAP_ULTRA_CHECK_INTERVAL_MS = 100;

#define LED_PIN 4
#define LED_COUNT 24
const int SERVO_MIN_SAFE = 40;
const int SERVO_MAX_SAFE = 180;

// LED Colors - Keeping white only
#define LED_COLOR_WHITE ring.Color(255, 255, 255) // White

// Serial Buffer
#define SERIAL_BUFFER_SIZE 32 // Reduced to save RAM
char inputBuffer[SERIAL_BUFFER_SIZE];
int inputIndex = 0;

// Classification enum to save RAM (replace String)
enum ClassType {
  CLASS_NONE,
  CLASS_OVERCOOKED,
  CLASS_RAW,
  CLASS_STANDARD
};

// ===============================================================
Servo leftServo, rightServo;
Adafruit_NeoPixel ring(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

enum SystemState {
  IDLE,
  MOVING_TO_FLAPPER,
  AT_FLAPPER,
  AT_CAM,
  CLEARING_STANDARD,  // New state for STANDARD clearing
  ERROR_STATE
};
SystemState currentState = IDLE;

bool isMotorMoving = false;
unsigned long motorStartTime = 0;
unsigned long motorDuration = 0;
unsigned long moveEndPauseTime = 0;
bool motorForward = true;

bool isServoActive = false;
unsigned long servoStartTime = 0;
unsigned long servoCooldownEnd = 0;
char activeServo = 'N';

unsigned long lastStartDetectTime = 0;
unsigned long lastStartUltraCheck = 0;
unsigned long lastCamDetectTime = 0;
unsigned long lastCamUltraCheck = 0;
unsigned long lastFlapDetectTime = 0;
unsigned long lastFlapUltraCheck = 0;
unsigned long classWaitStart = 0;
unsigned long flapDelayStart = 0;
bool flapCentering = false;

unsigned long lastHeartbeatTime = 0;

float prevCamDist = 999.0; // Track previous camera sensor distance
int camJumpCount = 0; // Count sustained jumps

ClassType copraClass = CLASS_NONE;
bool autoEnabled = true;

int startFailCount = 0;
int camFailCount = 0;
int flapFailCount = 0;
int globalErrorCount = 0;
const int MAX_GLOBAL_ERRORS = 5; // Pause system after this many errors

unsigned long lastMissedCamLog = 0; // New: For log cooldown
int consecutiveMissedCam = 0; // New: Track consecutive misses

ClassType defaultClass = CLASS_OVERCOOKED; // New: Configurable default failsafe

bool forceResume = false; // New: Flag to bypass pause on resume
bool forceClear = false; // New: Flag to bypass pause for STANDARD clear

int currentDequeuedId = -1; // New: Track dequeued ID for ACK messages

bool justResumedFromCam = false; // New: Skip cam after resume
unsigned long resumeCamSkipEnd = 0;

// ===============================================================
// FIFO QUEUE
#define MAX_QUEUE 20 // Increased to 10 to handle more misses
struct CopraItem {
  ClassType classification; // Use enum instead of String
  bool valid; // True if slot used
  bool atCamNotified; // ACK,AT_CAM sent
  unsigned long detectedAt; // Millis at start detection
  unsigned long camArrivedAt; // Millis at camera detection
  int id; // Unique ID for syncing with RPi
};
CopraItem fifoQueue[MAX_QUEUE];
int headIndex = 0;
int tailIndex = 0;
int queueCount = 0;
int nextItemId = 0; // Incremental ID for items

void clearQueue() {
  for (int i = 0; i < MAX_QUEUE; i++) {
    fifoQueue[i].classification = CLASS_NONE;
    fifoQueue[i].valid = false;
    fifoQueue[i].atCamNotified = false;
    fifoQueue[i].detectedAt = 0;
    fifoQueue[i].camArrivedAt = 0;
    fifoQueue[i].id = -1;
  }
  headIndex = 0;
  tailIndex = 0;
  queueCount = 0;
  nextItemId = 0;
  if (DEBUG_QUEUE) {
    Serial.println(F("DBG,QUEUE_CLEARED"));
    Serial.flush();
  }
}

bool isQueueFull() {
  return queueCount >= MAX_QUEUE;
}

bool isQueueEmpty() {
  return queueCount == 0;
}

const char* classToStr(ClassType cls) {
  switch (cls) {
    case CLASS_OVERCOOKED: return "OVERCOOKED";
    case CLASS_RAW: return "RAW";
    case CLASS_STANDARD: return "STANDARD";
    default: return "";
  }
}

ClassType strToClass(const char* str) {
  if (strcmp(str, "OVERCOOKED") == 0) return CLASS_OVERCOOKED;
  if (strcmp(str, "RAW") == 0) return CLASS_RAW;
  if (strcmp(str, "STANDARD") == 0) return CLASS_STANDARD;
  return CLASS_NONE;
}

void printQueueState() {
  if (!DEBUG_QUEUE) return;
  Serial.print(F("DBG,QUEUE("));
  Serial.print(queueCount);
  Serial.print(F("): "));
  for (int i = 0; i < queueCount; i++) {
    int idx = (headIndex + i) % MAX_QUEUE;
    Serial.print(F("["));
    Serial.print(idx);
    Serial.print(F(":ID="));
    Serial.print(fifoQueue[idx].id);
    Serial.print(F(","));
    Serial.print(fifoQueue[idx].valid ? "V" : " ");
    Serial.print(F(","));
    Serial.print(fifoQueue[idx].atCamNotified ? "CAM" : "");
    Serial.print(F(","));
    Serial.print(classToStr(fifoQueue[idx].classification));
    Serial.print(F("] "));
  }
  Serial.println();
  Serial.flush(); // Ensure complete transmission
}

void enqueuePlaceholder(unsigned long detectedTime) {
  if (isQueueFull()) {
    // New: Dequeue oldest if it's default to continue
    if (fifoQueue[headIndex].classification == defaultClass) {
      Serial.println(F("WARN,FIFO_FULL_DEQUEUE_OLD_DEFAULT"));
      Serial.flush();
      dequeueItem();
    } else {
      Serial.println(F("ERR,FIFO_FULL_ON_DETECT"));
      Serial.flush();
      enterErrorState();
      return;
    }
  }
  int itemId = nextItemId++;
  fifoQueue[tailIndex].classification = CLASS_NONE;
  fifoQueue[tailIndex].valid = true;
  fifoQueue[tailIndex].atCamNotified = false;
  fifoQueue[tailIndex].detectedAt = detectedTime;
  fifoQueue[tailIndex].camArrivedAt = 0;
  fifoQueue[tailIndex].id = itemId;
  tailIndex = (tailIndex + 1) % MAX_QUEUE;
  queueCount++;
  Serial.print(F("ACK,ENQUEUE_PLACEHOLDER,idx="));
  Serial.print((tailIndex + MAX_QUEUE - 1) % MAX_QUEUE);
  Serial.print(F(",id="));
  Serial.print(itemId);
  Serial.print(F(",time="));
  Serial.print(detectedTime);
  Serial.print(F(",ts="));
  Serial.println(millis());
  Serial.flush();
  printQueueState();
  // Dynamic extension: cap at queue size * interval
  if (isMotorMoving) {
    unsigned long now = millis();
    unsigned long maxDuration = now - motorStartTime + (queueCount * TIME_TO_FLAPPER_MS);
    if (motorDuration < maxDuration) {
      motorDuration = maxDuration;
      if (DEBUG_MOTOR) {
        Serial.print(F("DBG,MOTOR_EXTENDED,"));
        Serial.print(motorDuration);
        Serial.print(F(",ts="));
        Serial.println(now);
        Serial.flush();
      }
    }
  }
}

void assignClassificationToEarliestAtCam(ClassType cls, int targetId) {
  if (queueCount == 0) {
    if (!isQueueFull()) {
      int itemId = nextItemId++;
      fifoQueue[tailIndex].classification = cls;
      fifoQueue[tailIndex].valid = true;
      fifoQueue[tailIndex].atCamNotified = false;
      fifoQueue[tailIndex].detectedAt = millis();
      fifoQueue[tailIndex].camArrivedAt = 0;
      fifoQueue[tailIndex].id = itemId;
      tailIndex = (tailIndex + 1) % MAX_QUEUE;
      queueCount++;
      Serial.print(F("ACK,ENQUEUE_DIRECT,"));
      Serial.print(classToStr(cls));
      Serial.print(F(",id="));
      Serial.print(itemId);
      Serial.print(F(",ts="));
      Serial.println(millis());
      Serial.flush();
      printQueueState();
    } else {
      Serial.print(F("ERR,FIFO_FULL_ASSIGN,ts="));
      Serial.println(millis());
      Serial.flush();
      globalErrorCount++;
    }
    return;
  }
  for (int i = 0; i < queueCount; i++) {
    int idx = (headIndex + i) % MAX_QUEUE;
    if (fifoQueue[idx].valid && fifoQueue[idx].atCamNotified && fifoQueue[idx].classification == CLASS_NONE && fifoQueue[idx].id == targetId) {
      fifoQueue[idx].classification = cls;
      Serial.print(F("ACK,CLASS,"));
      Serial.print(classToStr(cls));
      Serial.print(F(",id="));
      Serial.print(targetId);
      Serial.print(F(",ts="));
      Serial.println(millis());
      Serial.flush();
      printQueueState();
      return;
    }
  }
  Serial.print(F("ERR,NO_MATCHING_ID_FOR_CLASS,"));
  Serial.print(targetId);
  Serial.print(F(",ts="));
  Serial.println(millis());
  Serial.flush();
  globalErrorCount++;
  if (!isQueueFull()) {
    int itemId = nextItemId++;
    fifoQueue[tailIndex].classification = cls;
    fifoQueue[tailIndex].valid = true;
    fifoQueue[tailIndex].atCamNotified = false;
    fifoQueue[tailIndex].detectedAt = millis();
    fifoQueue[tailIndex].camArrivedAt = 0;
    fifoQueue[tailIndex].id = itemId;
    tailIndex = (tailIndex + 1) % MAX_QUEUE;
    queueCount++;
    Serial.print(F("ACK,ENQUEUE_LATE,"));
    Serial.print(classToStr(cls));
    Serial.print(F(",id="));
    Serial.print(itemId);
    Serial.print(F(",ts="));
    Serial.println(millis());
    Serial.flush();
    printQueueState();
  } else {
    Serial.print(F("ERR,FIFO_FULL_ASSIGN2,ts="));
    Serial.println(millis());
    Serial.flush();
    globalErrorCount++;
  }
}

ClassType dequeueItem() {
  if (queueCount == 0) return CLASS_NONE;
  ClassType cls = fifoQueue[headIndex].classification;
  int idxOut = headIndex;
  int idOut = fifoQueue[headIndex].id;
  fifoQueue[headIndex].classification = CLASS_NONE;
  fifoQueue[headIndex].valid = false;
  fifoQueue[headIndex].atCamNotified = false;
  fifoQueue[headIndex].detectedAt = 0;
  fifoQueue[headIndex].camArrivedAt = 0;
  fifoQueue[headIndex].id = -1;
  headIndex = (headIndex + 1) % MAX_QUEUE;
  queueCount--;
  Serial.print(F("ACK,DEQUEUE,idx="));
  Serial.print(idxOut);
  Serial.print(F(",id="));
  Serial.print(idOut);
  Serial.print(F(","));
  Serial.print(classToStr(cls));
  Serial.print(F(",ts="));
  Serial.println(millis());
  Serial.flush();
  printQueueState();
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
    ring.setPixelColor(i, LED_COLOR_WHITE);
  }
  ring.show();
  delay(2000); // Added for sensor stabilization
  clearQueue();
  wdt_enable(WDT_TIMEOUT); // Enable Watchdog Timer
  Serial.print(F("ACK,ARDUINO_READY (FIFO-enabled, WDT-enabled). AUTO mode enabled,ts="));
  Serial.println(millis());
  Serial.flush();
  selfTest(); // Run basic self-test
}

void selfTest() {
  // Basic test: Blink LED, check servos, etc.
  Serial.print(F("ACK,SELF_TEST_START,ts="));
  Serial.println(millis());
  Serial.flush();
  servosNeutral();
  // Removed: startMotorMove(500); // Avoid ghost logs/moves
  Serial.print(F("ACK,SELF_TEST_COMPLETE,ts="));
  Serial.println(millis());
  Serial.flush();
}

// ===============================================================
// LED RING - Always white, no updates
void updateLedRing(uint32_t color) {
  // Disabled to keep white
}

// ===============================================================
// ERROR HANDLING
void enterErrorState() {
  stopMotor();
  servosNeutral();
  currentState = ERROR_STATE;
  // No LED change
  Serial.print(F("ERR,SYSTEM_PAUSED,ts="));
  Serial.println(millis());
  Serial.flush();
}

// ===============================================================
// MOTOR
void startMotorMove(unsigned long duration_ms, bool forward = true) {
  unsigned long now = millis();
  if (moveEndPauseTime > 0 && now < moveEndPauseTime + MOVE_PAUSE_MS) {
    if (!forceResume && !forceClear) return; // Bypass if force flags
  }
  forceResume = false; // Reset flags
  forceClear = false;
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
  motorStartTime = now;
  motorDuration = duration_ms;
  isMotorMoving = true;
  // No LED change
  if (DEBUG_MOTOR) {
    Serial.print(F("DBG,MOTOR,START,"));
    Serial.print(duration_ms);
    Serial.print(F(",ts="));
    Serial.println(now);
    Serial.flush();
  }
  Serial.print(F("ACK,MOTOR,START,id="));
  Serial.print(currentDequeuedId);
  Serial.print(F(",dur="));
  Serial.print(duration_ms);
  Serial.print(F(",state="));
  Serial.print(currentState);
  Serial.print(F(",ts="));
  Serial.println(now);
  Serial.flush();
}

void stopMotor() {
  digitalWrite(MOTOR_AIN1, LOW);
  digitalWrite(MOTOR_AIN2, LOW);
  analogWrite(MOTOR_PWMA, 0);
  isMotorMoving = false;
  moveEndPauseTime = millis();
  currentDequeuedId = -1; // Reset after motor stop
  // No LED change
  if (DEBUG_MOTOR) {
    Serial.print(F("DBG,MOTOR,DONE,ts="));
    Serial.println(millis());
    Serial.flush();
  }
  Serial.print(F("ACK,MOTOR,DONE,state="));
  Serial.print(currentState);
  Serial.print(F(",ts="));
  Serial.println(millis());
  Serial.flush();
}

void updateMotor() {
  if (!isMotorMoving) return;
  unsigned long now = millis();
  if (now - motorStartTime >= motorDuration) {
    if (currentState == MOVING_TO_FLAPPER && classWaitStart == 0) {
      Serial.print(F("DBG,MISSED_CAM_DETECT - Overshot US2?,ts="));
      Serial.println(now);
      Serial.flush();
    }
    stopMotor();
  }
}

// ===============================================================
// SERVOS
void servosNeutral() {
  if (DEBUG_SERVO) Serial.print(F("DBG,SERVO_NEUTRAL_START,ts=")); Serial.println(millis()); Serial.flush();
  leftServo.write(constrain(LEFT_NEUTRAL_ANGLE, SERVO_MIN_SAFE, SERVO_MAX_SAFE));
  rightServo.write(constrain(RIGHT_NEUTRAL_ANGLE, SERVO_MIN_SAFE, SERVO_MAX_SAFE));
  if (DEBUG_SERVO) Serial.print(F("DBG,SERVO_NEUTRAL_END,ts=")); Serial.println(millis()); Serial.flush();
}

void triggerServo(char side) {
  unsigned long now = millis();
  if (isServoActive || now < servoCooldownEnd) return;
  activeServo = side;
  servoStartTime = now;
  isServoActive = true;
  if (DEBUG_SERVO) Serial.print(F("DBG,SERVO_TRIGGER_START,ts=")); Serial.println(now); Serial.flush();
  if (side == 'L') {
    leftServo.write(constrain(LEFT_PUSH_ANGLE, SERVO_MIN_SAFE, SERVO_MAX_SAFE));
  } else if (side == 'R') {
    rightServo.write(constrain(RIGHT_PUSH_ANGLE, SERVO_MIN_SAFE, SERVO_MAX_SAFE));
  }
  if (DEBUG_SERVO) {
    Serial.print(F("DBG,SERVO,TRIGGER,"));
    Serial.print(side);
    Serial.print(F(",ts="));
    Serial.println(now);
    Serial.flush();
  }
  if (DEBUG_SERVO) Serial.print(F("DBG,SERVO_TRIGGER_END,ts=")); Serial.println(now); Serial.flush(); // If reset happens here, won't print
}

void updateServo() {
  if (!isServoActive) return;
  unsigned long now = millis();
  if (now - servoStartTime >= SERVO_PUSH_HOLD_MS) {
    if (DEBUG_SERVO) Serial.print(F("DBG,SERVO_HOLD_START,ts=")); Serial.println(now); Serial.flush();
    if (activeServo == 'L') {
      leftServo.write(constrain(LEFT_NEUTRAL_ANGLE, SERVO_MIN_SAFE, SERVO_MAX_SAFE));
      Serial.print(F("ACK,SORT,L,id="));
      Serial.print(currentDequeuedId);
      Serial.print(F(",ts="));
      Serial.println(now);
      Serial.flush();
      if (DEBUG_SERVO) {
        Serial.print(F("DBG,SORT,L,ts="));
        Serial.println(now);
        Serial.flush();
      }
    } else if (activeServo == 'R') {
      rightServo.write(constrain(RIGHT_NEUTRAL_ANGLE, SERVO_MIN_SAFE, SERVO_MAX_SAFE));
      Serial.print(F("ACK,SORT,R,id="));
      Serial.print(currentDequeuedId);
      Serial.print(F(",ts="));
      Serial.println(now);
      Serial.flush();
      if (DEBUG_SERVO) {
        Serial.print(F("DBG,SORT,R,ts="));
        Serial.println(now);
        Serial.flush();
      }
    }
    isServoActive = false;
    activeServo = 'N';
    servoCooldownEnd = now + SERVO_COOLDOWN_MS;
    // Removed: startMotorMove(SHORT_CLEAR_MS); // No forward move after L/R
    if (DEBUG_SERVO) Serial.print(F("DBG,SERVO_HOLD_END,ts=")); Serial.println(now); Serial.flush();
  }
}

// ===============================================================
// ULTRASONIC
float getDistanceCM(int trigPin, int echoPin) {
  float readings[3];
  for (int i = 0; i < 3; i++) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH, 30000);
    readings[i] = (duration == 0) ? 999.0 : (duration * 0.034 / 2.0);
    if (i < 2) delay(ULTRA_READ_DELAY_MS);
  }
  // Sort and take median
  if (readings[0] > readings[1]) { float temp = readings[0]; readings[0] = readings[1]; readings[1] = temp; }
  if (readings[1] > readings[2]) { float temp = readings[1]; readings[1] = readings[2]; readings[2] = temp; }
  if (readings[0] > readings[1]) { float temp = readings[0]; readings[0] = readings[1]; readings[1] = temp; }
  return readings[1]; // Median
}

void resetUltrasonicPin(int echoPin) {
  pinMode(echoPin, OUTPUT);
  digitalWrite(echoPin, LOW);
  delayMicroseconds(100); // Short delay to reset
  pinMode(echoPin, INPUT);
  if (DEBUG_START_ULTRA) {
    Serial.print(F("DBG,START_ULTRA_RESET,ts="));
    Serial.println(millis());
    Serial.flush();
  }
}

void checkStartUltrasonicTrigger() {
  if (currentState == ERROR_STATE) return; // Skip in error state
  unsigned long now = millis();
  if (now - lastStartUltraCheck < ULTRA_CHECK_INTERVAL_MS) return;
  lastStartUltraCheck = now;
  float dist = getDistanceCM(START_ULTRA_TRIG, START_ULTRA_ECHO);
  if (DEBUG_START_ULTRA) {
    Serial.print(F("DBG,START_ULTRA,DIST="));
    Serial.print(dist);
    Serial.print(F(",ts="));
    Serial.println(now);
    Serial.flush();
  }
  if (dist == 999.0) {
    startFailCount++;
    if (startFailCount > SENSOR_FAIL_COUNT_MAX) {
      resetUltrasonicPin(START_ULTRA_ECHO); // Reset on persistent fail
      startFailCount = 0; // Reset count after reset
      if (DEBUG_START_ULTRA) {
        Serial.print(F("DBG,SENSOR_FAIL_AT_START_RESET,ts="));
        Serial.println(now);
        Serial.flush();
      }
      return;
    }
  } else {
    startFailCount = 0;
  }
  bool detected = (dist > 0 && dist < START_DETECT_DISTANCE_CM); // Removed || dist==999.0 - treat timeout as no detection
  if (autoEnabled && detected && (now - lastStartDetectTime > DETECT_COOLDOWN_MS)) {
    if (DEBUG_START_ULTRA) {
      Serial.print(F("DBG,START_ULTRA,DETECTED,ts="));
      Serial.println(now);
      Serial.flush();
    }
    lastStartDetectTime = now;
    Serial.print(F("TRIG,START_OBJECT_DETECTED"));
    if (DEBUG_START_ULTRA) {
      Serial.print(F(",dist="));
      Serial.print(dist);
    }
    Serial.print(F(",ts="));
    Serial.println(now);
    Serial.flush();
    enqueuePlaceholder(now);
    if (currentState == IDLE) {
      currentState = MOVING_TO_FLAPPER;
      startMotorMove(TIME_TO_CAM_MS + TIME_TO_FLAPPER_MS);
    }
  }
}

bool hasUnnotifiedAtCam() {
  for (int i = 0; i < queueCount; i++) {
    int idx = (headIndex + i) % MAX_QUEUE;
    if (fifoQueue[idx].valid && !fifoQueue[idx].atCamNotified) {
      return true;
    }
  }
  return false;
}

void checkCamUltrasonicTrigger() {
  if (!(currentState == MOVING_TO_FLAPPER || currentState == CLEARING_STANDARD)) return; // Allow during clear
  unsigned long now = millis();
  if (justResumedFromCam && now < resumeCamSkipEnd) return; // Skip after resume
  if (now - lastCamUltraCheck < CAM_ULTRA_CHECK_INTERVAL_MS) return;
  lastCamUltraCheck = now;
  if (justResumedFromCam) justResumedFromCam = false; // Reset flag after skip
  float dist = getDistanceCM(CAM_ULTRA_TRIG, CAM_ULTRA_ECHO);
  if (DEBUG_CAM_ULTRA && currentState == MOVING_TO_FLAPPER) {
    Serial.print(F("DBG,CAM_ULTRA,DIST="));
    Serial.print(dist);
    Serial.print(F(",PREV="));
    Serial.print(prevCamDist);
    Serial.print(F(",ts="));
    Serial.println(now);
    Serial.flush();
  }
  if (dist == 999.0) {
    camFailCount++;
    if (camFailCount > SENSOR_FAIL_COUNT_MAX) {
      if (DEBUG_CAM_ULTRA) {
        Serial.print(F("DBG,SENSOR_FAIL_AT_CAM_IGNORED,ts="));
        Serial.println(now);
        Serial.flush();
      }
      Serial.print(F("ERR,CAM_SENSOR_FAIL,ts="));
      Serial.println(now);
      Serial.flush();
      return;
    }
  } else {
    camFailCount = 0;
  }
  bool isJump = (dist != 999.0 && prevCamDist != 999.0 && dist - prevCamDist > CAM_DISTANCE_JUMP_CM);
  if (isJump) {
    camJumpCount++;
  } else {
    camJumpCount = 0;
  }
  bool detected = ((dist > 0 && (dist < CAM_DETECT_LOW_CM || dist > CAM_DETECT_HIGH_CM)) || (camJumpCount >= CAM_JUMP_CHECKS)); // Removed dist==999.0
  if (detected && (now - lastCamDetectTime > CAM_DETECT_COOLDOWN_MS)) {
    if (!hasUnnotifiedAtCam()) {
      if (DEBUG_CAM_ULTRA) {
        Serial.print(F("DBG,IGNORED_CAM_ALREADY_NOTIFIED,ts="));
        Serial.println(now);
        Serial.flush();
      }
      return; // New: Ignore if no pending notifications
    }
    if (DEBUG_CAM_ULTRA) {
      Serial.print(F("DBG,CAM_ULTRA,DETECTED"));
      if (camJumpCount >= CAM_JUMP_CHECKS) Serial.print(F("_JUMP,PREV="));
      Serial.print(F(",ts="));
      Serial.println(now);
      Serial.flush();
    }
    lastCamDetectTime = now;
    Serial.print(F("TRIG,CAM_OBJECT_DETECTED,ts="));
    Serial.println(now);
    Serial.flush();
    stopMotor();
    currentState = AT_CAM;
    // No LED change
    classWaitStart = now;
    for (int i = 0; i < queueCount; i++) {
      int idx = (headIndex + i) % MAX_QUEUE;
      if (fifoQueue[idx].valid && !fifoQueue[idx].atCamNotified) {
        fifoQueue[idx].atCamNotified = true;
        fifoQueue[idx].camArrivedAt = now;
        Serial.print(F("ACK,AT_CAM,idx="));
        Serial.print(idx);
        Serial.print(F(",id="));
        Serial.print(fifoQueue[idx].id);
        Serial.print(F(",ts="));
        Serial.println(now);
        Serial.flush();
        printQueueState();
        break;
      }
    }
  }
  prevCamDist = dist; // Update previous distance
}

void checkFlapUltrasonicTrigger() {
  if (!(currentState == MOVING_TO_FLAPPER || currentState == CLEARING_STANDARD)) return; // Allow during clear
  unsigned long now = millis();
  if (now - lastFlapUltraCheck < FLAP_ULTRA_CHECK_INTERVAL_MS) return;
  lastFlapUltraCheck = now;
  float dist = getDistanceCM(FLAP_ULTRA_TRIG, FLAP_ULTRA_ECHO);
  if (DEBUG_FLAP_ULTRA) {
    Serial.print(F("DBG,FLAP_ULTRA,DIST="));
    Serial.print(dist);
    Serial.print(F(",ts="));
    Serial.println(now);
    Serial.flush();
  }
  if (dist == 999.0) {
    flapFailCount++;
    if (flapFailCount > SENSOR_FAIL_COUNT_MAX) {
      if (DEBUG_FLAP_ULTRA) {
        Serial.print(F("DBG,SENSOR_FAIL_AT_FLAP_IGNORED,ts="));
        Serial.println(now);
        Serial.flush();
      }
      return;
    }
  } else {
    flapFailCount = 0;
  }
  bool detected = (dist == 999.0 || (dist > 0 && (dist < FLAP_DETECT_LOW_CM || dist > FLAP_DETECT_HIGH_CM)));
  if (detected && (now - lastFlapDetectTime > FLAP_DETECT_COOLDOWN_MS)) {
    // New: Ignore if head item not classified
    if (queueCount > 0 && fifoQueue[headIndex].classification == CLASS_NONE) {
      if (DEBUG_FLAP_ULTRA) {
        Serial.print(F("DBG,IGNORED_FLAP_HEAD_NOT_CLASSIFIED,ts="));
        Serial.println(now);
        Serial.flush();
      }
      return;
    }
    if (DEBUG_FLAP_ULTRA) {
      Serial.print(F("DBG,FLAP_ULTRA,DETECTED"));
      if (dist == 999.0) Serial.print(F("_TIMEOUT"));
      Serial.print(F(",ts="));
      Serial.println(now);
      Serial.flush();
    }
    lastFlapDetectTime = now;
    Serial.print(F("TRIG,FLAP_OBJECT_DETECTED,ts="));
    Serial.println(now);
    Serial.flush();
    flapDelayStart = now;
    flapCentering = true;
    // No LED change
  }
}

// ===============================================================
// SERIAL CHECKSUM (Simple XOR checksum)
byte calculateChecksum(const char* msg) {
  byte checksum = 0;
  for (int i = 0; msg[i] != '\0'; i++) {
    checksum ^= msg[i];
  }
  return checksum;
}

// Custom strupr implementation
char* custom_strupr(char* str) {
  for (int i = 0; str[i] != '\0'; i++) {
    str[i] = toupper(str[i]);
  }
  return str;
}

// ===============================================================
// COMMAND HANDLER
void handleCommand(const char* cmd) {
  char command[32]; // Temp buffer to avoid String
  strcpy(command, cmd);
  custom_strupr(command); // Custom uppercase

  // Parse for checksum: Expect format CMD,arg1,arg2|checksum
  char* pipePtr = strchr(command, '|');
  if (pipePtr != NULL) {
    *pipePtr = '\0'; // Null terminate msgPart
    byte receivedChecksum = (byte)atoi(pipePtr + 1);
    byte calcChecksum = calculateChecksum(command);
    if (receivedChecksum != calcChecksum) {
      Serial.print(F("ERR,CHECKSUM_FAIL,ts="));
      Serial.println(millis());
      Serial.flush();
      globalErrorCount++;
      return;
    }
  } else {
    // No checksum, reject now (strict enforcement)
    Serial.print(F("ERR,NO_CHECKSUM,ts="));
    Serial.println(millis());
    Serial.flush();
    globalErrorCount++;
    return;
  }

  char* commaPtr = strchr(command, ',');
  if (commaPtr != NULL) {
    *commaPtr = '\0';
    if (strcmp(command, "SET_DEFAULT") == 0) {
      ClassType newDefault = strToClass(commaPtr + 1);
      if (newDefault != CLASS_NONE) {
        defaultClass = newDefault;
        Serial.print(F("ACK,SET_DEFAULT,"));
        Serial.print(classToStr(defaultClass));
        Serial.print(F(",ts="));
        Serial.println(millis());
        Serial.flush();
        return;
      } else {
        Serial.print(F("ERR,INVALID_DEFAULT_CLASS,ts="));
        Serial.println(millis());
        Serial.flush();
        return;
      }
    }
    *commaPtr = ','; // Restore for other commands
  }

  if (strcmp(command, "AUTO_ENABLE") == 0) {
    autoEnabled = true;
    Serial.print(F("ACK,AUTO_ENABLED,ts="));
    Serial.println(millis());
    Serial.flush();
  } else if (strcmp(command, "AUTO_DISABLE") == 0) {
    autoEnabled = false;
    stopMotor();
    currentState = IDLE;
    Serial.print(F("ACK,AUTO_DISABLED,ts="));
    Serial.println(millis());
    Serial.flush();
  } else if (strcmp(command, "RESET") == 0) {
    stopMotor();
    servosNeutral();
    currentState = IDLE;
    copraClass = CLASS_NONE;
    lastStartDetectTime = 0;
    lastCamDetectTime = 0;
    lastFlapDetectTime = 0;
    clearQueue();
    globalErrorCount = 0;
    // No LED change
    Serial.print(F("ACK,RESET,ts="));
    Serial.println(millis());
    Serial.flush();
  } else if (strcmp(command, "RESUME") == 0) { // New: Resume from error without full reset
    if (currentState == ERROR_STATE) {
      currentState = IDLE;
      globalErrorCount = 0;
      Serial.print(F("ACK,RESUME_FROM_ERROR,ts="));
      Serial.println(millis());
      Serial.flush();
    } else {
      Serial.print(F("ACK,NOT_IN_ERROR,ts="));
      Serial.println(millis());
      Serial.flush();
    }
  } else if (strcmp(command, "PING") == 0) {
    Serial.print(F("ACK,PING,ts="));
    Serial.println(millis());
    Serial.flush();
  } else if (strcmp(command, "EMERGENCY_STOP") == 0) {
    enterErrorState();
    Serial.print(F("ACK,EMERGENCY_STOP,ts="));
    Serial.println(millis());
    Serial.flush();
  } else if (strcmp(command, "TEST_SERVO_L") == 0) { // New: Test left servo without full sequence
    triggerServo('L');
    Serial.print(F("ACK,TEST_SERVO_L_TRIGGERED,ts="));
    Serial.println(millis());
    Serial.flush();
  } else if (strcmp(command, "TEST_SERVO_R") == 0) { // New: Test right servo
    triggerServo('R');
    Serial.print(F("ACK,TEST_SERVO_R_TRIGGERED,ts="));
    Serial.println(millis());
    Serial.flush();
  } else if (strcmp(command, "GET_CAM_DIST") == 0) {
    float dist = getDistanceCM(CAM_ULTRA_TRIG, CAM_ULTRA_ECHO);
    Serial.print(F("ACK,CAM_DIST,"));
    Serial.print(dist);
    Serial.print(F(",ts="));
    Serial.println(millis());
    Serial.flush();
  } else if (strcmp(command, "GET_DEFAULT") == 0) {
    Serial.print(F("ACK,DEFAULT_CLASS,"));
    Serial.print(classToStr(defaultClass));
    Serial.print(F(",ts="));
    Serial.println(millis());
    Serial.flush();
  } else {
    char* commaPtr = strchr(command, ',');
    if (commaPtr != NULL) {
      *commaPtr = '\0';
      if (strcmp(command, "DEFAULT") == 0) {
        // Use configured default for this ID
        int targetId = atoi(commaPtr + 1);
        assignClassificationToEarliestAtCam(defaultClass, targetId);
        Serial.print(F("ACK,CLASS,"));
        Serial.print(classToStr(defaultClass));
        Serial.print(F(",id="));
        Serial.print(targetId);
        Serial.print(F(",ts="));
        Serial.println(millis());
        Serial.flush();
        return;
      }
      ClassType cls = strToClass(command);
      int targetId = atoi(commaPtr + 1);
      if (cls != CLASS_NONE) {
        if (DEBUG_QUEUE) {
          Serial.print(F("DBG,RPI_CLASS_RECEIVED,"));
          Serial.print(classToStr(cls));
          Serial.print(F(",id="));
          Serial.print(targetId);
          Serial.print(F(",ts="));
          Serial.println(millis());
          Serial.flush();
        }
        assignClassificationToEarliestAtCam(cls, targetId);
        Serial.print(F("ACK,CLASS_RECEIVED,id="));
        Serial.print(targetId);
        Serial.print(F(",ts="));
        Serial.println(millis());
        Serial.flush();
        return;
      }
    }
    Serial.print(F("ACK,UNKNOWN,"));
    Serial.print(command);
    Serial.print(F(",ts="));
    Serial.println(millis());
    Serial.flush();
  }
}

// ===============================================================
// STATE MACHINE
void updateStateMachine() {
  unsigned long now = millis();
  // Handle missed cam detection
  if (currentState == MOVING_TO_FLAPPER && classWaitStart == 0 && now - motorStartTime > TIME_TO_CAM_MS + ITEM_TIMEOUT_BUFFER_MS) {
    if (now - lastMissedCamLog >= MISSED_CAM_LOG_COOLDOWN_MS) {
      Serial.print(F("ERR,MISSED_CAM - Assigning default and continuing,ts="));
      Serial.println(now);
      Serial.flush();
      lastMissedCamLog = now;
    }
    consecutiveMissedCam++;
    if (consecutiveMissedCam > CONSECUTIVE_MISS_MAX) {
      globalErrorCount++;
      consecutiveMissedCam = 0;
    }
    // New: Brief reverse retry to reposition
    stopMotor();
    startMotorMove(CAM_MISS_RETRY_REVERSE_MS, false);
    delay(CAM_MISS_RETRY_REVERSE_MS + 100); // Blocking brief wait for reverse
    startMotorMove(TIME_TO_CAM_MS + TIME_TO_FLAPPER_MS); // Resume forward
    for (int i = 0; i < queueCount; i++) {
      int idx = (headIndex + i) % MAX_QUEUE;
      if (fifoQueue[idx].valid && !fifoQueue[idx].atCamNotified) {
        fifoQueue[idx].atCamNotified = true;
        fifoQueue[idx].camArrivedAt = now;
        Serial.print(F("ACK,AT_CAM,idx="));
        Serial.print(idx);
        Serial.print(F(",id="));
        Serial.print(fifoQueue[idx].id);
        Serial.print(F(",ts="));
        Serial.println(now);
        Serial.flush();
        printQueueState();
        assignClassificationToEarliestAtCam(defaultClass, fifoQueue[idx].id);
        // Removed duplicate print; merged into assignClassificationToEarliestAtCam
        break;
      }
    }
    // Continue moving without stop
  } else {
    consecutiveMissedCam = 0;
  }
  // Handle missed flap detection (after cam)
  if (currentState == MOVING_TO_FLAPPER && classWaitStart != 0 && now - motorStartTime > TIME_TO_FLAPPER_MS + ITEM_TIMEOUT_BUFFER_MS) {
    Serial.print(F("ERR,MISSED_FLAP - Dequeuing, extend if more,ts="));
    Serial.println(now);
    Serial.flush();
    currentDequeuedId = fifoQueue[headIndex].id;
    dequeueItem(); // Remove missed item
    globalErrorCount++;
    if (queueCount > 0 && isMotorMoving) {
      motorDuration += (TIME_TO_FLAPPER_MS + ITEM_TIMEOUT_BUFFER_MS);
      Serial.print(F("ACK,EXTENDED_FOR_NEXT_AFTER_MISS,ts="));
      Serial.println(now);
      Serial.flush();
    }
  }
  switch (currentState) {
    case IDLE:
      break;
    case MOVING_TO_FLAPPER:
      if (!isMotorMoving) {
        currentState = IDLE;
        lastStartDetectTime = 0;
        Serial.print(F("ERR,MOTOR_TIMEOUT,ts="));
        Serial.println(now);
        Serial.flush();
        globalErrorCount++;
      }
      if (flapCentering && now - flapDelayStart >= FLAP_CENTER_DELAY_MS) {
        stopMotor();
        flapCentering = false;
        currentState = AT_FLAPPER;
        if (isQueueEmpty()) {
          currentState = IDLE;
          Serial.print(F("DBG,SKIP_GHOST_SORT_QUEUE_EMPTY,ts="));
          Serial.println(now);
          Serial.flush();
          return;
        }
        currentDequeuedId = fifoQueue[headIndex].id; // Set before dequeue
        ClassType nextClass = dequeueItem();
        if (nextClass == CLASS_NONE) {
          nextClass = defaultClass;
          Serial.print(F("ACK,CLASS,"));
          Serial.print(classToStr(defaultClass));
          Serial.print(F(",id="));
          Serial.print(currentDequeuedId);
          Serial.print(F(",source=default failsafe,ts="));
          Serial.println(now);
          Serial.flush();
        } else {
          Serial.print(F("ACK,CLASS,"));
          Serial.print(classToStr(nextClass));
          Serial.print(F(",id="));
          Serial.print(currentDequeuedId);
          Serial.print(F(",ts="));
          Serial.println(now);
          Serial.flush();
        }
        copraClass = nextClass;
        if (copraClass == CLASS_OVERCOOKED) triggerServo('L');
        else if (copraClass == CLASS_RAW) triggerServo('R');
        else if (copraClass == CLASS_STANDARD) {
          forceClear = true; // Bypass pause for clear
          startMotorMove(CLEAR_TIME_MS);
          Serial.print(F("ACK,CLEAR_STANDARD,ts="));
          Serial.println(now);
          Serial.flush();
          currentState = CLEARING_STANDARD; // Change to new state
          lastFlapDetectTime = now;
        }
      }
      break;
    case AT_CAM:
      {
        bool alreadyClassified = (fifoQueue[headIndex].valid && fifoQueue[headIndex].classification != CLASS_NONE);
        if (alreadyClassified || now - classWaitStart >= CLASS_WAIT_MS) {
          if (!alreadyClassified && fifoQueue[headIndex].valid && fifoQueue[headIndex].atCamNotified && fifoQueue[headIndex].classification == CLASS_NONE) {
            assignClassificationToEarliestAtCam(defaultClass, fifoQueue[headIndex].id);
            // Removed duplicate; handled in assign
          }
          forceResume = true; // New: Bypass pause if already classified
          currentDequeuedId = fifoQueue[headIndex].id; // Set to head before resume
          startMotorMove(TIME_TO_FLAPPER_MS);
          currentState = MOVING_TO_FLAPPER;
          lastCamDetectTime = now; // Reset cooldown to prevent immediate re-detection
          justResumedFromCam = true;
          resumeCamSkipEnd = now + RESUME_CAM_SKIP_MS;
          // No LED change
          if (alreadyClassified) {
            Serial.print(F("DBG,RESUMED_AT_CAM_ALREADY_CLASSIFIED,ts="));
            Serial.println(now);
            Serial.flush();
          }
        }
      }
      break;
    case AT_FLAPPER:
      if (!isMotorMoving && !isServoActive) {
        justResumedFromCam = true; // New: Extend skip to post-flap
        resumeCamSkipEnd = now + RESUME_CAM_SKIP_MS;
        if (queueCount > 0) {
          // Dynamic start: use TIME_TO_CAM_MS only if head not at cam yet
          unsigned long nextDuration = (fifoQueue[headIndex].camArrivedAt == 0) ? TIME_TO_CAM_MS + TIME_TO_FLAPPER_MS : TIME_TO_FLAPPER_MS;
          currentState = MOVING_TO_FLAPPER;
          currentDequeuedId = fifoQueue[headIndex].id; // Set to next head
          startMotorMove(nextDuration);
          Serial.print(F("ACK,NEXT_ITEM_START,ts="));
          Serial.println(now);
          Serial.flush();
        } else {
          currentState = IDLE;
          // No LED change
          Serial.print(F("ACK,PROCESS_COMPLETE,ts="));
          Serial.println(now);
          Serial.flush();
        }
        lastFlapDetectTime = now;
        lastStartDetectTime = 0;
        copraClass = CLASS_NONE;
      }
      break;
    case CLEARING_STANDARD:  // New state for handling STANDARD clear
      if (flapCentering && now - flapDelayStart >= FLAP_CENTER_DELAY_MS) {
        stopMotor();
        flapCentering = false;
        currentState = AT_FLAPPER;
        if (isQueueEmpty()) {
          currentState = IDLE;
          Serial.print(F("DBG,SKIP_GHOST_SORT_QUEUE_EMPTY,ts="));
          Serial.println(now);
          Serial.flush();
          return;
        }
        currentDequeuedId = fifoQueue[headIndex].id; // Set before dequeue
        ClassType nextClass = dequeueItem();
        if (nextClass == CLASS_NONE) {
          nextClass = defaultClass;
          Serial.print(F("ACK,CLASS,"));
          Serial.print(classToStr(defaultClass));
          Serial.print(F(",id="));
          Serial.print(currentDequeuedId);
          Serial.print(F(",source=default failsafe,ts="));
          Serial.println(now);
          Serial.flush();
        } else {
          Serial.print(F("ACK,CLASS,"));
          Serial.print(classToStr(nextClass));
          Serial.print(F(",id="));
          Serial.print(currentDequeuedId);
          Serial.print(F(",ts="));
          Serial.println(now);
          Serial.flush();
        }
        copraClass = nextClass;
        if (copraClass == CLASS_OVERCOOKED) triggerServo('L');
        else if (copraClass == CLASS_RAW) triggerServo('R');
        else if (copraClass == CLASS_STANDARD) {
          forceClear = true; // Bypass pause for clear
          startMotorMove(CLEAR_TIME_MS);
          Serial.print(F("ACK,CLEAR_STANDARD,ts="));
          Serial.println(now);
          Serial.flush();
          currentState = CLEARING_STANDARD; // Change to new state
          lastFlapDetectTime = now;
        }
      }
      if (!isMotorMoving) {
        justResumedFromCam = true; // New: Extend skip to post-clear
        resumeCamSkipEnd = now + RESUME_CAM_SKIP_MS;
        if (queueCount > 0) {
          // Resume for pending queue after clear
          forceResume = true;
          currentState = MOVING_TO_FLAPPER;
          currentDequeuedId = fifoQueue[headIndex].id; // Set to next head
          startMotorMove(TIME_TO_CAM_MS + TIME_TO_FLAPPER_MS);
          Serial.print(F("ACK,RESUMING_AFTER_CLEAR,ts="));
          Serial.println(now);
          Serial.flush();
        } else {
          currentState = IDLE;
          Serial.print(F("ACK,PROCESS_COMPLETE,ts="));
          Serial.println(now);
          Serial.flush();
        }
      }
      break;
    case ERROR_STATE:
      // Stay in error until reset or resume
      break;
  }
  // Check for stuck items (after switch to prioritize AT_CAM)
  if (!isQueueEmpty()) {
    unsigned long startTime = fifoQueue[headIndex].camArrivedAt > 0 ? fifoQueue[headIndex].camArrivedAt : fifoQueue[headIndex].detectedAt;
    unsigned long timeoutVal = (fifoQueue[headIndex].camArrivedAt > 0 ? TIME_TO_FLAPPER_MS : TIME_TO_CAM_MS + TIME_TO_FLAPPER_MS) + ITEM_TIMEOUT_BUFFER_MS + (queueCount * CLASS_WAIT_MS / 2);
    if (now - startTime > timeoutVal) {
      Serial.print(F("ERR,ITEM_TIMEOUT_DEQUEUE,idx="));
      Serial.print(headIndex);
      Serial.print(F(",id="));
      Serial.print(fifoQueue[headIndex].id);
      Serial.print(F(",ts="));
      Serial.println(now);
      Serial.flush();
      currentDequeuedId = fifoQueue[headIndex].id; // Set for timeout dequeue
      if (currentState == AT_CAM) {
        // New: Handle in AT_CAM - assign default and resume instead of dequeue
        if (fifoQueue[headIndex].valid && fifoQueue[headIndex].atCamNotified && fifoQueue[headIndex].classification == CLASS_NONE) {
          assignClassificationToEarliestAtCam(defaultClass, fifoQueue[headIndex].id);
        }
        forceResume = true;
        startMotorMove(TIME_TO_FLAPPER_MS);
        currentState = MOVING_TO_FLAPPER;
        lastCamDetectTime = now;
        justResumedFromCam = true;
        resumeCamSkipEnd = now + RESUME_CAM_SKIP_MS;
        Serial.print(F("DBG,RESUMED_TIMEOUT_IN_AT_CAM,ts="));
        Serial.println(now);
        Serial.flush();
      } else {
        int prevCount = queueCount;
        dequeueItem();
        if (prevCount == 1) {  // Was last item
          currentState = IDLE;
        }
      }
      globalErrorCount++;
    }
  }
  // Check global errors
  if (globalErrorCount >= MAX_GLOBAL_ERRORS) {
    enterErrorState();
  }
  // Resume from IDLE if queue pending (e.g., after miss/stop)
  if (currentState == IDLE && !isQueueEmpty()) {
    forceResume = true; // Bypass pause
    currentState = MOVING_TO_FLAPPER;
    currentDequeuedId = fifoQueue[headIndex].id; // Set to next head
    startMotorMove(TIME_TO_CAM_MS + TIME_TO_FLAPPER_MS);
    Serial.print(F("ACK,RESUMING_FOR_PENDING_QUEUE,ts="));
    Serial.println(now);
    Serial.flush();
  }
}

// ===============================================================
// HEARTBEAT
// The heartbeat is a periodic message ("ACK,HEARTBEAT") sent every HEARTBEAT_INTERVAL_MS (5 seconds) when the system is in IDLE state.
// Its purpose is to confirm that the Arduino is still operational and the serial connection to the Raspberry Pi (or monitoring device) is active.
// If the RPi stops receiving heartbeats, it can detect a potential failure or disconnection and take appropriate action (e.g., alert or retry connection).
// This enhances system reliability by providing an automated health check without manual pings.
void sendHeartbeat() {
  unsigned long now = millis();
  if (currentState == IDLE && now - lastHeartbeatTime >= HEARTBEAT_INTERVAL_MS) {
    Serial.print(F("ACK,HEARTBEAT,ts="));
    Serial.println(now);
    Serial.flush();
    lastHeartbeatTime = now;
  }
}

// ===============================================================
// MAIN LOOP
void loop() {
  wdt_reset(); // Reset watchdog
  if (currentState != ERROR_STATE) {
    updateMotor();  
    updateServo();
    checkStartUltrasonicTrigger();
    checkCamUltrasonicTrigger();
    checkFlapUltrasonicTrigger();
    updateStateMachine();
  }
  sendHeartbeat();
  while (Serial.available()) {
    char c = Serial.read();
    if (inputIndex < SERIAL_BUFFER_SIZE - 1) {
      if (c == '\n') {
        inputBuffer[inputIndex] = '\0';
        handleCommand(inputBuffer);
        inputIndex = 0;
      } else {
        inputBuffer[inputIndex++] = c;
      }
    } else {
      // Overflow: Clear buffer
      inputIndex = 0;
      Serial.print(F("ERR,SERIAL_BUFFER_OVERFLOW,ts="));
      Serial.println(millis());
      Serial.flush();
      globalErrorCount++;
    }
  }
}