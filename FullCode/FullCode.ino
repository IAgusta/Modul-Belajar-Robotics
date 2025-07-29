/*  ESP32 Robot – non-blocking refactor
 *  Original author: Ei
 *  Created : 2025
 *  Github : https://github.com/IAgusta/Modul-Belajar-Robotics
 */

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include <SPIFFS.h>
#include <Preferences.h>
#include <DFPlayerMini_Fast.h>

/*
=============== Pin Configuration ==================
*/
// DFPlayer Pin Configuration
#define DF_RX 1  // DFPlayer TX to ESP32 RX (input only)
#define DF_TX 3  // DFPlayer RX to ESP32 TX
// Bateries Capacities Program, using divider
const int voltagePin = 35;
// BFD-1000 Sensor Pin Mapping
const int irLeft2  = 18; // S1 GPIO pin
const int irLeft1  = 5; // S2 GPIO pin
const int irCenter = 17; // S3 GPIO pin
const int irRight1 = 16; // S4 GPIO pin
const int irRight2 = 4; // S5 GPIO pin
// Motor Driver Pin and Other Adjustment
// Motor 1
const int motor1Pin1 = 27;
const int motor1Pin2 = 26;
const int enable1Pin = 14;
// Motor 2
const int motor2Pin1 = 33;
const int motor2Pin2 = 25;
const int enable2Pin = 32;
// Setting PWM properties
const int freq = 30000;
const int resolution = 8;
// Ultrasonics Adjustment
const int TrigPin = 2;
const int EchoPin = 19;
// LED channels
const int redLed = 0;
const int greenLed = 12;
const int blueLed  = 23;
const int servoPin = 13; // GPIO Servo, change if needed
// OLED with U8g2 (SSD1306 128x64 I2C)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0);
// 16x2 I²C LCD (change address if needed)
LiquidCrystal_I2C lcd(0x27, 16, 2);

/*
==================== NET Handlers ======================
*/
const char* url = "https://alpaca-labs.online"; // Website Url Change if the website domain changes
const char* apiKey = "XXXXXXXXXXXXXXXXXXXXXXXXX"; // Change this based on your API Key
// Dont change this path url
String getUrl = String(url) + "/api/robot/command/" + apiKey;
String postUrl = String(url) + "/api/robot/command-status/" + apiKey;
String sensorUrl = String(url) + "/api/robot/sensor-data";
// Adjusment For API fetch from Database
unsigned long lastFetch = 0;
const unsigned long fetchInterval = 2000;

/*
=================== WIFI Handlers =====================
*/
#define MAX_NETWORKS 3 // Change the Max Network u want to save manually
#define WIFI_TIMEOUT_MS 10000  // 10 seconds timeout for each try
// Variable for WiFi Connection
unsigned long lastWiFiCheck = 0;
const unsigned long wifiCheckInterval = 10000; // Check every 10 seconds
bool softAPStarted = false;
bool wasEverConnected = false;
// Connection Settings, try to connect difference connection if the above is fail
String ssidList[MAX_NETWORKS] = {
  "Add Your SSID Here"
};
String passwordList[MAX_NETWORKS] = {
  "Your Password"
};
int ssidCount = 3;
Preferences preferences;
AsyncWebServer server(80); // ESP32 Webserver on Port 80

/*
=================== Main Adjustment and Configuration ===================
*/
int BASE_SPEED      = 200;   // forward PWM 0-255
const int LOST_TIMEOUT_MS = 2000;  // max time to search
const byte FINISH_PATTERN = 0b11111;  // all sensors on finish line
int   lastError = 0;          // for remembering direction
bool  finished  = false;      // true when finish line detected
unsigned long lostStart = 0;  // time when line disappeared
int prevError = 0;   // for crude derivative
float integral = 0; // Integral term
float KP = 0.15;  
float KD = 0.9;
float KI = 0.0005;
bool          lastWasLeft  = false;   // true = last correction was left
bool          lineWasLost  = false;

int avoidRange = 15; // Default Range

// Variables for distance measurement
long duration;
int distance;
int rightDistance, leftDistance;

unsigned long lastSensorDisplay = 0;
const unsigned long sensorDisplayInterval = 250;
int AVOIDANCE_DELAY = 1000;

// Servo Adjustment
Servo servo;
int leftAngle = 180; // Angle for servo to left
int middleAngle = 90; // Angle for default Servo
int rightAngle = 0; // Angle for servo to right
int currentAngle = 0;

bool lineFollowerActive = false;   // default Black Line on White Space
bool reverseLine = false;   // Reserve the Reading Sensor
bool wallAvoiderActive = false;    // Wall Avoider
bool avoidingObstacle = false; 

String oledCommand = "None";
String oledAudio = "None";
String oledIP = "";
String oledSSID = "";

// DFPlayer Adjustment
HardwareSerial dfSerial(1);  // Use UART1 for DFPlayer
// DFplayer Name Adjustment
DFPlayerMini_Fast DFPlayer;
bool dfplayerReady = false;

/*
=============== Batteries Check Adjustment ===============
*/
uint8_t batteryIcon[8] = {
  0b01110,
  0b11111,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b11111,
  0b00000
};

const float vRef = 3.3; // Adjust if using ADC calibration
const float r1 = 90000.0; // 100k ohm
const float r2 = 50000.0;  // 51k ohm
unsigned long lastBatteryBlink = 0;
bool isBatteryBlinking = false;
const float adcCalibration = 1.225;

float getBatteryVoltage() {
  // Take multiple readings for better accuracy
  float adcSum = 0;
  for (int i = 0; i < 10; i++) {
    adcSum += analogRead(voltagePin);
    delay(1);
  }
  float adcAverage = adcSum / 10.0;
  
  // Convert ADC reading to voltage
  float adcVoltage = (adcAverage * vRef / 4095.0);
  
  // Calculate original voltage using voltage divider formula
  float originalVoltage = adcVoltage * ((r1 + r2) / r2) * adcCalibration;
  
  return originalVoltage;
}

int estimateBatteryPercent(float voltage) {
  float minVoltage = 6.2; // 3~3.2V per cell (2S)
  float maxVoltage = 8.2; // 4~4.2V per cell (2S)

  if (voltage <= minVoltage) return 0;
  if (voltage >= maxVoltage) return 100;

  return (int)(((voltage - minVoltage) / (maxVoltage - minVoltage)) * 100);
}

// Oled Display Text
void displayOLED(const String &line1, const String &line2 = "", const String &line3 = "", const String &line4 = "") {
  oled.clearBuffer();
  oled.setFont(u8g2_font_9x15B_tr);  // Bigger, bolder font

  int lineHeight = 16;  // spacing between lines for 9x15B
  int y = lineHeight;

  if (line1 != "") { oled.drawStr(0, y, line1.c_str()); y += lineHeight; }
  if (line2 != "") { oled.drawStr(0, y, line2.c_str()); y += lineHeight; }
  if (line3 != "") { oled.drawStr(0, y, line3.c_str()); y += lineHeight; }
  if (line4 != "") { oled.drawStr(0, y, line4.c_str()); }

  oled.sendBuffer();
}

// Update Oled Messege
void updateOLEDStatus(bool showAudio = true) {
  oled.clearBuffer();
  oled.setFont(u8g2_font_6x10_tf); // Reduced font

  int y = 10;
  oled.drawStr(0, y, ("Batt: " + String(getBatteryVoltage(), 2) + "V").c_str()); y += 12;
  oled.drawStr(0, y, ("IP: " + oledIP).c_str()); y += 12;
  oled.drawStr(0, y, ("Conn: " + oledSSID).c_str()); y += 12;
  oled.drawStr(0, y, ("Cmd: " + oledCommand).c_str()); y += 12;

  if (showAudio) {
    oled.drawStr(0, y, ("Audio: " + oledAudio).c_str());
  }

  oled.sendBuffer();
}

// ======== MOVEMENT AUDIO (randomized) ========
struct AudioCommand {
  String label;
  int track;
  unsigned long timestamp;
};

AudioCommand currentAudio = {"None", 0, 0};
AudioCommand pendingAudio = {"None", 0, 0};
bool audioPlaying = false;
unsigned long audioStartTime = 0;
const unsigned long MAX_AUDIO_DURATION = 2000; // Max 2 seconds per audio
String lastDirection = "";

bool proximityAlertActive = false;
unsigned long lastProximityAlert = 0;
const unsigned long PROXIMITY_ALERT_COOLDOWN = 3000; // 3 seconds between alerts
const int PROXIMITY_THRESHOLD = 3; // 3cm threshold
bool wasCloseLastCheck = false;

int playKiri() {
  int track = random(9, 10);
  DFPlayer.play(track); 
  return track;
}

int playKiriAsync() { 
  int track = random(9, 10); 
  queueDirectionAudio("Left", track);
  return track; 
}

int playKanan() { 
  int track = random(11, 12); 
  DFPlayer.play(track); 
  return track; 
}

int playKananAsync() { 
  int track = random(11, 12); 
  queueDirectionAudio("Right", track);
  return track; 
}

int playMaju() {
  int track = random(13, 14); 
  DFPlayer.play(track); 
  return track; 
}

int playMajuAsync() { 
  int track = random(13, 14); 
  queueDirectionAudio("Forward", track);
  return track; 
}

int playMundur() { 
  int track = random(15, 16); 
  DFPlayer.play(track); 
  return track; 
}

int playMundurAsync() { 
  int track = random(15, 16); 
  queueDirectionAudio("Reverse", track);
  return track; 
}

// Sound FX Wrappers
void playConnected()      { playSFX("Connected", 17); }
void playDisconnecto()    { playSFX("Disconnect", 18); }
void playUgh()            { playSFX("Ugh", 19); }
void playWazza()          { playSFX("Wazza", 20); }
void playDramaticOuch()   { playSFX("DramaticOuch", 21); }
void playOuch()           { playSFX("Ouch", 22); }
void playDrumroll()       { playSFX("Drumroll", 23); }
void playWhaaat()         { playSFX("Whaat?", 24); }
void playHellow()         { playSFX("Hello", 25); }
void playGoodnight()      { playSFX("Goodnight", 26); }
void playReboot()         { playSFX("Reboot", 27); }
void playYeehaw()         { playSFX("Yeehaw", 28); }
void playYouAreGreat()    { playSFX("You Are Great", 29); }

bool isRobotMoving() {
  // Check if any motor is active by reading the enable pins
  int motor1Speed = ledcRead(enable1Pin);
  int motor2Speed = ledcRead(enable2Pin);
  
  // Robot is moving if either motor has speed > 0
  return (motor1Speed > 0 || motor2Speed > 0);
}

void queueDirectionAudio(const String& label, int track) {
  // If same direction, don't queue new audio
  if (label == lastDirection && audioPlaying) {
    return;
  }
  
  // Queue new audio (will interrupt current if different direction)
  pendingAudio.label = label;
  pendingAudio.track = track;
  pendingAudio.timestamp = millis();
  lastDirection = label;
}

// Direction Play main SFX with random track
int playSFX(const String& label, int trackMin, int trackMax) {
  int track = random(trackMin, trackMax + 1);
  oledAudio = label + " (" + String(track) + ")";
  displayOLED("Playing SFX", oledAudio);
  DFPlayer.play(track);
  updateOLEDStatus();
  return track;
}

// Overloaded Play SFX for single track
void playSFX(const String& label, int track) {
  playSFX(label, track, track);
}


// Add these global variables for non-blocking LCD
struct LCDMessage {
  String line1;
  String line2;
  unsigned long startTime;
  unsigned long duration;
  bool active;
};

LCDMessage currentLCDMessage = {"", "", 0, 0, false};

void showLCDMessage(const String &l1, const String &l2, int duration = 2000) {
  lcd.clear(); 
  lcd.setCursor(0, 0); 
  lcd.print(l1.substring(0, 16));
  lcd.setCursor(0, 1); 
  lcd.print(l2.substring(0, 16));
  
  // Set up the timer for clearing
  currentLCDMessage.line1 = l1;
  currentLCDMessage.line2 = l2;
  currentLCDMessage.startTime = millis();
  currentLCDMessage.duration = duration;
  currentLCDMessage.active = true;
}

void manageLCDMessages() {
  if (currentLCDMessage.active) {
    if (millis() - currentLCDMessage.startTime >= currentLCDMessage.duration) {
      // Time to clear the message
      lcd.clear(); 
      lcd.setCursor(0, 0); 
      lcd.print("Status: OK");
      lcd.setCursor(0, 1); 
      lcd.print("Waiting...");
      
      currentLCDMessage.active = false;
    }
  }
}

void setColor(bool red, bool green, bool blue) {
  digitalWrite(redLed,   red ? HIGH : LOW);
  digitalWrite(greenLed, green ? HIGH : LOW);
  digitalWrite(blueLed,  blue ? HIGH : LOW);
}

/* ----------  LINE FOLLOWING FUNCTIONS (BFD-1000)  ---------- */
int readSensors() {
  int bits = 0;
  if (reverseLine){
    bits |= (!digitalRead(irLeft2))  << 4;   // bit4
    bits |= (!digitalRead(irLeft1))  << 3;   // bit3
    bits |= (!digitalRead(irCenter)) << 2;   // bit2
    bits |= (!digitalRead(irRight1)) << 1;   // bit1
    bits |= (!digitalRead(irRight2));        // bit0
  } if (!reverseLine) {
    bits |= (digitalRead(irLeft2))  << 4;   // bit4
    bits |= (digitalRead(irLeft1))  << 3;   // bit3
    bits |= (digitalRead(irCenter)) << 2;   // bit2
    bits |= (digitalRead(irRight1)) << 1;   // bit1
    bits |= (digitalRead(irRight2));        // bit0
  }
  return bits;                             // 0bxxxxx  (5 bits)
}

/* ----------  LCD LINE PATERN DISPLAY  ---------- */
void displayLCD(int pattern) {
  lcd.setCursor(0, 0);
  // Add full movement description
  switch (pattern) {
    case 0b00100: lcd.print(F("FORWARD")); break;
    case 0b00110:
    case 0b00111:
    case 0b00011: lcd.print(F("RIGHT")); break;
    case 0b01100:
    case 0b11100:
    case 0b11000: lcd.print(F("LEFT")); break;
    case 0b00000:
    case 0b11111: lcd.print(F("LINE LOST")); break;
    default: lcd.print(F("             ")); break;
  }

  lcd.setCursor(0, 1);
  for (int i = 4; i >= 0; i--) {
    lcd.print((pattern >> i) & 1 ? "[O]" : "[X]");
  }
}


void followLine(int pattern) {
  /* 1.  FINISH LINE  */
  if (pattern == FINISH_PATTERN) {
    finished = true;
    handleStop();
    lcd.setCursor(0, 0);
    lcd.print(F("FINISHED!"));
    return;
  }

  /* 2.  LINE STILL PRESENT  */
  int error = 0;
  bool lineFound = false;

  float totalWeight = 0;
  int count = 0;

  for (int i = 0; i < 5; i++) {
    if (pattern & (1 << (4 - i))) {
      totalWeight += (i - 2);
      count++;
    }
  }

  if (count > 0) {
    error = totalWeight / count;
    lineFound = true;
  }

  // Override for hard turns (optional but useful)
  if (pattern == 0b10000) error = -4;
  if (pattern == 0b00001) error = 4;
  if (pattern == 0b11000) error = -3;
  if (pattern == 0b00011) error = 3;


  if (lineFound) {
    lostStart = 0;
    lastError = error;
    int delta  = error - prevError;                       // crude d/dt
    integral += error;                                    // accumulate error
    integral = constrain(integral, -50, 50);
    int diff = constrain(KP * error + KD * delta + KI * integral,
                     -BASE_SPEED, BASE_SPEED);  // clamp big swings
    prevError  = error;                                   // store for next loop
    int left   = BASE_SPEED + diff;
    int right  = BASE_SPEED - diff;
    driveMotors(left, right, error); // Pass error to driveMotors
    return;
  }

  /* 3.  LINE LOST – RECOVERY  */
  if (lostStart == 0) lostStart = millis();

  if (millis() - lostStart < LOST_TIMEOUT_MS) {
    /* continue the last turn direction with same base speed */
    int diff = (lastError >= 0) ? KP : -KP;
    int left  = BASE_SPEED + diff;
    int right = BASE_SPEED - diff;
    driveMotors(left, right, lastError); // Pass lastError to driveMotors
  } else {
    handleStop();
  }
}

/* ----------  MOTOR DRIVER  ---------- */
void driveMotors(int leftPWM, int rightPWM, int error) {
  leftPWM  = constrain(leftPWM, 0, 255);
  rightPWM = constrain(rightPWM, 0, 255);

  if (error == -4) {  // Hard left turn
    ledcWrite(enable1Pin, 0);       // Stop left motor
    ledcWrite(enable2Pin, rightPWM); // Activate right motor
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
  } else if (error == 4) { // Hard right turn
    ledcWrite(enable1Pin, leftPWM); // Activate left motor
    ledcWrite(enable2Pin, 0);       // Stop right motor
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
  } else {
    // Normal driving
    ledcWrite(enable1Pin, leftPWM);
    ledcWrite(enable2Pin, rightPWM);
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
  }
}

enum UltrasonicState {
  ULTRASONIC_IDLE,
  ULTRASONIC_TRIGGERED,
  ULTRASONIC_MEASURING
};

UltrasonicState ultrasonicState = ULTRASONIC_IDLE;
unsigned long triggerTime = 0;
unsigned long echoStartTime = 0;
int lastDistance = -1;
unsigned long lastMeasurementTime = 0;
const unsigned long MIN_MEASUREMENT_INTERVAL = 50; // Minimum 50ms between measurements

// IMPROVED: Fixed getDistance() with proper timing and error handling
int getDistance() {
  unsigned long now = millis();
  
  // Don't measure too frequently - ultrasonic needs recovery time
  if (now - lastMeasurementTime < MIN_MEASUREMENT_INTERVAL) {
    return lastDistance;
  }
  
  switch (ultrasonicState) {
    case ULTRASONIC_IDLE:
      // Start measurement
      digitalWrite(TrigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(TrigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(TrigPin, LOW);
      triggerTime = millis();
      ultrasonicState = ULTRASONIC_TRIGGERED;
      return lastDistance;
      
    case ULTRASONIC_TRIGGERED:
      // Wait for echo to start (but not forever)
      if (digitalRead(EchoPin) == HIGH) {
        echoStartTime = micros();
        ultrasonicState = ULTRASONIC_MEASURING;
      } else if (millis() - triggerTime > 50) { // Reduced timeout
        // Timeout - reset and try again
        ultrasonicState = ULTRASONIC_IDLE;
        lastMeasurementTime = millis();
        return -1; // Invalid reading
      }
      return lastDistance;
      
    case ULTRASONIC_MEASURING:
      // Wait for echo to end
      if (digitalRead(EchoPin) == LOW) {
        unsigned long duration = micros() - echoStartTime;
        int distance = duration * 0.034 / 2;
        
        // Validate reading - filter out obviously wrong values
        if (distance > 2 && distance < 400) { // Valid range 2-400cm
          lastDistance = distance;
        } else {
          lastDistance = -1; // Invalid reading
        }
        
        ultrasonicState = ULTRASONIC_IDLE;
        lastMeasurementTime = millis();
      } else if (micros() - echoStartTime > 25000) { // Reduced timeout
        // Timeout - echo too long
        ultrasonicState = ULTRASONIC_IDLE;
        lastMeasurementTime = millis();
        lastDistance = -1;
      }
      return lastDistance;
  }
  return lastDistance;
}

unsigned long lastObstacleTime = 0;
const unsigned long OBSTACLE_COOLDOWN = 2000; // Reduced from 3000
unsigned long lastValidDistanceTime = 0;
int stableDistance = -1;
int obstacleDetectionCount = 0;
const int REQUIRED_DETECTIONS = 3; // Reduced from 5 for faster response
const unsigned long DETECTION_WINDOW = 300; // Reduced from 500ms
unsigned long firstDetectionTime = 0;

// Add moving average filter to reduce noise
#define FILTER_SIZE 5
int distanceHistory[FILTER_SIZE];
int historyIndex = 0;
bool historyFull = false;

int getFilteredDistance() {
  int rawDistance = getDistance();
  
  if (rawDistance <= 0 || rawDistance > 400) {
    return -1; // Invalid reading
  }

  // Add to circular buffer
  distanceHistory[historyIndex] = rawDistance;
  historyIndex = (historyIndex + 1) % FILTER_SIZE;
  
  // Calculate median (better than average for outliers)
  int sorted[FILTER_SIZE];
  memcpy(sorted, distanceHistory, sizeof(sorted));
  std::sort(sorted, sorted + FILTER_SIZE);
  
  return sorted[FILTER_SIZE/2]; // Return median
}

void resetUltrasonicSensor() {
  // Force sensor state to idle
  ultrasonicState = ULTRASONIC_IDLE;
  lastDistance = -1;
  lastMeasurementTime = millis();
  
  // Clear detection history
  obstacleDetectionCount = 0;
  firstDetectionTime = 0;
  
  // Clear distance filter history
  for (int i = 0; i < FILTER_SIZE; i++) {
    distanceHistory[i] = 0;
  }
  historyIndex = 0;
  historyFull = false;
  
  // Reset timing variables
  lastValidDistanceTime = millis();
  stableDistance = -1;
}

void completeObstacleAvoidanceReset() {
  // Complete reset after obstacle avoidance
  resetUltrasonicSensor();
  
  // Set longer cooldown to prevent immediate re-detection
  lastObstacleTime = millis();
  
  // Force several clear readings to ensure sensor is working properly
  int clearReadings = 0;
  unsigned long resetStart = millis();
  
  while (clearReadings < 3 && (millis() - resetStart < 3000)) {
    delay(100);
    int dist = blockingGetDistance();
    
    if (dist > avoidRange + 5) { // Must be clearly beyond avoid range
      clearReadings++;
    } else if (dist > 0 && dist <= avoidRange) {
      // Still detecting obstacle - reset counter
      clearReadings = 0;
    }
    
    // Show progress on LCD
    lcd.setCursor(0, 1);
    lcd.print("Reset: ");
    lcd.print(clearReadings);
    lcd.print("/3");
  }
  
  if (clearReadings < 3) {
    // Couldn't get clear readings - extend cooldown
    lastObstacleTime = millis() + 2000; // Extra 2 seconds
    
    lcd.setCursor(0, 1);
    lcd.print("Sensor Issue");
    delay(1000);
  }
}

bool isObstacleDetected(int avoidRange) {
  // Skip detection if in cooldown period
  if (millis() - lastObstacleTime < OBSTACLE_COOLDOWN) {
    return false;
  }

  int currentDistance = getFilteredDistance();
  
  // Only process valid readings
  if (currentDistance <= 0 || currentDistance > 300) {
    return false;
  }

  // Hysteresis for more stable detection
  static bool wasDetecting = false;
  int detectThreshold = avoidRange;
  int clearThreshold = avoidRange + 5; // 5cm hysteresis

  if (!wasDetecting && currentDistance < detectThreshold) {
    wasDetecting = true;
    return true;
  } else if (wasDetecting && currentDistance > clearThreshold) {
    wasDetecting = false;
  }
  
  return false;
}

// For Checking Wall on Right or Left (why not use getDistance? because it have debounce time for make it return value -1)
int blockingGetDistance() {
  // Clear any pending state first
  ultrasonicState = ULTRASONIC_IDLE;
  
  digitalWrite(TrigPin, LOW);
  delayMicroseconds(5); // Increased clear time
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);

  unsigned long duration = pulseIn(EchoPin, HIGH, 30000);
  
  if (duration > 0) {
    int distance = duration * 0.034 / 2;
    return (distance > 2 && distance < 400) ? distance : -1;
  }
  
  return -1;
}

int getAverageDistance(int numReadings = 3) {
  int readings[numReadings];
  int validCount = 0;
  
  // Collect readings
  for (int i = 0; i < numReadings; i++) {
    int reading = blockingGetDistance();
    if (reading > 0) {
      readings[validCount] = reading;
      validCount++;
    }
    delay(60); // Increased delay between readings
  }
  
  if (validCount == 0) return -1;
  if (validCount == 1) return readings[0];
  
  // Simple outlier rejection - remove readings that differ too much from median
  // Sort readings
  for (int i = 0; i < validCount - 1; i++) {
    for (int j = i + 1; j < validCount; j++) {
      if (readings[i] > readings[j]) {
        int temp = readings[i];
        readings[i] = readings[j];
        readings[j] = temp;
      }
    }
  }
  
  // Use median and nearby values
  int median = readings[validCount / 2];
  int sum = 0;
  int count = 0;
  
  for (int i = 0; i < validCount; i++) {
    if (abs(readings[i] - median) < 10) { // Within 10cm of median
      sum += readings[i];
      count++;
    }
  }
  
  return count > 0 ? sum / count : median;
}

void handleObstacle(int avoidRange) {
  if (!wallAvoiderActive) return;

  int centerDist = blockingGetDistance();
  if (centerDist > avoidRange + 10) { // 10cm buffer
    return; // No obstacle in front
  }
  
  handleStop();
  avoidingObstacle = true;
  
  if (!lineFollowerActive) {
    // Original obstacle avoidance for non-line following mode
    servo.attach(servoPin);    
    // --- SCAN LEFT ---
    servo.write(leftAngle);
    delay(500);
    int leftDist = getAverageDistance(3);
    delay(100);
    
    // --- SCAN RIGHT ---
    servo.write(rightAngle);
    delay(500);
    int rightDist = getAverageDistance(3);
    delay(100);

    // --- RETURN TO CENTER ---
    servo.write(middleAngle);
    delay(300);
    servo.detach();

    // Show distances on LCD
    lcd.clear();
    lcd.print("L:");
    lcd.print(leftDist);
    lcd.print(" R:");
    lcd.print(rightDist);

    // ---- AVOIDANCE MANEUVER ----
    if (leftDist > rightDist) {
      avoidLeft();
      lcd.setCursor(0, 1); lcd.print(F("Belok Kiri"));
    } else {
      avoidRight();
      lcd.setCursor(0, 1); lcd.print(F("Belok Kanan"));
    }
  } else {
    // Wall following mode when line following is active
    LineWall();
  }
  obstacleDetectionCount = 0;
  firstDetectionTime = 0;
  
  avoidingObstacle = false;

  resetUltrasonicSensor();
  lastObstacleTime = millis(); 
}

void LineWall(){
  lcd.clear();
  lcd.print("Wall Avoid");
  lcd.setCursor(0, 1);
  lcd.print("Line Mode");

  // Step 1: Back up slightly
  handleReverse();
  delay(500);
  handleStop();
  delay(300);

  // Turn left
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  ledcWrite(enable1Pin, 200);
  ledcWrite(enable2Pin, 200);
  delay(300);

  handleForward();
  delay(2000);

  handleStop();
  delay(300);

  // Turn left
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
  ledcWrite(enable1Pin, 200);
  ledcWrite(enable2Pin, 200);
  delay(300);

    // Reset control values
  lastError = 0;
  prevError = 0;
  integral = 0;
  lostStart = 0;

  // Forward search
  bool lineFound = false;
  unsigned long searchStart = millis();
  while (millis() - searchStart < 4000 && !lineFound) {
    int pattern = readSensors();
    if (pattern != 0b00000) {
      lineFound = true;
      handleStop();
      delay(300);
      lcd.clear();
      lcd.print("Line Found!");
      delay(1000);
      break;
    }
    handleForward();
    delay(50);
  }
}

void avoidLeft() {
  // Back up slightly
  handleReverse();
  delay(300);
  
  int track = -1;
  track = playSFX("Left", 9, 10);

  // Wait a bit
  handleStop();
  delay(300);

  TurnLeft();

  handleStop();
  delay(300);
}

void avoidRight() {
  // Back up slightly
  handleReverse();
  delay(300);
  
  int track = -1;
  track = playSFX("Right", 11, 12);
  
  // Wait a bit
  handleStop();
  delay(300);

  TurnRight();
  handleStop();
  delay(300);
}

void TurnLeft(){
  // Turn left
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  ledcWrite(enable1Pin, 200);
  ledcWrite(enable2Pin, 200);
  delay(AVOIDANCE_DELAY);
}

void TurnRight(){
  // Turn right
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
  ledcWrite(enable1Pin, 200);
  ledcWrite(enable2Pin, 200);
  delay(AVOIDANCE_DELAY);
}

void indicateWiFiStatus(bool connected, bool softAP) {
  if (connected) {
    // Blink green 3 times (3 sec)
    for (int i = 0; i < 3; i++) {
      setColor(false, true, false); delay(500);
      setColor(false, false, false); delay(500);
    }
    setColor(false, true, true); // Turquoise (connected)
  } else if (softAP) {
    // Blink red 3 times (3 sec)
    for (int i = 0; i < 3; i++) {
      setColor(true, false, false); delay(500);
      setColor(false, false, false); delay(500);
    }
    setColor(false, false, true); // Blue (SoftAP ready)
  }
}

void checkingConnectionOptimized(){
  // Only check WiFi when not actively line following or much less frequently
  if (lineFollowerActive || wallAvoiderActive) {
    // Check much less frequently when line following (every 30 seconds)
    static unsigned long lastWiFiCheckLineMode = 0;
    if (millis() - lastWiFiCheckLineMode > 30000) {
      lastWiFiCheckLineMode = millis();
      
      // Quick check without delays
      if (WiFi.status() != WL_CONNECTED && !softAPStarted) {
        // Don't do full reconnection during line following
        // Just note the disconnection
        wasEverConnected = false;
      }
    }
    return;
  }
  
  // Original logic when not line following
  if (millis() - lastWiFiCheck > wifiCheckInterval) {
    lastWiFiCheck = millis();
    if (WiFi.status() != WL_CONNECTED && !softAPStarted) {
      if (wasEverConnected) {
        playDisconnecto();
      }
      lcd.clear(); lcd.setCursor(0, 0); lcd.print("WiFi Terputus");
      lcd.setCursor(0, 1); lcd.print("Mulai SoftAP");
      displayOLED("WiFi Terputus", "Mulai SoftAP");
      setColor(false, false, true);
      WiFi.disconnect(true);
      delay(1000);
      startSoftAPConfig();
      softAPStarted = true;
    } else {
      if (WiFi.status() != WL_CONNECTED) {
        WiFi.reconnect();
      }
    }
  }
}

void displayLoadingAnimation(int step) {
  String bar = "[";
  for (int i = 0; i < 3; i++) {
    bar += (i < step) ? "=" : " ";
  }
  bar += "]";

  lcd.setCursor(11, 1);
  lcd.print("[     ]");       // Clear old bar
  lcd.setCursor(11, 1);
  lcd.print(bar);
}

bool connectToWiFi() {
  for (int i = 0; i < ssidCount; i++) {
    WiFi.begin(ssidList[i].c_str(), passwordList[i].c_str());
    displayOLED("Connecting to:", ssidList[i]);
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Menghubungkan");
    lcd.setCursor(0, 1); lcd.print("Kejaringan");

    unsigned long startAttemptTime = millis();
    int loadingStep = 0;
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT_MS) {
      displayLoadingAnimation(loadingStep);
      loadingStep = (loadingStep + 1) % 6;
      delay(250);
    }

    if (WiFi.status() == WL_CONNECTED) {
      lcd.clear();
      lcd.setCursor(0, 0); lcd.print("Berhasil Terhubung");
      lcd.setCursor(0, 1); lcd.print(WiFi.localIP());
      displayOLED("Connected!", ssidList[i], WiFi.localIP().toString());
      oledIP = WiFi.localIP().toString();
      oledSSID = ssidList[i];
      updateOLEDStatus();
      indicateWiFiStatus(true, false);
      wasEverConnected = true;
      return true;
    } else {
      lcd.clear();
      lcd.setCursor(0, 0); lcd.print("Gagal Terhubung");
      lcd.setCursor(0, 1); lcd.print("Mencoba:");
      lcd.print(ssidList[i]);

      setColor(true, false, false);
      delay(500);
      WiFi.disconnect(true);
    }
  }
  return false;
}

void startSoftAPConfig() {
  WiFi.softAP("ESP32_Config");
  IPAddress IP = WiFi.softAPIP();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Mode SoftAP");
  lcd.setCursor(0, 1); lcd.print(IP);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.on("/save", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("ssid") && request->hasParam("password")) {
      String newSSID = request->getParam("ssid")->value();
      String newPass = request->getParam("password")->value();

      if (ssidCount < MAX_NETWORKS) {
      int index = ssidCount;
      ssidList[index] = newSSID;
      passwordList[index] = newPass;
      ssidCount++;

      preferences.begin("wifi", false);
      preferences.putString(("ssid" + String(index)).c_str(), newSSID);
      preferences.putString(("pass" + String(index)).c_str(), newPass);
      preferences.putInt("count", ssidCount);
      preferences.end();
      }

      request->send(200, "text/html", "Saved. Rebooting...");
      
      playReboot();
      delay(1000);
      ESP.restart();
    } else {
      request->send(400, "text/plain", "Missing ssid or password");
    }
  });
  server.begin();
}

void loadSavedWiFi() {
  preferences.begin("wifi", true);
  int count = preferences.getInt("count", ssidCount);
  for (int i = ssidCount; i < count && i < MAX_NETWORKS; i++) {
    ssidList[i] = preferences.getString(("ssid" + String(i)).c_str(), "");
    passwordList[i] = preferences.getString(("pass" + String(i)).c_str(), "");
  }
  ssidCount = count;
  preferences.end();
}

/*
===================* Proxy Mode : Hashing Command *===================
*/
void proxyConnection(){
  // /move?dir=forward|backward|left|right|stop
  server.on("/move", HTTP_GET, [](AsyncWebServerRequest *request) {
    String dir = request->getParam("dir") ? request->getParam("dir")->value() : "stop";
    moveRobot(dir);
    oledCommand = dir;
    oledAudio = "Move" + dir;
    updateOLEDStatus();
    showLCDMessage("New Command:", dir);
    request->send(200, "application/json", "{\"result\": \"ok\", \"action\": \"" + dir + "\"}");
  });

  // /line?active=1|0
  server.on("/line", HTTP_GET, [](AsyncWebServerRequest *request) {
    // Only set active if the parameter is provided
    if (request->hasParam("active")) {
      bool active = request->getParam("active")->value() == "1";
      setLineFollower(active);
    }
    
    // Only set reverse if the parameter is provided
    if (request->hasParam("reverse")) {
      bool reverse = request->getParam("reverse")->value() == "1";
      setLineReverse(reverse);
    }

    // Respond with current states (not the parameters)
    String response = "{\"result\":\"ok\", \"line_active\":";
    response += (lineFollowerActive ? "true" : "false");
    response += ", \"reverse\":";
    response += (reverseLine ? "true" : "false");
    response += "}";

    request->send(200, "application/json", response);
  });

  // /wall?active=1|0
  server.on("/wall", HTTP_GET, [](AsyncWebServerRequest *request) {
    bool active = request->getParam("active") && request->getParam("active")->value() == "1";
    setWallAvoider(active);
    request->send(200, "application/json", "{\"result\": \"ok\", \"wall_active\": " + String(active ? "true" : "false") + "}");
  });

  // /speed?value=XX
  server.on("/speed", HTTP_GET, [](AsyncWebServerRequest *request) {
    int speed = request->getParam("value") ? request->getParam("value")->value().toInt() : 100;
    setRobotSpeed(speed);
    request->send(200, "application/json", "{\"result\": \"ok\", \"speed\": " + String(speed) + "}");
  });

  // /distance?value=XX
  server.on("/distance", HTTP_GET, [](AsyncWebServerRequest *request) {
    int dist = request->getParam("value") ? request->getParam("value")->value().toInt() : 25;
    setAvoidDistance(dist);
    request->send(200, "application/json", "{\"result\": \"ok\", \"distance\": " + String(dist) + "}");
  });

  // Status endpoint for connection test
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "application/json", "{\"result\": \"ok\", \"status\": \"online\"}");
  });

  server.on("/sensor", HTTP_GET, [](AsyncWebServerRequest *request) {
  int left = (digitalRead(irLeft2) == 0 || digitalRead(irLeft1) == 0) ? 1 : 0;
  int mid  = (digitalRead(irCenter) == 0) ? 1 : 0;
  int right = (digitalRead(irRight1) == 0 || digitalRead(irRight2) == 0) ? 1 : 0;

  String json = "{\"left\":" + String(left) + ",\"mid\":" + String(mid) + ",\"right\":" + String(right) + "}";
  request->send(200, "application/json", json);
  });

  // /kp?value=1.23
  server.on("/kp", HTTP_GET, [](AsyncWebServerRequest *request){
    float v = request->getParam("value")->value().toFloat();
    setPidKp(v);
    request->send(200, "application/json", "{\"result\":\"ok\",\"kp\":"+String(v)+"}");
  });

  // /ki?value=0.05
  server.on("/ki", HTTP_GET, [](AsyncWebServerRequest *request){
    float v = request->getParam("value")->value().toFloat();
    setPidKi(v);
    request->send(200, "application/json", "{\"result\":\"ok\",\"ki\":"+String(v)+"}");
  });

  // /kd?value=0.2
  server.on("/kd", HTTP_GET, [](AsyncWebServerRequest *request){
    float v = request->getParam("value")->value().toFloat();
    setPidKd(v);
    request->send(200, "application/json", "{\"result\":\"ok\",\"kd\":"+String(v)+"}");
  });
}

/*
==========================* API HANDLERS *==========================
*/

enum APIState {
  API_IDLE,
  API_REQUESTING,
  API_PROCESSING
};

APIState apiState = API_IDLE;
HTTPClient apiHttp;
unsigned long apiStartTime = 0;
String pendingCommand = "";

void fetchCommandFromAPIAsync() {
  if (WiFi.status() != WL_CONNECTED) return;
  
  switch (apiState) {
    case API_IDLE:
      // Start new request only if it's time
      if (millis() - lastFetch > fetchInterval) {
        apiHttp.begin(getUrl);
        apiHttp.setTimeout(1000); // 1 second timeout
        
        // Start the request
        int httpCode = apiHttp.GET();
        
        if (httpCode > 0) {
          apiState = API_REQUESTING;
          apiStartTime = millis();
        } else {
          apiHttp.end();
        }
        lastFetch = millis();
      }
      break;
      
    case API_REQUESTING:
      // Check if response is ready or timeout
      if (millis() - apiStartTime > 1000) {
        // Timeout
        apiHttp.end();
        apiState = API_IDLE;
      } else {
        // Try to get response
        int size = apiHttp.getSize();
        if (size > 0 || !apiHttp.connected()) {
          if (size > 0) {
            pendingCommand = apiHttp.getString();
            apiState = API_PROCESSING;
          } else {
            apiHttp.end();
            apiState = API_IDLE;
          }
        }
      }
      break;
      
    case API_PROCESSING:
      // Process the command (this should be fast)
      if (pendingCommand.length() > 0) {
        DynamicJsonDocument doc(512);
        DeserializationError error = deserializeJson(doc, pendingCommand);
        
        if (!error) {
          JsonObject command = doc["command"];
          handleAPICommand(command);
          sendCommandStatus(1); // Quick acknowledgment
        }
        
        pendingCommand = "";
      }
      
      apiHttp.end();
      apiState = API_IDLE;
      break;
  }
}

void sendCommandStatus(int statusCode) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(postUrl); // POST /command-status/{apiKey}
    http.addHeader("Content-Type", "application/json");

    String json = "{\"status\": " + String(statusCode) + "}";
    int httpCode = http.POST(json);
    http.end();
  }
}

void sendSensorData(String type, float value) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(sensorUrl);
    http.addHeader("Content-Type", "application/json");

    DynamicJsonDocument doc(256);
    doc["api_key"] = apiKey;
    doc["sensor_type"] = type;
    doc["value"] = value;

    String body;
    serializeJson(doc, body);

    int httpCode = http.POST(body);
    http.end();
  }
}

void handleAPICommand(JsonObject command) {
  String action = command["action"];

  if (action == "forward" || action == "backward" || action == "reverse" || 
      action == "left" || action == "right" || action == "stop") {
    moveRobot(action); // existing function
  }

  else if (action == "line") {
    bool status = command["status"];
    setLineFollower(status);

    // OPTIONAL: check and apply reverse if provided
    if (command.containsKey("reverse")) {
      bool reverse = command["reverse"];
      setLineReverse(reverse);
    }
  }

  else if (action == "wall") {
    bool status = command["status"];
    setWallAvoider(status);
  }

  else if (action == "speed") {
    int speed = command["speed"];
    setRobotSpeed(speed);
  }

  else if (action == "distance") {
    int dist = command["distance"];
    setAvoidDistance(dist);
  }

  else if (action == "kp"){
    int KP = command["value"];
    setPidKp(KP);
  }

  else if (action == "ki"){
    int KI = command["value"];
    setPidKp(KI);
  }

  else if (action == "kd"){
    int KD = command["value"];
    setPidKp(KD);
  }

  else {
    displayOLED("Unknown Cmd", action);
    playWhaaat();
  }

  oledCommand = action;
  updateOLEDStatus();
}

// Robot COntrol by manual
void moveRobot(String dir) {
  oledCommand = dir;
  int track = -1;

  if (dir == "forward") {
    track = playSFX("Forward", 13, 14);
    handleForward();
  } else if (dir == "left") {
    track = playSFX("Left", 9, 10);
    handleLeft();
  } else if (dir == "right") {
    track = playSFX("Right", 11, 12);
    handleRight();
  } else if (dir == "reverse" || dir == "backward") {
    track = playSFX("Reverse", 15, 16);
    handleReverse();
  } else {
    handleStop();
    return;
  }
}

void moveRobotAsync(String dir) {
  oledCommand = dir;

  if (dir == "forward") {
    playMajuAsync();
    handleForward();
  } else if (dir == "left") {
    playKiriAsync();
    handleLeft();
  } else if (dir == "right") {
    playKananAsync();
    handleRight();
  } else if (dir == "reverse" || dir == "backward") {
    playMundurAsync();
    handleReverse();
  } else {
    // For stop/unknown commands, play immediately (not queued)
    playSFX("Ugh", 19, 19);
    handleStop();
    audioPlaying = false; // Stop direction audio
    lastDirection = "";
    return;
  }
}

// Proxy Mode : LineFolower
void setLineFollower(bool active) {
  // Enable/disable line follower mode
  lineFollowerActive = active;
  lcd.clear(); lcd.print("Line: ");
  lcd.print(active ? "ON" : "OFF");
  if (!active) {
    handleStop();
  }
}

void setLineReverse(bool reverse) {
  reverseLine = reverse;
  lcd.clear();
  lcd.print("Line");
  lcd.print(reverse ? "-R:" : ":");
  lcd.print(lineFollowerActive ? "ON" : "OFF");
}

// Proxy Mode : WallAvoider
void setWallAvoider(bool active) {
  // Enable/disable wall avoidance
  wallAvoiderActive = active;
  lcd.clear(); lcd.print("Wall: ");
  lcd.print(active ? "ON" : "OFF");
  if (!active) {
    handleStop();
  }
}

// Proxy Mode : Set Speed PWM
void setRobotSpeed(int speed) {
  // Adjust PWM or motor controller as needed
  BASE_SPEED = speed;
  lcd.clear(); lcd.print("Speed: ");
  lcd.print(speed);
}

// Proxy Mode : Set Distance Range
void setAvoidDistance(int dist) {
  // Use value for your ultrasonic logic
  avoidRange = dist;
  lcd.clear(); lcd.print("AvoidDist: ");
  lcd.print(dist);
}

// Proxy Mode : Set PID Value
void setPidKp(float v) {
  KP = v; 
  lcd.clear(); lcd.print("KP: ");
  lcd.print(v);
}

void setPidKi(float v) { 
  KI = v;
  lcd.clear(); lcd.print("KI: ");
  lcd.print(v);
}

void setPidKd(float v) {
  KD = v;
  lcd.clear(); lcd.print("KD: ");
  lcd.print(v);
}

void handleForward() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  ledcWrite(enable1Pin, BASE_SPEED);
  ledcWrite(enable2Pin, BASE_SPEED);
}

void handleLeft() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  ledcWrite(enable1Pin, BASE_SPEED);
  ledcWrite(enable2Pin, BASE_SPEED);
}

void handleStop() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
  ledcWrite(enable1Pin, 0);
  ledcWrite(enable2Pin, 0);
}

void handleRight() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
  ledcWrite(enable1Pin, BASE_SPEED);
  ledcWrite(enable2Pin, BASE_SPEED);
}

void handleReverse() {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  ledcWrite(enable1Pin, BASE_SPEED);
  ledcWrite(enable2Pin, BASE_SPEED);
}

void handleSPIFFS() {
  if (!SPIFFS.begin(true)) {
    lcd.clear(); lcd.print("SPIFFS Gagal");
    lcd.setCursor(0, 1); lcd.print("Periksa Flash");
    displayOLED("SPIFFS Mount", "Gagal");
    return;
  }

  displayOLED("SPIFFS Mount", "Sukses", "Loading Files...");
  delay(1000);
}

void checkBatteryWarning() {
  float voltage = getBatteryVoltage();  // use your existing function
  int percent = estimateBatteryPercent(voltage);

  unsigned long now = millis();
  if (percent <= 50 && now - lastBatteryBlink >= 60000) {
    isBatteryBlinking = true;
    lastBatteryBlink = now;

    // Blink yellow for 2 sec (on-off)
    setColor(true, true, false); // Yellow
    delay(2000);
    setColor(false, false, false); // Off
    isBatteryBlinking = false;
  }
}

/*
=================* MAIN FUNCTION *=================
*/
void setup() {
  handleSPIFFS();
  initializeAll(); // Initializing Setup for Robot
}

void loop() {
  // 0. Manage async audio and lcd
  manageAsyncAudio();
  manageLCDMessages();
  // checkProximityAlert();

  // 1. ALWAYS handle API commands (very fast, non-blocking)
  fetchCommandFromAPIAsync();
  
  // 2. WALL AVOIDANCE HAS HIGHEST PRIORITY - Check this FIRST
  if (wallAvoiderActive && !avoidingObstacle) {
    // Check cooldown period - don't detect obstacles too soon after last avoidance
    if (millis() - lastObstacleTime > OBSTACLE_COOLDOWN) {
      if (isObstacleDetected(avoidRange)) {
        lastObstacleTime = millis(); // Set cooldown timer
        handleObstacle(avoidRange);
        return; // Exit after handling obstacle
      }
    }
  }
  
  // 3. Line following - only if not avoiding obstacle
  if (lineFollowerActive && !avoidingObstacle) {
    int pattern = readSensors();
    followLine(pattern);
    
    // Update display less frequently
    if (millis() - lastSensorDisplay > sensorDisplayInterval) {
      displayLCD(pattern);
      lastSensorDisplay = millis();
    }
    return; // Exit after line following
  }
  
  // 4. If only wall avoidance is active (no line following), move forward
  else if (wallAvoiderActive && !avoidingObstacle) {
    handleForward();
  }

  // 5. Less frequent WiFi checking
  static unsigned long lastWiFiCheck = 0;
  if (millis() - lastWiFiCheck > 10000) {
    checkingConnectionOptimized();
    lastWiFiCheck = millis();
  }
}

/*
==================* ALL INITIALIZE HANDLERS *===================
*/
// All of the initialize
void initializeAll(){
  initializeServo(); 
  initializeLCD();    
  initializeInfrared(); 
  initializeLED();     
  initializeUltrasonic();
  initializeMotorDriver();
  initializeDFPlayer();
  initializeBatteries();  
  proxyConnection();  
  loadSavedWiFi();  

  displayOLED("Initializing...", "Loading Wi-Fi...");
  
  if (!connectToWiFi()) {
    playWazza();
    displayOLED("Wi-Fi Failed!", "AP Mode: ESP32_Config", "Use browser to setup");
    delay(1000);
    startSoftAPConfig();  
  } else {
    if (dfplayerReady) {
      playConnected();
      displayOLED("Connected:", WiFi.SSID(), WiFi.localIP().toString());
      delay(2000);
    }
    server.begin(); 
  }
}

void initializeServo(){
  servo.attach(servoPin);
  servo.write(middleAngle);
  delay(300);
  servo.detach();
}

// Show voltage for Setup
void initializeBatteries() {
  float voltage = getBatteryVoltage();  
  int percent = estimateBatteryPercent(voltage);

  String statusMsg;
  if (percent >= 80) {
    statusMsg = "Baterai Terisi";
  } else if (percent >= 50) {
    statusMsg = "Baterai Cukup";
  } else if (percent < 50 && percent != 0) {
    statusMsg = "Baterai Kurang, Charge Baterai";
  } else {
    statusMsg = "Tidak Terdeteksi, Cek Kabel";
  }

  // Line 1: battery icon + percentage + voltage
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.write((uint8_t)0); // battery icon slot
  lcd.print(" ");
  lcd.print(String(percent) + "% (" + String(voltage, 2) + "V)");

  // Line 2: scroll the message over 3 seconds
  unsigned long start = millis();
  while (millis() - start < 3000) {
    for (int i = 0; i < statusMsg.length() + 16; i++) {
      lcd.setCursor(0, 1);
      lcd.print("                "); // Clear line
      lcd.setCursor(0, 1);
      lcd.print(statusMsg.substring(i, i + 16));
      delay(250); // Scroll speed
      if (millis() - start >= 3000) break;
    }
  }
}

// Setup LED RGB
void initializeLED(){
  pinMode(redLed, OUTPUT);
  pinMode(blueLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  setColor(true, false, false);    // Solid red
}

// Setup ultrasonic
void initializeUltrasonic(){
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
}

// Motor DC adjusment
void initializeMotorDriver(){
  // Set the Motor pins as outputs
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  // Configure PWM Pins
  ledcAttach(enable1Pin, freq, resolution);
  ledcAttach(enable2Pin, freq, resolution);
    
  // Initialize PWM with 0 duty cycle
  ledcWrite(enable1Pin, 0);
  ledcWrite(enable2Pin, 0);

  // Initialize PWM with 0 duty cycle
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}

// Initialize DFPlayer using Serial
void initializeDFPlayer() {
  dfSerial.begin(9600, SERIAL_8N1, DF_RX, DF_TX);
  delay(1000); // Give DFPlayer time to boot

  if (DFPlayer.begin(dfSerial)) {
    DFPlayer.volume(20); // Volume: 0–30
    playHellow();
    dfplayerReady = true;
  } else {
    lcd.clear(); lcd.print("DFPlayer Gagal");
    lcd.setCursor(0, 1); lcd.print("Cek Kabel");
  }
}

// Setup LCD and OLED
void initializeLCD(){
  Wire.begin(21, 22);  // Explicitly set SDA, SCL pins

  lcd.begin();
  lcd.backlight();
  lcd.createChar(0, batteryIcon);
  oled.begin();  // Must be after Wire.begin
  displayOLED("ESP32 ROBOT", "Starting...", "by Ei", "https://github/IAgusta");
}

// Setup Infrared BFD-1000 or 5 Infrared Sensor
void initializeInfrared(){
  pinMode(irLeft2, INPUT);
  pinMode(irLeft1, INPUT);
  pinMode(irCenter, INPUT);
  pinMode(irRight1, INPUT);
  pinMode(irRight2, INPUT);
}

/*
Other Functionality Just Ignore it if Doesnt Need it
*/
void manageAsyncAudio() {
  unsigned long now = millis();
  
  // Check if we have pending audio
  if (pendingAudio.label != "None") {
    // If no audio playing, or different direction, start new audio
    if (!audioPlaying || pendingAudio.label != currentAudio.label) {
      // Start new audio
      DFPlayer.play(pendingAudio.track);
      currentAudio = pendingAudio;
      audioPlaying = true;
      audioStartTime = now;
      
      // Update OLED
      oledAudio = currentAudio.label + " (" + String(currentAudio.track) + ")";
      updateOLEDStatus();
    }
    
    // Clear pending
    pendingAudio = {"None", 0, 0};
  }
  
  // Check if current audio should timeout (safety mechanism)
  if (audioPlaying && (now - audioStartTime > MAX_AUDIO_DURATION)) {
    audioPlaying = false;
    currentAudio = {"None", 0, 0};
    lastDirection = "";
  }
}

// Main proximity alert function
void checkProximityAlert() {
  // Only check when NOT in wall avoidance mode and NOT in obstacle avoidance
  if (wallAvoiderActive || avoidingObstacle) {
    return;
  }
  
  int frontDist = getDistance();
  unsigned long now = millis();
  
  // Check if distance is valid and below threshold
  if (frontDist > 0 && frontDist < PROXIMITY_THRESHOLD) {
    // Only play sound if enough time has passed since last alert
    if (now - lastProximityAlert > PROXIMITY_ALERT_COOLDOWN) {
      
      if (isRobotMoving()) {
        // Robot is moving - play movement collision sounds
        int randomChoice = random(0, 2); // 0 or 1
        if (randomChoice == 0) {
          playOuch();
        } else {
          playDramaticOuch();
        }
        
        // Optional: Show warning on LCD
        showLCDMessage("WARNING!", "Too Close!", 1500);
        
      } else {
        // Robot is stopped - play stationary warning
        playUgh();
        
        // Optional: Show different message for stationary
        showLCDMessage("OBSTACLE!", "Very Close", 1500);
      }
      
      lastProximityAlert = now;
      wasCloseLastCheck = true;
      
      // // Optional: Flash red LED for visual warning
      // setColor(true, false, false); // Red warning
      // delay(100); // Brief flash
      // setColor(false, false, false); // Turn off (or return to normal state)
    }
  } else {
    // Reset the close detection flag when object moves away
    if (wasCloseLastCheck && frontDist >= PROXIMITY_THRESHOLD + 2) { // Add 2cm hysteresis
      wasCloseLastCheck = false;
      // Optional: Play "clear" sound or show "clear" message
      showLCDMessage("Path Clear", "", 1000);
    }
  }
}
