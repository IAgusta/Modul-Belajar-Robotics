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
const char* apiKey = "UzalZNU3vJ76NLfwK4VgupaVNDaFv3xl"; // Change this based on your API Key
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
  "MSI 5916",
  "OZARA ATHIRA",
  "Redmi Note 7"
};
String passwordList[MAX_NETWORKS] = {
  "msimodern",
  "Eri979669",
  "yunitamuhtya"
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
const int OBSTACLE_DISTANCE = 15;  // obstacle threshold in cm
const int AVOIDANCE_DELAY = 500;   // avoidance maneuver time
// PID Variable
int   lastError = 0;          // for remembering direction
bool  finished  = false;      // true when finish line detected
unsigned long lostStart = 0;  // time when line disappeared
int prevError = 0;   // for crude derivative
float integral = 0; // Integral term
// PID Value
float KP = 0.15;  
float KD = 0.9;
float KI = 0.0005;

int avoidRange = 15; // Default Range

// Variables for distance measurement
long duration;
int distance;
int rightDistance, leftDistance;

unsigned long lastSensorDisplay = 0;
const unsigned long sensorDisplayInterval = 250;

// Servo Adjustment
Servo servo;
int leftAngle = 0; // Angle for servo to left
int middleAngle = 90; // Angle for default Servo
int rightAngle = 180; // Angle for servo to right
int currentAngle = 0;

bool lineFollowerActive = false;   // default Black Line on White Space
bool WhiteLineActive  = false;     // White Line on Black Space
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
const float r1 = 100000.0; // 100k ohm
const float r2 = 51000.0;  // 51k ohm
unsigned long lastBatteryBlink = 0;
bool isBatteryBlinking = false;

float getBatteryVoltage() {
  float adc = analogRead(voltagePin) * vRef / 4095.0;
  return adc * ((r1 + r2) / r2);
}

int estimateBatteryPercent(float voltage) {
  float minVoltage = 6.0; // 3~3.2V per cell (2S)
  float maxVoltage = 8.0; // 4~4.2V per cell (2S)

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
int playKiri() { int track = random(9, 10); DFPlayer.play(track); return track; }
int playKanan() { int track = random(11, 12); DFPlayer.play(track); return track; }
int playMaju() { int track = random(13, 14); DFPlayer.play(track); return track; }
int playMundur() { int track = random(15, 16); DFPlayer.play(track); return track; }

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

void showLCDMessage(const String &l1, const String &l2, int duration = 2000) {
  lcd.clear(); lcd.setCursor(0, 0); lcd.print(l1.substring(0, 16));
  lcd.setCursor(0, 1); lcd.print(l2.substring(0, 16));
  delay(duration);
  lcd.clear(); lcd.setCursor(0, 0); lcd.print("Status: OK");
  lcd.setCursor(0, 1); lcd.print("Waiting...");
}

void setColor(bool red, bool green, bool blue) {
  digitalWrite(redLed,   red ? HIGH : LOW);
  digitalWrite(greenLed, green ? HIGH : LOW);
  digitalWrite(blueLed,  blue ? HIGH : LOW);
}

/* ----------  LINE FOLLOWING FUNCTIONS (BFD-1000)  ---------- */
int readSensors() {
  int bits = 0;
  bits |= (!digitalRead(irLeft2))  << 4;   // bit4  (TCRT-5000 remove '!'")
  bits |= (!digitalRead(irLeft1))  << 3;   // bit3  (TCRT-5000 remove '!'")
  bits |= (!digitalRead(irCenter)) << 2;   // bit2  (TCRT-5000 remove '!'")
  bits |= (!digitalRead(irRight1)) << 1;   // bit1  (TCRT-5000 remove '!'")
  bits |= (!digitalRead(irRight2));        // bit0  (TCRT-5000 remove '!'")
  return bits;                             // 0bxxxxx  (5 bits)
}

void followLine(int pattern) {
  // Display sensor values every interval
  if (millis() - lastSensorDisplay > sensorDisplayInterval) {
    lastSensorDisplay = millis();

    lcd.setCursor(0, 0); lcd.print("Sensor:");
    lcd.setCursor(0, 1);
    for (int i = 4; i >= 0; i--) {         // print 5 bits
      lcd.print((pattern >> i) & 1 ? 'O' : 'X');
    }
  }

  /* 1.  FINISH LINE  */
  if (pattern == FINISH_PATTERN) {
    finished = true;
    handleStop();
    lcd.setCursor(0, 1);
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

  // Override for hard turns
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
    driveMotors(left, right, error);
    return;
  }

  /* 3.  LINE LOST – RECOVERY  */
  if (lostStart == 0) lostStart = millis();

  if (millis() - lostStart < LOST_TIMEOUT_MS) {
    /* continue the last turn direction */
    int diff = (lastError >= 0) ? KP : -KP;
    int left  = BASE_SPEED + diff;
    int right = BASE_SPEED - diff;
    driveMotors(left, right, lastError);
  } else {
    handleStop();
  }
}

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

// Function to calculate distance
int getDistance() {
  digitalWrite(TrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);
  duration = pulseIn(EchoPin, HIGH);
  distance = duration * 0.034 / 2;
  return distance;
}

enum WallAvoiderState {
  WA_DRIVE,
  WA_BACK,
  WA_TURN,
  WA_DELAY
};

void handleObstacle(int avoidRange) {
  if (!wallAvoiderActive) return;
  
  avoidingObstacle = true;
  servo.attach(servoPin);
  
  // --- SCAN LEFT ---
  servo.write(leftAngle);
  delay(400);
  int leftDist = getDistance();

  // --- SCAN RIGHT ---
  servo.write(rightAngle);
  delay(400);
  int rightDist = getDistance();

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
    avoidLeft(lineFollowerActive);
  } else {
    avoidRight(lineFollowerActive);
  }
  
  avoidingObstacle = false;
}

void avoidLeft(bool findLine) {
  // Back up slightly
  handleReverse();
  delay(500);
  
  // Turn left
  handleLeft();
  delay(AVOIDANCE_DELAY);
  
  // Move forward
  handleForward();
  delay(AVOIDANCE_DELAY * 1.5);
  
  // Turn right
  handleRight();
  delay(AVOIDANCE_DELAY);
  
  // Try to find line if in line following mode
  if (findLine) {
    unsigned long start = millis();
    while (millis() - start < 3000) {
      int pattern = readSensors();
      if (pattern != 0b00000) break;
      handleForward();
    }
  }
  
  handleStop();
}

void avoidRight(bool findLine) {
  // Back up slightly
  handleReverse();
  delay(400);
  
  // Turn right
  handleRight();
  delay(AVOIDANCE_DELAY);
  
  // Move forward
  handleForward();
  delay(AVOIDANCE_DELAY * 1.5);
  
  // Turn left
  handleLeft();
  delay(AVOIDANCE_DELAY);
  
  // Try to find line if in line following mode
  if (findLine) {
    unsigned long start = millis();
    while (millis() - start < 3000) {
      int pattern = readSensors();
      if (pattern != 0b00000) break;
      handleForward();
    }
  }
  
  handleStop();
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

void checkingConnection(){
  // Check Wi-Fi connection periodically
  if (millis() - lastWiFiCheck > wifiCheckInterval) {
    lastWiFiCheck = millis();
    if (WiFi.status() != WL_CONNECTED && !softAPStarted) {
      if (wasEverConnected) {
        playDisconnecto();  // Only play sound if it was connected before
      }
      lcd.clear(); lcd.setCursor(0, 0); lcd.print("WiFi Lost");
      lcd.setCursor(0, 1); lcd.print("Switching to AP");
      displayOLED("WiFi Lost", "Switching to AP");
      setColor(false, false, true);
      WiFi.disconnect(true);
      delay(1000);
      startSoftAPConfig();
      softAPStarted = true;
    } else {
      // Optionally, reconnect if the connection is lost
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
  lcd.setCursor(0, 0); lcd.print("SoftAP Mode");
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
    bool active = request->getParam("active") && request->getParam("active")->value() == "1";
    setLineFollower(active);
    request->send(200, "application/json", "{\"result\": \"ok\", \"line_active\": " + String(active ? "true" : "false") + "}");
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
void fetchCommandFromAPI() {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.begin(getUrl);
  int httpCode = http.GET();

  if (httpCode == 200) {
    String payload = http.getString();
    DynamicJsonDocument doc(512);
    DeserializationError error = deserializeJson(doc, payload);
    if (error) return;

    JsonObject command = doc["command"];
    handleAPICommand(command); // Pass the whole JSON object

    sendCommandStatus(1);  // Acknowledge command
  }
  http.end();
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
    playSFX("Ugh", 19, 19);  // Just plays track 19
    handleStop();
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
  // 1. Wall avoidance has priority
  int frontDist = getDistance();
  if (wallAvoiderActive && !avoidingObstacle && 
      frontDist > 0 && frontDist < avoidRange) {
    handleObstacle(avoidRange);
  }
  
  // 2. Line following runs when active and not avoiding
  else if (lineFollowerActive && !avoidingObstacle) {
    int pattern = readSensors();
    followLine(pattern);
  }
  
  // 3. If only wall avoidance is active, move forward
  else if (wallAvoiderActive && !avoidingObstacle) {
    handleForward();
  }

  checkingConnection(); // Check Your Connection if the connection fail go to softAP instead.
  
  if (millis() - lastFetch > fetchInterval) {
    fetchCommandFromAPI();
    lastFetch = millis();
  }

  delay(10);
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