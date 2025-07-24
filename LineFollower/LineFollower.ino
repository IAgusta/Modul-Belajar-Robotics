#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/* ----------  LCD  ---------- */
LiquidCrystal_I2C lcd(0x27, 16, 2);

/* ----------  5-IR SENSOR PINS  ---------- */
const int irLeft2  = 18;
const int irLeft1  = 5;
const int irCenter = 17;
const int irRight1 = 16;
const int irRight2 = 4;

/* ----------  MOTOR PINS (unchanged)  ---------- */
const int motor1Pin1 = 27;
const int motor1Pin2 = 26;
const int enable1Pin = 14;
const int motor2Pin1 = 33;
const int motor2Pin2 = 25;
const int enable2Pin = 32;

const int freq = 30000;
const int resolution = 8;

const int BASE_SPEED      = 200;   // forward PWM 0-255
const int LOST_TIMEOUT_MS = 2000;  // max time to search
const byte FINISH_PATTERN = 0b11111;  // all sensors on finish line

int   lastError = 0;          // for remembering direction
bool  finished  = false;      // true when finish line detected
unsigned long lostStart = 0;  // time when line disappeared

int prevError = 0;   // for crude derivative
float integral = 0; // Integral term

const float KP = 0.15;  
const float KD = 0.9;
const float KI = 0.0005;

bool          lastWasLeft  = false;   // true = last correction was left
bool          lineWasLost  = false;

bool lineFollowerActive = true;

/* ----------  SETUP  ---------- */
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);    // your I²C pins
  lcd.begin();
  lcd.backlight();
  lcd.print(F("5-IR READY"));
  delay(1000);

  initializeMotorDriver();
  initializeInfrared();
}

/* ----------  LOOP  ---------- */
void loop() {
  if (lineFollowerActive) {
    int pattern = readSensors();
    followLine(pattern);
    displayLCD(pattern);
  } else {
    handleStop();
  }
}

/* ----------  SENSOR READ (5-BIT)  ---------- */
int readSensors() {
  int bits = 0;
  bits |= (!digitalRead(irLeft2))  << 4;   // bit4
  bits |= (!digitalRead(irLeft1))  << 3;   // bit3
  bits |= (!digitalRead(irCenter)) << 2;   // bit2
  bits |= (!digitalRead(irRight1)) << 1;   // bit1
  bits |= (!digitalRead(irRight2));        // bit0
  return bits;                             // 0bxxxxx  (5 bits)
}

void followLine(int pattern) {
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

void handleStop() {
  ledcWrite(enable1Pin, 0);
  ledcWrite(enable2Pin, 0);
}

/* stubs so the rest compiles */
void handleForward() {}
void handleLeft()   {}
void handleRight()  {}

/* ----------  LCD DISPLAY  ---------- */
void displayLCD(int pattern) {
  lcd.setCursor(0, 0);
  lcd.print(F("S:"));
  for (int i = 4; i >= 0; i--) {         // print 5 bits
    lcd.print((pattern >> i) & 1 ? '1' : '0');
  }
  lcd.print(F(" "));

  switch (pattern) {
    case 0b00100: lcd.print(F("FWD")); break;
    case 0b00110:
    case 0b00111:
    case 0b00011: lcd.print(F("RGT")); break;
    case 0b01100:
    case 0b11100:
    case 0b11000: lcd.print(F("LFT")); break;
    case 0b00000:
    case 0b11111: lcd.print(F("LOST")); break;
    default:  break;
  }

  lcd.setCursor(0, 1);
  lcd.print(F("Pat:0b"));
  for (int i = 4; i >= 0; i--) {
    lcd.print((pattern >> i) & 1);
  }
  lcd.print(F("   "));
}

void initializeMotorDriver() {
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  ledcAttach(enable1Pin, freq, resolution);
  ledcAttach(enable2Pin, freq, resolution);
  ledcWrite(enable1Pin, 0);
  ledcWrite(enable2Pin, 0);
}

void initializeInfrared() {
  pinMode(irLeft2, INPUT);
  pinMode(irLeft1, INPUT);
  pinMode(irCenter, INPUT);
  pinMode(irRight1, INPUT);
  pinMode(irRight2, INPUT);
}
