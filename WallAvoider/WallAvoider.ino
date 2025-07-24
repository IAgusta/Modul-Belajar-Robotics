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
int dutyCycle = 200; // Default speed

// Ultrasonics Adjustment
const int TrigPin = 2;
const int EchoPin = 19;
int avoidRange = 15; // Default Range

// Variables for distance measurement
long duration;
int distance;

bool wallAvoiderActive = true;
// ------------------  WALL AVOIDER  ------------------
enum WallAvoiderState {
  WA_DRIVE,     // normal forward
  WA_BACK,      // backing up
  WA_TURN,      // turning away
  WA_DELAY      // small pause before resume
};

WallAvoiderState waState = WA_DRIVE;
unsigned long waTimer = 0;
bool turnRightNext = true;          // direction flag

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


void handleForward() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  ledcWrite(enable1Pin, dutyCycle);
  ledcWrite(enable2Pin, dutyCycle);
}

void handleLeft() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  ledcWrite(enable1Pin, dutyCycle);
  ledcWrite(enable2Pin, dutyCycle);
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
  ledcWrite(enable1Pin, dutyCycle);
  ledcWrite(enable2Pin, dutyCycle);
}

void handleReverse() {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  ledcWrite(enable1Pin, dutyCycle);
  ledcWrite(enable2Pin, dutyCycle);
}


// Setup ultrasonic
void initializeUltrasonic(){
  pinMode(TrigPin, OUTPUT);
  pinMode(EchoPin, INPUT);
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

void wallAvoider(int avoidRange) {
  if (!wallAvoiderActive) return;

  int dist = getDistance();

  switch (waState) {
    case WA_DRIVE:
      handleForward();
      if (dist > 0 && dist < avoidRange) {
        handleStop();
        turnRightNext = random(2); // 0 = left, 1 = right
        waState = WA_BACK;
        waTimer = millis();
      }
      break;

    case WA_BACK:
      if (millis() - waTimer < 2000) {
        handleReverse();
      } else {
        handleStop();
        waState = WA_TURN;
        waTimer = millis();
      }
      break;

    case WA_TURN:
      if (millis() - waTimer < 1000) {
        turnRightNext ? handleRight() : handleLeft();
      } else {
        handleStop();
        waState = WA_DELAY;
        waTimer = millis();
      }
      break;

    case WA_DELAY:
      if (millis() - waTimer > 200) {
        waState = WA_DRIVE;
      }
      break;
  }
}


void setup(){
  initializeMotorDriver();
  initializeUltrasonic();
}

void loop(){
  wallAvoider(avoidRange);
  delay(10);
}