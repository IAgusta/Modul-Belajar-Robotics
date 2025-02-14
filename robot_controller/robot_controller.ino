#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "ESP32_Robot_Car";
const char* password = "12345678";

WebServer server(80);

int motor1Pin1 = 27; 
int motor1Pin2 = 26; 
int enable1Pin = 14;

int motor2Pin1 = 33; 
int motor2Pin2 = 25; 
int enable2Pin = 32;

const int freq = 30000;
const int resolution = 8;
int dutyCycle = 0;

String valueString = String(0);

void setup() {
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  ledcAttach(enable1Pin, freq, resolution);
  ledcAttach(enable2Pin, freq, resolution);
    
  ledcWrite(enable1Pin, 0);
  ledcWrite(enable2Pin, 0);

  Serial.begin(115200);

  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  server.on("/", handleRoot);
  server.on("/forward", handleForward);
  server.on("/left", handleLeft);
  server.on("/stop", handleStop);
  server.on("/right", handleRight);
  server.on("/reverse", handleReverse);
  server.on("/speed", handleSpeed);

  server.begin();
}

void loop() {
  server.handleClient();
}

void handleRoot() {
  const char html[] PROGMEM = R"rawliteral(
  <!DOCTYPE HTML>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="icon" href="data:,">
  <style>
    html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center; }
    .button {
      -webkit-user-select: none; -moz-user-select: none; -ms-user-select: none; user-select: none;
      background-color: #4CAF50; border: none; color: white; padding: 12px 28px; text-decoration: none;
      font-size: 26px; margin: 1px; cursor: pointer; transition: background-color 0.3s;
    }
    .button:active { background-color: #45a049; }
    .button2 { background-color: #555555; }
    .button2:active { background-color: #444444; }
    .slider { width: 80%; margin: 20px auto; }
    .slider input[type="range"] { width: 100%; }
  </style>
  <script>
    function moveForward() { fetch('/forward'); }
    function moveLeft() { fetch('/left'); }
    function stopRobot() { fetch('/stop'); }
    function moveRight() { fetch('/right'); }
    function moveReverse() { fetch('/reverse'); }

    function updateMotorSpeed(pos) {
      document.getElementById('motorSpeed').innerHTML = pos;
      fetch(`/speed?value=${pos}`);
    }

    function setupButton(button, action) {
      button.addEventListener('mousedown', action);
      button.addEventListener('touchstart', action);
      button.addEventListener('mouseup', stopRobot);
      button.addEventListener('touchend', stopRobot);
    }

    window.onload = function () {
      setupButton(document.getElementById('forward'), moveForward);
      setupButton(document.getElementById('left'), moveLeft);
      setupButton(document.getElementById('right'), moveRight);
      setupButton(document.getElementById('reverse'), moveReverse);
      setupButton(document.getElementById('stop'), stopRobot);
    };
  </script>
</head>
<body>
  <h1>ESP32 Motor Control</h1>
  <p>
    <button id="forward" class="button">FORWARD</button>
  </p>
  <div style="clear: both;">
    <p>
      <button id="left" class="button">LEFT</button>
      <button id="stop" class="button button2">STOP</button>
      <button id="right" class="button">RIGHT</button>
    </p>
  </div>
  <p>
    <button id="reverse" class="button">REVERSE</button>
  </p>
  <div class="slider">
    <p>Motor Speed: <span id="motorSpeed">0</span></p>
    <input type="range" min="0" max="100" step="25" id="motorSlider" oninput="updateMotorSpeed(this.value)" value="0"/>
  </div>
</body>
</html>)rawliteral";
  server.send(200, "text/html", html);
}

void handleForward() {
  Serial.println("Forward");
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH); 
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  server.send(200);
}

void handleLeft() {
  Serial.println("Left");
  digitalWrite(motor1Pin1, LOW); 
  digitalWrite(motor1Pin2, LOW); 
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  server.send(200);
}

void handleStop() {
  Serial.println("Stop");
  digitalWrite(motor1Pin1, LOW); 
  digitalWrite(motor1Pin2, LOW); 
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);   
  server.send(200);
}

void handleRight() {
  Serial.println("Right");
  digitalWrite(motor1Pin1, LOW); 
  digitalWrite(motor1Pin2, HIGH); 
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);    
  server.send(200);
}

void handleReverse() {
  Serial.println("Reverse");
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW); 
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);          
  server.send(200);
}

void handleSpeed() {
  if (server.hasArg("value")) {
    valueString = server.arg("value");
    int value = valueString.toInt();
    if (value == 0) {
      ledcWrite(enable1Pin, 0);
      ledcWrite(enable2Pin, 0);
      digitalWrite(motor1Pin1, LOW); 
      digitalWrite(motor1Pin2, LOW); 
      digitalWrite(motor2Pin1, LOW);
      digitalWrite(motor2Pin2, LOW);   
    } else { 
      dutyCycle = map(value, 25, 100, 200, 255);
      ledcWrite(enable1Pin, dutyCycle);
      ledcWrite(enable2Pin, dutyCycle);
      Serial.println("Motor speed set to " + String(value));
    }
  }
  server.send(200);
}