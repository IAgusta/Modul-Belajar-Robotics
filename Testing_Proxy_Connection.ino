#include <WiFi.h>
#include <ESPAsyncWebServer.h>

// Replace with your WiFi credentials
const char* ssid = "MSI 5916";
const char* password = "msimodern";

AsyncWebServer server(80);

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println();
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // /move?dir=forward|backward|left|right|stop
  server.on("/move", HTTP_GET, [](AsyncWebServerRequest *request) {
    String dir = request->getParam("dir") ? request->getParam("dir")->value() : "stop";
    moveRobot(dir); // Implement your logic
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

  server.begin();
}

// ------------------------
// Dummy implementations
void moveRobot(String dir) {
  Serial.println("Move: " + dir);
  // Add your motor control logic here
}

void setLineFollower(bool active) {
  Serial.println("Line Follower: " + String(active ? "ON" : "OFF"));
  // Enable/disable line follower mode
}

void setWallAvoider(bool active) {
  Serial.println("Wall Avoider: " + String(active ? "ON" : "OFF"));
  // Enable/disable wall avoidance
}

void setRobotSpeed(int speed) {
  Serial.println("Set Speed: " + String(speed));
  // Adjust PWM or motor controller as needed
}

void setAvoidDistance(int dist) {
  Serial.println("Set Avoid Distance: " + String(dist));
  // Use value for your ultrasonic logic
}
// ------------------------

void loop() {
  // No need to put anything here for AsyncWebServer
}
