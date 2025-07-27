#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

/*
  ========== CONFIGURATION ==========
*/
const char* ssid     = "YOUR_SSID";            // Your SSID
const char* password = "YOUR_PASSWORD";           // Your password

const char* apiKey = "YOUR_API_KEY";
const char* url = "https://alpaca-labs.online";
String getUrl = String(url) + "/api/robot/command/" + apiKey;
String postUrl = String(url) + "/api/robot/command-status/" + apiKey;
const unsigned long fetchInterval = 2000;

unsigned long lastFetch = 0;

/*
  ========== HANDLE API COMMAND ==========
  Replace with your own logic!
*/
void handleAPICommand(JsonObject command) {
  String action = command["action"] | "none";
  
  Serial.println("=== EXECUTING COMMAND ===");
  Serial.print("Action: ");
  Serial.println(action);

  // Add actual command execution here
  if (action == "forward") {
    Serial.println("🚗 EXECUTING: Moving forward!");
    // Add your motor control code here
    // digitalWrite(motorPin1, HIGH);
    // digitalWrite(motorPin2, LOW);
    
  } else if (action == "left") {
    Serial.println("↩️ EXECUTING: Turning left!");
    // Add your motor control code here
    
  } else if (action == "right") {
    Serial.println("↪️ EXECUTING: Turning right!");
    // Add your motor control code here
    
  } else if (action == "stop") {
    Serial.println("⛔ EXECUTING: Stopping!");
    // Add your motor control code here
    // digitalWrite(motorPin1, LOW);
    // digitalWrite(motorPin2, LOW);
    
  } else if (action == "line") {
    bool reversed = command["reverse"] | false;
    Serial.printf("📏 EXECUTING: Line follower mode: %s\n", reversed ? "REVERSED" : "NORMAL");
    // Add your line following code here
    
  } else {
    Serial.println("❓ UNKNOWN COMMAND - No action taken");
    return;
  }
  
  Serial.println("✅ Command execution completed!");
  Serial.println("========================");
}

/*
  ========== SIMPLIFIED API FETCHER ==========
*/
void fetchAndExecuteCommand() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("❌ WiFi not connected!");
    return;
  }

  // Check if it's time to fetch
  if (millis() - lastFetch < fetchInterval) {
    return;
  }

  Serial.println("\n[API] Checking for new commands...");
  
  HTTPClient http;
  http.begin(getUrl);
  http.setTimeout(10000); // 10 second timeout
  http.addHeader("User-Agent", "ESP32-Robot");
  http.addHeader("Accept", "application/json");
  http.addHeader("Connection", "close");

  Serial.println("[HTTP] Sending GET request...");
  int httpCode = http.GET();
  
  Serial.print("[HTTP] Response Code: ");
  Serial.println(httpCode);

  if (httpCode == 200) {
    Serial.println("[HTTP] ✅ Command available!");
    
    // Get the response immediately
    String payload = http.getString();
    Serial.println("\n[HTTP] 📨 Raw Response:");
    Serial.println("--- START ---");
    Serial.println(payload);
    Serial.println("--- END ---");
    
    // Process the JSON immediately
    if (payload.length() > 0) {
      Serial.println("\n[JSON] 🔄 Processing command...");
      
      DynamicJsonDocument doc(1024);
      DeserializationError error = deserializeJson(doc, payload);

      if (error) {
        Serial.print("[JSON] ❌ Parse error: ");
        Serial.println(error.c_str());
        Serial.println("[JSON] Payload length: " + String(payload.length()));
      } else {
        Serial.println("[JSON] ✅ Successfully parsed JSON");
        
        // Debug: Print entire JSON structure
        Serial.println("[JSON] Complete structure:");
        serializeJsonPretty(doc, Serial);
        Serial.println();
        
        if (doc.containsKey("command")) {
          JsonObject command = doc["command"];
          Serial.println("[JSON] 📋 Found command object!");
          
          // Execute the command
          handleAPICommand(command);
          
          // Send completion status
          Serial.println("[API] 📤 Sending completion status...");
          sendCommandStatus(2);
          
        } else {
          Serial.println("[JSON] ❌ No 'command' key found!");
          Serial.println("[JSON] Available keys:");
          for (JsonPair kv : doc.as<JsonObject>()) {
            Serial.print("  - ");
            Serial.println(kv.key().c_str());
          }
        }
      }
    } else {
      Serial.println("[HTTP] ❌ Empty response body!");
    }
    
  } else if (httpCode == 204) {
    Serial.println("[HTTP] 📭 No new commands (204)");
    
  } else if (httpCode > 0) {
    Serial.printf("[HTTP] ⚠️ Unexpected response: %d\n", httpCode);
    String errorBody = http.getString();
    if (errorBody.length() > 0) {
      Serial.println("[HTTP] Error body: " + errorBody);
    }
    
  } else {
    Serial.printf("[HTTP] ❌ Connection failed: %d\n", httpCode);
    Serial.println("[HTTP] Error: " + http.errorToString(httpCode));
  }

  http.end();
  lastFetch = millis();
  
  Serial.println("[API] 🔄 Ready for next check\n");
}

void sendCommandStatus(int statusCode) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[POST] ❌ WiFi not connected!");
    return;
  }

  Serial.println("[POST] 📤 Sending status update...");
  
  HTTPClient http;
  http.begin(postUrl);
  http.setTimeout(10000);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("User-Agent", "ESP32-Robot");
  http.addHeader("Connection", "close");

  String json = "{\"status\": " + String(statusCode) + "}";
  Serial.println("[POST] Payload: " + json);
  
  int code = http.POST(json);
  Serial.print("[POST] Response Code: ");
  Serial.println(code);

  if (code > 0) {
    String response = http.getString();
    Serial.println("[POST] Response: " + response);
    
    if (code == 200) {
      Serial.println("[POST] ✅ Status updated successfully!");
    } else {
      Serial.printf("[POST] ⚠️ Unexpected response code: %d\n", code);
    }
  } else {
    Serial.printf("[POST] ❌ Failed to send: %d\n", code);
    Serial.println("[POST] Error: " + http.errorToString(code));
  }

  http.end();
}

/*
  ========== WIFI CONNECTION ==========
*/
void connectWiFi() {
  Serial.println("\n🌐 Connecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  Serial.print("Connecting");
  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 15000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi Connected!");
    Serial.print("📡 IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("📶 Signal Strength: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  } else {
    Serial.println("\n❌ WiFi Connection Failed!");
  }
}

/*
  ========== SETUP ==========
*/
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n🤖 ESP32 Robot Command Handler Starting...");
  Serial.println("==========================================");
  
  // Initialize your pins here
  // pinMode(motorPin1, OUTPUT);
  // pinMode(motorPin2, OUTPUT);
  
  connectWiFi();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("🚀 System ready!");
    Serial.println("📡 Listening for commands...");
    
    // Test connection immediately
    Serial.println("\n🧪 Testing API connection...");
    fetchAndExecuteCommand();
  }
  
  Serial.println("==========================================\n");
}

/*
  ========== LOOP ==========
*/
void loop() {
  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("❌ WiFi disconnected! Reconnecting...");
    connectWiFi();
    delay(5000);
    return;
  }
  
  // Fetch and handle commands
  fetchAndExecuteCommand();
  
  // Small delay
  delay(100);
}