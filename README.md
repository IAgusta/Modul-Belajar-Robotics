# Modul Belajar Robotics
A comprehensive ESP32-based robotics learning module for **alpaca-labs.online** with web-based robot control, featuring various autonomous behaviors and remote control capabilities.

## Overview
This repository contains a complete robotics learning platform built around the ESP32 microcontroller. The system provides both manual remote control via web interface and autonomous behaviors like wall avoidance and line following. Perfect for learning robotics fundamentals, IoT integration, and embedded programming.

## Features
- **Web-based Robot Control**: Control your robot remotely through alpaca-labs.online interface
- **Proxy Testing**: Built-in proxy functionality for testing network connections
- **Fetch API Integration**: RESTful API endpoints for seamless web communication
- **Wall Avoider**: Autonomous obstacle detection and avoidance using ultrasonic sensors
- **Line Follower**: Precise line tracking using IR sensors
- **Real-time Control**: Instant response via WiFi connectivity
- **Modular Code Structure**: Easy to understand and modify for learning purposes

## Hardware Requirements
- ESP32 Development Board (ESP32-WROOM-32 or similar)
- Motor Driver (L298N or similar)
- DC Motors (2x for differential drive)
- Ultrasonic Sensor (HC-SR04 for wall detection)
- IR Sensors (5 Arrays for line following)
- Servo Motor (optional, for sensor mounting)
- Jumper wires and breadboard
- Power supply (7.4V Li-Po battery recommended)
- Chassis and wheels

## Quick Start

### 1. Hardware Setup
1. Connect the motors to the motor driver
2. Wire the ultrasonic sensor for obstacle detection
3. Install IR sensors for line following
4. Connect all components to the ESP32 according to the wiring diagram

### 2. Software Installation
1. Clone this repository:
```bash
git clone https://github.com/IAgusta/Modul-Belajar-Robotics.git
cd Modul-Belajar-Robotics
```

2. Install required libraries in Arduino IDE:
   - ESP32 Board Package
   - WiFi Library
   - WebServer Library
   - ArduinoJson Library
   - U8g2lib
   - LiquidCrystal_I2C
   - ESP32Servo
   - SPIFFS
   - DFPlayerMini_Fast

3. Configure WiFi credentials in:
```
String ssidList[MAX_NETWORKS] = {
  "SSID 1",
  "SSID 2",
  "SSID 3"
};
String passwordList[MAX_NETWORKS] = {
  "PASSWORD 1",
  "PASSWORD 2",
  "PASSWORD 3"
};
```

### 3. Upload and Run

1. Open `FullCode/FullCode.ino` in Arduino IDE
2. Select your ESP32 board and port
3. Upload the code to your ESP32
4. Open Serial Monitor to get the robot's IP address
5. Navigate to the IP address in your browser to access the control interface

## Control Interface
### Web Control Panel
Access the robot through your browser at the ESP32's IP address. The interface provides:
- **Directional Controls**: Forward, backward, left, right movement
- **Speed Control**: Adjustable motor speed slider
- **Mode Selection**: Switch between manual and autonomous modes

## Operating Modes
### 1. Manual Control Mode
- Direct control via web interface
- Real-time response to user commands
- Speed and direction control
- Emergency stop functionality

### 2. Wall Avoider Mode
- Autonomous navigation with obstacle avoidance
- Uses ultrasonic sensor for distance measurement
- Implements basic pathfinding algorithm
- Configurable detection sensitivity

### 3. Line Follower Mode
- Follows black lines on white surfaces
- PID-controlled steering for smooth tracking
- Adjustable following speed
- Corner and intersection handling

## Configuration
### Pin Configuration
Modify pin assignments in:

```
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
```

### Network Configuration
Configure network settings for your environment:

```cpp
#define WEB_SERVER_PORT   80
#define PROXY_ENABLED     true
#define API_TIMEOUT       5000
```

## 🧪 Testing

### Proxy Testing
Run the proxy test to verify network connectivity:
```cpp
// Load and run Testing Code/Testing_Proxy_Connection/Testing_Proxy_Connection.ino
```

### API Testing
Test the fetch API functionality:
```cpp
// Load and run Testing Code/Testing_API_Fetching/Testing_API_Fetching.ino
```

## 📚 Learning Resources
This project is designed for educational purposes. Each module includes:
- **Commented Code**: Extensive comments explaining functionality
- **Step-by-step Tutorials**: Progressive learning approach
- **Troubleshooting Guide**: Common issues and solutions
- **Extension Ideas**: Suggestions for further development

## Troubleshooting
### Common Issues
**Robot not connecting to WiFi:**
- Check WiFi credentials in config.h
- Ensure router is in range
- Verify ESP32 is not in deep sleep mode

**Web interface not loading:**
- Confirm ESP32 IP address in Serial Monitor
- Check if port 80 is accessible
- Try accessing from different browser

**Sensors not responding:**
- Verify wiring connections
- Run sensor calibration script
- Check power supply voltage

**Motors not moving:**
- Check motor driver connections
- Verify battery charge level
- Test motor driver with simple code

## Author

**IAgusta** - [GitHub Profile](https://github.com/IAgusta)

## Support
For support and questions:
- Create an issue in this repository
- Visit [alpaca-labs.online](https://alpaca-labs.online) for platform-specific help
- Or email on helper@alpaca-labs.online
