# Miners-Safety-Helmet


An IoT-based wearable safety helmet designed to enhance underground miner safety
through multi-sensor hazard detection, real-time audio-visual alerts, and cloud 
telemetry via ThingSpeak.

------------------------------------------------------------------------------
TABLE OF CONTENTS
------------------------------------------------------------------------------
1. Overview & Objectives
2. Key Features
3. Hardware Architecture & Components
4. Firmware & Software Details
5. Pin Mapping & Circuitry
6. System Workflow & Hazard Logic
7. PCB Design & Hardware Layout
8. Installation & Setup Guide
9. Testing & Validation Results
10. Future Enhancements
11. Project Team & Acknowledgments
12. References

------------------------------------------------------------------------------
1. OVERVIEW & OBJECTIVES
------------------------------------------------------------------------------
Underground mining involves extreme working conditions where hazardous gas leaks,
high temperature/humidity, sudden impacts/falls, and low overhead clearance pose
severe threats to human life. 

Objectives:
- Provide clear, immediate audio-visual alerts (buzzer chimes & flashing LED) 
  tailored for all workers, including illiterate miners.
- Integrate multi-sensor environmental and motion monitoring into a lightweight,
  wearable, low-cost helmet.
- Deliver real-time cloud data logging via ThingSpeak for supervisory oversight
  and remote safety management.

------------------------------------------------------------------------------
2. KEY FEATURES
------------------------------------------------------------------------------
- Environmental Sensing: High-accuracy temperature and humidity monitoring.
- Combustible Gas Detection: Real-time monitoring of dangerous gas/smoke levels.
- Motion & Impact Tracking: IMU-based 6-axis accelerometer and gyroscope tracking
  with complementary filtering to detect sudden tilts or impacts.
- Overhead Clearance Warnings: Ultrasonic distance detection to warn of low ceiling 
  hazards in tight shafts.
- Accessible Audio-Visual Alarms: High-frequency buzzer tones and blinking LED patterns.
- Manual SOS & Reset Controls: Tactile push-buttons for manual emergency activation 
  and alert reset with software debouncing.
- IoT Cloud Telemetry: Automatic Wi-Fi data upload to ThingSpeak every 15 seconds.

------------------------------------------------------------------------------
3. HARDWARE ARCHITECTURE & COMPONENTS
------------------------------------------------------------------------------
- Microcontroller: Espressif ESP32-S3 WROOM / ESP32 Development Board
  (Integrated Wi-Fi, ADC channels, native I2C support)
- Temperature & Humidity Sensor: DHT22 (Precision digital sensor)
- Combustible Gas Sensor: MQ-2 Gas Module (Smoke, CO, LPG detection)
- Motion & Inertial Sensor: MPU6050 6-Axis IMU (Accelerometer + Gyroscope)
- Proximity / Distance Sensor: HC-SR04 Ultrasonic Sensor
- Alert Outputs: Active Piezo Buzzer & High-Brightness LED
- Manual Controls: 2x Tactile Push-Buttons (Trigger SOS / Reset)
- Power Management: Rechargeable Battery Module & LDO Voltage Regulator

------------------------------------------------------------------------------
4. FIRMWARE & SOFTWARE DETAILS
------------------------------------------------------------------------------
- Language / Framework: C++ / Arduino Framework for ESP32
- Key Libraries:
  * WiFi.h & Wire.h (ESP32 Built-in)
  * ThingSpeak.h (ThingSpeak Communication)
  * DHT.h (Adafruit DHT Sensor Library)
  * Adafruit_MPU6050.h & Adafruit_Sensor.h (Adafruit MPU6050 Driver)

Data Filter Algorithm:
- Complementary Filter applied to MPU6050 readings:
  tiltDeg = 0.98 * gyroTilt + 0.02 * accelTilt
  (Distinguishes dynamic jerk/vibration from sustained tilt/impacts).

------------------------------------------------------------------------------
5. PIN MAPPING & CIRCUITRY
------------------------------------------------------------------------------
Component           | Module Pin | ESP32 GPIO / Pin Code
--------------------|------------|----------------------
DHT22 Temp/Hum      | DATA       | GPIO 2 (D0)
MQ-2 Gas Sensor     | AOUT       | GPIO 3 (D1 / ADC1)
Piezo Buzzer        | Positive   | GPIO 8 (D8)
Status LED          | Anode      | GPIO 10 (D10)
Manual Trigger SOS  | Pin        | GPIO 5 (D3 - INPUT_PULLUP)
Alarm Reset Button  | Pin        | GPIO 9 (D9 - INPUT_PULLUP)
HC-SR04 Ultrasonic  | TRIG       | GPIO 4 (D2)
HC-SR04 Ultrasonic  | ECHO       | GPIO 6 (D4)
MPU6050 IMU         | SDA        | GPIO 6 (D4)
MPU6050 IMU         | SCL        | GPIO 7 (D5)

------------------------------------------------------------------------------
6. SYSTEM WORKFLOW & HAZARD LOGIC
------------------------------------------------------------------------------
Sensors are sampled every 500ms. An alarm state triggers if ANY of the 
following thresholds are exceeded:

Parameter              | Threshold Level
-----------------------|------------------------------------
Temperature            | > 35.0 °C
Humidity               | > 80.0 %
Gas Concentration      | > 1800 Raw ADC Value
Tilt Angle             | > 50.0 Degrees
Overhead Clearance     | < 30.0 cm ( sustained for > 30 mins )
Manual SOS Trigger     | Button Pressed (LOW)

Cloud Upload:
- Upload Interval: Every 15 seconds to ThingSpeak.
- Fields Map:
  Field 1: Temperature (°C)
  Field 2: Humidity (%)
  Field 3: Gas Raw Reading
  Field 4: Calculated Tilt Angle (Degrees)
  Field 5: Clearance Distance (cm)
  Field 6: Hazard Alarm State (1 = Active, 0 = Normal)
  Field 7: Manual Alarm State (1 = Active, 0 = Off)

------------------------------------------------------------------------------
7. PCB DESIGN & HARDWARE LAYOUT
------------------------------------------------------------------------------
- Two-layer custom PCB designed using Altium Designer / KiCad.
- Features solid ground plane pour on top and bottom layers for noise immunity.
- Short I2C traces to avoid clock jitter and signal attenuation.
- Isolated high-frequency/noisy routing from analog sensor traces.
- Keep-out zone maintained under ESP32 PCB antenna to optimize Wi-Fi range.

------------------------------------------------------------------------------
8. INSTALLATION & SETUP GUIDE
------------------------------------------------------------------------------
1. Prerequisites:
   - Install Arduino IDE (v2.0 or higher recommended).
   - Install ESP32 Board Manager core in Arduino IDE.
   - Install required libraries: `DHT sensor library`, `Adafruit MPU6050`, 
     `Adafruit Unified Sensor`, `ThingSpeak`.

2. Configuration:
   - Open `miner_safety_helmet.ino`.
   - Update Wi-Fi Credentials:
     const char* ssid = "YOUR_WIFI_SSID";
     const char* password = "YOUR_WIFI_PASSWORD";
   - Update ThingSpeak Credentials:
     unsigned long channelID = YOUR_CHANNEL_ID;
     const char* writeAPIKey = "YOUR_WRITE_API_KEY";

3. Uploading Firmware:
   - Connect ESP32 board via USB.
   - Select correct COM Port and Board model (e.g., ESP32 Dev Module / ESP32-S3).
   - Compile and upload the sketch.

------------------------------------------------------------------------------
9. TESTING & VALIDATION RESULTS
------------------------------------------------------------------------------
- Environmental Chamber Testing: Successfully triggered alerts at > 35°C and 
  elevated smoke exposure.
- Motion Simulation: Complementary filter correctly filtered high-frequency 
  jerks while catching sustained tilt (>50°).
- Distance Validation: Accurate proximity alerts up to 30 cm clearance limit.
- Cloud Integration: Continuous 24-hour test verified 99.8% successful payload 
  delivery to ThingSpeak.

------------------------------------------------------------------------------
10. FUTURE ENHANCEMENTS
------------------------------------------------------------------------------
- Edge AI / TinyML: On-device gesture and fall-pattern recognition.
- Hybrid Long-Range Connectivity: LoRaWAN / GSM fallback for deep underground 
  tunnels without Wi-Fi.
- Predictive Analytics: Machine learning algorithms for environmental trend analysis.
- Advanced Web Dashboard: Shift-wise safety analytics and real-time GPS tracking.

------------------------------------------------------------------------------
11. PROJECT TEAM & ACKNOWLEDGMENTS
------------------------------------------------------------------------------
Project Members:
- Rohit Amrut Patki
- Vaibhav Hanumant Waghmode
- Omkar Pandurang Barakale

Project Guide:
- Prof. Mrs. Meena Karad

Department & Institution:
- Department of Electronics and Telecommunication Engineering (E&TC)
- D. Y. Patil College of Engineering, Akurdi, Pune (Academic Year 2025–2026)

------------------------------------------------------------------------------
12. REFERENCES
------------------------------------------------------------------------------
- IEEE Xplore: "IoT-Enabled Safety Helmet for Coal Miners" 
  (https://ieeexplore.ieee.org/document/10561018/)
- KSCST Project Reports: "An IoT Based Smart Helmet for Industrial And Mining Workers"
- IJRASET: "IOT Mining Tracking & Worker Safety Helmet" (2024)
- Random Nerd Tutorials: "ESP32 Publish Sensor Readings to ThingSpeak"
==============================================================================
