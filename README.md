# Smart Greenhouse – ESP32 IoT Project

**Automated smart greenhouse with real-time optimization and cloud-based control**

---

## Table of Contents

- [Project Overview](#project-overview)  
- [Features](#features)  
- [System Architecture](#system-architecture)  
- [Components](#components)  
- [Protocols & Tools](#protocols--tools)  
- [Setup Instructions](#setup-instructions)  
- [Usage](#usage)  
- [License](#license)  

---

## Project Overview

This project implements an automated smart greenhouse using an ESP32 microcontroller. The system monitors environmental conditions such as temperature, humidity, soil moisture, light intensity, and water level. Actuators such as fans, water pumps, growth LEDs, and servo-controlled windows adjust conditions to optimize plant growth.

The system uses multitasking on ESP32 with FreeRTOS to manage real-time operations and communicates with a cloud-based Node-RED dashboard through the MQTT protocol for monitoring and remote control.

All hardware components are simulated in **Wokwi**, so no physical setup is required to test the system.

---

## Features

- Real-time monitoring and control of greenhouse conditions  
- Multi-tasking support with FreeRTOS for ESP32  
- MQTT communication with HiveMQ broker for secure data transfer  
- Cloud dashboard management via Node-RED  
- Automatic and manual modes for system operation  
- Simulation environment using Wokwi  

---

## System Architecture

The project follows a **Cyber-Physical System (CPS)** model:

**Sensors:**

- **DHT22:** Measures temperature and humidity  
- **Soil Moisture Sensor:** Measures soil moisture level  
- **Ultrasonic Sensor:** Measures water tank level  
- **LDR:** Measures ambient light intensity  

**Actuators:**

- **Fan:** Relay-controlled  
- **Water Pump:** Relay-controlled  
- **Growth LED:** PWM-controlled  
- **Servo Motor:** Controls greenhouse window  

**Dashboard & Communication:**

- **Node-RED dashboard** for monitoring and control  
- **MQTT protocol** for publishing/subscribing sensor data and commands  

---

## Components

- ESP32 Microcontroller  
- DHT22 Sensor  
- Soil Moisture Sensor (simulated by potentiometer)  
- Ultrasonic Sensor  
- LDR Sensor  
- Relay modules for fan and pump  
- Growth LED (PWM controlled)  
- Servo Motor  
- OLED Display for local status visualization  
- Push button for manual reset and mode switching  

---

## Protocols & Tools

- **MQTT (Message Queuing Telemetry Transport):** Lightweight publish/subscribe messaging protocol for IoT devices  
- **Node-RED:** Visual programming environment for creating dashboards and managing IoT flows  
- **Wokwi Simulator:** Hardware simulation environment  
- **FreeRTOS:** Real-time task management on ESP32  

---

## Setup Instructions

### HiveMQ Broker

1. Create a free account on [HiveMQ](https://www.hivemq.com/?utm_source=adwords&utm_campaign=&utm_term=hive%20mq&utm_medium=ppc&hsa_tgt=kwd-1156701646538&hsa_cam=22496895017&hsa_src=g&hsa_net=adwords&hsa_kw=hive%20mq&hsa_ad=653297813212&hsa_grp=185625595384&hsa_ver=3&hsa_acc=3585854406&hsa_mt=e&gad_source=1&gad_campaignid=22496895017&gbraid=0AAAAADusSG6q7OGCOkTVrC9VUQhCkS-kV&gclid=CjwKCAjw89jGBhB0EiwA2o1On16HDdhYvWlA51tcHHlOKC258SsvpQez0zkLK5WYRzRaDV0Sv-vZiBoC9XUQAvD_BwE) and obtain a broker URL, username, and password.  

### Secure MQTT Connection

2. Enable TLS and provide certificates for encrypted communication.  

### Node-RED Installation

3. Install [Node.js](https://nodejs.org/en/download), then install [Node-RED](https://nodered.org/docs/getting-started/local) following official instructions.
4. Run the command below in your terminal to start Node-RED:

```bash
node-red
```

5. Follow the created link for the node-red server.
6. Install **node-red-dashboard** nodes for UI widgets
7. Import the flow.json file in your flows section and deploy the flow.
  

### ESP32 Configuration

5. Flash the ESP32 code using **PlatformIO** or **Arduino IDE**.  
6. Set up your MQTT credentials and broker URL.
7. To run the code as simulated in **Wokwi**, you have to follow the instructions in the [Link](https://docs.wokwi.com/vscode/getting-started).
 

---

## Usage

- **Manual Mode:** Control actuators directly from the Node-RED dashboard.  
- **Auto Mode:** The system optimizes greenhouse conditions based on sensor inputs and predefined scenarios:

  - Fan control based on temperature and humidity  
  - Water pump activation based on soil moisture and water level  
  - Growth LED control based on ambient light  
  - Servo window control based on temperature and humidity thresholds  

- Real-time sensor data and actuator status are displayed on the dashboard and OLED display.  

---

