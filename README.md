# Industrial IoT Smart HVAC Controller

Author: Ahnaf Mohammed  
This project is an ESP32-based Industrial IoT Smart HVAC Controller created for Module: COMP50069 Hardware, Micro-controllers and Sensors.

## Wokwi Simulation Link
https://wokwi.com/projects/463680487131753473

## Overview
The system reads temperature and humidity from a DHT22 sensor and reads analogue load from a potentiometer using the ESP32 ADC. It uses FreeRTOS tasks, interrupt-driven buttons, mutex-protected shared state, ADC calibration/filtering, LCD output, serial diagnostics and fail-safe safety logic.

## Main Features
- ESP32 DevKit V1 Wokwi simulation
- DHT22 temperature and humidity monitoring
- Potentiometer as 12-bit ADC load sensor
- Two simulated relay modules for fan/cooling and heater actuator outputs
- Emergency and reset buttons using interrupts
- FreeRTOS task architecture
- Mutex-protected SystemState struct
- LCD1602 I2C display
- Serial diagnostic logging
- Heating, cooling, idle, ventilation, fault and emergency modes
- Latched overheat/emergency faults with manual safe reset
- Software task-heartbeat monitoring and button-flood protection

## How to Test
- IDLE/OFF: 24 C, 45% humidity, 50% load
- HEATING: 15 C, 45% humidity, 50% load
- COOLING: 30 C, 45% humidity, 50% load
- VENT: 24 C, load above 75%
- OVERHEAT: 42 C or above; heater and fan outputs forced off, red fault LED and buzzer on
- EMERGENCY: press the red emergency button; fault output and buzzer latch until reset
- RESET: return to safe values below 40 C with valid sensor data, then press the green reset button

## Files
- sketch.ino: main Wokwi/Arduino file with FreeRTOS tasks
- diagram.json: Wokwi circuit wiring
- hvac_types.h: shared structs, constants and enums
- hvac_logic.h: function declarations
- hvac_logic.cpp: HVAC logic, ADC calibration and safety decisions
- libraries.txt: required Wokwi libraries
