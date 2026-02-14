# Smart Greenhouse Monitoring & Control System 🌿
> **Automated Environmental Control System based on STM32F103 (BluePill)**

## 📋 Project Overview
This project is an embedded system designed to monitor and automate a greenhouse environment. It utilizes an **STM32F103C8T6** microcontroller to track temperature, light intensity, and soil moisture levels. The system features two operational modes: **AUTO** for intelligent self-regulation and **MANUAL** for remote user control via UART.

## ✨ Key Features
- **Real-time Monitoring:** Tracks temperature (DS18B20), ambient light (LDR), and soil moisture (Potentiometer).
- **Dual Mode Operation:**
  - **AUTO:** Autonomous control of fans, pumps, heaters, and lighting based on environmental thresholds.
  - **MANUAL:** Direct control of actuators via Serial Terminal commands.
- **Precision Control:** PWM-based regulation for heating and lighting systems.
- **Visual Feedback:** Integrated OLED display for local data visualization and Virtual Terminal for remote logging.
- **Safety Interlocks:** Digital input for door status monitoring.

## 🛠️ Hardware Requirements
- **Microcontroller:** STM32F103C8T6 (BluePill)
- **Sensors:** 
  - DS18B20 (Digital Temperature)
  - LDR (Analog Light)
  - 10K Potentiometer (Soil Moisture Simulation)
- **Actuators (LED Simulation):** 
  - Ventilation Fan (Green)
  - Irrigation Pump (Blue)
  - Heater (Red - PWM)
  - Grow Light (Yellow - PWM)
- **Display:** SSD1306 OLED (I2C)
- **Communication:** USB-to-TTL (UART)

## 📌 Pin Mapping
| Component | STM32 Pin | Function |
| :--- | :--- | :--- |
| **DS18B20** | PA0 | One-Wire Data |
| **LDR** | PA1 | ADC1_IN1 |
| **Moisture Pot**| PA2 | ADC1_IN2 |
| **Fan LED** | PA3 | Digital Output |
| **Pump LED** | PA4 | Digital Output |
| **Heater LED** | PA6 | TIM3_CH1 (PWM) |
| **Light LED** | PA7 | TIM3_CH2 (PWM) |
| **Mode Button** | PB0 | GPIO_Input (Pull-up) |
| **Door Sensor** | PB1 | GPIO_Input (Pull-up) |
| **OLED SCL/SDA**| PB6 / PB7 | I2C1 |
| **UART TX/RX** | PA9 / PA10 | USART1 |

## 💻 Software Stack
- **IDE:** STM32CubeIDE
- **Configuration:** STM32CubeMX
- **Library:** HAL (Hardware Abstraction Layer)
- **Simulation:** Proteus 8.13

## 🚀 How to Run
1. **Simulation:**
   - Open the `.pdsprj` file in Proteus 8.13.
   - Double-click the STM32 MCU and load the `Greenhouse.elf` or `Greenhouse.hex` file.
   - Ensure the MCU clock is set to **72MHz**.
2. **Serial Control:**
   - Open the Virtual Terminal in Proteus.
   - Use 'F' to toggle Fan and 'P' to toggle Pump in MANUAL mode.

## ⚠️ Known Issues / Technical Notes
*   **OLED Display:** In the current simulation version, the OLED may not initialize correctly due to a driver protocol mismatch (Parallel vs. I2C command set).
*   **Timing Sensitivity:** The DS18B20 sensor is highly sensitive to the simulation time-step in Proteus; a stable 72MHz clock is mandatory for correct readings.
